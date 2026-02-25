# UzlsFour — Verlet Rope Physics Puzzle

## Overview

iOS-игра с резинками (rubber bands), натянутыми между дырками на доске. Резинки симулируются физическим движком (Verlet integration + PBD constraints). Пользователь перетаскивает концы резинок между дырками, распутывая их. Когда резинка больше не пересекает другие — она исчезает с fade-out анимацией. Уровень пройден когда все резинки убраны.

## Architecture

```
App entry → ContentView → GameView (UIViewRepresentable) → Renderer (Metal)
                ↕                                              ↕
         GameController                               VerletSimulator
        (physics params,                           (Verlet integration,
         level state,                               constraints, collision,
         FPS counter)                               drag handling)
                                                        ↕
                                                   RopePhysics
                                                  (linking numbers,
                                                   pass-through detection)
```

### Data Flow

1. Level loading: JSON (`LevelLoader`) или процедурная генерация (`LevelGenerator`) → `LevelDefinition`
2. Initialization: `Renderer.loadLevel()` → `VerletSimulator.initializeLevel(actions:)` — реплеит pin/drag actions
3. Physics loop: `Renderer.draw()` → `simulator.update(deltaTime:)` → `updateRopeMesh()` → Metal rendering
4. Input: Touch → `GameMTKView` → `Renderer.handleTouch()` → `simulator.beginDrag/updateDrag/endDrag()`
5. Win check: `removeUntangledRopes()` → 2D crossing check → fade-out → level complete

## Physics Engine — VerletSimulator

Файл: `Game/Simulation/VerletSimulator.swift`

### Band (rope state)

```
Band:
  pos: [SIMD3<Float>]         — текущие позиции частиц
  old: [SIMD3<Float>]         — предыдущие позиции (для Verlet)
  segLen: Float               — длина сегмента (натуральная)
  radius: Float               — радиус для коллизий и рендеринга
  color: SIMD3<Float>         — цвет
  pins: [Int: SIMD3<Float>]   — закреплённые частицы (индекс → позиция)
  active: Bool                — активна ли
  fadeOut: Float?              — прогресс fade-out анимации
```

### Fixed Timestep

Accumulator-based, фиксированный dt = 1/120s. Внутри каждого шага:
1. Verlet integration: `new = pos + (pos - old) * damping + gravity * dt²`
2. Constraint solving (N итераций):
   - Distance constraints с tension (сохранение длины сегментов)
   - Pin constraints (фиксация концов в дырках)
   - Board collision (Z ≥ 0, частицы не проваливаются сквозь доску)
3. Collision detection & resolution:
   - Broadphase: AABB sweep по сегментам
   - Narrowphase: segment-segment 3D collision с учётом радиусов

### Collision — collideSegments()

3D segment-segment collision:
1. Находим ближайшие точки двух отрезков (параметры s, t ∈ [0,1])
2. Если расстояние < сумма радиусов → push apart вдоль нормали
3. Коррекция применяется к обоим сегментам (если не pinned)

### Drag System

- `beginDrag(bandIndex:, endIndex:)` — начало перетаскивания, запоминает начальную позицию
- `updateDrag(to:)` — интерполяция позиции с lift height (поднимает конец над доской)
- `endDrag(holePosition:)` — snap к дырке, запуск settle-анимации (LowerAnimation)
- `LowerAnimation` — плавное опускание конца в дырку после отпускания

### Level Initialization — initializeLevel(actions:)

Уровни инициализируются через последовательность actions:
1. **Pin actions**: создаёт Band, закрепляет концы в дырках, settle с коллизией
2. **Drag actions**: симулирует перетаскивание конца к другой дырке (создаёт запутывание)

`simulateDrag()` — реплеит drag action: поднимает конец, двигает по дуге к целевой дырке, опускает. Между шагами — settle с физикой.

### Ключевые параметры

```
gravity = -5.0                — гравитация (Z вниз)
damping = 0.97                — демпфирование скорости
constraintIterations = 8      — итераций constraint solver
ropeTension = 0.98            — множитель натяжения (< 1 = резинка короче натуральной длины)
particleCount = 6..60         — частиц на резинку
settleSteps = 300             — шагов settle при инициализации
dragHeight = 0.6              — высота подъёма при драге
liftHeight = 0.3              — высота подъёма при settle
```

## RopePhysics — Linking Numbers

Файл: `Game/Simulation/RopePhysics.swift`

Вычисляет signed linking number между парами резинок:
- 2D crossing detection по сегментам
- Z-depth ordering в точках пересечения определяет знак (+1 или -1)
- Суммарный linking number = количество пересечений с учётом знака
- `logStateIfNeeded()` — логирует изменения топологии и pass-throughs

## Rendering Pipeline

### Passes

1. **Shadow pass** — depth map с позиции источника света (PCSS shadows)
2. **HDR pass** — table (процедурная текстура дерева), holes (instanced), ropes → HDR texture
3. **Bloom pass** — compute shaders: threshold → horizontal blur → vertical blur
4. **Composite pass** — HDR + bloom → screen с tone mapping

### Rope Mesh — RopeMeshBuilder

Файл: `Game/Renderer/RopeMeshBuilder.swift`

`buildRect()` генерирует цилиндрический mesh резинки:
- Circular profile (16 сегментов)
- Frenet-Serret frame для ориентации
- Twist events support
- Stretch/pinch эффекты (latex deformation)
- Цвет зависит от натяжения

### Hole Mesh — HoleMeshBuilder

Файл: `Game/Renderer/HoleMeshBuilder.swift`

Генерирует mesh дырки: top ring, wall, bottom cap.
- segments = 48, innerRadius = 0.76, outerRadius = 1.0, depth = 1.25

### Metal Shaders — Base.metal

Файл: `Shaders/Base.metal`

- `ropeVertex/ropeFragment` — rubber material с matte shading
- `holeVertex/holeFragment` — hole shading с тенями
- `tableFragment` — процедурная текстура дерева (FBM noise)
- `bloomThreshold/bloomBlurH/bloomBlurV` — compute bloom
- `postFragment` — HDR tone mapping + bloom composite
- `shadowVisibility()` — PCSS shadow mapping

## Win Condition

Файл: `Game/Renderer/Renderer+WinCheck.swift`

1. `removeUntangledRopes()` — проверяет все активные резинки
2. `isRopeUntangled()` — резинка распутана если нет 2D пересечений с другими активными резинками (`segmentsCross2D()`)
3. Распутанная резинка → `startFadeOut()` → fade-out анимация → деактивация
4. `checkLevelComplete()` — все резинки деактивированы → victory

## Level System

### LevelDefinition

Файл: `Game/Level/LevelDefinition.swift`

```
LevelDefinition:
  id: Int
  holeRadius: Float
  particlesPerRope: Int
  holes: [Vec2]              — позиции дырок
  ropes: [Rope]              — startHole, endHole, color, radius
  hooks: [Hook]?             — опционально: предзаданные зацепы (legacy)
  actions: [Action]?         — последовательность pin/drag для инициализации

Action:
  type: "pin" | "drag"
  ropeIndex: Int
  endIndex: Int              — 0 (start) или 1 (end)
  holeIndex: Int             — целевая дырка
```

### LevelLoader

Файл: `Game/Level/LevelLoader.swift`

Загружает `level_XXX.json` из bundle (папка `levels/` или корень).

### LevelGenerator — процедурная генерация

Файл: `Game/Level/LevelGenerator.swift`

Детерминистичная генерация через `SeededRNG`:
1. `HoleLayout` — 8 вариантов раскладки: grid4x5, circle12, hexagon, diamond, cross, twoRings, triangle, star
2. `difficulty(levelId:)` — количество резинок и драгов по уровню
3. `pickStructuredPairs()` — пары примерно противоположных дырок (star pattern)
4. Генерирует actions: сначала pin всех резинок, потом drag'и для создания запутывания
5. Drag strategy: выбирает целевую резинку для пересечения, двигает конец на противоположную сторону

## TopologyEngine (legacy, частично используется)

Файл: `Game/Topology/TopologyEngine.swift`

Старая система трекинга топологии через HookSequence. Сейчас используется для:
- Загрузки hooks из JSON уровней
- `buildInitialHooks()` — построение начальных hooks из пересечений

Win detection теперь использует physics-based 2D crossing check вместо topology.

## Input System

Файл: `Game/GameMTKView.swift`

- Single touch: drag резинок
- Two-finger: camera pan/rotation/zoom
- Triple-tap: toggle camera debug mode

`Renderer+Interaction.swift`:
- `beginDrag()` — находит ближайший конец резинки, проверяет Z-order для выбора верхнего
- `endDrag()` — snap к ближайшей дырке
- `screenToWorld()` — конвертация экранных координат в мировые

## UI — ContentView

Файл: `App/ContentView.swift`

- `GameController: ObservableObject` — управление параметрами физики и состоянием
- Level picker (1–200)
- Physics controls (3 вкладки: Rope, Solver, Drag)
- Restart / Dump topology кнопки
- Victory overlay с переходом на следующий уровень

## Haptics

Файл: `Game/Haptics.swift`

`light()`, `medium()`, `success()` — тактильная обратная связь.

## File Structure

```
App/
  UzlsFourApp.swift              — @main entry point
  ContentView.swift              — UI, GameController, physics controls

Game/
  GameView.swift                 — UIViewRepresentable wrapper для MTKView
  GameMTKView.swift              — custom MTKView с touch handling
  Haptics.swift                  — тактильная обратная связь

  Simulation/
    VerletSimulator.swift        — Verlet integration, constraints, collision, drag
    RopePhysics.swift            — linking number computation, pass-through detection

  Renderer/
    Renderer.swift               — Metal setup, pipelines, state
    Renderer+MTKViewDelegate.swift — render loop, passes
    Renderer+Interaction.swift   — touch → drag, camera controls
    Renderer+RopeMesh.swift      — rope mesh from physics positions
    Renderer+WinCheck.swift      — untangle detection, fade-out
    Renderer+Bloom.swift         — bloom post-processing
    Renderer+Holes.swift         — hole instances
    FrameTypes.swift             — Metal shader data structures
    Camera.swift                 — camera system (ortho, lookAt)
    RopeMeshBuilder.swift        — cylindrical rope mesh generation
    HoleMeshBuilder.swift        — hole mesh generation

  Level/
    LevelDefinition.swift        — level data structures
    LevelLoader.swift            — JSON level loader
    LevelGenerator.swift         — procedural level generation

  Topology/
    TopologyEngine.swift         — legacy topology tracking
    TopologyTypes.swift          — topology data structures
    SegmentIntersection.swift    — 2D segment intersection utility

  Utils/
    Array+Safe.swift             — safe array subscript

Shaders/
  Base.metal                     — all Metal shaders

levels/
  level_001.json                 — 20 holes (4x5 grid), 3 ropes
  level_002.json                 — 20 holes, 4 ropes
  level_003.json                 — 16 holes (diamond), 4 ropes

tests/
  LevelGeneratorTests.swift      — level generation tests
```
