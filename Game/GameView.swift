import MetalKit
import SwiftUI

class GameController: ObservableObject {
    weak var renderer: Renderer?

    struct Defaults {
        // Rope
        static let particleCount: Float = 60
        static let gravity: Float = -5.0
        static let damping: Float = 0.97
        // Solver
        static let constraintIterations: Float = 8
        static let settleSteps: Float = 5
        // Drag
        static let dragHeight: Float = 0.35
        static let liftHeight: Float = 0.30
        static let ropeTension: Float = 0.98
    }

    // Rope
    @Published var particleCount: Float = Defaults.particleCount {
        didSet { renderer?.physicsParticleCount = Int(particleCount); persist("p.ptc", particleCount) }
    }
    @Published var gravity: Float = Defaults.gravity {
        didSet { renderer?.physicsGravity = gravity; persist("p.grv", gravity) }
    }
    @Published var damping: Float = Defaults.damping {
        didSet { renderer?.physicsDamping = damping; persist("p.dmp", damping) }
    }

    // Solver
    @Published var constraintIterations: Float = Defaults.constraintIterations {
        didSet { renderer?.physicsConstraintIterations = Int(constraintIterations); persist("p.cit", constraintIterations) }
    }
    @Published var settleSteps: Float = Defaults.settleSteps {
        didSet { renderer?.physicsSettleSteps = Int(settleSteps); persist("p.stl", settleSteps) }
    }

    // Drag
    @Published var dragHeight: Float = Defaults.dragHeight {
        didSet { renderer?.dragHeight = dragHeight; persist("p.drg", dragHeight) }
    }
    @Published var liftHeight: Float = Defaults.liftHeight {
        didSet { renderer?.physicsLiftHeight = liftHeight; persist("p.lft", liftHeight) }
    }
    @Published var ropeTension: Float = Defaults.ropeTension {
        didSet { renderer?.physicsRopeTension = ropeTension; persist("p.rtn", ropeTension) }
    }

    @Published var fps: Float = 0
    @Published var currentLevel: Int = 1
    @Published var showLevelComplete: Bool = false

    private var fpsTimer: Timer?

    private func persist(_ key: String, _ v: Float) { UserDefaults.standard.set(v, forKey: key) }

    init() {
        loadSaved()
        fpsTimer = Timer.scheduledTimer(withTimeInterval: 0.25, repeats: true) { [weak self] _ in
            guard let self, let r = self.renderer else { return }
            self.fps = r.currentFPS
            if self.currentLevel != r.currentLevelId {
                self.currentLevel = r.currentLevelId
            }
        }
    }

    private func loadSaved() {
        let ud = UserDefaults.standard
        func f(_ k: String) -> Float? { ud.object(forKey: k) != nil ? ud.float(forKey: k) : nil }

        if let v = f("p.ptc") { particleCount = v }
        if let v = f("p.grv") { gravity = v }
        if let v = f("p.dmp") { damping = v }
        if let v = f("p.cit") { constraintIterations = v }
        if let v = f("p.stl") { settleSteps = v }
        if let v = f("p.drg") { dragHeight = v }
        if let v = f("p.lft") { liftHeight = v }
        if let v = f("p.rtn") { ropeTension = v }
    }

    private static let allKeys = [
        "p.ptc", "p.grv", "p.dmp", "p.cit", "p.stl", "p.drg", "p.lft", "p.rtn"
    ]

    func resetToDefaults() {
        particleCount = Defaults.particleCount
        gravity = Defaults.gravity
        damping = Defaults.damping
        constraintIterations = Defaults.constraintIterations
        settleSteps = Defaults.settleSteps
        dragHeight = Defaults.dragHeight
        liftHeight = Defaults.liftHeight
        ropeTension = Defaults.ropeTension
        Self.allKeys.forEach { UserDefaults.standard.removeObject(forKey: $0) }
    }

    func restartLevel() {
        guard let renderer = renderer else { return }
        renderer.loadLevel(levelId: renderer.currentLevelId)
    }

    func loadLevel(_ id: Int) {
        guard let renderer = renderer else { return }
        renderer.loadLevel(levelId: id)
        currentLevel = id
    }

    func dumpTopology() -> URL? {
        guard let renderer = renderer,
              let topology = renderer.topology else { return nil }

        let holes = renderer.holePositions.map {
            LevelDefinition.Vec2(xPosition: $0.x, yPosition: $0.y)
        }

        let levelRopes = renderer.ropes.map { rope in
            LevelDefinition.Rope(
                startHole: rope.startHole,
                endHole: rope.endHole,
                color: LevelDefinition.Color(
                    redChannel: rope.color.x,
                    greenChannel: rope.color.y,
                    blueChannel: rope.color.z
                ),
                radius: rope.radius
            )
        }

        var levelHooks: [LevelDefinition.Hook] = []
        for (_, hook) in topology.hooks.sorted(by: { $0.key < $1.key }) {
            guard hook.N != 0 else { continue }
            levelHooks.append(LevelDefinition.Hook(
                ropeA: .init(fromType: "hole", index: hook.ropeA, hookIndex: nil),
                ropeB: .init(fromType: "hole", index: hook.ropeB, hookIndex: nil),
                N: hook.N,
                ropeAStartIsOver: hook.ropeAStartIsOver
            ))
        }

        let def = LevelDefinition(
            id: renderer.currentLevelId,
            holeRadius: renderer.holeRadius,
            particlesPerRope: renderer.physicsParticleCount,
            holes: holes,
            ropes: levelRopes,
            hooks: levelHooks.isEmpty ? nil : levelHooks,
            actions: nil
        )

        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let data = try? encoder.encode(def) else { return nil }

        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        let ts = Int(Date().timeIntervalSince1970)
        let url = docs.appendingPathComponent("level_\(renderer.currentLevelId)_\(ts).json")
        try? data.write(to: url)
        return url
    }
}

struct GameView: UIViewRepresentable {
    @ObservedObject var controller: GameController

    func makeUIView(context: Context) -> MTKView {
        let view = GameMTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float_stencil8
        view.clearColor = MTLClearColor(red: 0.07, green: 0.08, blue: 0.12, alpha: 1.0)
        view.preferredFramesPerSecond = 120
        view.framebufferOnly = false
        view.isOpaque = true
        view.backgroundColor = .black
        view.alpha = 1.0

        let renderer = Renderer(view: view)
        context.coordinator.renderer = renderer
        controller.renderer = renderer
        renderer.onLevelComplete = { [weak controller] in
            DispatchQueue.main.async {
                controller?.showLevelComplete = true
                DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
                    controller?.showLevelComplete = false
                }
            }
        }

        renderer.physicsParticleCount = Int(controller.particleCount)
        renderer.physicsGravity = controller.gravity
        renderer.physicsDamping = controller.damping
        renderer.physicsConstraintIterations = Int(controller.constraintIterations)
        renderer.physicsSettleSteps = Int(controller.settleSteps)
        renderer.dragHeight = controller.dragHeight
        renderer.physicsLiftHeight = controller.liftHeight
        renderer.physicsRopeTension = controller.ropeTension

        view.delegate = renderer
        view.onTouch = { phase, location in
            renderer.handleTouch(phase: phase, location: location, in: view)
        }
        view.onCameraPan = { translation in
            renderer.handleCameraPan(translation: translation, in: view)
        }
        view.onCameraRotation = { delta in
            renderer.handleCameraRotation(delta: delta)
        }
        view.onCameraZoom = { scale in
            renderer.handleCameraZoom(scale: scale)
        }
        view.onCameraDebugToggle = {
            renderer.cameraDebugMode.toggle()
        }

        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {
        _ = uiView
        _ = context
    }

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    final class Coordinator {
        var renderer: Renderer?
    }
}
