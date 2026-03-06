import MetalKit
import os.log

@MainActor
final class Renderer: NSObject, MTKViewDelegate {
    static let logger = Logger(subsystem: "com.uzls.four", category: "Renderer")

    let device: MTLDevice
    let commandQueue: MTLCommandQueue
    let tablePipeline: MTLRenderPipelineState
    let holePipeline: MTLRenderPipelineState
    let ropePipeline: MTLRenderPipelineState
    let postPipeline: MTLRenderPipelineState
    let boardPipeline: MTLRenderPipelineState
    let shadowRopePipeline: MTLRenderPipelineState
    let shadowHolePipeline: MTLRenderPipelineState
    let shadowBoardPipeline: MTLRenderPipelineState
    let bloomThreshold: MTLComputePipelineState
    let bloomBlurH: MTLComputePipelineState
    let bloomBlurV: MTLComputePipelineState
    let bakeWoodPipeline: MTLComputePipelineState
    let bakeBoardWoodVolumePipeline: MTLComputePipelineState

    var depthStateScene: MTLDepthStencilState
    var depthStateBackground: MTLDepthStencilState
    var depthStateShadow: MTLDepthStencilState

    var shadowDepthTex: MTLTexture?
    let shadowMapSize: Int = 2048

    var camera = Camera()
    var cameraZoomScale: Float = 0.95764452219009399
    var cameraBaseOrthoHalfHeight: Float = 2.05
    var cameraDebugMode = false
    var cameraDebugTouchStart: CGPoint?
    var cameraDragActive = false
    var cameraDragStart: CGPoint?
    var time: Float = 0
    var dragVisualEnergy: Float = 0
    var lastDeltaTime: Float = 1.0 / 60.0
    var lastDrawTime: Double = 0
    var currentFPS: Float = 0


    var frameUniforms: MTLBuffer?
    var postParamsBuffer: MTLBuffer?
    var holeInstances: MTLBuffer?
    var holeVB: MTLBuffer?
    var holeIB: MTLBuffer?
    var holeIndexCount: Int = 0

    var holePositions: [SIMD2<Float>] = []
    var holeElevations: [Float] = []
    var holeRadius: Float = 0.105
    var holeOccupied: [Bool] = []
    var profileSegments: Int = 10
    var holeRadiusScale: Float = 0.73367023468017578 { didSet { rebuildHoleInstancesIfNeeded() } }
    var holeTint: SIMD4<Float> = SIMD4<Float>(1, 0.90870898962020874, 1, 1)
    var holeSegments: Int = 19 { didSet { rebuildHoleMeshIfNeeded() } }
    var ropeRadiusScale: Float = 1.0618677139282227
    var squareCrossSection: Bool = false {
        didSet {
            rebuildHoleMeshIfNeeded()
            simulator?.squareCrossSection = squareCrossSection
        }
    }
    var stretchThinning: Float = 0.5 {
        didSet { simulator?.stretchThinning = stretchThinning }
    }
    var exposure: Float = 0.5
    var lightIntensity: Float = 0.40238875150680542
    var lightDir: SIMD3<Float> = SIMD3<Float>(-0.029445827007293701, -0.22127896547317505, 0.87485504150390625)
    var bloomStrength: Float = 0
    var ambient: Float = 0.10394008457660675
    var shadowBias: Float = 0.0013682411517947912
    var shadowDarkness: Float = 0.15453849732875824
    var lightSize: Float = 0.073253527283668518
    var shadowsEnabled: Bool = true
    var shadowType: Int = 2
    var cartoonShaderEnabled: Bool = true
    var cartoonExposure: Float = 1.3291559219360352
    var cartoonBloom: Float = 0
    var cartoonEdgeStrength: Float = 1
    var cartoonLevels: Int = 2
    var cartoonShadowBright: Float = 0.38
    var cartoonWrap: Float = 0.15
    var cartoonEdgeSmooth: Float = 0.5
    var tableStyle: Int = 0
    var tableColor1: SIMD3<Float> = SIMD3<Float>(0.079999998211860657, 0.090000003576278687, 0.12999999523162842)
    var tableColor2: SIMD3<Float> = SIMD3<Float>(0.11999999731779099, 0.12999999523162842, 0.20000000298023224)
    var woodSeed: Float = 0.045950364321470261 { didSet { bakeWoodTexture() } }
    var woodBrightness: Float = 1.0 { didSet { bakeWoodTexture() } }
    var woodPatternScale: Float = 1.7154961824417114 { didSet { bakeWoodTexture() } }
    var boardElevation: Float = 0.12 {
        didSet {
            if levelLoaded { loadLevel(levelId: currentLevelId) }
        }
    }
    private var levelLoaded = false
    var capRadiusScale: Float = 0.90614998340606689
    var capSegments: Int = 12
    var capRings: Int = 6
    var capDarken: Float = 0
    var wormMode: Bool = false
    var wormSegFreq: Float = 28.0
    var wormSegBulge: Float = 0.12
    var wormThickness: Float = 1.35
    var wormTaperLen: Float = 0.12
    var wormGrooveDepth: Float = 0.35
    var wormBellyBright: Float = 1.15
    var wormBackDark: Float = 0.7
    var wormSkinNoise: Float = 0.08
    var wormSSS: Float = 0.25
    var wormRoughness: Float = 0.25
    var wormSpecular: Float = 0.8
    var wormRimStrength: Float = 0.08
    var wormEyeSize: Float = 0.015
    var wormPulseSpeed: Float = 2.5
    var wormPulseAmp: Float = 0.02
    var wormCrawlSpeed: Float = 3.5
    var wormCrawlAmp: Float = 0.012
    var wormSideAmp: Float = 0.008

    var ropeMatte: Float = 0.33392396569252014
    var ropeGloss: Float = 0.69268685579299927
    var ropeDiffuseWrap: Float = 0.0080702323466539383
    var ropeSubsurface: Float = 0.33765289187431335
    var ropeEdgeLight: Float = 0.41452157497406006
    var ropeSaturation: Float = 1.35732102394104
    var ropeMicroBump: Float = 0.142847940325737
    var ropeBumpScale: Float = 3.0
    var ropeContactAO: Float = 1
    var ropeLiftGlow: Float = 0.97152864933013916
    var ropeStretchGloss: Float = 0.7
    var ropeStretchSpec: Float = 1.0
    var ropeEnvReflect: Float = 0.15
    var ropeEnvSpread: Float = 0.15
    var ropeEnvDebug: Bool = false
    var shadowDebugMode: Int = 0
    struct RopeEndpoints {
        var startHole: Int
        var endHole: Int
        var color: SIMD3<Float>
        var radius: Float
        var crossSection: CrossSection
    }

    var ropes: [RopeEndpoints] = []
    var topology: TopologyEngine?
    var simulator: VerletSimulator?
    var lastDragWorld: SIMD2<Float> = .zero
    var dragStartWorld: SIMD2<Float> = .zero

    struct DragState {
        var ropeIndex: Int
        var endIndex: Int
        var originalHoleIndex: Int
    }

    var dragState: DragState?
    var dragWorld: SIMD2<Float> = .zero
    var dragHeight: Float = 0.34999999403953552
    var highlightHoleIndex: Int = -1

    var ropeVB: MTLBuffer?
    var ropeIB: MTLBuffer?
    var ropeIndexCount: Int = 0

    var hdrTex: MTLTexture?
    var envTex: MTLTexture?
    var sceneDepthTex: MTLTexture?
    var bloomA: MTLTexture?
    var bloomB: MTLTexture?
    var bakedWoodTex: MTLTexture?
    var bakedBoardWoodVolumeTex: MTLTexture?
    var noiseTexture: MTLTexture?
    var woodBoundsMin: SIMD2<Float> = .zero
    var woodBoundsMax: SIMD2<Float> = .zero
    var boardWoodZMin: Float = 0
    var boardWoodZMax: Float = 1
    var lastViewSize: CGSize = CGSize(width: 400, height: 600)
    var renderScale: Float = 1.0 {
        didSet { invalidateOffscreenTextures() }
    }
    
    var boards: [LevelDefinition.Board] = []
    var boardMeshVB: MTLBuffer?
    var boardMeshIB: MTLBuffer?
    var boardMeshIndexCount: Int = 0

    var lastPhysicsLogTime: Double = 0
    let frictionSound = RubberFrictionSound()

    struct MeshStats: Equatable {
        let vertices: Int
        let indices: Int
        let ropeCount: Int
    }
    var lastMeshStats: MeshStats?

    var currentLevelId: Int = 1
    var moveCount: Int = 0
    var onLevelComplete: (() -> Void)?
    var onMoveCountChanged: ((Int) -> Void)?
    var onUndoStackChanged: ((Bool) -> Void)?
    var onZoomChanged: ((Float) -> Void)?

    // MARK: - Undo

    struct UndoEntry {
        var simulatorSnapshot: VerletSimulator.Snapshot
        var ropeEndpoints: [RopeEndpoints]
        var holeOccupied: [Bool]
        var moveCount: Int
    }

    private var undoStore = RendererUndoStore<UndoEntry>()
    var levelFlow = RendererLevelFlowCoordinator(settleCheckDelay: 0.5)

    var canUndo: Bool { undoStore.canUndo }

    func pushUndoState() {
        guard let sim = simulator else { return }
        let entry = UndoEntry(
            simulatorSnapshot: sim.takeSnapshot(),
            ropeEndpoints: ropes,
            holeOccupied: holeOccupied,
            moveCount: moveCount
        )
        undoStore.push(entry)
        onUndoStackChanged?(true)
    }

    func performUndo() {
        guard let entry = undoStore.pop(), let sim = simulator else { return }
        dragState = nil
        highlightHoleIndex = -1
        levelFlow.cancelSettleCheck()
        sim.restoreSnapshot(entry.simulatorSnapshot)
        ropes = entry.ropeEndpoints
        holeOccupied = entry.holeOccupied
        moveCount = entry.moveCount
        onMoveCountChanged?(moveCount)
        onUndoStackChanged?(undoStore.canUndo)
    }
    
    // Physics parameters (forwarded to simulator)
    var physicsGravity: Float = -5.0 {
        didSet { simulator?.gravity = physicsGravity }
    }
    var physicsDamping: Float = 0.97 {
        didSet { simulator?.damping = physicsDamping }
    }
    var physicsConstraintIterations: Int = 8 {
        didSet { simulator?.constraintIterations = physicsConstraintIterations }
    }
    var physicsParticleCount: Int = 60 {
        didSet { simulator?.particleCount = physicsParticleCount }
    }
    var physicsSettleSteps: Int = 5 {
        didSet { simulator?.settleSteps = physicsSettleSteps }
    }
    var physicsLiftHeight: Float = 0.30 {
        didSet { simulator?.liftHeight = physicsLiftHeight }
    }
    var physicsRopeTension: Float = 0.98 {
        didSet { simulator?.ropeTension = physicsRopeTension }
    }
    var physicsBendCompliance: Float = 0.0015 {
        didSet { simulator?.bendCompliance = physicsBendCompliance }
    }
    var physicsBendVelocityCoupling: Float = 0.45 {
        didSet { simulator?.bendVelocityCoupling = physicsBendVelocityCoupling }
    }

    init(view: MTKView) {
        guard let device = view.device else { fatalError("Metal device is required") }
        guard let commandQueue = device.makeCommandQueue() else { fatalError("Command queue is required") }

        self.device = device
        self.commandQueue = commandQueue

        guard let library = device.makeDefaultLibrary() else {
            fatalError("Failed to create default Metal library")
        }

        self.tablePipeline = Self.makeTablePipeline(device: device, view: view, library: library)
        self.holePipeline = Self.makeHolePipeline(device: device, view: view, library: library)
        self.ropePipeline = Self.makeRopePipeline(device: device, view: view, library: library)
        self.postPipeline = Self.makePostPipeline(device: device, view: view, library: library)
        self.boardPipeline = Self.makeBoardPipeline(device: device, view: view, library: library)
        self.shadowRopePipeline = Self.makeShadowRopePipeline(device: device, library: library)
        self.shadowHolePipeline = Self.makeShadowHolePipeline(device: device, library: library)
        self.shadowBoardPipeline = Self.makeShadowBoardPipeline(device: device, library: library)
        (self.bloomThreshold, self.bloomBlurH, self.bloomBlurV) = Self.makeBloomPipelines(device: device, library: library)
        self.bakeWoodPipeline = Self.makeComputePipeline(device: device, function: library.makeFunction(name: "bakeWoodKernel")!)
        self.bakeBoardWoodVolumePipeline = Self.makeComputePipeline(device: device, function: library.makeFunction(name: "bakeBoardWoodVolumeKernel")!)
        (self.depthStateScene, self.depthStateBackground, self.depthStateShadow) = Self.makeDepthStates(device: device)

        self.frameUniforms = device.makeBuffer(length: MemoryLayout<FrameUniforms>.stride, options: [.storageModeShared])
        self.postParamsBuffer = device.makeBuffer(length: MemoryLayout<PostParams>.stride, options: [.storageModeShared])
        Self.buildHoleMeshBuffers(device: device, segments: holeSegments, square: squareCrossSection, vertexBuffer: &holeVB, indexBuffer: &holeIB, indexCount: &holeIndexCount)

        self.noiseTexture = Self.generateNoiseTexture(device: device, size: 2048)

        super.init()
    }

    func loadLevel(levelId: Int) {
        let loadStart = CACurrentMediaTime()
        currentLevelId = levelId
        Self.logger.info("Loading level \(levelId)...")
        
        dragState = nil
        dragWorld = .zero
        simulator = nil
        undoStore.clear()
        levelFlow.clearAll()
        moveCount = 0
        onMoveCountChanged?(0)
        onUndoStackChanged?(false)
        
        var t0 = CACurrentMediaTime()
        let level: LevelDefinition
        if let jsonLevel = LevelLoader.load(levelId: levelId) {
            Self.logger.info("Level \(levelId) loaded from JSON: \(jsonLevel.ropes.count) ropes, \(jsonLevel.holes.count) holes")
            level = jsonLevel
        } else {
            Self.logger.info("Level \(levelId) generated procedurally")
            level = LevelGenerator.generate(levelId: levelId, boardElevation: boardElevation)
        }
        let tGenerate = CACurrentMediaTime()

        let levelHoles = level.holes.map { $0.simd }
        let levelHoleElevations = level.holes.map { $0.zPosition }
        let levelHoleRadius = level.holeRadius

        let validatedRopes = level.ropes.filter { rope in
            guard levelHoles.indices.contains(rope.startHole) else { return false }
            guard levelHoles.indices.contains(rope.endHole) else { return false }
            return rope.startHole != rope.endHole
        }

        holePositions = levelHoles
        holeElevations = levelHoleElevations
        holeRadius = levelHoleRadius
        holeOccupied = Array(repeating: false, count: levelHoles.count)
        boards = level.boards ?? []
        rebuildHoleInstances()
        rebuildHoleMeshIfNeeded()
        rebuildBoardMesh()
        let tMesh = CACurrentMediaTime()

        ropes = validatedRopes.map { rope in
            RopeEndpoints(startHole: rope.startHole, endHole: rope.endHole, color: rope.color.simd, radius: rope.radius, crossSection: rope.crossSection)
        }

        let ropeConfigs = ropes.map { rope in
            (startHole: rope.startHole, endHole: rope.endHole, color: rope.color)
        }
        let hookDefs: [TopologyEngine.HookDefinition]? = level.hooks.flatMap { jsonHooks in
            jsonHooks.compactMap { hook -> TopologyEngine.HookDefinition? in
                guard let ropeAIdx = Self.resolveRopeRef(hook.ropeA, hooks: jsonHooks),
                      let ropeBIdx = Self.resolveRopeRef(hook.ropeB, hooks: jsonHooks) else {
                    Self.logger.warning("Failed to resolve hook rope references")
                    return nil
                }
                return TopologyEngine.HookDefinition(
                    ropeA: ropeAIdx,
                    ropeB: ropeBIdx,
                    N: hook.N,
                    ropeAStartIsOver: hook.ropeAStartIsOver
                )
            }
        }
        topology = TopologyEngine(holePositions: levelHoles, ropeConfigs: ropeConfigs, hookDefinitions: hookDefs)
        let tTopology = CACurrentMediaTime()

        let sim = VerletSimulator(holePositions: levelHoles, holeElevations: levelHoleElevations, holeRadius: levelHoleRadius, boards: boards)
        sim.gravity = physicsGravity
        sim.damping = physicsDamping
        sim.constraintIterations = physicsConstraintIterations
        sim.particleCount = physicsParticleCount
        sim.settleSteps = physicsSettleSteps
        sim.liftHeight = physicsLiftHeight
        sim.ropeTension = physicsRopeTension
        sim.bendCompliance = physicsBendCompliance
        sim.bendVelocityCoupling = physicsBendVelocityCoupling
        sim.stretchThinning = stretchThinning
        sim.squareCrossSection = squareCrossSection

        let simRopeConfigs = ropes.map { rope in
            VerletSimulator.RopeConfig(startHole: rope.startHole, endHole: rope.endHole, radius: rope.radius, crossSection: rope.crossSection)
        }

        let simActions: [VerletSimulator.LevelAction] = level.actions?.compactMap { action in
            guard let actionType = VerletSimulator.LevelAction.ActionType(rawValue: action.type) else { return nil }
            return VerletSimulator.LevelAction(type: actionType, ropeIndex: action.ropeIndex, endIndex: action.endIndex, holeIndex: action.holeIndex)
        } ?? []

        t0 = CACurrentMediaTime()
        sim.initializeLevel(ropeConfigs: simRopeConfigs, actions: simActions)
        let tPhysics = CACurrentMediaTime()
        self.simulator = sim

        for ropeIndex in ropes.indices {
            guard sim.bands.indices.contains(ropeIndex) else { continue }
            let band = sim.bands[ropeIndex]
            if let pinStart = band.pinStart {
                ropes[ropeIndex].startHole = pinStart
            }
            if let pinEnd = band.pinEnd {
                ropes[ropeIndex].endHole = pinEnd
            }
        }

        for ropeIndex in ropes.indices {
            let startHoleIndex = ropes[ropeIndex].startHole
            let endHoleIndex = ropes[ropeIndex].endHole
            guard holeOccupied.indices.contains(startHoleIndex), holeOccupied.indices.contains(endHoleIndex) else { continue }
            holeOccupied[startHoleIndex] = true
            holeOccupied[endHoleIndex] = true
        }

        let aspect = Float(lastViewSize.width / max(1, lastViewSize.height))
        let maxElev = holeElevations.max() ?? 0
        camera.fitToHoles(holePositions, holeRadius: holeRadius, aspect: aspect, maxElevation: maxElev)
        cameraBaseOrthoHalfHeight = camera.orthoHalfHeight
        camera.orthoHalfHeight = cameraBaseOrthoHalfHeight / cameraZoomScale

        let tCamera = CACurrentMediaTime()
        bakeWoodTexture()
        bakeBoardWoodVolumeTexture()
        let tWood = CACurrentMediaTime()

        let ms = { (a: Double, b: Double) -> String in String(format: "%.1f", (b - a) * 1000) }
        Self.logger.warning("""
            [LOAD-PROFILE] level=\(levelId) ropes=\(validatedRopes.count) holes=\(levelHoles.count) actions=\(simActions.count) \
            generate=\(ms(loadStart, tGenerate))ms mesh=\(ms(tGenerate, tMesh))ms topology=\(ms(tMesh, tTopology))ms \
            physics=\(ms(t0, tPhysics))ms camera=\(ms(tPhysics, tCamera))ms wood=\(ms(tCamera, tWood))ms \
            TOTAL=\(ms(loadStart, tWood))ms
            """)
        levelLoaded = true
    }

    /// Resolves a HookRopeRef to a rope index.
    /// - "hole": index is the rope index directly
    /// - "hook": follows the chain through referenced hook's ropeA (hookIndex=0) or ropeB (hookIndex=1)
    private static func resolveRopeRef(_ ref: LevelDefinition.HookRopeRef, hooks: [LevelDefinition.Hook], depth: Int = 0) -> Int? {
        guard depth < 16 else {
            logger.error("resolveRopeRef: recursion depth exceeded")
            return nil
        }
        switch ref.fromType {
        case "hole":
            return ref.index
        case "hook":
            guard hooks.indices.contains(ref.index) else {
                logger.error("resolveRopeRef: hook index \(ref.index) out of range")
                return nil
            }
            let refHook = hooks[ref.index]
            let side = ref.hookIndex ?? 0
            let sideRef = (side == 0) ? refHook.ropeA : refHook.ropeB
            return resolveRopeRef(sideRef, hooks: hooks, depth: depth + 1)
        default:
            logger.error("resolveRopeRef: unknown fromType '\(ref.fromType)'")
            return nil
        }
    }

}
