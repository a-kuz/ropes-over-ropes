import MetalKit
#if os(iOS)
import CoreMotion
#endif
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
    let copyDepthPipeline: MTLComputePipelineState
    let bakeWoodPipeline: MTLComputePipelineState
    let bakeBoardWoodVolumePipeline: MTLComputePipelineState
    let debug2DPipeline: MTLRenderPipelineState

    var depthStateScene: MTLDepthStencilState
    var depthStateBackground: MTLDepthStencilState
    var depthStateShadow: MTLDepthStencilState

    var shadowDepthTex: MTLTexture?
    var shadowMapSize: Int = 2048 {
        didSet { if oldValue != shadowMapSize { resizeShadowTexture() } }
    }

    var camera = Camera()
    var cameraZoomScale: Float = 0.95764452219009399
    var cameraBaseOrthoHalfHeight: Float = 2.05
    var cameraDragActive = false
    var cameraDragStart: CGPoint?
    var time: Float = 0
    var lastDeltaTime: Float = 1.0 / 60.0
    var lastDrawTime: Double = 0
    var currentFPS: Float = 0
    var potentialFPS: Float = 0

    func updatePotentialFPS(_ fps: Float) {
        if potentialFPS == 0 {
            potentialFPS = fps
        } else {
            potentialFPS = potentialFPS * 0.95 + fps * 0.05
        }
    }


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
    var holeRadiusScale: Float = 0.73367023468017578 {
        didSet {
            rebuildHoleInstancesIfNeeded()
            if levelLoaded { bakeHoleMaskTexture() }
        }
    }
    var holeSegments: Int = 19 { didSet { rebuildHoleMeshIfNeeded() } }
    var ropeRadiusScale: Float = 1.0618677139282227
    var squareCrossSection: Bool = false {
        didSet {
            rebuildHoleMeshIfNeeded()
            simulator?.squareCrossSection = squareCrossSection
            if levelLoaded { bakeHoleMaskTexture() }
        }
    }
    var stretchThinning: Float = 0.5 {
        didSet { simulator?.stretchThinning = stretchThinning }
    }

    // MARK: - Shader / visual parameters (extracted)
    var shaderParams = RendererShaderParams()

    // Forwarding properties for params with side effects
    var woodSeed: Float {
        get { shaderParams.woodSeed }
        set { shaderParams.woodSeed = newValue; bakeWoodTexture() }
    }
    var woodBrightness: Float {
        get { shaderParams.woodBrightness }
        set { shaderParams.woodBrightness = newValue; bakeWoodTexture() }
    }
    var woodPatternScale: Float {
        get { shaderParams.woodPatternScale }
        set { shaderParams.woodPatternScale = newValue; bakeWoodTexture() }
    }

    var boardElevation: Float = 0.12 {
        didSet {
            if levelLoaded { loadLevel(levelId: currentLevelId) }
        }
    }
    private var levelLoaded = false
    var debug2DOverlay: Bool = false
    var debug2DLineVB: MTLBuffer?
    var debug2DLineCount: Int = 0
    var lastDebug2DLogTime: Double = 0
    struct RopeEndpoints {
        var startHole: Int
        var endHole: Int
        var color: SIMD3<Float>
        var radius: Float
        var crossSection: CrossSection
    }

    var ropes: [RopeEndpoints] = []
    var simulator: VerletSimulator?
    var lastDragWorld: SIMD2<Float> = .zero
    var dragInputResponse: Float = 20.0

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
    var envDepthTex: MTLTexture?
    var envDisabledTex: MTLTexture
    var woodDebugTex: MTLTexture
    var holeMaskDisabledTex: MTLTexture
    var sceneDepthTex: MTLTexture?
    var bloomA: MTLTexture?
    var bloomB: MTLTexture?
    var bakedWoodTex: MTLTexture?
    var bakedHoleMaskTex: MTLTexture?
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

    let frictionSound = RubberFrictionSound()

    #if os(iOS)
    nonisolated(unsafe) let motionManager = CMMotionManager()
    #endif
    var deviceTilt: SIMD2<Float> = .zero

    struct MeshStats: Equatable {
        let vertices: Int
        let indices: Int
        let ropeCount: Int
    }
    var lastMeshStats: MeshStats?

    var currentLevelId: Int = 1
    var moveCount: Int = 0

    // Recording: captures drag actions for braid development
    var isRecording: Bool = false
    var recordedActions: [(ropeIndex: Int, endIndex: Int, fromHole: Int, toHole: Int)] = []
    var isTensionMode: Bool = false
    var tensionLevelCompleted: Bool = false

    // Tension mode rendering
    struct WeightRenderInfo {
        var position: SIMD2<Float>
        var radius: Float
        var settled: Bool
    }
    struct TargetRenderInfo {
        var position: SIMD2<Float>
        var radius: Float
        var weightIndex: Int
        var satisfied: Bool
    }
    var weightRenderInfos: [WeightRenderInfo] = []
    var targetRenderInfos: [TargetRenderInfo] = []
    var weightVB: MTLBuffer?
    var weightIB: MTLBuffer?
    var weightIndexCount: Int = 0
    var targetVB: MTLBuffer?
    var targetIB: MTLBuffer?
    var targetIndexCount: Int = 0

    // Braid mode
    var isBraidMode: Bool = false
    var braidLevelCompleted: Bool = false
    var braidTargets: [Int] = []        // braidTargets[ropeIndex] = target bottom hole index
    var braidMinCrossings: Int = 0
    var braidBottomHoleStart: Int = 0   // index of first bottom hole
    var braidStrandCount: Int = 0

    // Slow braid replay: execute drag actions one at a time so the user can watch
    var pendingBraidDrags: [VerletSimulator.LevelAction] = []
    var braidDragTimer: Float = 0
    let braidDragInterval: Float = 0.3  // seconds pause between drags

    // Animated drag state machine
    enum BraidDragPhase { case idle, lift, traverse, lower, settle }
    var braidDragPhase: BraidDragPhase = .idle
    var braidDragStep: Int = 0
    var braidDragAction: VerletSimulator.LevelAction?
    var braidDragFromPos: SIMD3<Float> = .zero
    var braidDragLiftFrom: SIMD3<Float> = .zero
    var braidDragLiftTo: SIMD3<Float> = .zero
    var braidDragToPos: SIMD3<Float> = .zero
    var braidDragIdx: Int = 0 // particle index (0 or last)

    // Runtime braid computation: computes drag targets from actual particle positions
    var braidRemainingCrossings: Int = 0
    // isLower[ropeIndex][endIndex] — true = this end is under the other rope
    var braidIsLower: [[Bool]] = [[false, false], [true, true]]

    // Rail mode rendering
    var isRailMode: Bool = false
    var railLevelCompleted: Bool = false
    struct RailRenderInfo {
        var points: [SIMD2<Float>]
    }
    struct CartRenderInfo {
        var position: SIMD2<Float>
        var radius: Float
        var settled: Bool
    }
    struct StationRenderInfo {
        var position: SIMD2<Float>
        var radius: Float
        var cartIndex: Int
        var satisfied: Bool
    }
    var railRenderInfos: [RailRenderInfo] = []
    var cartRenderInfos: [CartRenderInfo] = []
    var stationRenderInfos: [StationRenderInfo] = []
    var railVB: MTLBuffer?
    var railIB: MTLBuffer?
    var railIndexCount: Int = 0
    var cartVB: MTLBuffer?
    var cartIB: MTLBuffer?
    var cartIndexCount: Int = 0
    var stationVB: MTLBuffer?
    var stationIB: MTLBuffer?
    var stationIndexCount: Int = 0

    // Victory camera orbit
    var victoryOrbitActive: Bool = false
    var victoryOrbitTime: Float = 0
    let victoryOrbitDuration: Float = 5.0        // orbit duration (UI shown immediately, this is just visual)
    var victoryOrbitSavedCamera: Camera?

    // Victory replay
    struct ReplayMove {
        var ropeIndex: Int
        var endIndex: Int
        var fromHole: Int
        var toHole: Int
    }
    var moveHistory: [ReplayMove] = []
    var victoryReplayPending: Bool = false
    var victoryReplayMovesBuffer: [ReplayMove] = []
    var victoryReplayActive: Bool = false
    var victoryReplayMoves: [ReplayMove] = []
    var victoryReplayMoveIndex: Int = 0
    enum ReplayPhase { case pause, lift, traverse, lower, settle, done }
    var victoryReplayPhase: ReplayPhase = .done
    var victoryReplayStep: Int = 0
    var victoryReplayFromPos: SIMD3<Float> = .zero
    var victoryReplayLiftFrom: SIMD3<Float> = .zero
    var victoryReplayLiftTo: SIMD3<Float> = .zero
    var victoryReplayToPos: SIMD3<Float> = .zero
    var victoryReplayParticleIdx: Int = 0
    let replayPauseFrames: Int = 15
    let replayLiftFrames: Int = 30
    let replayTraverseFrames: Int = 70
    let replayLowerFrames: Int = 11
    let replaySettleFrames: Int = 30
    // ropeIndex → moveHistory.count at moment of vanish (recorded during gameplay)
    var replayVanishRecords: [Int: Int] = [:]
    // copy used during active replay
    var victoryReplayVanishAtMove: [Int: Int] = [:]
    // waiting for suck animation to finish before starting replay
    var victoryWaitingForSuck: Bool = false

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
    var physicsBroadphaseInterval: Int = 3 {
        didSet { simulator?.broadphaseRebuildInterval = physicsBroadphaseInterval }
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
    var physicsFriction: Float = 0.8 {
        didSet { simulator?.frictionCoefficient = physicsFriction }
    }
    var physicsCollisionResponse: Float = 0.35 {
        didSet { simulator?.collisionResponse = physicsCollisionResponse }
    }
    var physicsZSeparation: Float = 1.0 {
        didSet { simulator?.zSeparationStrength = physicsZSeparation }
    }
    var physicsFrictionDampingRatio: Float = 0.3 {
        didSet { simulator?.frictionDampingRatio = physicsFrictionDampingRatio }
    }
    var physicsMaxFrictionCap: Float = 0.25 {
        didSet { simulator?.maxFrictionCap = physicsMaxFrictionCap }
    }
    var physicsBoardFrictionRatio: Float = 0.5 {
        didSet { simulator?.boardFrictionRatio = physicsBoardFrictionRatio }
    }
    var physicsTwistStiffness: Float = 0.15 {
        didSet { simulator?.twistStiffness = physicsTwistStiffness }
    }
    var physicsTwistDamping: Float = 0.4 {
        didSet { simulator?.twistDamping = physicsTwistDamping }
    }
    var physicsGravityTorque: Float = 0.8 {
        didSet { simulator?.gravityTorqueStrength = physicsGravityTorque }
    }
    var physicsDragPickupDuration: Float = 0.12 {
        didSet { simulator?.dragPickupDuration = physicsDragPickupDuration }
    }
    var physicsDragMinSubsteps: Int = 3 {
        didSet { simulator?.dragMinSubsteps = physicsDragMinSubsteps }
    }
    var physicsFadeOutSpeed: Float = 45.0 {
        didSet { simulator?.fadeOutSpeed = physicsFadeOutSpeed }
    }
    var physicsLowerAnimDuration: Float = 0.55 {
        didSet { simulator?.lowerAnimDuration = physicsLowerAnimDuration }
    }
    var physicsIdleTimeout: Float = 3.0 {
        didSet { simulator?.idleTimeout = physicsIdleTimeout }
    }
    var useParticleBraid: Bool = false
    var physicsPaused: Bool = false

    // Auto-drag test (level 998)
    var autoDragActive: Bool = false
    var autoDragStep: Int = 0
    var autoDragTotalSteps: Int = 600
    var autoDragStart: SIMD2<Float> = .zero
    var autoDragEnd: SIMD2<Float> = .zero
    var autoDragBand: Int = 0
    var autoDragEndIdx: Int = 0
    var autoDragPrevCrossings: Int = -1

    // Animated level init: replay drag actions visually (always on)
    var slowInitVisualization: Bool = true
    var pendingInitDrags: [VerletSimulator.LevelAction] = []

    // Snapshot of bands after slowInit completes, keyed by levelId
    var postInitBandsSnapshot: (levelId: Int, bands: [VerletSimulator.Band])? = nil

    // Per-drag/swap animation state
    enum SlowDragPhase { case idle, lift, traverse, lower, settle }
    var slowDragPhase: SlowDragPhase = .idle
    var slowDragStep: Int = 0
    var slowDragIsSwap: Bool = false
    // Band A (primary, or only for drag)
    var slowDragBandIdx: Int = 0
    var slowDragParticleIdx: Int = 0
    var slowDragFromPos: SIMD3<Float> = .zero
    var slowDragLiftFrom: SIMD3<Float> = .zero
    var slowDragLiftTo: SIMD3<Float> = .zero
    var slowDragToPos: SIMD3<Float> = .zero
    // Band B (for swap only)
    var slowDragBandIdx2: Int = 0
    var slowDragParticleIdx2: Int = 0
    var slowDragFromPos2: SIMD3<Float> = .zero
    var slowDragLiftFrom2: SIMD3<Float> = .zero
    var slowDragLiftTo2: SIMD3<Float> = .zero
    var slowDragToPos2: SIMD3<Float> = .zero
    var slowDragSettleCount: Int = 0

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
        self.copyDepthPipeline = Self.makeComputePipeline(device: device, function: library.makeFunction(name: "copyDepthKernel")!)
        self.bakeWoodPipeline = Self.makeComputePipeline(device: device, function: library.makeFunction(name: "bakeWoodKernel")!)
        self.bakeBoardWoodVolumePipeline = Self.makeComputePipeline(device: device, function: library.makeFunction(name: "bakeBoardWoodVolumeKernel")!)
        self.debug2DPipeline = Self.makeDebug2DPipeline(device: device, view: view, library: library)
        (self.depthStateScene, self.depthStateBackground, self.depthStateShadow) = Self.makeDepthStates(device: device)

        self.frameUniforms = device.makeBuffer(length: MemoryLayout<FrameUniforms>.stride, options: [.storageModeShared])
        self.postParamsBuffer = device.makeBuffer(length: MemoryLayout<PostParams>.stride, options: [.storageModeShared])
        Self.buildHoleMeshBuffers(device: device, segments: holeSegments, square: squareCrossSection, vertexBuffer: &holeVB, indexBuffer: &holeIB, indexCount: &holeIndexCount)
        let envDisabledDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba16Float, width: 1, height: 1, mipmapped: false)
        envDisabledDesc.usage = .shaderRead
        guard let envDisabledTex = device.makeTexture(descriptor: envDisabledDesc) else {
            fatalError("Failed to create disabled env texture")
        }
        let woodDebugDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba16Float, width: 1, height: 1, mipmapped: false)
        woodDebugDesc.usage = .shaderRead
        guard let woodDebugTex = device.makeTexture(descriptor: woodDebugDesc) else {
            fatalError("Failed to create debug wood texture")
        }
        let holeMaskDisabledDesc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .r8Unorm, width: 1, height: 1, mipmapped: false)
        holeMaskDisabledDesc.usage = .shaderRead
        guard let holeMaskDisabledTex = device.makeTexture(descriptor: holeMaskDisabledDesc) else {
            fatalError("Failed to create disabled hole mask texture")
        }
        self.envDisabledTex = envDisabledTex
        self.woodDebugTex = woodDebugTex
        self.holeMaskDisabledTex = holeMaskDisabledTex

        var envDisabledPixel = [UInt16](repeating: 0, count: 4)
        envDisabledTex.replace(region: MTLRegionMake2D(0, 0, 1, 1), mipmapLevel: 0, withBytes: &envDisabledPixel, bytesPerRow: MemoryLayout<UInt16>.stride * 4)
        var woodDebugPixel = [UInt16](arrayLiteral: 0, 15360, 0, 15360)
        woodDebugTex.replace(region: MTLRegionMake2D(0, 0, 1, 1), mipmapLevel: 0, withBytes: &woodDebugPixel, bytesPerRow: MemoryLayout<UInt16>.stride * 4)
        var holeMaskDisabledPixel = [UInt8](repeating: 0, count: 1)
        holeMaskDisabledTex.replace(region: MTLRegionMake2D(0, 0, 1, 1), mipmapLevel: 0, withBytes: &holeMaskDisabledPixel, bytesPerRow: 1)

        self.noiseTexture = Self.generateNoiseTexture(device: device, size: 2048)

        super.init()

        startMotionUpdates()
    }

    private func startMotionUpdates() {
        #if os(iOS)
        guard motionManager.isDeviceMotionAvailable else { return }
        motionManager.deviceMotionUpdateInterval = 1.0 / 60.0
        motionManager.startDeviceMotionUpdates()
        #endif
    }

    func loadLevelDefinition(_ def: LevelDefinition) {
        _pendingLevelDef = def
        loadLevel(levelId: def.id)
    }

    private var _pendingLevelDef: LevelDefinition?

    func loadLevel(levelId: Int) {
        let loadStart = CACurrentMediaTime()
        currentLevelId = levelId
        Self.logger.info("Loading level \(levelId)...")

        dragState = nil
        dragWorld = .zero
        simulator = nil
        pendingBraidDrags = []
        braidDragTimer = 0
        victoryOrbitActive = false
        victoryOrbitTime = 0
        victoryOrbitSavedCamera = nil
        victoryReplayActive = false
        victoryReplayPending = false
        victoryReplayPhase = .done
        victoryWaitingForSuck = false
        replayVanishRecords = [:]
        victoryReplayVanishAtMove = [:]
        moveHistory.removeAll()
        undoStore.clear()
        levelFlow.clearAll()
        moveCount = 0
        onMoveCountChanged?(0)
        onUndoStackChanged?(false)

        var t0 = CACurrentMediaTime()
        let level: LevelDefinition
        if let pending = _pendingLevelDef {
            level = pending
            _pendingLevelDef = nil
        } else if levelId == 998 {
            Self.logger.info("Level 998: tunneling test")
            level = Self.makeTunnelingTestLevel()
        } else if levelId == 2 {
            Self.logger.info("Level 2: central knot showcase")
            level = LevelGenerator.generateStructureShowcase(levelId: 3115, particleCount: physicsParticleCount)
        } else if let jsonLevel = LevelLoader.load(levelId: levelId) {
            Self.logger.info("Level \(levelId) loaded from JSON: \(jsonLevel.ropes.count) ropes, \(jsonLevel.holes.count) holes")
            level = jsonLevel
        } else if levelId >= 3100 {
            Self.logger.info("Level \(levelId): structure showcase")
            level = LevelGenerator.generateStructureShowcase(levelId: levelId, particleCount: physicsParticleCount)
        } else if levelId >= 3001 {
            if useParticleBraid {
                Self.logger.info("Level \(levelId) generated as particle braid")
                level = LevelGenerator.generateParticleBraidLevel(levelId: levelId, particleCount: physicsParticleCount)
            } else {
                Self.logger.info("Level \(levelId) generated as braid mode")
                level = LevelGenerator.generateBraidLevel(levelId: levelId, particleCount: physicsParticleCount)
            }
        } else if levelId >= 2001 {
            Self.logger.info("Level \(levelId) generated as rail mode")
            level = LevelGenerator.generateRailLevel(levelId: levelId, particleCount: physicsParticleCount)
        } else if levelId >= 1001 {
            Self.logger.info("Level \(levelId) generated as tension mode")
            level = LevelGenerator.generateTensionLevel(levelId: levelId, particleCount: physicsParticleCount)
        } else {
            Self.logger.info("Level \(levelId) generated procedurally")
            level = LevelGenerator.generate(levelId: levelId, boardElevation: boardElevation, particleCount: physicsParticleCount)
        }
        let tGenerate = CACurrentMediaTime()

        let levelHoles = level.holes.map { $0.simd }
        let levelHoleElevations = level.holes.map { $0.zPosition }
        let levelHoleRadius = level.holeRadius

        let maxValidIndex = levelHoles.count + (level.weights?.count ?? 0) - 1
        let validatedRopes = level.ropes.filter { rope in
            guard rope.startHole >= 0 && rope.startHole <= maxValidIndex else { return false }
            guard rope.endHole >= 0 && rope.endHole <= maxValidIndex else { return false }
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

        let tTopology = CACurrentMediaTime()

        let sim = VerletSimulator(holePositions: levelHoles, holeElevations: levelHoleElevations, holeRadius: levelHoleRadius, boards: boards)
        sim.gravity = physicsGravity
        sim.damping = physicsDamping
        sim.constraintIterations = physicsConstraintIterations
        sim.broadphaseRebuildInterval = physicsBroadphaseInterval
        sim.particleCount = physicsParticleCount
        sim.settleSteps = physicsSettleSteps
        sim.liftHeight = physicsLiftHeight
        sim.ropeTension = physicsRopeTension
        sim.bendCompliance = physicsBendCompliance
        sim.bendVelocityCoupling = physicsBendVelocityCoupling
        sim.frictionCoefficient = physicsFriction
        sim.collisionResponse = physicsCollisionResponse
        sim.zSeparationStrength = physicsZSeparation
        sim.frictionDampingRatio = physicsFrictionDampingRatio
        sim.maxFrictionCap = physicsMaxFrictionCap
        sim.boardFrictionRatio = physicsBoardFrictionRatio
        sim.twistStiffness = physicsTwistStiffness
        sim.twistDamping = physicsTwistDamping
        sim.gravityTorqueStrength = physicsGravityTorque
        sim.dragPickupDuration = physicsDragPickupDuration
        sim.dragMinSubsteps = physicsDragMinSubsteps
        sim.fadeOutSpeed = physicsFadeOutSpeed
        sim.lowerAnimDuration = physicsLowerAnimDuration
        sim.idleTimeout = physicsIdleTimeout
        sim.stretchThinning = stretchThinning
        sim.squareCrossSection = squareCrossSection

        let simRopeConfigs = ropes.map { rope in
            VerletSimulator.RopeConfig(startHole: rope.startHole, endHole: rope.endHole, radius: rope.radius, crossSection: rope.crossSection)
        }

        let simActions: [VerletSimulator.LevelAction] = level.actions?.compactMap { action in
            guard let actionType = VerletSimulator.LevelAction.ActionType(rawValue: action.type) else { return nil }
            var la = VerletSimulator.LevelAction(type: actionType, ropeIndex: action.ropeIndex, endIndex: action.endIndex, holeIndex: action.holeIndex)
            la.ropeIndex2 = action.ropeIndex2 ?? 0
            la.endIndex2 = action.endIndex2 ?? 0
            return la
        } ?? []

        // Tension mode setup
        isTensionMode = level.isTensionMode
        tensionLevelCompleted = false
        sim.isTensionMode = isTensionMode

        // Initialize weights BEFORE level init (actions reference them)
        if isTensionMode, let weightDefs = level.weights, let targetDefs = level.targets {
            let weightConfigs = weightDefs.enumerated().map { i, w in
                let target = targetDefs.first(where: { $0.weightIndex == i })
                return VerletSimulator.WeightConfig(
                    position: w.position,
                    mass: w.mass ?? 2.0,
                    radius: w.radius ?? 0.15,
                    targetPosition: target?.position,
                    targetRadius: target?.radius ?? 0.2
                )
            }
            sim.initializeWeights(weightConfigs)

            weightRenderInfos = weightDefs.map {
                WeightRenderInfo(position: $0.position, radius: $0.radius ?? 0.15, settled: false)
            }
            targetRenderInfos = targetDefs.map {
                TargetRenderInfo(position: $0.position, radius: $0.radius ?? 0.2, weightIndex: $0.weightIndex, satisfied: false)
            }
            buildWeightMesh()
            buildTargetMesh()
        } else {
            weightRenderInfos = []
            targetRenderInfos = []
        }

        // Braid mode setup
        isBraidMode = level.isBraidMode
        braidLevelCompleted = false
        if isBraidMode, let targets = level.braidTargets {
            braidTargets = targets
            braidMinCrossings = level.braidMinCrossings ?? 1
            braidStrandCount = level.ropes.count
            braidBottomHoleStart = braidStrandCount // bottom holes start after top holes
        } else {
            braidTargets = []
            braidMinCrossings = 0
            braidStrandCount = 0
            braidBottomHoleStart = 0
        }

        // Rail mode setup
        isRailMode = level.isRailMode
        railLevelCompleted = false
        sim.isRailMode = isRailMode
        if isRailMode, let railDefs = level.rails, let cartDefs = level.carts, let stationDefs = level.stations {
            sim.initializeRails(railDefs: railDefs, cartDefs: cartDefs, stationDefs: stationDefs)
            railRenderInfos = sim.rails.map { rail in
                RailRenderInfo(points: rail.points)
            }
            cartRenderInfos = sim.carts.map { cart in
                let pos = sim.rails[cart.railIndex].position(at: cart.t)
                return CartRenderInfo(position: pos, radius: cart.radius, settled: false)
            }
            stationRenderInfos = sim.railStations.map { station in
                let pos = sim.rails[station.railIndex].position(at: station.t)
                return StationRenderInfo(position: pos, radius: station.radius, cartIndex: station.cartIndex, satisfied: false)
            }
            buildRailMesh()
            buildCartMesh()
            buildStationMesh()
        } else {
            railRenderInfos = []
            cartRenderInfos = []
            stationRenderInfos = []
        }

        let simParticles: [[SIMD3<Float>]]? = level.ropeParticles.map { ropes in
            ropes.map { particles in particles.map { $0.simd3 } }
        }

        t0 = CACurrentMediaTime()
        if let snapshot = postInitBandsSnapshot, snapshot.levelId == levelId {
            // Restore from snapshot — skip slowInit entirely
            sim.initializeLevel(ropeConfigs: simRopeConfigs, actions: simActions.filter { $0.type == .pin }, ropeParticles: simParticles)
            sim.bands = snapshot.bands
            pendingInitDrags = []
        } else if slowInitVisualization {
            // Split actions: run pins immediately, queue drags for per-frame replay
            let pinActions = simActions.filter { $0.type == .pin }
            let dragActions = simActions.filter { $0.type == .drag || $0.type == .swap }
            sim.initializeLevel(ropeConfigs: simRopeConfigs, actions: pinActions, ropeParticles: simParticles)
            pendingInitDrags = dragActions
        } else {
            sim.initializeLevel(ropeConfigs: simRopeConfigs, actions: simActions, ropeParticles: simParticles)
            pendingInitDrags = []
        }
        pendingBraidDrags = []
        braidRemainingCrossings = 0

        let tPhysics = CACurrentMediaTime()
        self.simulator = sim

        // Sync pin indices back to ropes array.
        // Weight pins (negative) are converted to holeCount + weightIndex for Renderer use.
        let holeCount = levelHoles.count
        for ropeIndex in ropes.indices {
            guard sim.bands.indices.contains(ropeIndex) else { continue }
            let band = sim.bands[ropeIndex]
            if let pinStart = band.pinStart {
                if pinStart < 0 {
                    ropes[ropeIndex].startHole = holeCount + VerletSimulator.weightIndex(from: pinStart)
                } else {
                    ropes[ropeIndex].startHole = pinStart
                }
            }
            if let pinEnd = band.pinEnd {
                if pinEnd < 0 {
                    ropes[ropeIndex].endHole = holeCount + VerletSimulator.weightIndex(from: pinEnd)
                } else {
                    ropes[ropeIndex].endHole = pinEnd
                }
            }
        }

        for ropeIndex in ropes.indices {
            let startHoleIndex = ropes[ropeIndex].startHole
            let endHoleIndex = ropes[ropeIndex].endHole
            if holeOccupied.indices.contains(startHoleIndex) {
                holeOccupied[startHoleIndex] = true
            }
            if holeOccupied.indices.contains(endHoleIndex) {
                holeOccupied[endHoleIndex] = true
            }
        }

        let aspect = Float(lastViewSize.width / max(1, lastViewSize.height))
        let maxElev = holeElevations.max() ?? 0
        camera.fitToHoles(holePositions, holeRadius: holeRadius, aspect: aspect, maxElevation: maxElev)
        cameraBaseOrthoHalfHeight = camera.orthoHalfHeight
        camera.orthoHalfHeight = cameraBaseOrthoHalfHeight / cameraZoomScale

        let tCamera = CACurrentMediaTime()
        bakeWoodTexture()
        bakeHoleMaskTexture()
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

        // Auto-drag test: after level 998 loads, start dragging rope 1 end across rope 0
        if levelId == 998 {
            autoDragActive = true
            autoDragStep = 0
            autoDragTotalSteps = 18000  // 30x slower
            autoDragBand = 1       // blue rope (under)
            autoDragEndIdx = 1
            autoDragStart = holePositions[3]  // (0.25, 0.4)
            autoDragEnd = SIMD2<Float>(-7.75, -12.6)  // ~x30 stretch through red rope
        }
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

    func autoDragCountCrossings(_ sim: VerletSimulator) -> Int {
        guard sim.bands.count >= 2 else { return 0 }
        let posA = sim.bands[0].positions
        let posB = sim.bands[1].positions
        let skip = 3
        var count = 0
        for si in skip..<max(skip, posA.count-1-skip) {
            let a0 = SIMD2<Float>(posA[si].x, posA[si].y)
            let a1 = SIMD2<Float>(posA[si+1].x, posA[si+1].y)
            for sj in skip..<max(skip, posB.count-1-skip) {
                let b0 = SIMD2<Float>(posB[sj].x, posB[sj].y)
                let b1 = SIMD2<Float>(posB[sj+1].x, posB[sj+1].y)
                let d1 = a1 - a0, d2 = b1 - b0
                let cross = d1.x * d2.y - d1.y * d2.x
                if abs(cross) < 1e-9 { continue }
                let d = b0 - a0
                let t = (d.x * d2.y - d.y * d2.x) / cross
                let u = (d.x * d1.y - d.y * d1.x) / cross
                if t > 1e-6 && t < (1-1e-6) && u > 1e-6 && u < (1-1e-6) { count += 1 }
            }
        }
        return count
    }

    private static func makeTunnelingTestLevel() -> LevelDefinition {
        typealias V = LevelDefinition.Vec2
        typealias C = LevelDefinition.Color
        typealias R = LevelDefinition.Rope
        typealias A = LevelDefinition.Action
        let particlesPerRope = 15
        // Rope 0 (red): diagonal \  — pinned first, so it lies ON TOP
        // Rope 1 (blue): diagonal / — pinned second + dragged across, so it goes UNDER
        return LevelDefinition(
            mode: nil, id: 998, holeRadius: 0.08, particlesPerRope: particlesPerRope,
            holes: [
                V(xPosition: -0.4, yPosition:  0.25),  // 0
                V(xPosition:  0.4, yPosition: -0.25),  // 1
                V(xPosition: -0.25, yPosition: -0.4),  // 2
                V(xPosition:  0.25, yPosition:  0.4),  // 3
            ],
            ropes: [
                R(startHole: 0, endHole: 1, color: C(redChannel: 0.9, greenChannel: 0.2, blueChannel: 0.2), radius: 0.04),
                R(startHole: 2, endHole: 3, color: C(redChannel: 0.2, greenChannel: 0.4, blueChannel: 0.9), radius: 0.04),
            ],
            hooks: nil,
            actions: [
                A(type: "pin", ropeIndex: 1, endIndex: 0, holeIndex: 2),
                A(type: "pin", ropeIndex: 1, endIndex: 1, holeIndex: 3),
                A(type: "pin", ropeIndex: 0, endIndex: 0, holeIndex: 0),
                A(type: "pin", ropeIndex: 0, endIndex: 1, holeIndex: 1),
            ],
            boards: nil, weights: nil, targets: nil, rails: nil, carts: nil, stations: nil)
    }

    func applyRopePalette(_ palette: [SIMD3<Float>]) {
        guard !palette.isEmpty else { return }
        for i in ropes.indices {
            ropes[i].color = palette[i % palette.count]
        }
    }
}
