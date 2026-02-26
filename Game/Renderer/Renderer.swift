import MetalKit
import os.log

final class Renderer: NSObject, MTKViewDelegate {
    static let logger = Logger(subsystem: "com.uzls.four", category: "Renderer")

    let device: MTLDevice
    let commandQueue: MTLCommandQueue
    let tablePipeline: MTLRenderPipelineState
    let holePipeline: MTLRenderPipelineState
    let ropePipeline: MTLRenderPipelineState
    let postPipeline: MTLRenderPipelineState
    let shadowRopePipeline: MTLRenderPipelineState
    let shadowHolePipeline: MTLRenderPipelineState
    let bloomThreshold: MTLComputePipelineState
    let bloomBlurH: MTLComputePipelineState
    let bloomBlurV: MTLComputePipelineState

    var depthStateScene: MTLDepthStencilState
    var depthStateBackground: MTLDepthStencilState
    var depthStateShadow: MTLDepthStencilState

    var shadowDepthTex: MTLTexture?
    let shadowMapSize: Int = 2048

    var camera = Camera()
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
    var holeInstances: MTLBuffer?
    var holeVB: MTLBuffer?
    var holeIB: MTLBuffer?
    var holeIndexCount: Int = 0

    var holePositions: [SIMD2<Float>] = []
    var holeRadius: Float = 0.105
    var holeOccupied: [Bool] = []
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
    var dragHeight: Float = 0.35
    var highlightHoleIndex: Int = -1

    var ropeVB: MTLBuffer?
    var ropeIB: MTLBuffer?
    var ropeIndexCount: Int = 0

    var hdrTex: MTLTexture?
    var bloomA: MTLTexture?
    var bloomB: MTLTexture?
    var lastViewSize: CGSize = CGSize(width: 400, height: 600)
    
    var ropePhysicsLogger = RopePhysics()
    var lastPhysicsLogTime: Double = 0
    let frictionSound = RubberFrictionSound()

    struct MeshStats: Equatable {
        let vertices: Int
        let indices: Int
        let ropeCount: Int
    }
    var lastMeshStats: MeshStats?

    var currentLevelId: Int = 49
    var onLevelComplete: (() -> Void)?
    var onUndoStackChanged: ((Bool) -> Void)?

    // MARK: - Undo

    struct UndoEntry {
        var simulatorSnapshot: VerletSimulator.Snapshot
        var ropeEndpoints: [RopeEndpoints]
        var holeOccupied: [Bool]
    }

    private var undoStack: [UndoEntry] = []

    var canUndo: Bool { !undoStack.isEmpty }

    func pushUndoState() {
        guard let sim = simulator else { return }
        let entry = UndoEntry(
            simulatorSnapshot: sim.takeSnapshot(),
            ropeEndpoints: ropes,
            holeOccupied: holeOccupied
        )
        undoStack.append(entry)
        onUndoStackChanged?(true)
    }

    func performUndo() {
        guard let entry = undoStack.popLast(), let sim = simulator else { return }
        dragState = nil
        highlightHoleIndex = -1
        settleCheckTimer = nil
        sim.restoreSnapshot(entry.simulatorSnapshot)
        ropes = entry.ropeEndpoints
        holeOccupied = entry.holeOccupied
        onUndoStackChanged?(!undoStack.isEmpty)
    }

    /// Timer for delayed win check after drag ends.
    /// Rope needs time to settle before we check crossings.
    var settleCheckTimer: Float? = nil
    let settleCheckDelay: Float = 0.5
    var nextLevelTimer: Float? = nil
    
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
        self.shadowRopePipeline = Self.makeShadowRopePipeline(device: device, library: library)
        self.shadowHolePipeline = Self.makeShadowHolePipeline(device: device, library: library)
        (self.bloomThreshold, self.bloomBlurH, self.bloomBlurV) = Self.makeBloomPipelines(device: device, library: library)
        (self.depthStateScene, self.depthStateBackground, self.depthStateShadow) = Self.makeDepthStates(device: device)

        self.frameUniforms = device.makeBuffer(length: MemoryLayout<FrameUniforms>.stride, options: [.storageModeShared])
        Self.buildHoleMeshBuffers(device: device, vertexBuffer: &holeVB, indexBuffer: &holeIB, indexCount: &holeIndexCount)

        super.init()

        loadLevel(levelId: 49)
    }
    
    func loadLevel(levelId: Int) {
        currentLevelId = levelId
        Self.logger.info("Loading level \(levelId)...")
        
        dragState = nil
        dragWorld = .zero
        simulator = nil
        undoStack.removeAll()
        onUndoStackChanged?(false)
        
        // Try JSON first, fallback to procedural generation
        let level: LevelDefinition
        if let jsonLevel = LevelLoader.load(levelId: levelId) {
            Self.logger.info("Level \(levelId) loaded from JSON: \(jsonLevel.ropes.count) ropes, \(jsonLevel.holes.count) holes")
            level = jsonLevel
        } else {
            Self.logger.info("Level \(levelId) generated procedurally")
            level = LevelGenerator.generate(levelId: levelId)
        }

        let levelHoles = level.holes.map { $0.simd }
        let levelHoleRadius = level.holeRadius

        let validatedRopes = level.ropes.filter { rope in
            guard levelHoles.indices.contains(rope.startHole) else { return false }
            guard levelHoles.indices.contains(rope.endHole) else { return false }
            return rope.startHole != rope.endHole
        }

        holePositions = levelHoles
        holeRadius = levelHoleRadius
        holeOccupied = Array(repeating: false, count: levelHoles.count)
        holeInstances = Self.makeHoleInstances(device: device, positions: levelHoles, radius: levelHoleRadius)

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

        // Initialize Verlet simulator
        let sim = VerletSimulator(holePositions: levelHoles, holeRadius: levelHoleRadius)
        sim.gravity = physicsGravity
        sim.damping = physicsDamping
        sim.constraintIterations = physicsConstraintIterations
        sim.particleCount = physicsParticleCount
        sim.settleSteps = physicsSettleSteps
        sim.liftHeight = physicsLiftHeight
        sim.ropeTension = physicsRopeTension

        let simRopeConfigs = ropes.map { rope in
            VerletSimulator.RopeConfig(startHole: rope.startHole, endHole: rope.endHole, radius: rope.radius, crossSection: rope.crossSection)
        }

        let simActions: [VerletSimulator.LevelAction] = level.actions?.compactMap { action in
            guard let actionType = VerletSimulator.LevelAction.ActionType(rawValue: action.type) else { return nil }
            return VerletSimulator.LevelAction(type: actionType, ropeIndex: action.ropeIndex, endIndex: action.endIndex, holeIndex: action.holeIndex)
        } ?? []

        sim.initializeLevel(ropeConfigs: simRopeConfigs, actions: simActions)
        self.simulator = sim

        // Sync rope endpoints and holeOccupied with simulator's actual positions
        // (decompose may place ropes at different holes than JSON specifies)
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
        camera.fitToHoles(holePositions, holeRadius: holeRadius, aspect: aspect)
    }

    private static func makePipeline(device: MTLDevice, descriptor: MTLRenderPipelineDescriptor) -> MTLRenderPipelineState {
        do {
            return try device.makeRenderPipelineState(descriptor: descriptor)
        } catch {
            fatalError(String(describing: error))
        }
    }

    private static func makeComputePipeline(device: MTLDevice, function: MTLFunction) -> MTLComputePipelineState {
        do {
            return try device.makeComputePipelineState(function: function)
        } catch {
            fatalError(String(describing: error))
        }
    }

    private static func makeTablePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "fullscreenVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "tableFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeHolePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "holeVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "holeFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeHoleVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeRopePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "ropeVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "ropeFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeRopeVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeShadowRopePipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "ropeShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeRopeVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeShadowHolePipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "holeShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeHoleVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makePostPipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "fullscreenVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "postFragment")
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeBloomPipelines(device: MTLDevice, library: MTLLibrary) -> (MTLComputePipelineState, MTLComputePipelineState, MTLComputePipelineState) {
        guard let thresholdFunction = library.makeFunction(name: "bloomThreshold"),
              let blurHFunction = library.makeFunction(name: "bloomBlurH"),
              let blurVFunction = library.makeFunction(name: "bloomBlurV") else {
            fatalError("Failed to load bloom functions")
        }
        return (
            makeComputePipeline(device: device, function: thresholdFunction),
            makeComputePipeline(device: device, function: blurHFunction),
            makeComputePipeline(device: device, function: blurVFunction)
        )
    }

    private static func makeDepthStates(device: MTLDevice) -> (MTLDepthStencilState, MTLDepthStencilState, MTLDepthStencilState) {
        let backgroundDescriptor = MTLDepthStencilDescriptor()
        backgroundDescriptor.depthCompareFunction = .always
        backgroundDescriptor.isDepthWriteEnabled = false

        let sceneDescriptor = MTLDepthStencilDescriptor()
        sceneDescriptor.depthCompareFunction = .lessEqual
        sceneDescriptor.isDepthWriteEnabled = true

        let shadowDescriptor = MTLDepthStencilDescriptor()
        shadowDescriptor.depthCompareFunction = .lessEqual
        shadowDescriptor.isDepthWriteEnabled = true

        guard let backgroundState = device.makeDepthStencilState(descriptor: backgroundDescriptor),
              let sceneState = device.makeDepthStencilState(descriptor: sceneDescriptor),
              let shadowState = device.makeDepthStencilState(descriptor: shadowDescriptor) else {
            fatalError("Failed to create depth states")
        }

        return (sceneState, backgroundState, shadowState)
    }

    private static func makeRopeVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = MemoryLayout<RopeVertex>.offset(of: \.position) ?? 0
        descriptor.attributes[0].bufferIndex = 0

        descriptor.attributes[1].format = .float3
        descriptor.attributes[1].offset = MemoryLayout<RopeVertex>.offset(of: \.normal) ?? 0
        descriptor.attributes[1].bufferIndex = 0

        descriptor.attributes[2].format = .float3
        descriptor.attributes[2].offset = MemoryLayout<RopeVertex>.offset(of: \.color) ?? 0
        descriptor.attributes[2].bufferIndex = 0

        descriptor.attributes[3].format = .float2
        descriptor.attributes[3].offset = MemoryLayout<RopeVertex>.offset(of: \.texCoord) ?? 0
        descriptor.attributes[3].bufferIndex = 0

        descriptor.attributes[4].format = .float4
        descriptor.attributes[4].offset = MemoryLayout<RopeVertex>.offset(of: \.params) ?? 0
        descriptor.attributes[4].bufferIndex = 0

        descriptor.layouts[0].stride = MemoryLayout<RopeVertex>.stride
        return descriptor
    }

    private static func makeHoleVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = 0
        descriptor.attributes[0].bufferIndex = 0

        descriptor.attributes[1].format = .float3
        descriptor.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        descriptor.attributes[1].bufferIndex = 0

        descriptor.layouts[0].stride = MemoryLayout<HoleVertex>.stride
        return descriptor
    }

    private static func buildHoleMeshBuffers(device: MTLDevice, vertexBuffer: inout MTLBuffer?, indexBuffer: inout MTLBuffer?, indexCount: inout Int) {
        let mesh = HoleMeshBuilder.build()
        indexCount = mesh.indices.count
        vertexBuffer = device.makeBuffer(bytes: mesh.vertices, length: mesh.vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        indexBuffer = device.makeBuffer(bytes: mesh.indices, length: mesh.indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
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
