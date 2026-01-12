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
    var time: Float = 0
    var dragVisualEnergy: Float = 0

    var simulation: RopeSimulation!

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
    }

    var ropes: [RopeEndpoints] = []
    var topology: TopologyEngine?
    var lastDragWorld: SIMD2<Float> = .zero
    var dragStartWorld: SIMD2<Float> = .zero

    struct DragState {
        var ropeIndex: Int
        var endIndex: Int
        var originalHoleIndex: Int
        var topologySnapshot: TopologySnapshot
        var restLength: Float
    }

    var dragState: DragState?
    var dragWorld: SIMD2<Float> = .zero
    var dragWorldLazy: SIMD2<Float> = .zero
    var dragWorldTarget: SIMD2<Float> = .zero
    var dragSagProgress: Float = 0.0
    var dragHeight: Float = 0.35
    var dragLiftCurrent: Float = 0
    var dragStretchRatio: Float = 1.0
    var dragOscillationPhase: Float = 0.0
    var dragOscillationVelocity: Float = 0.0
    var dragOscillationRopeIndex: Int? = nil

    struct SnapAnimationState {
        var ropeIndex: Int
        var endIndex: Int
        var targetHoleIndex: Int
        var startPosition: SIMD2<Float>
        var targetPosition: SIMD2<Float>
        var startZ: Float
        var progress: Float
        var topologySnapshot: TopologySnapshot
        var originalHoleIndex: Int
    }
    var snapAnimationState: SnapAnimationState?
    
    struct RopeTensionState {
        var currentLength: Float
        var velocity: Float
    }
    var ropeTensionStates: [Int: RopeTensionState] = [:]
    var globalTensionActive: Bool = false
    var ropeRestLengths: [Int: Float] = [:]
    var tensionLogCounter: Int = 0

    var ropeVB: MTLBuffer?
    var ropeIB: MTLBuffer?
    var ropeIndexCount: Int = 0

    var hdrTex: MTLTexture?
    var bloomA: MTLTexture?
    var bloomB: MTLTexture?
    
    var ropePhysicsLogger = RopePhysics()
    var lastPhysicsLogTime: Double = 0
    
    struct MeshStats: Equatable {
        let vertices: Int
        let indices: Int
        let ropeCount: Int
    }
    var lastMeshStats: MeshStats?
    
    var currentLevelId: Int = 1
    
    var ropeRenderSimpleMode: Bool = true
    var ropeRenderDisableBandRepulsion: Bool = true
    var ropeRenderDisableCrossingDeform: Bool = true
    var ropeRenderDisableHoleDeform: Bool = true
    var ropeRenderDisableDragCrossingPhysics: Bool = false
    
    var hookStepMultiplier: Float = 0.0900 {
        didSet { TopologySampler.hookStepMultiplier = hookStepMultiplier }
    }
    var hookRadiusMultiplier: Float = 0.920 {
        didSet { TopologySampler.hookRadiusMultiplier = hookRadiusMultiplier }
    }
    var hookStepLimitMultiplier: Float = 2.5000 {
        didSet { TopologySampler.hookStepLimitMultiplier = hookStepLimitMultiplier }
    }
    var debugSegmentColors: Bool = true {
        didSet { TopologySampler.debugSegmentColors = debugSegmentColors }
    }
    var smoothSubdivisions: Int = 4 {
        didSet { TopologySampler.smoothSubdivisions = smoothSubdivisions }
    }
    var smoothIterations: Int = 0 {
        didSet { TopologySampler.smoothIterations = smoothIterations }
    }
    var smoothStrength: Float = 0.00 {
        didSet { TopologySampler.smoothStrength = smoothStrength }
    }
    var smoothZone: Float = 0.001 {
        didSet { TopologySampler.smoothZone = smoothZone }
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
        
        TopologySampler.hookStepMultiplier = hookStepMultiplier
        TopologySampler.hookRadiusMultiplier = hookRadiusMultiplier
        TopologySampler.hookStepLimitMultiplier = hookStepLimitMultiplier
        TopologySampler.debugSegmentColors = debugSegmentColors
        TopologySampler.smoothSubdivisions = smoothSubdivisions
        TopologySampler.smoothIterations = smoothIterations
        TopologySampler.smoothStrength = smoothStrength
        TopologySampler.smoothZone = smoothZone
        
        loadLevel(levelId: 1)
    }
    
    func loadLevel(levelId: Int) {
        currentLevelId = levelId
        Self.logger.info("Loading level \(levelId)...")
        
        dragState = nil
        snapAnimationState = nil
        dragWorld = .zero
        dragWorldLazy = .zero
        dragWorldTarget = .zero
        dragSagProgress = 0.0
        dragLiftCurrent = 0
        dragStretchRatio = 1.0
        dragOscillationPhase = 0.0
        dragOscillationVelocity = 0.0
        dragOscillationRopeIndex = nil
        ropeTensionStates = [:]
        globalTensionActive = false
        ropeRestLengths = [:]
        tensionLogCounter = 0
        
        let fallbackLayout = Self.makeHoleLayout()
        let level = LevelLoader.load(levelId: levelId)

        if level == nil {
            Self.logger.warning("Level \(levelId) failed to load, using fallback")
        } else {
            Self.logger.info("Level \(levelId) loaded successfully: \(level!.ropes.count) ropes, \(level!.holes.count) holes from JSON")
        }

        let decodedHoles = level?.holes.map { $0.simd }
        let levelHoles = (decodedHoles?.isEmpty == false) ? (decodedHoles ?? fallbackLayout.positions) : fallbackLayout.positions
        let levelHoleRadius = level?.holeRadius ?? fallbackLayout.radius
        
        let defaultRopes: [LevelDefinition.Rope] = [
            LevelDefinition.Rope(
                startHole: 0,
                endHole: min(14, levelHoles.count - 1),
                color: .init(redChannel: 0.20, greenChannel: 0.95, blueChannel: 0.35),
                radius: 0.045
            ),
            LevelDefinition.Rope(
                startHole: 3,
                endHole: min(17, levelHoles.count - 1),
                color: .init(redChannel: 0.30, greenChannel: 0.55, blueChannel: 0.98),
                radius: 0.044
            ),
            LevelDefinition.Rope(
                startHole: min(6, levelHoles.count - 1),
                endHole: min(11, levelHoles.count - 1),
                color: .init(redChannel: 0.96, greenChannel: 0.28, blueChannel: 0.33),
                radius: 0.043
            )
        ]
        let candidateRopes = (level != nil && !level!.ropes.isEmpty) ? level!.ropes : defaultRopes

        var validatedRopes = candidateRopes.filter { rope in
            guard levelHoles.indices.contains(rope.startHole) else { return false }
            guard levelHoles.indices.contains(rope.endHole) else { return false }
            if rope.startHole == rope.endHole { return false }
            return true
        }

        if validatedRopes.isEmpty, levelHoles.count >= 2 {
            validatedRopes = [
                LevelDefinition.Rope(
                    startHole: 0,
                    endHole: 1,
                    color: .init(redChannel: 0.85, greenChannel: 0.85, blueChannel: 0.92),
                    radius: 0.045
                )
            ]
        }

        let particlesPerRope = max(8, level?.particlesPerRope ?? 6400)
        simulation = RopeSimulation(device: device, ropeCount: max(1, validatedRopes.count), particlesPerRope: particlesPerRope)

        holePositions = levelHoles
        holeRadius = levelHoleRadius
        holeOccupied = Array(repeating: false, count: levelHoles.count)
        holeInstances = Self.makeHoleInstances(device: device, positions: levelHoles, radius: levelHoleRadius)

        ropes = validatedRopes.map { rope in
            RopeEndpoints(startHole: rope.startHole, endHole: rope.endHole, color: rope.color.simd, radius: rope.radius)
        }

        let ropeConfigs = ropes.map { rope in
            (startHole: rope.startHole, endHole: rope.endHole, color: rope.color)
        }
        topology = TopologyEngine(holePositions: levelHoles, ropeConfigs: ropeConfigs)

        for ropeIndex in ropes.indices {
            let startHoleIndex = ropes[ropeIndex].startHole
            let endHoleIndex = ropes[ropeIndex].endHole
            guard holeOccupied.indices.contains(startHoleIndex), holeOccupied.indices.contains(endHoleIndex) else { continue }
            guard let pinStart = holePositions[safe: startHoleIndex], let pinEnd = holePositions[safe: endHoleIndex] else { continue }
            holeOccupied[startHoleIndex] = true
            holeOccupied[endHoleIndex] = true
            simulation.setPins(
                ropeIndex: ropeIndex,
                pinStart: SIMD3<Float>(pinStart.x, pinStart.y, 0),
                pinEnd: SIMD3<Float>(pinEnd.x, pinEnd.y, 0)
            )
            ropeRestLengths[ropeIndex] = simd_length(pinEnd - pinStart)
        }
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

}
