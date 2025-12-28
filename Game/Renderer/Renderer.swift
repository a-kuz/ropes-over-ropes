import MetalKit

final class Renderer: NSObject, MTKViewDelegate {
    let device: MTLDevice
    let commandQueue: MTLCommandQueue
    let tablePipeline: MTLRenderPipelineState
    let scenePipeline: MTLRenderPipelineState
    let ropePipeline: MTLRenderPipelineState
    let postPipeline: MTLRenderPipelineState
    let bloomThreshold: MTLComputePipelineState
    let bloomBlurH: MTLComputePipelineState
    let bloomBlurV: MTLComputePipelineState

    var depthStateScene: MTLDepthStencilState
    var depthStateBackground: MTLDepthStencilState

    var camera = Camera()
    var time: Float = 0

    var simulation: RopeSimulation

    var frameUniforms: MTLBuffer?
    var holeInstances: MTLBuffer?

    var holePositions: [SIMD2<Float>] = []
    var holeRadius: Float = 0.105
    var holeOccupied: [Bool] = []
    struct RopeEndpoints {
        var startHole: Int
        var endHole: Int
        var color: SIMD3<Float>
        var width: Float
        var height: Float
    }

    var ropes: [RopeEndpoints] = []
    var topology: TopologyEngine?
    var lastDragWorld: SIMD2<Float> = .zero

    struct DragState {
        var ropeIndex: Int
        var endIndex: Int
        var originalHoleIndex: Int
    }

    var dragState: DragState?
    var dragWorld: SIMD2<Float> = .zero
    var dragHeight: Float = 0.65
    var dragLiftCurrent: Float = 0

    var ropeVB: MTLBuffer?
    var ropeIB: MTLBuffer?
    var ropeIndexCount: Int = 0

    var hdrTex: MTLTexture?
    var bloomA: MTLTexture?
    var bloomB: MTLTexture?

    init(view: MTKView) {
        guard let device = view.device else { fatalError("Metal device is required") }
        guard let commandQueue = device.makeCommandQueue() else { fatalError("Command queue is required") }

        self.device = device
        self.commandQueue = commandQueue

        let fallbackLayout = Self.makeHoleLayout()
        let level = LevelLoader.load(levelId: 1)
        let decodedHoles = level?.holes.map { $0.simd }
        let levelHoles = (decodedHoles?.isEmpty == false) ? (decodedHoles ?? fallbackLayout.positions) : fallbackLayout.positions
        let levelHoleRadius = level?.holeRadius ?? fallbackLayout.radius
        let defaultRopes: [LevelDefinition.Rope] = [
            LevelDefinition.Rope(startHole: 0, endHole: min(14, levelHoles.count - 1), color: .init(redChannel: 0.20, greenChannel: 0.95, blueChannel: 0.35), width: 0.090, height: 0.030),
            LevelDefinition.Rope(startHole: 3, endHole: min(17, levelHoles.count - 1), color: .init(redChannel: 0.30, greenChannel: 0.55, blueChannel: 0.98), width: 0.088, height: 0.029),
            LevelDefinition.Rope(startHole: min(6, levelHoles.count - 1), endHole: min(11, levelHoles.count - 1), color: .init(redChannel: 0.96, greenChannel: 0.28, blueChannel: 0.33), width: 0.086, height: 0.028)
        ]
        let candidateRopes = level?.ropes ?? defaultRopes
        var validatedRopes = candidateRopes.filter { rope in
            guard levelHoles.indices.contains(rope.startHole) else { return false }
            guard levelHoles.indices.contains(rope.endHole) else { return false }
            return rope.startHole != rope.endHole
        }
        if validatedRopes.isEmpty, levelHoles.count >= 2 {
            validatedRopes = [LevelDefinition.Rope(startHole: 0, endHole: 1, color: .init(redChannel: 0.85, greenChannel: 0.85, blueChannel: 0.92), width: 0.090, height: 0.030)]
        }
        let particlesPerRope = max(8, level?.particlesPerRope ?? 64)

        self.simulation = RopeSimulation(device: device, ropeCount: max(1, validatedRopes.count), particlesPerRope: particlesPerRope)

        guard let library = device.makeDefaultLibrary() else {
            fatalError("Failed to create default Metal library")
        }

        self.tablePipeline = Self.makeTablePipeline(device: device, view: view, library: library)
        self.scenePipeline = Self.makeScenePipeline(device: device, view: view, library: library)
        self.ropePipeline = Self.makeRopePipeline(device: device, view: view, library: library)
        self.postPipeline = Self.makePostPipeline(device: device, view: view, library: library)
        (self.bloomThreshold, self.bloomBlurH, self.bloomBlurV) = Self.makeBloomPipelines(device: device, library: library)
        (self.depthStateScene, self.depthStateBackground) = Self.makeDepthStates(device: device)

        self.frameUniforms = device.makeBuffer(length: MemoryLayout<FrameUniforms>.stride, options: [.storageModeShared])
        self.holePositions = levelHoles
        self.holeRadius = levelHoleRadius
        self.holeOccupied = Array(repeating: false, count: levelHoles.count)
        self.holeInstances = Self.makeHoleInstances(device: device, positions: levelHoles, radius: levelHoleRadius)

        self.ropes = validatedRopes.map { rope in
            RopeEndpoints(startHole: rope.startHole, endHole: rope.endHole, color: rope.color.simd, width: rope.width, height: rope.height)
        }

        let topoRopes = self.ropes.map { rope in
            TopologyRope(nodes: [.hole(rope.startHole), .hole(rope.endHole)], color: rope.color, active: true)
        }
        self.topology = TopologyEngine(holePositions: levelHoles, ropes: topoRopes)

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
        }

        super.init()
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
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeScenePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "sceneVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "sceneFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makeRopePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "ropeVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "ropeFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeRopeVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    private static func makePostPipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "fullscreenVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "postFragment")
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
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

    private static func makeDepthStates(device: MTLDevice) -> (MTLDepthStencilState, MTLDepthStencilState) {
        let backgroundDescriptor = MTLDepthStencilDescriptor()
        backgroundDescriptor.depthCompareFunction = .always
        backgroundDescriptor.isDepthWriteEnabled = false

        let sceneDescriptor = MTLDepthStencilDescriptor()
        sceneDescriptor.depthCompareFunction = .lessEqual
        sceneDescriptor.isDepthWriteEnabled = true

        guard let backgroundState = device.makeDepthStencilState(descriptor: backgroundDescriptor),
              let sceneState = device.makeDepthStencilState(descriptor: sceneDescriptor) else {
            fatalError("Failed to create depth states")
        }

        return (sceneState, backgroundState)
    }

    private static func makeRopeVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = 0
        descriptor.attributes[0].bufferIndex = 0

        descriptor.attributes[1].format = .float3
        descriptor.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        descriptor.attributes[1].bufferIndex = 0

        descriptor.attributes[2].format = .float3
        descriptor.attributes[2].offset = MemoryLayout<SIMD3<Float>>.stride * 2
        descriptor.attributes[2].bufferIndex = 0

        descriptor.attributes[3].format = .float2
        descriptor.attributes[3].offset = MemoryLayout<SIMD3<Float>>.stride * 3
        descriptor.attributes[3].bufferIndex = 0

        descriptor.layouts[0].stride = MemoryLayout<RopeVertex>.stride
        return descriptor
    }

}
