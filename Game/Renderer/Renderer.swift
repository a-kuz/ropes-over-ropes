import MetalKit

final class Renderer: NSObject, MTKViewDelegate {
    let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private let tablePipeline: MTLRenderPipelineState
    private let scenePipeline: MTLRenderPipelineState
    private let ropePipeline: MTLRenderPipelineState
    private let postPipeline: MTLRenderPipelineState
    let bloomThreshold: MTLComputePipelineState
    let bloomBlurH: MTLComputePipelineState
    let bloomBlurV: MTLComputePipelineState

    private var depthStateScene: MTLDepthStencilState
    private var depthStateBackground: MTLDepthStencilState

    var camera = Camera()
    private var time: Float = 0

    var simulation: RopeSimulation

    private var frameUniforms: MTLBuffer?
    private var holeInstances: MTLBuffer?

    var holePositions: [SIMD2<Float>] = []
    var holeRadius: Float = 0.105
    var holeOccupied: [Bool] = []
    struct RopeEndpoints {
        var startHole: Int
        var endHole: Int
        var color: SIMD3<Float>
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

    var ropeVB: MTLBuffer?
    var ropeIB: MTLBuffer?
    var ropeIndexCount: Int = 0

    private var hdrTex: MTLTexture?
    private var bloomA: MTLTexture?
    private var bloomB: MTLTexture?

    init(view: MTKView) {
        guard let device = view.device else { fatalError("Metal device is required") }
        guard let commandQueue = device.makeCommandQueue() else { fatalError("Command queue is required") }

        self.device = device
        self.commandQueue = commandQueue

        let level = LevelLoader.load(levelId: 1)
        let levelHoles = level?.holes.map { $0.simd } ?? Self.makeHoleLayout().positions
        let levelHoleRadius = level?.holeRadius ?? Self.makeHoleLayout().radius
        let levelRopes = level?.ropes ?? [
            LevelDefinition.Rope(startHole: 0, endHole: min(14, levelHoles.count - 1), color: .init(red: 0.20, green: 0.95, blue: 0.35)),
            LevelDefinition.Rope(startHole: 3, endHole: min(17, levelHoles.count - 1), color: .init(red: 0.30, green: 0.55, blue: 0.98)),
            LevelDefinition.Rope(startHole: min(6, levelHoles.count - 1), endHole: min(11, levelHoles.count - 1), color: .init(red: 0.96, green: 0.28, blue: 0.33))
        ]
        let particlesPerRope = level?.particlesPerRope ?? 64

        self.simulation = RopeSimulation(device: device, ropeCount: max(1, levelRopes.count), particlesPerRope: particlesPerRope)

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

        self.ropes = levelRopes.map { rope in
            RopeEndpoints(startHole: rope.startHole, endHole: rope.endHole, color: rope.color.simd)
        }

        let topoRopes = self.ropes.enumerated().map { index, rope in
            TopologyRope(nodes: [.hole(rope.startHole), .hole(rope.endHole)], color: rope.color, active: true)
        }
        self.topology = TopologyEngine(holePositions: levelHoles, ropes: topoRopes)

        for ropeIndex in ropes.indices {
            let startHoleIndex = ropes[ropeIndex].startHole
            let endHoleIndex = ropes[ropeIndex].endHole
            holeOccupied[startHoleIndex] = true
            holeOccupied[endHoleIndex] = true

            let pinStart = holePositions[startHoleIndex]
            let pinEnd = holePositions[endHoleIndex]
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

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        resizeTextures(size: size)
    }

    func draw(in view: MTKView) {
        guard let drawable = view.currentDrawable else { return }
        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }

        let deltaTime = 1.0 / Float(max(1, view.preferredFramesPerSecond))
        time += deltaTime

        updateFrameUniforms(view: view)

        if let topology {
            topology.relaxCrossingPositions()
            let lift: Float = 0.14
            let dragLift: Float = dragHeight
            for ropeIndex in 0..<min(simulation.ropeCount, topology.ropes.count) {
                let targets = TopologySampler.sampleRope(
                    engine: topology,
                    ropeIndex: ropeIndex,
                    count: simulation.particlesPerRope,
                    lift: lift,
                    dragLift: dragLift
                )
                simulation.updateTargets(ropeIndex: ropeIndex, positions: targets)
            }
        }
        if let dragState {
            let ropeIndex = dragState.ropeIndex
            let endpoints = ropes[ropeIndex]
            let startHole = endpoints.startHole
            let endHole = endpoints.endHole
            let startPin = holePositions[startHole]
            let endPin = holePositions[endHole]

            if dragState.endIndex == 0 {
                simulation.setPins(
                    ropeIndex: ropeIndex,
                    pinStart: SIMD3<Float>(dragWorld.x, dragWorld.y, dragHeight),
                    pinEnd: SIMD3<Float>(endPin.x, endPin.y, 0)
                )
            } else {
                simulation.setPins(
                    ropeIndex: ropeIndex,
                    pinStart: SIMD3<Float>(startPin.x, startPin.y, 0),
                    pinEnd: SIMD3<Float>(dragWorld.x, dragWorld.y, dragHeight)
                )
            }
        }
        simulation.step(deltaTime: deltaTime)
        updateRopeMesh()

        if hdrTex == nil {
            resizeTextures(size: view.drawableSize)
        }

        guard let hdrTex else { return }
        guard let bloomA else { return }
        guard let bloomB else { return }
        guard let depth = view.depthStencilTexture else { return }

        let renderPass = MTLRenderPassDescriptor()
        renderPass.colorAttachments[0].texture = hdrTex
        renderPass.colorAttachments[0].loadAction = .clear
        renderPass.colorAttachments[0].storeAction = .store
        renderPass.colorAttachments[0].clearColor = MTLClearColor(red: 0.07, green: 0.08, blue: 0.11, alpha: 1)
        renderPass.depthAttachment.texture = depth
        renderPass.depthAttachment.loadAction = .clear
        renderPass.depthAttachment.storeAction = .dontCare
        renderPass.depthAttachment.clearDepth = 1
        renderPass.stencilAttachment.texture = depth
        renderPass.stencilAttachment.loadAction = .clear
        renderPass.stencilAttachment.storeAction = .dontCare
        renderPass.stencilAttachment.clearStencil = 0

        if let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPass) {
            encoder.setRenderPipelineState(tablePipeline)
            encoder.setDepthStencilState(depthStateBackground)
            if let frameUniforms {
                encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
            }
            encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)

            encoder.setDepthStencilState(depthStateScene)
            encoder.setRenderPipelineState(scenePipeline)
            if let frameUniforms {
                encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
                encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
            }
            if let holeInstances {
                encoder.setVertexBuffer(holeInstances, offset: 0, index: 2)
                encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 6, instanceCount: holeInstances.length / MemoryLayout<HoleInstance>.stride)
            }

            encoder.setRenderPipelineState(ropePipeline)
            if let ropeVB, let ropeIB, ropeIndexCount > 0 {
                encoder.setVertexBuffer(ropeVB, offset: 0, index: 0)
                if let frameUniforms {
                    encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
                    encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
                }
                encoder.drawIndexedPrimitives(type: .triangle, indexCount: ropeIndexCount, indexType: .uint32, indexBuffer: ropeIB, indexBufferOffset: 0)
            }

            encoder.endEncoding()
        }

        if let encoder = commandBuffer.makeComputeCommandEncoder() {
            encodeBloom(encoder: encoder, hdrTexture: hdrTex, bloomTextureA: bloomA, bloomTextureB: bloomB)
            encoder.endEncoding()
        }

        if let rpd = view.currentRenderPassDescriptor, let enc = commandBuffer.makeRenderCommandEncoder(descriptor: rpd) {
            enc.setRenderPipelineState(postPipeline)
            enc.setFragmentTexture(hdrTex, index: 0)
            enc.setFragmentTexture(bloomA, index: 1)
            enc.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
            enc.endEncoding()
        }

        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    private func resizeTextures(size: CGSize) {
        let width = max(1, Int(size.width.rounded()))
        let height = max(1, Int(size.height.rounded()))

        let hdrDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rgba16Float,
            width: width,
            height: height,
            mipmapped: false
        )
        hdrDesc.usage = [.renderTarget, .shaderRead, .shaderWrite]
        hdrTex = device.makeTexture(descriptor: hdrDesc)

        let bloomWidth = max(1, width / 2)
        let bloomHeight = max(1, height / 2)
        let bloomDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rgba16Float,
            width: bloomWidth,
            height: bloomHeight,
            mipmapped: false
        )
        bloomDesc.usage = [.shaderRead, .shaderWrite]
        bloomA = device.makeTexture(descriptor: bloomDesc)
        bloomB = device.makeTexture(descriptor: bloomDesc)
    }

    private func updateFrameUniforms(view: MTKView) {
        guard let frameUniforms else { return }
        let aspect = Float(view.drawableSize.width / max(1, view.drawableSize.height))
        let viewProjection = camera.viewProj(aspect: aspect)

        let lightDir = simd_normalize(SIMD3<Float>(-0.35, 0.25, 0.9))
        let uniforms = FrameUniforms(
            viewProj: viewProjection,
            lightDir_intensity: SIMD4<Float>(lightDir.x, lightDir.y, lightDir.z, 2.8),
            ambientColor: SIMD4<Float>(0.08, 0.09, 0.12, 1),
            cameraPos: SIMD4<Float>(0, 0, camera.distance, 1)
        )
        frameUniforms.contents().copyMemory(from: [uniforms], byteCount: MemoryLayout<FrameUniforms>.stride)
    }
}

