import MetalKit
import simd

extension Renderer {
    static func generateNoiseTexture(device: MTLDevice, size: Int) -> MTLTexture? {
        let desc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rg8Unorm,
            width: size,
            height: size,
            mipmapped: false
        )
        desc.usage = .shaderRead
        guard let tex = device.makeTexture(descriptor: desc) else { return nil }

        let gridSize = 128
        var rng: UInt32 = 0xDEAD_BEEF
        func lcg() -> Float {
            rng = rng &* 1_664_525 &+ 1_013_904_223
            return Float(rng >> 8) / 16_777_216.0
        }
        var grid = [Float](repeating: 0, count: gridSize * gridSize * 2)
        for i in 0..<grid.count { grid[i] = lcg() }

        func gridVal(_ gx: Int, _ gy: Int, _ ch: Int) -> Float {
            let wx = ((gx % gridSize) + gridSize) % gridSize
            let wy = ((gy % gridSize) + gridSize) % gridSize
            return grid[(wy * gridSize + wx) * 2 + ch]
        }
        func smooth(_ t: Float) -> Float { t * t * (3 - 2 * t) }
        func valueNoise(_ x: Float, _ y: Float, _ freq: Int, _ ch: Int) -> Float {
            let fx = x * Float(freq), fy = y * Float(freq)
            let ix = Int(floor(fx)), iy = Int(floor(fy))
            let tx = smooth(fx - floor(fx)), ty = smooth(fy - floor(fy))
            let c00 = gridVal(ix, iy, ch), c10 = gridVal(ix+1, iy, ch)
            let c01 = gridVal(ix, iy+1, ch), c11 = gridVal(ix+1, iy+1, ch)
            let a = c00 + (c10 - c00) * tx
            let b = c01 + (c11 - c01) * tx
            return a + (b - a) * ty
        }

        var pixels = [UInt8](repeating: 0, count: size * size * 2)
        for y in 0..<size {
            for x in 0..<size {
                let u = Float(x) / Float(size)
                let v = Float(y) / Float(size)
                let idx = (y * size + x) * 2
                for ch in 0..<2 {
                    let o1 = valueNoise(u, v, 16, ch)
                    let o2 = valueNoise(u, v, 37, ch)
                    let o3 = valueNoise(u, v, 79, ch)
                    let o4 = valueNoise(u, v, 128, ch)
                    let fbm = o1 * 0.15 + o2 * 0.30 + o3 * 0.30 + o4 * 0.15
                    let white = lcg()
                    let val = fbm * 0.85 + white * 0.15
                    pixels[idx + ch] = UInt8(min(max(val, 0), 1) * 255)
                }
            }
        }

        tex.replace(
            region: MTLRegionMake2D(0, 0, size, size),
            mipmapLevel: 0,
            withBytes: pixels,
            bytesPerRow: size * 2
        )
        return tex
    }

    static func makePipeline(device: MTLDevice, descriptor: MTLRenderPipelineDescriptor) -> MTLRenderPipelineState {
        do {
            return try device.makeRenderPipelineState(descriptor: descriptor)
        } catch {
            fatalError(String(describing: error))
        }
    }

    static func makeComputePipeline(device: MTLDevice, function: MTLFunction) -> MTLComputePipelineState {
        do {
            return try device.makeComputePipelineState(function: function)
        } catch {
            fatalError(String(describing: error))
        }
    }

    static func makeTablePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "fullscreenVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "tableFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeHolePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
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

    static func makeRopePipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "ropeVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "ropeFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = true
        descriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        descriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        descriptor.colorAttachments[0].sourceAlphaBlendFactor = .sourceAlpha
        descriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeRopeVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeShadowRopePipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "ropeShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeRopeVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeShadowHolePipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "holeShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeHoleVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeBoardPipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "boardVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "boardFragment")
        descriptor.colorAttachments[0].pixelFormat = .rgba16Float
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeBoardVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeShadowBoardPipeline(device: MTLDevice, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "boardShadowVertex")
        descriptor.depthAttachmentPixelFormat = .depth32Float
        descriptor.vertexDescriptor = makeBoardVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeBoardVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float3
        descriptor.attributes[0].offset = 0
        descriptor.attributes[0].bufferIndex = 0
        descriptor.attributes[1].format = .float3
        descriptor.attributes[1].offset = MemoryLayout<SIMD3<Float>>.stride
        descriptor.attributes[1].bufferIndex = 0
        descriptor.attributes[2].format = .float2
        descriptor.attributes[2].offset = MemoryLayout<SIMD3<Float>>.stride * 2
        descriptor.attributes[2].bufferIndex = 0
        descriptor.layouts[0].stride = MemoryLayout<BoardVertex>.stride
        return descriptor
    }

    static func makePostPipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "fullscreenVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "postFragment")
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        descriptor.colorAttachments[0].isBlendingEnabled = false
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeBloomPipelines(device: MTLDevice, library: MTLLibrary) -> (MTLComputePipelineState, MTLComputePipelineState, MTLComputePipelineState) {
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

    static func makeDepthStates(device: MTLDevice) -> (MTLDepthStencilState, MTLDepthStencilState, MTLDepthStencilState) {
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

    static func makeRopeVertexDescriptor() -> MTLVertexDescriptor {
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
        descriptor.attributes[5].format = .float
        descriptor.attributes[5].offset = MemoryLayout<RopeVertex>.offset(of: \.ropeSeed) ?? 0
        descriptor.attributes[5].bufferIndex = 0
        descriptor.layouts[0].stride = MemoryLayout<RopeVertex>.stride
        return descriptor
    }

    static func makeDebug2DPipeline(device: MTLDevice, view: MTKView, library: MTLLibrary) -> MTLRenderPipelineState {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: "debug2DVertex")
        descriptor.fragmentFunction = library.makeFunction(name: "debug2DFragment")
        descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
        descriptor.colorAttachments[0].isBlendingEnabled = true
        descriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
        descriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
        descriptor.colorAttachments[0].sourceAlphaBlendFactor = .one
        descriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
        descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.stencilAttachmentPixelFormat = view.depthStencilPixelFormat
        descriptor.vertexDescriptor = makeDebug2DVertexDescriptor()
        return makePipeline(device: device, descriptor: descriptor)
    }

    static func makeDebug2DVertexDescriptor() -> MTLVertexDescriptor {
        let descriptor = MTLVertexDescriptor()
        descriptor.attributes[0].format = .float2
        descriptor.attributes[0].offset = 0
        descriptor.attributes[0].bufferIndex = 0
        descriptor.attributes[1].format = .float4
        descriptor.attributes[1].offset = MemoryLayout<Debug2DVertexData>.offset(of: \.color)!
        descriptor.attributes[1].bufferIndex = 0
        descriptor.layouts[0].stride = MemoryLayout<Debug2DVertexData>.stride
        return descriptor
    }

    static func makeHoleVertexDescriptor() -> MTLVertexDescriptor {
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

    static func buildHoleMeshBuffers(device: MTLDevice, segments: Int, square: Bool = false, padMode: Bool = false, padHeight: Float = 0.18, vertexBuffer: inout MTLBuffer?, indexBuffer: inout MTLBuffer?, indexCount: inout Int) {
        let mesh: HoleMesh
        if padMode {
            mesh = HoleMeshBuilder.buildPad(segments: segments, height: padHeight)
        } else if square {
            mesh = HoleMeshBuilder.buildSquare()
        } else {
            mesh = HoleMeshBuilder.build(segments: segments)
        }
        indexCount = mesh.indices.count
        vertexBuffer = device.makeBuffer(bytes: mesh.vertices, length: mesh.vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        indexBuffer = device.makeBuffer(bytes: mesh.indices, length: mesh.indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
    }
}
