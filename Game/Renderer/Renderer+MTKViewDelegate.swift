import MetalKit
import simd

extension Renderer {
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        lastViewSize = size
        let offscreen = scaledOffscreenSize(from: size)
        resizeTextures(size: offscreen)
        let aspect = Float(size.width / max(1, size.height))
        let maxElev = holeElevations.max() ?? 0
        camera.fitToHoles(holePositions, holeRadius: holeRadius, aspect: aspect, maxElevation: maxElev)
    }

    func draw(in view: MTKView) {
        guard let drawable = view.currentDrawable else { return }
        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }

        let now = CACurrentMediaTime()
        let realDt = lastDrawTime > 0 ? Float(now - lastDrawTime) : 1.0 / Float(max(1, view.preferredFramesPerSecond))
        lastDrawTime = now
        let deltaTime = min(realDt, 1.0 / 15.0)
        lastDeltaTime = deltaTime
        time += deltaTime

        // Exponential moving average FPS
        let instantFPS = 1.0 / max(realDt, 0.0001)
        currentFPS = currentFPS == 0 ? instantFPS : currentFPS * 0.95 + instantFPS * 0.05

        updateFrameUniforms(view: view)

        updateFrameSimulation(deltaTime: deltaTime)

        lastViewSize = view.drawableSize
        if hdrTex == nil {
            let offscreen = scaledOffscreenSize(from: view.drawableSize)
            resizeTextures(size: offscreen)
        }

        guard let hdrTex else { return }
        guard let bloomA else { return }
        guard let bloomB else { return }
        guard let sceneDepth = sceneDepthTex else { return }

        encodeShadowPass(commandBuffer: commandBuffer)
        encodeHDRPass(commandBuffer: commandBuffer, hdrTexture: hdrTex, depthTexture: sceneDepth)
        encodeBloomPass(commandBuffer: commandBuffer, hdrTexture: hdrTex, bloomTextureA: bloomA, bloomTextureB: bloomB)
        encodeCompositePass(commandBuffer: commandBuffer, view: view, hdrTexture: hdrTex, bloomTextureA: bloomA, depthTexture: sceneDepth)

        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    private func encodeHDRPass(commandBuffer: MTLCommandBuffer, hdrTexture: MTLTexture, depthTexture: MTLTexture) {
        let renderPass = MTLRenderPassDescriptor()
        renderPass.colorAttachments[0].texture = hdrTexture
        renderPass.colorAttachments[0].loadAction = .clear
        renderPass.colorAttachments[0].storeAction = .store
        renderPass.colorAttachments[0].clearColor = MTLClearColor(red: 0, green: 0, blue: 0, alpha: 1)
        renderPass.depthAttachment.texture = depthTexture
        renderPass.depthAttachment.loadAction = .clear
        renderPass.depthAttachment.storeAction = .store
        renderPass.depthAttachment.clearDepth = 1
        renderPass.stencilAttachment.texture = depthTexture
        renderPass.stencilAttachment.loadAction = .clear
        renderPass.stencilAttachment.storeAction = .dontCare
        renderPass.stencilAttachment.clearStencil = 0

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPass) else { return }

        encoder.setRenderPipelineState(tablePipeline)
        encoder.setDepthStencilState(depthStateScene)
        if let frameUniforms {
            encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
        }
        if let holeInstances {
            var holeCount: UInt32 = UInt32(holeInstances.length / MemoryLayout<HoleInstance>.stride)
            encoder.setFragmentBytes(&holeCount, length: MemoryLayout<UInt32>.stride, index: 3)
            encoder.setFragmentBuffer(holeInstances, offset: 0, index: 4)
        } else {
            var holeCount: UInt32 = 0
            encoder.setFragmentBytes(&holeCount, length: MemoryLayout<UInt32>.stride, index: 3)
        }
        if let shadowDepthTex {
            encoder.setFragmentTexture(shadowDepthTex, index: 2)
        }
        if let bakedWoodTex {
            encoder.setFragmentTexture(bakedWoodTex, index: 3)
        }
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)

        if let boardVB = boardMeshVB, let boardIB = boardMeshIB, boardMeshIndexCount > 0 {
            encoder.setDepthStencilState(depthStateScene)
            encoder.setRenderPipelineState(boardPipeline)
            encoder.setVertexBuffer(boardVB, offset: 0, index: 0)
            if let frameUniforms {
                encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
                encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
            }
            if let holeInstances {
                var holeCount: UInt32 = UInt32(holeInstances.length / MemoryLayout<HoleInstance>.stride)
                encoder.setFragmentBytes(&holeCount, length: MemoryLayout<UInt32>.stride, index: 3)
                encoder.setFragmentBuffer(holeInstances, offset: 0, index: 4)
            } else {
                var holeCount: UInt32 = 0
                encoder.setFragmentBytes(&holeCount, length: MemoryLayout<UInt32>.stride, index: 3)
            }
            if let shadowDepthTex {
                encoder.setFragmentTexture(shadowDepthTex, index: 2)
            }
            if let boardTex = bakedBoardWoodVolumeTex {
                encoder.setFragmentTexture(boardTex, index: 3)
            }
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: boardMeshIndexCount, indexType: .uint32, indexBuffer: boardIB, indexBufferOffset: 0)
        }

        encoder.setDepthStencilState(depthStateScene)
        encoder.setRenderPipelineState(holePipeline)
        if let holeVB, let holeIB, holeIndexCount > 0, let holeInstances {
            encoder.setVertexBuffer(holeVB, offset: 0, index: 0)
            if let frameUniforms {
                encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
                encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
            }
            encoder.setVertexBuffer(holeInstances, offset: 0, index: 2)
            if let shadowDepthTex {
                encoder.setFragmentTexture(shadowDepthTex, index: 2)
            }
            let instanceCount = holeInstances.length / MemoryLayout<HoleInstance>.stride
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: holeIndexCount, indexType: .uint16, indexBuffer: holeIB, indexBufferOffset: 0, instanceCount: instanceCount)
        }

        encoder.setRenderPipelineState(ropePipeline)
        if let ropeVB, let ropeIB, ropeIndexCount > 0 {
            encoder.setVertexBuffer(ropeVB, offset: 0, index: 0)
            if let frameUniforms {
                encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
                encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
            }
            if let shadowDepthTex {
                encoder.setFragmentTexture(shadowDepthTex, index: 2)
            }
            if let noiseTexture {
                encoder.setFragmentTexture(noiseTexture, index: 3)
            }
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: ropeIndexCount, indexType: .uint32, indexBuffer: ropeIB, indexBufferOffset: 0)
        }

        encoder.endEncoding()
    }

    private func encodeShadowPass(commandBuffer: MTLCommandBuffer) {
        if shadowDepthTex == nil {
            resizeTextures(size: CGSize(width: 1, height: 1))
        }
        guard let shadowDepthTex else { return }

        let renderPass = MTLRenderPassDescriptor()
        renderPass.depthAttachment.texture = shadowDepthTex
        renderPass.depthAttachment.loadAction = .clear
        renderPass.depthAttachment.storeAction = .store
        renderPass.depthAttachment.clearDepth = 1

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPass) else { return }
        encoder.setDepthStencilState(depthStateShadow)

        if let holeVB, let holeIB, holeIndexCount > 0, let holeInstances {
            encoder.setRenderPipelineState(shadowHolePipeline)
            encoder.setVertexBuffer(holeVB, offset: 0, index: 0)
            if let frameUniforms {
                encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
            }
            encoder.setVertexBuffer(holeInstances, offset: 0, index: 2)
            let instanceCount = holeInstances.length / MemoryLayout<HoleInstance>.stride
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: holeIndexCount, indexType: .uint16, indexBuffer: holeIB, indexBufferOffset: 0, instanceCount: instanceCount)
        }

        if let boardVB = boardMeshVB, let boardIB = boardMeshIB, boardMeshIndexCount > 0 {
            encoder.setRenderPipelineState(shadowBoardPipeline)
            encoder.setVertexBuffer(boardVB, offset: 0, index: 0)
            if let frameUniforms {
                encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
            }
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: boardMeshIndexCount, indexType: .uint32, indexBuffer: boardIB, indexBufferOffset: 0)
        }

        if let ropeVB, let ropeIB, ropeIndexCount > 0 {
            encoder.setRenderPipelineState(shadowRopePipeline)
            encoder.setVertexBuffer(ropeVB, offset: 0, index: 0)
            if let frameUniforms {
                encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
            }
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: ropeIndexCount, indexType: .uint32, indexBuffer: ropeIB, indexBufferOffset: 0)
        }

        encoder.endEncoding()
    }

    private func encodeBloomPass(commandBuffer: MTLCommandBuffer, hdrTexture: MTLTexture, bloomTextureA: MTLTexture, bloomTextureB: MTLTexture) {
        guard let encoder = commandBuffer.makeComputeCommandEncoder() else { return }
        encodeBloom(encoder: encoder, hdrTexture: hdrTexture, bloomTextureA: bloomTextureA, bloomTextureB: bloomTextureB)
        encoder.endEncoding()
    }

    private func encodeCompositePass(commandBuffer: MTLCommandBuffer, view: MTKView, hdrTexture: MTLTexture, bloomTextureA: MTLTexture, depthTexture: MTLTexture) {
        guard let rpd = view.currentRenderPassDescriptor else { return }
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: rpd) else { return }
        encoder.setRenderPipelineState(postPipeline)
        encoder.setFragmentTexture(hdrTexture, index: 0)
        encoder.setFragmentTexture(bloomTextureA, index: 1)
        encoder.setFragmentTexture(depthTexture, index: 2)
        if let postParamsBuffer {
            encoder.setFragmentBuffer(postParamsBuffer, offset: 0, index: 0)
        }
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
        encoder.endEncoding()
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

        let sceneDepthDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .depth32Float_stencil8,
            width: width,
            height: height,
            mipmapped: false
        )
        sceneDepthDesc.usage = [.renderTarget, .shaderRead]
        sceneDepthTex = device.makeTexture(descriptor: sceneDepthDesc)

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

        let shadowDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .depth32Float,
            width: shadowMapSize,
            height: shadowMapSize,
            mipmapped: false
        )
        shadowDesc.usage = [.renderTarget, .shaderRead]
        shadowDesc.storageMode = .private
        shadowDepthTex = device.makeTexture(descriptor: shadowDesc)
    }

    private func updateFrameUniforms(view: MTKView) {
        guard let frameUniforms else { return }
        let aspect = Float(view.drawableSize.width / max(1, view.drawableSize.height))
        let viewProjection = camera.viewProj(aspect: aspect)

        let lightDir = simd_normalize(lightDir)
        let halfH = camera.orthoHalfHeight
        let halfW = halfH * aspect
        let lightViewProj = makeLightViewProj(lightDir: lightDir, halfW: halfW, halfH: halfH)
        let invShadow = 1.0 / Float(max(1, shadowMapSize))

        let invViewProj = viewProjection.inverse

        let uniforms = FrameUniforms(
            viewProj: viewProjection,
            invViewProj: invViewProj,
            lightViewProj: lightViewProj,
            lightDirIntensity: SIMD4<Float>(lightDir.x, lightDir.y, lightDir.z, lightIntensity),
            ambientColor: SIMD4<Float>(0, 0, 0, Float(highlightHoleIndex)),
            cameraPos: SIMD4<Float>(camera.center.x, camera.center.y + camera.distance * sin(camera.tiltAngle), camera.center.z + camera.distance * cos(camera.tiltAngle), 1),
            orthoHalfSizeShadowBias: SIMD4<Float>(halfW, halfH, shadowBias, Float(shadowType)),
            shadowInvSizeUnused: SIMD4<Float>(invShadow, invShadow, camera.center.x, camera.center.y),
            timeDrag: SIMD4<Float>(time, 0, Float(currentLevelId), dragState != nil ? 1 : 0),
            woodBoundsMin: SIMD4<Float>(woodBoundsMin.x, woodBoundsMin.y, boardWoodZMin, 0),
            woodBoundsMax: SIMD4<Float>(woodBoundsMax.x, woodBoundsMax.y, boardWoodZMax, 0),
            holeTint: holeTint,
            visualParams: SIMD4<Float>(exposure, bloomStrength, cartoonShaderEnabled ? 1 : 0, Float(cartoonLevels)),
            lightingParams: SIMD4<Float>(ambient, shadowDarkness, lightSize, shadowsEnabled ? 1 : 0),
            tableParams: SIMD4<Float>(Float(tableStyle), tableColor1.x, tableColor1.y, tableColor1.z),
            tableParams2: SIMD4<Float>(tableColor2.x, tableColor2.y, tableColor2.z, squareCrossSection ? 1 : 0),
            ropeMatParams: SIMD4<Float>(ropeMatte, ropeGloss, ropeDiffuseWrap, ropeSubsurface),
            ropeMatParams2: SIMD4<Float>(ropeEdgeLight, ropeSaturation, ropeMicroBump, ropeContactAO),
            ropeMatParams3: SIMD4<Float>(ropeLiftGlow, ropeBumpScale, ropeStretchGloss, ropeStretchSpec),
            cartoonParams: SIMD4<Float>(cartoonShadowBright, cartoonWrap, cartoonEdgeSmooth, 0),
            wormParams1: SIMD4<Float>(wormGrooveDepth, wormBellyBright, wormBackDark, wormSkinNoise),
            wormParams2: SIMD4<Float>(wormSSS, wormRoughness, wormSpecular, wormRimStrength),
            wormParams3: SIMD4<Float>(wormEyeSize, wormPulseSpeed, wormPulseAmp, wormSegFreq),
            wormParams4: SIMD4<Float>(wormCrawlSpeed, wormCrawlAmp, wormSideAmp, 0)
        )
        frameUniforms.contents().copyMemory(from: [uniforms], byteCount: MemoryLayout<FrameUniforms>.stride)

        let useCartoon = cartoonShaderEnabled
        let effExposure = useCartoon ? cartoonExposure : exposure
        let effBloom = useCartoon ? cartoonBloom : bloomStrength
        let postParams = PostParams(exposure: effExposure, bloomStrength: effBloom, cartoonEdgeStrength: cartoonEdgeStrength, cartoonMode: useCartoon ? 1 : 0, cartoonEdgeSmooth: cartoonEdgeSmooth)
        postParamsBuffer?.contents().copyMemory(from: [postParams], byteCount: MemoryLayout<PostParams>.stride)
    }

    private func makeLightViewProj(lightDir: SIMD3<Float>, halfW: Float, halfH: Float) -> simd_float4x4 {
        let target = camera.center
        let eye = target + lightDir * 4.9
        var up = SIMD3<Float>(0, 1, 0)
        if abs(simd_dot(up, lightDir)) > 0.95 {
            up = SIMD3<Float>(1, 0, 0)
        }
        let view = simd_float4x4.lookAt(eye: eye, center: target, up: up)
        let proj = simd_float4x4.ortho(left: -halfW * 1.08, right: halfW * 1.08, bottom: -halfH * 1.08, top: halfH * 1.08, near: 0.01, far: 12.0)
        return proj * view
    }
}
