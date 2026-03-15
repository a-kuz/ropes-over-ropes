import MetalKit
import simd

extension Renderer {
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        lastViewSize = size
        let offscreen = scaledOffscreenSize(from: size, view: view)
        resizeTextures(size: offscreen)
        let aspect = Float(size.width / max(1, size.height))
        let maxElev = holeElevations.max() ?? 0
        camera.fitToHoles(holePositions, holeRadius: holeRadius, aspect: aspect, maxElevation: maxElev)
    }

    func draw(in view: MTKView) {
        let cpuStart = CACurrentMediaTime()
        guard let drawable = view.currentDrawable else { return }
        guard let compositeRenderPass = view.currentRenderPassDescriptor else { return }
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
            let offscreen = scaledOffscreenSize(from: view.drawableSize, view: view)
            resizeTextures(size: offscreen)
        }

        guard let hdrTex else { return }
        guard let bloomA else { return }
        guard let bloomB else { return }
        guard let sceneDepth = sceneDepthTex else { return }

        encodeShadowPass(commandBuffer: commandBuffer)
        encodeHDRPass(commandBuffer: commandBuffer, hdrTexture: hdrTex, depthTexture: sceneDepth)
        if shaderParams.ropeEnvReflect > 0.001, let envTex, let envDepthTex {
            encodeEnvBlit(commandBuffer: commandBuffer, src: hdrTex, dst: envTex)
            encodeCopyDepth(commandBuffer: commandBuffer, src: sceneDepth, dst: envDepthTex)
            encodeRopeReflectionPass(commandBuffer: commandBuffer, hdrTexture: hdrTex, depthTexture: sceneDepth, envTexture: envTex, envDepthTexture: envDepthTex)
        }
        if shaderParams.bloomEnabled {
            encodeBloomPass(commandBuffer: commandBuffer, hdrTexture: hdrTex, bloomTextureA: bloomA, bloomTextureB: bloomB)
        }
        buildDebug2DData()
        encodeCompositePass(commandBuffer: commandBuffer, renderPass: compositeRenderPass, view: view, hdrTexture: hdrTex, bloomTextureA: bloomA, depthTexture: sceneDepth)

        let cpuEnd = CACurrentMediaTime()
        let cpuDuration = cpuEnd - cpuStart

        commandBuffer.addCompletedHandler { [weak self] buffer in
            let gpuDuration = buffer.gpuEndTime - buffer.gpuStartTime
            let duration: Double
            if gpuDuration > 0.000001 {
                duration = max(cpuDuration, gpuDuration)
            } else {
                duration = cpuDuration
            }
            let fps = duration > 0.0001 ? 1.0 / duration : 0
            self?.updatePotentialFPS(Float(fps))
        }

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
        if let shadowDepthTex {
            encoder.setFragmentTexture(shadowDepthTex, index: 2)
        }
        encoder.setFragmentTexture(bakedWoodTex ?? woodDebugTex, index: 3)
        encoder.setFragmentTexture(bakedHoleMaskTex ?? holeMaskDisabledTex, index: 4)
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

        // Draw weights and targets (tension mode)
        drawWeightsAndTargets(encoder: encoder)

        // Draw rails, carts, stations (rail mode)
        drawRailsAndCarts(encoder: encoder)

        // Draw ropes without env reflection (base lighting only)
        drawRopes(encoder: encoder)

        encoder.endEncoding()
    }

    private func drawRopes(encoder: MTLRenderCommandEncoder, envTexture: MTLTexture? = nil, envDepthTexture: MTLTexture? = nil) {
        encoder.setRenderPipelineState(ropePipeline)
        guard let ropeVB, let ropeIB, ropeIndexCount > 0 else { return }
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
        encoder.setFragmentTexture(envTexture ?? envDisabledTex, index: 4)
        if let envDepthTexture {
            encoder.setFragmentTexture(envDepthTexture, index: 5)
        }
        encoder.drawIndexedPrimitives(type: .triangle, indexCount: ropeIndexCount, indexType: .uint32, indexBuffer: ropeIB, indexBufferOffset: 0)
    }

    /// Second rope draw: overwrites base-lit ropes with reflection-enhanced versions.
    /// Uses loadAction=.load so the existing HDR + depth are preserved; depth=lessEqual
    /// means the same rope geometry overwrites exactly the same pixels.
    private func encodeRopeReflectionPass(commandBuffer: MTLCommandBuffer, hdrTexture: MTLTexture, depthTexture: MTLTexture, envTexture: MTLTexture, envDepthTexture: MTLTexture) {
        let renderPass = MTLRenderPassDescriptor()
        renderPass.colorAttachments[0].texture = hdrTexture
        renderPass.colorAttachments[0].loadAction = .load
        renderPass.colorAttachments[0].storeAction = .store
        renderPass.depthAttachment.texture = depthTexture
        renderPass.depthAttachment.loadAction = .load
        renderPass.depthAttachment.storeAction = .store
        renderPass.stencilAttachment.texture = depthTexture
        renderPass.stencilAttachment.loadAction = .load
        renderPass.stencilAttachment.storeAction = .dontCare

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPass) else { return }
        encoder.setDepthStencilState(depthStateScene)
        drawRopes(encoder: encoder, envTexture: envTexture, envDepthTexture: envDepthTexture)
        encoder.endEncoding()
    }

    private func encodeCopyDepth(commandBuffer: MTLCommandBuffer, src: MTLTexture, dst: MTLTexture) {
        guard let encoder = commandBuffer.makeComputeCommandEncoder() else { return }
        encoder.setComputePipelineState(copyDepthPipeline)
        encoder.setTexture(src, index: 0)
        encoder.setTexture(dst, index: 1)
        let w = copyDepthPipeline.threadExecutionWidth
        let h = copyDepthPipeline.maxTotalThreadsPerThreadgroup / w
        let threadsPerGroup = MTLSize(width: w, height: h, depth: 1)
        let gridSize = MTLSize(width: dst.width, height: dst.height, depth: 1)
        encoder.dispatchThreads(gridSize, threadsPerThreadgroup: threadsPerGroup)
        encoder.endEncoding()
    }

    func resizeShadowTexture() {
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

    private func encodeShadowPass(commandBuffer: MTLCommandBuffer) {
        if shadowDepthTex == nil {
            resizeShadowTexture()
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

    private func encodeCompositePass(commandBuffer: MTLCommandBuffer, renderPass: MTLRenderPassDescriptor, view: MTKView, hdrTexture: MTLTexture, bloomTextureA: MTLTexture, depthTexture: MTLTexture) {
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPass) else { return }
        encoder.setRenderPipelineState(postPipeline)
        encoder.setFragmentTexture(hdrTexture, index: 0)
        encoder.setFragmentTexture(bloomTextureA, index: 1)
        encoder.setFragmentTexture(depthTexture, index: 2)
        if let postParamsBuffer {
            encoder.setFragmentBuffer(postParamsBuffer, offset: 0, index: 0)
        }
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)

        // Debug 2D overlay (draws on top of composite)
        encodeDebug2DOverlay(encoder: encoder, view: view)

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

        let envDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .rgba16Float,  
            width: width,
            height: height,
            mipmapped: false
        )
        envDesc.usage = [.shaderRead, .shaderWrite]
        envTex = device.makeTexture(descriptor: envDesc)

        let envDepthDesc = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .r32Float,
            width: width,
            height: height,
            mipmapped: false
        )
        envDepthDesc.usage = [.shaderRead, .shaderWrite]
        envDepthTex = device.makeTexture(descriptor: envDepthDesc)

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

        resizeShadowTexture()
    }

    private func updateFrameUniforms(view: MTKView) {
        guard let frameUniforms else { return }
        let aspect = Float(view.drawableSize.width / max(1, view.drawableSize.height))
        let viewProjection = camera.viewProj(aspect: aspect)

        let sp = shaderParams
        if let motion = motionManager.deviceMotion {
            let g = motion.gravity
            let raw = SIMD2<Float>(Float(g.x), Float(g.y))
            deviceTilt += (raw - deviceTilt) * 0.4
        }
        let tiltOffset = SIMD3<Float>(deviceTilt.x * sp.tiltStrength, deviceTilt.y * sp.tiltStrength, 0)
        let lightDir = simd_normalize(sp.lightDir + tiltOffset)
        let halfH = camera.orthoHalfHeight
        let halfW = halfH * aspect
        let lightViewProj = makeLightViewProj(lightDir: lightDir, halfW: halfW, halfH: halfH)
        let invShadow = 1.0 / Float(max(1, shadowMapSize))

        let invViewProj = viewProjection.inverse

        let uniforms = FrameUniforms(
            viewProj: viewProjection,
            invViewProj: invViewProj,
            lightViewProj: lightViewProj,
            lightDirIntensity: SIMD4<Float>(lightDir.x, lightDir.y, lightDir.z, sp.lightIntensity),
            ambientColor: SIMD4<Float>(0, 0, 0, Float(highlightHoleIndex)),
            cameraPos: SIMD4<Float>(camera.center.x, camera.center.y + camera.distance * sin(camera.tiltAngle), camera.center.z + camera.distance * cos(camera.tiltAngle), 1),
            orthoHalfSizeShadowBias: SIMD4<Float>(halfW, halfH, sp.shadowBias, Float(sp.shadowType)),
            shadowInvSizeUnused: SIMD4<Float>(invShadow, invShadow, camera.center.x, camera.center.y),
            timeDrag: SIMD4<Float>(time, sp.ropeOpacity, Float(currentLevelId), dragState != nil ? 1 : 0),
            woodBoundsMin: SIMD4<Float>(woodBoundsMin.x, woodBoundsMin.y, boardWoodZMin, 0),
            woodBoundsMax: SIMD4<Float>(woodBoundsMax.x, woodBoundsMax.y, boardWoodZMax, 0),
            holeTint: sp.holeTint,
            visualParams: SIMD4<Float>(sp.exposure, sp.bloomStrength, sp.cartoonShaderEnabled ? 1 : 0, Float(sp.cartoonLevels)),
            lightingParams: SIMD4<Float>(sp.ambient, sp.shadowDarkness, sp.lightSize, sp.shadowsEnabled ? 1 : 0),
            tableParams: SIMD4<Float>(Float(sp.tableStyle), sp.tableColor1.x, sp.tableColor1.y, sp.tableColor1.z),
            tableParams2: SIMD4<Float>(sp.tableColor2.x, sp.tableColor2.y, sp.tableColor2.z, squareCrossSection ? 1 : 0),
            ropeMatParams: SIMD4<Float>(sp.ropeMatte, sp.ropeGloss, sp.ropeDiffuseWrap, sp.ropeSubsurface),
            ropeMatParams2: SIMD4<Float>(sp.ropeEdgeLight, sp.ropeSaturation, sp.ropeMicroBump, sp.ropeContactAO),
            ropeMatParams3: SIMD4<Float>(sp.ropeLiftGlow, sp.ropeBumpScale, sp.ropeStretchGloss, sp.ropeStretchSpec),
            cartoonParams: SIMD4<Float>(sp.cartoonShadowBright, sp.cartoonWrap, sp.pcssPenumbraScale, sp.ropeFlatNormals ? 1 : 0),
            wormParams1: SIMD4<Float>(sp.wormGrooveDepth, sp.wormBellyBright, sp.wormBackDark, sp.wormSkinNoise),
            wormParams2: SIMD4<Float>(sp.wormSSS, sp.wormRoughness, sp.wormSpecular, sp.wormRimStrength),
            wormParams3: SIMD4<Float>(sp.wormEyeSize, sp.wormPulseSpeed, sp.wormPulseAmp, sp.wormSegFreq),
            wormParams4: SIMD4<Float>(sp.wormCrawlSpeed, sp.wormCrawlAmp, sp.wormSideAmp, 0),
            ropeMatParams4: SIMD4<Float>(sp.ropeEnvReflect, sp.ropeEnvDebug ? 1 : 0, sp.ropeEnvSpread, Float(sp.shadowDebugMode))
        )
        frameUniforms.contents().copyMemory(from: [uniforms], byteCount: MemoryLayout<FrameUniforms>.stride)

        let useCartoon = sp.cartoonShaderEnabled
        let effExposure = useCartoon ? sp.cartoonExposure : sp.exposure
        let effBloom = sp.bloomEnabled ? (useCartoon ? sp.cartoonBloom : sp.bloomStrength) : 0
        let postParams = PostParams(exposure: effExposure, bloomStrength: effBloom, cartoonEdgeStrength: sp.cartoonEdgeStrength, cartoonMode: useCartoon ? 1 : 0, cartoonEdgeSmooth: sp.cartoonEdgeSmooth)
        postParamsBuffer?.contents().copyMemory(from: [postParams], byteCount: MemoryLayout<PostParams>.stride)
    }

    private func encodeEnvBlit(commandBuffer: MTLCommandBuffer, src: MTLTexture, dst: MTLTexture) {
        guard let blitEncoder = commandBuffer.makeBlitCommandEncoder() else { return }
        let w = min(src.width, dst.width)
        let h = min(src.height, dst.height)
        blitEncoder.copy(
            from: src, sourceSlice: 0, sourceLevel: 0,
            sourceOrigin: MTLOrigin(x: 0, y: 0, z: 0), sourceSize: MTLSize(width: w, height: h, depth: 1),
            to: dst, destinationSlice: 0, destinationLevel: 0,
            destinationOrigin: MTLOrigin(x: 0, y: 0, z: 0))
        blitEncoder.endEncoding()
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
