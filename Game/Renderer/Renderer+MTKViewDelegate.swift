import MetalKit
import simd

extension Renderer {
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        resizeTextures(size: size)
    }

    func draw(in view: MTKView) {
        guard let drawable = view.currentDrawable else { return }
        guard let commandBuffer = commandQueue.makeCommandBuffer() else { return }

        let deltaTime = 1.0 / Float(max(1, view.preferredFramesPerSecond))
        time += deltaTime

        if dragState != nil {
            let v = (dragWorldTarget - lastDragWorld) / max(1e-4, deltaTime)
            lastDragWorld = dragWorldTarget
            let speed = simd_length(v)
            let target = min(1, speed * 0.025)
            dragVisualEnergy += (target - dragVisualEnergy) * min(1, deltaTime * 18)
            dragVisualEnergy *= pow(0.5, deltaTime * 1.4)
            
            if dragOscillationRopeIndex != nil {
                let damping: Float = 0.88
                let spring: Float = 14.0
                let oldPhase = dragOscillationPhase
                dragOscillationPhase += dragOscillationVelocity * deltaTime
                dragOscillationVelocity -= dragOscillationPhase * spring * deltaTime
                dragOscillationVelocity *= damping
                dragOscillationPhase *= damping
                
                if Int(time * 10) % 10 == 0 {
                    Self.logger.info("🌊 Oscillation during drag: phase=\(String(format: "%.4f", self.dragOscillationPhase)) velocity=\(String(format: "%.4f", self.dragOscillationVelocity))")
                }
                
                if abs(dragOscillationPhase) < 0.0005 && abs(dragOscillationVelocity) < 0.0005 {
                    dragOscillationPhase = 0.0
                    dragOscillationVelocity = 0.0
                }
            }
        } else {
            dragVisualEnergy *= pow(0.5, deltaTime * 6.0)
            lastDragWorld = dragWorld
            
            if dragOscillationRopeIndex != nil {
                let damping: Float = 0.92
                let spring: Float = 12.0
                dragOscillationPhase += dragOscillationVelocity * deltaTime
                dragOscillationVelocity -= dragOscillationPhase * spring * deltaTime
                dragOscillationVelocity *= damping
                dragOscillationPhase *= damping
                
                if abs(dragOscillationPhase) < 0.001 && abs(dragOscillationVelocity) < 0.001 {
                    dragOscillationPhase = 0.0
                    dragOscillationVelocity = 0.0
                    dragOscillationRopeIndex = nil
                }
            }
        }

        updateFrameUniforms(view: view)
        updateSnapAnimation(deltaTime: deltaTime)
        updateLazyDrag(deltaTime: deltaTime)
        updateSimulationTargets(deltaTime: deltaTime)
        applyDragPinsIfNeeded()

        simulation.step(deltaTime: deltaTime)
        updateRopeMesh()

        if hdrTex == nil {
            resizeTextures(size: view.drawableSize)
        }

        guard let hdrTex else { return }
        guard let bloomA else { return }
        guard let bloomB else { return }
        guard let depth = view.depthStencilTexture else { return }

        encodeShadowPass(commandBuffer: commandBuffer)
        encodeHDRPass(commandBuffer: commandBuffer, hdrTexture: hdrTex, depthTexture: depth)
        encodeBloomPass(commandBuffer: commandBuffer, hdrTexture: hdrTex, bloomTextureA: bloomA, bloomTextureB: bloomB)
        encodeCompositePass(commandBuffer: commandBuffer, view: view, hdrTexture: hdrTex, bloomTextureA: bloomA)

        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    private func updateSimulationTargets(deltaTime: Float) {
        guard let topology else {
            simulation.simulationEnabled = true
            return
        }

        let targetLift = (dragState != nil || snapAnimationState != nil) ? dragHeight : 0
        dragLiftCurrent += (targetLift - dragLiftCurrent) * min(1, deltaTime * 18)

        let dragLift: Float = dragLiftCurrent
        let ropeCount = min(simulation.ropeCount, topology.ropes.count)
        let animatingRopeIndex = snapAnimationState?.ropeIndex
        if ropeCount > 0 {
            for ropeIndex in 0..<ropeCount {
                if ropeIndex == animatingRopeIndex { continue }
                let ropeHeight = ropes[safe: ropeIndex]?.height ?? 0.03
                let lift = max(ropeHeight * 1.35, 0.02)
                let targets = TopologySampler.sampleRope(
                    engine: topology,
                    ropeIndex: ropeIndex,
                    count: simulation.particlesPerRope,
                    lift: lift,
                    dragLift: dragLift,
                    ropeWidth: ropes[safe: ropeIndex]?.width ?? 0.085,
                    ropeWidthForIndex: { idx in
                        self.ropes[safe: idx]?.width ?? 0.085
                    },
                    ropeHeightForIndex: { idx in
                        self.ropes[safe: idx]?.height ?? 0.03
                    },
                    holeRadius: holeRadius
                )
                simulation.updateTargets(ropeIndex: ropeIndex, positions: targets)
            }
        }

        simulation.projectAlpha = 0.65
        simulation.collisionsEnabled = true
        simulation.simulationEnabled = true
    }

    private func updateSnapAnimation(deltaTime: Float) {
        guard var snapState = snapAnimationState else { return }
        guard ropes.indices.contains(snapState.ropeIndex) else {
            snapAnimationState = nil
            return
        }

        let animationSpeed: Float = 8.0
        snapState.progress += deltaTime * animationSpeed
        snapState.progress = min(1.0, snapState.progress)

        let t = snapState.progress
        let easeOut = 1.0 - pow(1.0 - t, 3.0)
        let currentPos = snapState.startPosition + (snapState.targetPosition - snapState.startPosition) * easeOut
        let currentZ = snapState.startZ * (1.0 - easeOut)

        let endpoints = ropes[snapState.ropeIndex]
        let fixedHoleIndex = (snapState.endIndex == 0) ? endpoints.endHole : endpoints.startHole
        guard let fixedPos = holePositions[safe: fixedHoleIndex] else {
            snapAnimationState = nil
            return
        }

        if snapState.endIndex == 0 {
            simulation.setPins(
                ropeIndex: snapState.ropeIndex,
                pinStart: SIMD3<Float>(currentPos.x, currentPos.y, currentZ),
                pinEnd: SIMD3<Float>(fixedPos.x, fixedPos.y, 0)
            )
        } else {
            simulation.setPins(
                ropeIndex: snapState.ropeIndex,
                pinStart: SIMD3<Float>(fixedPos.x, fixedPos.y, 0),
                pinEnd: SIMD3<Float>(currentPos.x, currentPos.y, currentZ)
            )
        }

        topology?.setFloating(ropeIndex: snapState.ropeIndex, position: currentPos)

        if snapState.progress >= 1.0 {
            topology?.endDrag(ropeIndex: snapState.ropeIndex, endIndex: snapState.endIndex, holeIndex: snapState.targetHoleIndex)
            
            if let pinStart = holePositions[safe: endpoints.startHole],
               let pinEnd = holePositions[safe: endpoints.endHole] {
                simulation.setPins(
                    ropeIndex: snapState.ropeIndex,
                    pinStart: SIMD3<Float>(pinStart.x, pinStart.y, 0),
                    pinEnd: SIMD3<Float>(pinEnd.x, pinEnd.y, 0)
                )
            } else {
                simulation.deactivateRope(ropeIndex: snapState.ropeIndex)
                topology?.deactivateRope(ropeIndex: snapState.ropeIndex)
            }

            removeUntangledRopes()
            snapAnimationState = nil
        } else {
            snapAnimationState = snapState
        }
    }

    private func updateLazyDrag(deltaTime: Float) {
        guard let dragState, ropes.indices.contains(dragState.ropeIndex) else { return }

        let endpoints = ropes[dragState.ropeIndex]
        let fixedHoleIndex = (dragState.endIndex == 0) ? endpoints.endHole : endpoints.startHole
        guard let fixedPos = holePositions[safe: fixedHoleIndex] else { return }

        let targetLength = simd_length(dragWorldTarget - fixedPos)
        let currentLazyLength = simd_length(dragWorldLazy - fixedPos)
        let restLength = dragState.restLength

        let isStretching = targetLength > currentLazyLength
        let isShrinking = targetLength < currentLazyLength

        if isStretching {
            let stretchSpeed: Float = 12.0
            let toTarget = dragWorldTarget - dragWorldLazy
            let moveAmount = min(simd_length(toTarget), stretchSpeed * deltaTime)
            if moveAmount > 1e-6 {
                let moveDir = toTarget / simd_length(toTarget)
                dragWorldLazy += moveDir * moveAmount
            }
            dragSagProgress = 0.0
        } else if isShrinking {
            let sagThreshold: Float = 0.02
            let lengthDiff = currentLazyLength - targetLength
            
            if lengthDiff > sagThreshold {
                let sagSpeed: Float = 4.0
                dragSagProgress += deltaTime * sagSpeed
                dragSagProgress = min(1.0, dragSagProgress)
                
                let sagFactor = 1.0 - dragSagProgress
                let sagAmount = lengthDiff * sagFactor * 0.4
                
                let toTarget = dragWorldTarget - fixedPos
                let toTargetLen = simd_length(toTarget)
                if toTargetLen > 1e-6 {
                    let targetDir = toTarget / toTargetLen
                    let saggedLength = targetLength + sagAmount
                    let targetLazyPos = fixedPos + targetDir * saggedLength
                    
                    let pullSpeed: Float = 8.0
                    dragWorldLazy = dragWorldLazy + (targetLazyPos - dragWorldLazy) * min(1.0, deltaTime * pullSpeed)
                }
            } else {
                let shrinkSpeed: Float = 15.0
                let toTarget = dragWorldTarget - dragWorldLazy
                let moveAmount = min(simd_length(toTarget), shrinkSpeed * deltaTime)
                if moveAmount > 1e-6 {
                    let moveDir = toTarget / simd_length(toTarget)
                    dragWorldLazy += moveDir * moveAmount
                }
                if dragSagProgress > 0.0 {
                    dragSagProgress = max(0.0, dragSagProgress - deltaTime * 5.0)
                }
            }
        } else {
            if dragSagProgress > 0.0 {
                dragSagProgress = max(0.0, dragSagProgress - deltaTime * 5.0)
            }
        }

        topology?.setFloating(ropeIndex: dragState.ropeIndex, position: dragWorldLazy)
    }

    private func applyDragPinsIfNeeded() {
        guard let dragState, ropes.indices.contains(dragState.ropeIndex) else { return }

        let ropeIndex = dragState.ropeIndex
        let endpoints = ropes[ropeIndex]

        guard let startPin = holePositions[safe: endpoints.startHole],
              let endPin = holePositions[safe: endpoints.endHole] else {
            self.dragState = nil
            simulation.deactivateRope(ropeIndex: ropeIndex)
            topology?.deactivateRope(ropeIndex: ropeIndex)
            return
        }

        if dragState.endIndex == 0 {
            simulation.setPins(
                ropeIndex: ropeIndex,
                pinStart: SIMD3<Float>(dragWorldLazy.x, dragWorldLazy.y, dragLiftCurrent),
                pinEnd: SIMD3<Float>(endPin.x, endPin.y, 0)
            )
        } else {
            simulation.setPins(
                ropeIndex: ropeIndex,
                pinStart: SIMD3<Float>(startPin.x, startPin.y, 0),
                pinEnd: SIMD3<Float>(dragWorldLazy.x, dragWorldLazy.y, dragLiftCurrent)
            )
        }
    }

    private func encodeHDRPass(commandBuffer: MTLCommandBuffer, hdrTexture: MTLTexture, depthTexture: MTLTexture) {
        let renderPass = MTLRenderPassDescriptor()
        renderPass.colorAttachments[0].texture = hdrTexture
        renderPass.colorAttachments[0].loadAction = .clear
        renderPass.colorAttachments[0].storeAction = .store
        renderPass.colorAttachments[0].clearColor = MTLClearColor(red: 0.07, green: 0.08, blue: 0.11, alpha: 1)
        renderPass.depthAttachment.texture = depthTexture
        renderPass.depthAttachment.loadAction = .clear
        renderPass.depthAttachment.storeAction = .dontCare
        renderPass.depthAttachment.clearDepth = 1
        renderPass.stencilAttachment.texture = depthTexture
        renderPass.stencilAttachment.loadAction = .clear
        renderPass.stencilAttachment.storeAction = .dontCare
        renderPass.stencilAttachment.clearStencil = 0

        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPass) else { return }

        encoder.setRenderPipelineState(tablePipeline)
        encoder.setDepthStencilState(depthStateBackground)
        if let frameUniforms {
            encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
        }
        if let shadowDepthTex {
            encoder.setFragmentTexture(shadowDepthTex, index: 2)
        }
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)

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

    private func encodeCompositePass(commandBuffer: MTLCommandBuffer, view: MTKView, hdrTexture: MTLTexture, bloomTextureA: MTLTexture) {
        guard let rpd = view.currentRenderPassDescriptor else { return }
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: rpd) else { return }
        encoder.setRenderPipelineState(postPipeline)
        encoder.setFragmentTexture(hdrTexture, index: 0)
        encoder.setFragmentTexture(bloomTextureA, index: 1)
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

        let lightDir = simd_normalize(SIMD3<Float>(-0.92, -0.18, 0.35))
        let halfH = camera.orthoHalfHeight
        let halfW = halfH * aspect
        let lightViewProj = makeLightViewProj(lightDir: lightDir, halfW: halfW, halfH: halfH)
        let invShadow = 1.0 / Float(max(1, shadowMapSize))

        let uniforms = FrameUniforms(
            viewProj: viewProjection,
            lightViewProj: lightViewProj,
            lightDirIntensity: SIMD4<Float>(lightDir.x, lightDir.y, lightDir.z, 5.2),
            ambientColor: SIMD4<Float>(0, 0, 0, 1),
            cameraPos: SIMD4<Float>(camera.center.x, camera.center.y + camera.distance * sin(camera.tiltAngle), camera.center.z + camera.distance * cos(camera.tiltAngle), 1),
            orthoHalfSizeShadowBias: SIMD4<Float>(halfW, halfH, 0.0012, 0),
            shadowInvSizeUnused: SIMD4<Float>(invShadow, invShadow, 0, 0),
            timeDrag: SIMD4<Float>(time, dragVisualEnergy, dragLiftCurrent, dragState != nil ? 1 : 0)
        )
        frameUniforms.contents().copyMemory(from: [uniforms], byteCount: MemoryLayout<FrameUniforms>.stride)
    }

    private func makeLightViewProj(lightDir: SIMD3<Float>, halfW: Float, halfH: Float) -> simd_float4x4 {
        let target = SIMD3<Float>(0, 0, 0)
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

