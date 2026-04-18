import MetalKit
import simd

extension Renderer {

    // MARK: - Weight mesh (cylinder on board surface) — uses HoleVertex format

    func buildWeightMesh() {
        guard !weightRenderInfos.isEmpty else { return }

        let segments = 32
        let height: Float = 0.12

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        for info in weightRenderInfos {
            let baseIdx = UInt16(vertices.count)
            let cx = info.position.x
            let cy = info.position.y
            let r = info.radius > 0 ? info.radius : 0.15

            // Top cap center
            vertices.append(HoleVertex(position: SIMD3(cx, cy, height), normal: SIMD3(0, 0, 1)))

            // Top ring
            for i in 0...segments {
                let angle = Float(i) / Float(segments) * 2 * .pi
                let x = cx + cos(angle) * r
                let y = cy + sin(angle) * r
                vertices.append(HoleVertex(position: SIMD3(x, y, height), normal: SIMD3(0, 0, 1)))
            }

            // Top cap triangles
            for i in 0..<segments {
                indices.append(baseIdx)
                indices.append(baseIdx + UInt16(i) + 1)
                indices.append(baseIdx + UInt16(i) + 2)
            }

            // Side wall
            let wallBase = UInt16(vertices.count)
            for i in 0...segments {
                let angle = Float(i) / Float(segments) * 2 * .pi
                let nx = cos(angle)
                let ny = sin(angle)
                let x = cx + nx * r
                let y = cy + ny * r
                vertices.append(HoleVertex(position: SIMD3(x, y, height), normal: SIMD3(nx, ny, 0)))
                vertices.append(HoleVertex(position: SIMD3(x, y, 0), normal: SIMD3(nx, ny, 0)))
            }

            for i in 0..<segments {
                let i0 = wallBase + UInt16(i) * 2
                indices.append(i0)
                indices.append(i0 + 1)
                indices.append(i0 + 2)
                indices.append(i0 + 2)
                indices.append(i0 + 1)
                indices.append(i0 + 3)
            }
        }

        weightVB = device.makeBuffer(bytes: vertices, length: vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        weightIB = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
        weightIndexCount = indices.count
    }

    // MARK: - Target zone mesh (ring on board) — uses HoleVertex format

    func buildTargetMesh() {
        guard !targetRenderInfos.isEmpty else { return }

        let segments = 48
        let innerScale: Float = 0.85
        let ringHeight: Float = 0.002

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        for info in targetRenderInfos {
            let baseIdx = UInt16(vertices.count)
            let cx = info.position.x
            let cy = info.position.y
            let outerR = info.radius
            let innerR = outerR * innerScale

            for i in 0...segments {
                let angle = Float(i) / Float(segments) * 2 * .pi
                let cosA = cos(angle)
                let sinA = sin(angle)
                let ox = cx + cosA * outerR
                let oy = cy + sinA * outerR
                let ix = cx + cosA * innerR
                let iy = cy + sinA * innerR
                vertices.append(HoleVertex(position: SIMD3(ox, oy, ringHeight), normal: SIMD3(0, 0, 1)))
                vertices.append(HoleVertex(position: SIMD3(ix, iy, ringHeight), normal: SIMD3(0, 0, 1)))
            }

            for i in 0..<segments {
                let i0 = baseIdx + UInt16(i) * 2
                indices.append(i0)
                indices.append(i0 + 1)
                indices.append(i0 + 2)
                indices.append(i0 + 2)
                indices.append(i0 + 1)
                indices.append(i0 + 3)
            }
        }

        targetVB = device.makeBuffer(bytes: vertices, length: vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        targetIB = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
        targetIndexCount = indices.count
    }

    // MARK: - Update weight positions from simulator

    func updateWeightRenderState() {
        guard isTensionMode, let sim = simulator else { return }
        for wi in weightRenderInfos.indices {
            guard sim.weights.indices.contains(wi) else { continue }
            weightRenderInfos[wi].position = sim.weights[wi].position
            weightRenderInfos[wi].settled = sim.weights[wi].settled
        }
        for ti in targetRenderInfos.indices {
            let wi = targetRenderInfos[ti].weightIndex
            if sim.weights.indices.contains(wi) {
                targetRenderInfos[ti].satisfied = sim.weights[wi].settled
            }
        }
        // Rebuild weight mesh with updated positions
        buildWeightMesh()
    }

    // MARK: - Draw weights and targets in HDR pass

    func drawWeightsAndTargets(encoder: MTLRenderCommandEncoder) {
        guard isTensionMode else { return }

        // Use holePipeline with identity instance (pos=0, radius=1, elev=0)
        // so world-space positions in HoleVertex pass through unchanged
        encoder.setRenderPipelineState(holePipeline)
        encoder.setDepthStencilState(depthStateScene)
        if let frameUniforms {
            encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
            encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
        }
        if let shadowDepthTex {
            encoder.setFragmentTexture(shadowDepthTex, index: 2)
        }

        // Identity instance: position=(0,0,0), radius=1
        var identityInstance = HoleInstance(positionRadius: SIMD4<Float>(0, 0, 0, 1), tintColor: .zero)
        let identityBuf = device.makeBuffer(bytes: &identityInstance, length: MemoryLayout<HoleInstance>.stride, options: [.storageModeShared])
        encoder.setVertexBuffer(identityBuf, offset: 0, index: 2)

        // Draw target zones (rings on surface)
        if let targetVB, let targetIB, targetIndexCount > 0 {
            encoder.setVertexBuffer(targetVB, offset: 0, index: 0)
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: targetIndexCount, indexType: .uint16, indexBuffer: targetIB, indexBufferOffset: 0, instanceCount: 1)
        }

        // Draw weights (cylinders)
        if let weightVB, let weightIB, weightIndexCount > 0 {
            encoder.setVertexBuffer(weightVB, offset: 0, index: 0)
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: weightIndexCount, indexType: .uint16, indexBuffer: weightIB, indexBufferOffset: 0, instanceCount: 1)
        }
    }
}
