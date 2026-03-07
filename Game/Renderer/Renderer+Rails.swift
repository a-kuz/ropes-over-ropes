import MetalKit
import simd

extension Renderer {

    // MARK: - Rail mesh (tube along polyline) — uses HoleVertex format

    func buildRailMesh() {
        guard !railRenderInfos.isEmpty else { return }

        let segments = 8   // circle segments for tube
        let tubeRadius: Float = 0.02

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        for info in railRenderInfos {
            guard info.points.count >= 2 else { continue }

            for pi in 0..<info.points.count {
                let p = info.points[pi]
                let tangent: SIMD2<Float>
                if pi == 0 {
                    tangent = simd_normalize(info.points[1] - info.points[0])
                } else if pi == info.points.count - 1 {
                    tangent = simd_normalize(info.points[pi] - info.points[pi-1])
                } else {
                    tangent = simd_normalize(info.points[pi+1] - info.points[pi-1])
                }
                let normal2D = SIMD2<Float>(-tangent.y, tangent.x)
                let z: Float = 0.01  // slightly above board

                let ringBase = UInt16(vertices.count)

                for si in 0...segments {
                    let angle = Float(si) / Float(segments) * 2 * .pi
                    let nx = normal2D.x * cos(angle)
                    let ny = normal2D.y * cos(angle)
                    let nz = sin(angle)
                    let x = p.x + nx * tubeRadius
                    let y = p.y + ny * tubeRadius
                    let vz = z + nz * tubeRadius
                    vertices.append(HoleVertex(position: SIMD3(x, y, vz), normal: SIMD3(nx, ny, nz)))
                }

                // Connect to previous ring
                if pi > 0 {
                    let prevBase = ringBase - UInt16(segments + 1)
                    for si in 0..<segments {
                        let i0 = prevBase + UInt16(si)
                        let i1 = ringBase + UInt16(si)
                        indices.append(i0)
                        indices.append(i1)
                        indices.append(i0 + 1)
                        indices.append(i0 + 1)
                        indices.append(i1)
                        indices.append(i1 + 1)
                    }
                }
            }
        }

        guard !vertices.isEmpty else { return }
        railVB = device.makeBuffer(bytes: vertices, length: vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        railIB = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
        railIndexCount = indices.count
    }

    // MARK: - Cart mesh (cylinder) — same as weight mesh

    func buildCartMesh() {
        guard !cartRenderInfos.isEmpty else { return }

        let segments = 24
        let height: Float = 0.15

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        for info in cartRenderInfos {
            let baseIdx = UInt16(vertices.count)
            let cx = info.position.x
            let cy = info.position.y
            let r = info.radius

            // Top cap center
            vertices.append(HoleVertex(position: SIMD3(cx, cy, height), normal: SIMD3(0, 0, 1)))

            // Top ring
            for i in 0...segments {
                let angle = Float(i) / Float(segments) * 2 * .pi
                vertices.append(HoleVertex(position: SIMD3(cx + cos(angle) * r, cy + sin(angle) * r, height), normal: SIMD3(0, 0, 1)))
            }

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
                vertices.append(HoleVertex(position: SIMD3(cx + nx * r, cy + ny * r, height), normal: SIMD3(nx, ny, 0)))
                vertices.append(HoleVertex(position: SIMD3(cx + nx * r, cy + ny * r, 0), normal: SIMD3(nx, ny, 0)))
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

        cartVB = device.makeBuffer(bytes: vertices, length: vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        cartIB = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
        cartIndexCount = indices.count
    }

    // MARK: - Station mesh (ring on surface)

    func buildStationMesh() {
        guard !stationRenderInfos.isEmpty else { return }

        let segments = 32
        let innerScale: Float = 0.85
        let ringHeight: Float = 0.003

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        for info in stationRenderInfos {
            let baseIdx = UInt16(vertices.count)
            let cx = info.position.x
            let cy = info.position.y
            let outerR = info.radius
            let innerR = outerR * innerScale

            for i in 0...segments {
                let angle = Float(i) / Float(segments) * 2 * .pi
                let cosA = cos(angle)
                let sinA = sin(angle)
                vertices.append(HoleVertex(position: SIMD3(cx + cosA * outerR, cy + sinA * outerR, ringHeight), normal: SIMD3(0, 0, 1)))
                vertices.append(HoleVertex(position: SIMD3(cx + cosA * innerR, cy + sinA * innerR, ringHeight), normal: SIMD3(0, 0, 1)))
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

        stationVB = device.makeBuffer(bytes: vertices, length: vertices.count * MemoryLayout<HoleVertex>.stride, options: [.storageModeShared])
        stationIB = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt16>.stride, options: [.storageModeShared])
        stationIndexCount = indices.count
    }

    // MARK: - Update cart positions from simulator

    func updateCartRenderState() {
        guard isRailMode, let sim = simulator else { return }
        for ci in cartRenderInfos.indices {
            guard sim.carts.indices.contains(ci) else { continue }
            let cart = sim.carts[ci]
            guard sim.rails.indices.contains(cart.railIndex) else { continue }
            let pos = sim.rails[cart.railIndex].position(at: cart.t)
            cartRenderInfos[ci].position = pos
            cartRenderInfos[ci].settled = cart.settled
        }

        // Update station satisfaction
        for si in stationRenderInfos.indices {
            let ci = stationRenderInfos[si].cartIndex
            stationRenderInfos[si].satisfied = cartRenderInfos.indices.contains(ci) && cartRenderInfos[ci].settled
        }

        buildCartMesh()
    }

    // MARK: - Draw rails, carts, stations in HDR pass

    func drawRailsAndCarts(encoder: MTLRenderCommandEncoder) {
        guard isRailMode else { return }

        encoder.setRenderPipelineState(holePipeline)
        encoder.setDepthStencilState(depthStateScene)
        if let frameUniforms {
            encoder.setVertexBuffer(frameUniforms, offset: 0, index: 1)
            encoder.setFragmentBuffer(frameUniforms, offset: 0, index: 1)
        }
        if let shadowDepthTex {
            encoder.setFragmentTexture(shadowDepthTex, index: 2)
        }

        var identityInstance = HoleInstance(positionRadius: SIMD4<Float>(0, 0, 0, 1))
        let identityBuf = device.makeBuffer(bytes: &identityInstance, length: MemoryLayout<HoleInstance>.stride, options: [.storageModeShared])
        encoder.setVertexBuffer(identityBuf, offset: 0, index: 2)

        // Draw rails
        if let railVB, let railIB, railIndexCount > 0 {
            encoder.setVertexBuffer(railVB, offset: 0, index: 0)
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: railIndexCount, indexType: .uint16, indexBuffer: railIB, indexBufferOffset: 0, instanceCount: 1)
        }

        // Draw stations
        if let stationVB, let stationIB, stationIndexCount > 0 {
            encoder.setVertexBuffer(stationVB, offset: 0, index: 0)
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: stationIndexCount, indexType: .uint16, indexBuffer: stationIB, indexBufferOffset: 0, instanceCount: 1)
        }

        // Draw carts
        if let cartVB, let cartIB, cartIndexCount > 0 {
            encoder.setVertexBuffer(cartVB, offset: 0, index: 0)
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: cartIndexCount, indexType: .uint16, indexBuffer: cartIB, indexBufferOffset: 0, instanceCount: 1)
        }
    }
}
