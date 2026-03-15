import MetalKit
import simd

extension Renderer {

    /// Build vertex buffers for the 2D debug overlay drawn over the full screen.
    func buildDebug2DData() {
        guard debug2DOverlay, let sim = simulator else {
            debug2DLineCount = 0
            return
        }

        let aspect = Float(lastViewSize.width / max(1, lastViewSize.height))
        let halfH = camera.orthoHalfHeight
        let halfW = halfH * aspect
        let cx = camera.center.x
        let cy = camera.center.y

        func toNDC(_ wx: Float, _ wy: Float) -> SIMD2<Float> {
            SIMD2<Float>((wx - cx) / halfW, -((wy - cy) / halfH))
        }

        // Thickness in NDC space (maps to ~3-4 pixels)
        let thickness: Float = 4.0 / Float(lastViewSize.height)

        let skip = 3
        let ropeColors: [SIMD4<Float>] = [
            SIMD4<Float>(1, 0.3, 0.3, 0.95),
            SIMD4<Float>(0.3, 1, 0.3, 0.95),
            SIMD4<Float>(0.4, 0.6, 1, 0.95),
            SIMD4<Float>(1, 1, 0.3, 0.95),
            SIMD4<Float>(1, 0.3, 1, 0.95),
            SIMD4<Float>(0.3, 1, 1, 0.95),
            SIMD4<Float>(1, 0.6, 0.2, 0.95),
            SIMD4<Float>(0.7, 0.3, 1, 0.95),
        ]

        // Build thick line quads (2 triangles per segment = 6 vertices)
        var triVerts: [Debug2DVertexData] = []
        triVerts.reserveCapacity(sim.bands.count * 120 * 6)

        for bi in sim.bands.indices {
            guard sim.bands[bi].active else { continue }
            let color = ropeColors[bi % ropeColors.count]
            let positions = sim.bands[bi].positions
            for i in 0..<(positions.count - 1) {
                let p0 = toNDC(positions[i].x, positions[i].y)
                let p1 = toNDC(positions[i + 1].x, positions[i + 1].y)
                let dir = p1 - p0
                let len = simd_length(dir)
                guard len > 1e-6 else { continue }
                let perp = SIMD2<Float>(-dir.y, dir.x) / len * thickness

                let a = p0 + perp
                let b = p0 - perp
                let c = p1 + perp
                let d = p1 - perp
                // Triangle 1: a, b, c
                triVerts.append(Debug2DVertexData(position: a, color: color))
                triVerts.append(Debug2DVertexData(position: b, color: color))
                triVerts.append(Debug2DVertexData(position: c, color: color))
                // Triangle 2: b, d, c
                triVerts.append(Debug2DVertexData(position: b, color: color))
                triVerts.append(Debug2DVertexData(position: d, color: color))
                triVerts.append(Debug2DVertexData(position: c, color: color))
            }
        }

        // Build intersection markers — small diamonds
        var markerVerts: [Debug2DVertexData] = []
        let crossColor = SIMD4<Float>(1, 0, 0, 1)
        let markerSize: Float = 0.04

        for i in 0..<sim.bands.count {
            guard sim.bands[i].active && sim.bands[i].fadeOut == 0 else { continue }
            let posA = sim.bands[i].positions
            let nA = posA.count
            let startA = skip
            let endA = max(startA, nA - 1 - skip)

            for j in (i + 1)..<sim.bands.count {
                guard sim.bands[j].active && sim.bands[j].fadeOut == 0 else { continue }
                let posB = sim.bands[j].positions
                let nB = posB.count
                let startB = skip
                let endB = max(startB, nB - 1 - skip)

                for si in startA..<endA {
                    let a0 = SIMD2<Float>(posA[si].x, posA[si].y)
                    let a1 = SIMD2<Float>(posA[si + 1].x, posA[si + 1].y)

                    for sj in startB..<endB {
                        let b0 = SIMD2<Float>(posB[sj].x, posB[sj].y)
                        let b1 = SIMD2<Float>(posB[sj + 1].x, posB[sj + 1].y)

                        if let pt = segmentIntersection2D(a0, a1, b0, b1) {
                            let c = toNDC(pt.x, pt.y)
                            let s = markerSize
                            // Diamond: 4 triangles
                            let top = c + SIMD2<Float>(0, s)
                            let bot = c - SIMD2<Float>(0, s)
                            let lft = c - SIMD2<Float>(s, 0)
                            let rgt = c + SIMD2<Float>(s, 0)
                            markerVerts.append(Debug2DVertexData(position: c, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: top, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: rgt, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: c, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: rgt, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: bot, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: c, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: bot, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: lft, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: c, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: lft, color: crossColor))
                            markerVerts.append(Debug2DVertexData(position: top, color: crossColor))
                        }
                    }
                }
            }
        }

        // Background quad (semi-transparent dark)
        let bgColor = SIMD4<Float>(0, 0, 0, 0.5)
        var bgVerts: [Debug2DVertexData] = [
            Debug2DVertexData(position: SIMD2<Float>(-1, -1), color: bgColor),
            Debug2DVertexData(position: SIMD2<Float>( 1, -1), color: bgColor),
            Debug2DVertexData(position: SIMD2<Float>(-1,  1), color: bgColor),
            Debug2DVertexData(position: SIMD2<Float>( 1, -1), color: bgColor),
            Debug2DVertexData(position: SIMD2<Float>( 1,  1), color: bgColor),
            Debug2DVertexData(position: SIMD2<Float>(-1,  1), color: bgColor),
        ]

        // Combine: bg + lines + markers
        let allVerts = bgVerts + triVerts + markerVerts
        debug2DLineCount = allVerts.count

        // Debug log (throttled)
        let now = CACurrentMediaTime()
        if now - lastDebug2DLogTime > 2.0 {
            lastDebug2DLogTime = now
            let activeBands = sim.bands.filter { $0.active && $0.fadeOut == 0 }.count
            // Log first few marker NDC positions
            let markerCount = markerVerts.count / 12
            var posLog = ""
            for m in 0..<min(markerCount, 4) {
                let v = markerVerts[m * 12] // center vertex
                posLog += " (\(String(format:"%.3f",v.position.x)),\(String(format:"%.3f",v.position.y)))"
            }
            Self.logger.info("[DEBUG2D] bands=\(activeBands) triVerts=\(triVerts.count) markers=\(markerCount)\(posLog)")
        }

        if allVerts.isEmpty {
            debug2DLineVB = nil
        } else {
            let size = allVerts.count * MemoryLayout<Debug2DVertexData>.stride
            if let existing = debug2DLineVB, existing.length >= size {
                existing.contents().copyMemory(from: allVerts, byteCount: size)
            } else {
                debug2DLineVB = device.makeBuffer(bytes: allVerts, length: size, options: .storageModeShared)
            }
        }
    }

    private func segmentIntersection2D(
        _ a0: SIMD2<Float>, _ a1: SIMD2<Float>,
        _ b0: SIMD2<Float>, _ b1: SIMD2<Float>
    ) -> SIMD2<Float>? {
        let d1 = a1 - a0
        let d2 = b1 - b0
        let cross = d1.x * d2.y - d1.y * d2.x
        if abs(cross) < 1e-9 { return nil }
        let d = b0 - a0
        let t = (d.x * d2.y - d.y * d2.x) / cross
        let u = (d.x * d1.y - d.y * d1.x) / cross
        guard t > 1e-6 && t < (1 - 1e-6) && u > 1e-6 && u < (1 - 1e-6) else { return nil }
        return a0 + d1 * t
    }

    func encodeDebug2DOverlay(encoder: MTLRenderCommandEncoder, view: MTKView) {
        guard debug2DOverlay, debug2DLineCount > 0, let vb = debug2DLineVB else { return }

        encoder.setRenderPipelineState(debug2DPipeline)
        encoder.setDepthStencilState(depthStateBackground)
        encoder.setVertexBuffer(vb, offset: 0, index: 0)
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: debug2DLineCount)
    }
}
