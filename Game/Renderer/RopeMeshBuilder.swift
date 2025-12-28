import simd

struct RopeMesh {
    var vertices: [RopeVertex]
    var indices: [UInt32]
}

enum RopeMeshBuilder {
    struct TwistEvent {
        let dist: Float
        let angle: Float
        let window: Float
    }

    static func buildRect(points: UnsafeBufferPointer<SIMD3<Float>>, width: Float, height: Float, color: SIMD3<Float>, twistEvents: [TwistEvent]) -> RopeMesh {
        let n = points.count
        if n < 2 { return RopeMesh(vertices: [], indices: []) }

        let w = max(0.0005, width)
        let h = max(0.0005, height)

        var totalLen: Float = 0
        for i in 1..<n {
            totalLen += simd_length(points[i] - points[i - 1])
        }
        totalLen = max(1e-6, totalLen)

        var vertices: [RopeVertex] = []
        vertices.reserveCapacity(n * 8)

        var indices: [UInt32] = []
        indices.reserveCapacity((n - 1) * 24)

        let up = SIMD3<Float>(0, 0, 1)

        var s: Float = 0
        var tPrev = simd_normalize(points[1] - points[0])
        var nrmPrev: SIMD3<Float> = {
            var nrm = simd_cross(up, tPrev)
            if simd_length_squared(nrm) < 1e-8 { nrm = SIMD3<Float>(1, 0, 0) }
            return simd_normalize(nrm)
        }()

        for i in 0..<n {
            let p = points[i]
            if i > 0 {
                s += simd_length(points[i] - points[i - 1])
            }

            let t: SIMD3<Float>
            if i == 0 {
                t = simd_normalize(points[1] - points[0])
            } else if i == n - 1 {
                t = simd_normalize(points[n - 1] - points[n - 2])
            } else {
                t = simd_normalize(points[i + 1] - points[i - 1])
            }

            var nrm = nrmPrev
            if i > 0 {
                let axis = simd_cross(tPrev, t)
                let axisLen = simd_length(axis)
                if axisLen > 1e-6 {
                    let axisN = axis / axisLen
                    let c = max(-1.0 as Float, min(1.0 as Float, simd_dot(tPrev, t)))
                    let ang = atan2(axisLen, c)
                    nrm = rotate(v: nrmPrev, axis: axisN, angle: ang)
                    let proj = nrm - t * simd_dot(nrm, t)
                    if simd_length_squared(proj) > 1e-10 {
                        nrm = simd_normalize(proj)
                    }
                }
            }

            var bin = simd_cross(t, nrm)
            if simd_length_squared(bin) < 1e-8 {
                var fallback = simd_cross(up, t)
                if simd_length_squared(fallback) < 1e-8 { fallback = SIMD3<Float>(1, 0, 0) }
                nrm = simd_normalize(fallback)
                bin = simd_cross(t, nrm)
            }
            bin = simd_normalize(bin)

            let twist = twistAngle(at: s, events: twistEvents)
            if abs(twist) > 1e-6 {
                let n2 = nrm * cos(twist) + bin * sin(twist)
                let b2 = -nrm * sin(twist) + bin * cos(twist)
                nrm = n2
                bin = b2
            }

            let u = s / totalLen
            let hx = w * 0.5
            let hy = h * 0.5

            let c00 = p + (-nrm * hx) + (-bin * hy)
            let c01 = p + (-nrm * hx) + ( bin * hy)
            let c10 = p + ( nrm * hx) + (-bin * hy)
            let c11 = p + ( nrm * hx) + ( bin * hy)

            vertices.append(RopeVertex(position: c00, normal: -nrm, color: color, uv: SIMD2<Float>(u, 0)))
            vertices.append(RopeVertex(position: c01, normal: -nrm, color: color, uv: SIMD2<Float>(u, 1)))
            vertices.append(RopeVertex(position: c10, normal: nrm, color: color, uv: SIMD2<Float>(u, 0)))
            vertices.append(RopeVertex(position: c11, normal: nrm, color: color, uv: SIMD2<Float>(u, 1)))

            vertices.append(RopeVertex(position: c00, normal: -bin, color: color, uv: SIMD2<Float>(u, 0)))
            vertices.append(RopeVertex(position: c10, normal: -bin, color: color, uv: SIMD2<Float>(u, 1)))
            vertices.append(RopeVertex(position: c01, normal: bin, color: color, uv: SIMD2<Float>(u, 0)))
            vertices.append(RopeVertex(position: c11, normal: bin, color: color, uv: SIMD2<Float>(u, 1)))

            if i < n - 1 {
                let a = UInt32(i * 8)
                let b = UInt32((i + 1) * 8)
                appendQuad(indices: &indices, a0: a + 0, a1: a + 1, b0: b + 0, b1: b + 1)
                appendQuad(indices: &indices, a0: a + 2, a1: a + 3, b0: b + 2, b1: b + 3)
                appendQuad(indices: &indices, a0: a + 4, a1: a + 5, b0: b + 4, b1: b + 5)
                appendQuad(indices: &indices, a0: a + 6, a1: a + 7, b0: b + 6, b1: b + 7)
            }

            tPrev = t
            nrmPrev = nrm
        }

        return RopeMesh(vertices: vertices, indices: indices)
    }

    private static func appendQuad(indices: inout [UInt32], a0: UInt32, a1: UInt32, b0: UInt32, b1: UInt32) {
        indices.append(a0)
        indices.append(b0)
        indices.append(b1)
        indices.append(a0)
        indices.append(b1)
        indices.append(a1)
    }

    private static func rotate(v: SIMD3<Float>, axis: SIMD3<Float>, angle: Float) -> SIMD3<Float> {
        let c = cos(angle)
        let s = sin(angle)
        return v * c + simd_cross(axis, v) * s + axis * simd_dot(axis, v) * (1 - c)
    }

    private static func twistAngle(at s: Float, events: [TwistEvent]) -> Float {
        if events.isEmpty { return 0 }
        var a: Float = 0
        for e in events {
            let d = abs(s - e.dist)
            let t = min(1, d / max(1e-6, e.window))
            let w = smoothstep(edge0: 1, edge1: 0, x: t)
            a += e.angle * w
        }
        return a
    }

    private static func smoothstep(edge0: Float, edge1: Float, x: Float) -> Float {
        let t = max(0, min(1, (x - edge0) / (edge1 - edge0)))
        return t * t * (3 - 2 * t)
    }
}

