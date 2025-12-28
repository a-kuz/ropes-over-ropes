import simd

struct RopeMesh {
    var vertices: [RopeVertex]
    var indices: [UInt32]
}

enum RopeMeshBuilder {
    static func build(points: UnsafeBufferPointer<SIMD3<Float>>, radius: Float, sides: Int, color: SIMD3<Float>) -> RopeMesh {
        let clampedSides = max(6, min(32, sides))
        let n = points.count
        if n < 2 {
            return RopeMesh(vertices: [], indices: [])
        }

        var vertices: [RopeVertex] = []
        vertices.reserveCapacity(n * clampedSides)

        let up = SIMD3<Float>(0, 0, 1)

        for i in 0..<n {
            let p = points[i]
            let t: SIMD3<Float>
            if i == 0 {
                t = simd_normalize(points[1] - points[0])
            } else if i == n - 1 {
                t = simd_normalize(points[n - 1] - points[n - 2])
            } else {
                t = simd_normalize(points[i + 1] - points[i - 1])
            }

            var nrm = simd_cross(up, t)
            if simd_length_squared(nrm) < 1e-8 {
                nrm = SIMD3<Float>(1, 0, 0)
            } else {
                nrm = simd_normalize(nrm)
            }
            let bin = simd_normalize(simd_cross(t, nrm))

            let u = Float(i) / Float(n - 1)
            for s in 0..<clampedSides {
                let a = (Float(s) / Float(clampedSides)) * (Float.pi * 2)
                let dir = nrm * cos(a) + bin * sin(a)
                let pos = p + dir * radius
                vertices.append(RopeVertex(position: pos, normal: dir, color: color, uv: SIMD2<Float>(u, Float(s) / Float(clampedSides))))
            }
        }

        var indices: [UInt32] = []
        indices.reserveCapacity((n - 1) * clampedSides * 6)

        for i in 0..<(n - 1) {
            let base0 = UInt32(i * clampedSides)
            let base1 = UInt32((i + 1) * clampedSides)
            for s in 0..<clampedSides {
                let s0 = UInt32(s)
                let s1 = UInt32((s + 1) % clampedSides)

                indices.append(base0 + s0)
                indices.append(base1 + s0)
                indices.append(base1 + s1)

                indices.append(base0 + s0)
                indices.append(base1 + s1)
                indices.append(base0 + s1)
            }
        }

        return RopeMesh(vertices: vertices, indices: indices)
    }
}

