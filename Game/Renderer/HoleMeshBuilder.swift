import simd

struct HoleMesh {
    var vertices: [HoleVertex]
    var indices: [UInt16]
}

enum HoleMeshBuilder {
    static func build(segments: Int = 48, innerRadius: Float = 0.76, outerRadius: Float = 1.0, depth: Float = 1.25) -> HoleMesh {
        let segCount = max(12, min(128, segments))
        let inner = max(0.05, min(0.98, innerRadius))
        let outer = max(inner + 0.01, min(1.5, outerRadius))
        let holeDepth = max(0.1, depth)

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        vertices.reserveCapacity(segCount * 6)
        indices.reserveCapacity(segCount * 18)

        func ringPoint(radius: Float, angle: Float, z: Float) -> SIMD3<Float> {
            SIMD3<Float>(cos(angle) * radius, sin(angle) * radius, z)
        }

        for segIndex in 0..<segCount {
            let a0 = (Float(segIndex) / Float(segCount)) * (Float.pi * 2)
            let a1 = (Float(segIndex + 1) / Float(segCount)) * (Float.pi * 2)

            let o0 = ringPoint(radius: outer, angle: a0, z: 0)
            let o1 = ringPoint(radius: outer, angle: a1, z: 0)
            let i0 = ringPoint(radius: inner, angle: a0, z: 0)
            let i1 = ringPoint(radius: inner, angle: a1, z: 0)

            let base = UInt16(vertices.count)
            vertices.append(HoleVertex(position: o0, normal: SIMD3<Float>(0, 0, 1)))
            vertices.append(HoleVertex(position: o1, normal: SIMD3<Float>(0, 0, 1)))
            vertices.append(HoleVertex(position: i0, normal: SIMD3<Float>(0, 0, 1)))
            vertices.append(HoleVertex(position: i1, normal: SIMD3<Float>(0, 0, 1)))

            indices.append(contentsOf: [
                base + 0, base + 2, base + 1,
                base + 1, base + 2, base + 3
            ])
        }

        for segIndex in 0..<segCount {
            let a0 = (Float(segIndex) / Float(segCount)) * (Float.pi * 2)
            let a1 = (Float(segIndex + 1) / Float(segCount)) * (Float.pi * 2)

            let top0 = ringPoint(radius: inner, angle: a0, z: 0)
            let top1 = ringPoint(radius: inner, angle: a1, z: 0)
            let bot0 = ringPoint(radius: inner, angle: a0, z: -holeDepth)
            let bot1 = ringPoint(radius: inner, angle: a1, z: -holeDepth)

            let n0 = simd_normalize(SIMD3<Float>(-cos(a0), -sin(a0), 0))
            let n1 = simd_normalize(SIMD3<Float>(-cos(a1), -sin(a1), 0))

            let base = UInt16(vertices.count)
            vertices.append(HoleVertex(position: top0, normal: n0))
            vertices.append(HoleVertex(position: bot0, normal: n0))
            vertices.append(HoleVertex(position: top1, normal: n1))
            vertices.append(HoleVertex(position: bot1, normal: n1))

            indices.append(contentsOf: [
                base + 0, base + 1, base + 2,
                base + 2, base + 1, base + 3
            ])
        }

        for segIndex in 0..<segCount {
            let a0 = (Float(segIndex) / Float(segCount)) * (Float.pi * 2)
            let a1 = (Float(segIndex + 1) / Float(segCount)) * (Float.pi * 2)

            let center = SIMD3<Float>(0, 0, -holeDepth)
            let p0 = ringPoint(radius: inner, angle: a0, z: -holeDepth)
            let p1 = ringPoint(radius: inner, angle: a1, z: -holeDepth)

            let base = UInt16(vertices.count)
            vertices.append(HoleVertex(position: center, normal: SIMD3<Float>(0, 0, 1)))
            vertices.append(HoleVertex(position: p0, normal: SIMD3<Float>(0, 0, 1)))
            vertices.append(HoleVertex(position: p1, normal: SIMD3<Float>(0, 0, 1)))

            indices.append(contentsOf: [
                base + 0, base + 1, base + 2
            ])
        }

        return HoleMesh(vertices: vertices, indices: indices)
    }
}

