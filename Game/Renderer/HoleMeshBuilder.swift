import simd

struct HoleMesh {
    var vertices: [HoleVertex]
    var indices: [UInt16]
}

enum HoleMeshBuilder {
    static func build(segments: Int = 48, innerRadius: Float = 0.76, outerRadius: Float = 1.0, depth: Float = 1.25, ringHeight: Float = 0.1) -> HoleMesh {
        let segCount = max(12, min(128, segments))
        let inner = max(0.05, min(0.98, innerRadius))
        let outer = max(inner + 0.01, min(1.5, outerRadius))
        let holeDepth = max(0.1, depth)
        let rh = max(0.02, min(0.2, ringHeight))

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        func ringPoint(radius: Float, angle: Float, z: Float) -> SIMD3<Float> {
            SIMD3<Float>(cos(angle) * radius, sin(angle) * radius, z)
        }

        for segIndex in 0..<segCount {
            let a0 = (Float(segIndex) / Float(segCount)) * (Float.pi * 2)
            let a1 = (Float(segIndex + 1) / Float(segCount)) * (Float.pi * 2)

            let o0 = ringPoint(radius: outer, angle: a0, z: rh)
            let o1 = ringPoint(radius: outer, angle: a1, z: rh)
            let i0 = ringPoint(radius: inner, angle: a0, z: rh)
            let i1 = ringPoint(radius: inner, angle: a1, z: rh)

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

            let top0 = ringPoint(radius: outer, angle: a0, z: rh)
            let top1 = ringPoint(radius: outer, angle: a1, z: rh)
            let bot0 = ringPoint(radius: outer, angle: a0, z: 0)
            let bot1 = ringPoint(radius: outer, angle: a1, z: 0)

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

            let top0 = ringPoint(radius: inner, angle: a0, z: rh)
            let top1 = ringPoint(radius: inner, angle: a1, z: rh)
            let bot0 = ringPoint(radius: inner, angle: a0, z: 0)
            let bot1 = ringPoint(radius: inner, angle: a1, z: 0)

            let n0 = simd_normalize(SIMD3<Float>(cos(a0), sin(a0), 0))
            let n1 = simd_normalize(SIMD3<Float>(cos(a1), sin(a1), 0))

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

            let o0 = ringPoint(radius: outer, angle: a0, z: 0)
            let o1 = ringPoint(radius: outer, angle: a1, z: 0)
            let i0 = ringPoint(radius: inner, angle: a0, z: 0)
            let i1 = ringPoint(radius: inner, angle: a1, z: 0)

            let baseRing = UInt16(vertices.count)
            vertices.append(HoleVertex(position: o0, normal: SIMD3<Float>(0, 0, -1)))
            vertices.append(HoleVertex(position: o1, normal: SIMD3<Float>(0, 0, -1)))
            vertices.append(HoleVertex(position: i0, normal: SIMD3<Float>(0, 0, -1)))
            vertices.append(HoleVertex(position: i1, normal: SIMD3<Float>(0, 0, -1)))

            indices.append(contentsOf: [
                baseRing + 0, baseRing + 1, baseRing + 2,
                baseRing + 1, baseRing + 3, baseRing + 2
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

    static func buildSquare(innerHalf: Float = 0.76, outerHalf: Float = 1.0, depth: Float = 1.25, ringHeight: Float = 0.1) -> HoleMesh {
        let ih = max(0.05, innerHalf)
        let oh = max(ih + 0.01, outerHalf)
        let d = max(0.1, depth)
        let rh = max(0.02, min(0.2, ringHeight))

        var vertices: [HoleVertex] = []
        var indices: [UInt16] = []

        func quad(_ p0: SIMD3<Float>, _ p1: SIMD3<Float>, _ p2: SIMD3<Float>, _ p3: SIMD3<Float>, normal n: SIMD3<Float>) {
            let base = UInt16(vertices.count)
            vertices.append(HoleVertex(position: p0, normal: n))
            vertices.append(HoleVertex(position: p1, normal: n))
            vertices.append(HoleVertex(position: p2, normal: n))
            vertices.append(HoleVertex(position: p3, normal: n))
            indices.append(contentsOf: [base, base+1, base+2, base, base+2, base+3])
        }

        let outerCorners: [SIMD2<Float>] = [
            SIMD2<Float>( oh,  oh),
            SIMD2<Float>(-oh,  oh),
            SIMD2<Float>(-oh, -oh),
            SIMD2<Float>( oh, -oh),
        ]
        let innerCorners: [SIMD2<Float>] = [
            SIMD2<Float>( ih,  ih),
            SIMD2<Float>(-ih,  ih),
            SIMD2<Float>(-ih, -ih),
            SIMD2<Float>( ih, -ih),
        ]

        let upN = SIMD3<Float>(0, 0, 1)
        let downN = SIMD3<Float>(0, 0, -1)

        for i in 0..<4 {
            let j = (i + 1) % 4
            let o0 = SIMD3<Float>(outerCorners[i].x, outerCorners[i].y, rh)
            let o1 = SIMD3<Float>(outerCorners[j].x, outerCorners[j].y, rh)
            let i0 = SIMD3<Float>(innerCorners[i].x, innerCorners[i].y, rh)
            let i1 = SIMD3<Float>(innerCorners[j].x, innerCorners[j].y, rh)
            quad(o0, o1, i1, i0, normal: upN)
        }

        let outerWallNormals: [SIMD3<Float>] = [
            SIMD3<Float>( 0, -1, 0),
            SIMD3<Float>( 1,  0, 0),
            SIMD3<Float>( 0,  1, 0),
            SIMD3<Float>(-1,  0, 0),
        ]
        let outerWallEdges: [(Int, Int)] = [(3, 0), (0, 1), (1, 2), (2, 3)]
        for (edgeIdx, (a, b)) in outerWallEdges.enumerated() {
            let n = outerWallNormals[edgeIdx]
            let t0 = SIMD3<Float>(outerCorners[a].x, outerCorners[a].y, rh)
            let t1 = SIMD3<Float>(outerCorners[b].x, outerCorners[b].y, rh)
            let b0 = SIMD3<Float>(outerCorners[a].x, outerCorners[a].y, 0)
            let b1 = SIMD3<Float>(outerCorners[b].x, outerCorners[b].y, 0)
            quad(t0, t1, b1, b0, normal: n)
        }

        let innerWallNormals: [SIMD3<Float>] = [
            SIMD3<Float>( 0,  1, 0),
            SIMD3<Float>(-1,  0, 0),
            SIMD3<Float>( 0, -1, 0),
            SIMD3<Float>( 1,  0, 0),
        ]
        let innerWallEdges: [(Int, Int)] = [(3, 0), (0, 1), (1, 2), (2, 3)]
        for (edgeIdx, (a, b)) in innerWallEdges.enumerated() {
            let n = innerWallNormals[edgeIdx]
            let t0 = SIMD3<Float>(innerCorners[a].x, innerCorners[a].y, rh)
            let t1 = SIMD3<Float>(innerCorners[b].x, innerCorners[b].y, rh)
            let b0 = SIMD3<Float>(innerCorners[a].x, innerCorners[a].y, 0)
            let b1 = SIMD3<Float>(innerCorners[b].x, innerCorners[b].y, 0)
            quad(t1, t0, b0, b1, normal: n)
        }

        for i in 0..<4 {
            let j = (i + 1) % 4
            let o0 = SIMD3<Float>(outerCorners[i].x, outerCorners[i].y, 0)
            let o1 = SIMD3<Float>(outerCorners[j].x, outerCorners[j].y, 0)
            let i0 = SIMD3<Float>(innerCorners[i].x, innerCorners[i].y, 0)
            let i1 = SIMD3<Float>(innerCorners[j].x, innerCorners[j].y, 0)
            quad(o1, o0, i0, i1, normal: downN)
        }

        for (edgeIdx, (a, b)) in innerWallEdges.enumerated() {
            let n = innerWallNormals[edgeIdx]
            let t0 = SIMD3<Float>(innerCorners[a].x, innerCorners[a].y, 0)
            let t1 = SIMD3<Float>(innerCorners[b].x, innerCorners[b].y, 0)
            let b0 = SIMD3<Float>(innerCorners[a].x, innerCorners[a].y, -d)
            let b1 = SIMD3<Float>(innerCorners[b].x, innerCorners[b].y, -d)
            quad(t1, t0, b0, b1, normal: n)
        }

        let center = SIMD3<Float>(0, 0, -d)
        for i in 0..<4 {
            let j = (i + 1) % 4
            let p0 = SIMD3<Float>(innerCorners[i].x, innerCorners[i].y, -d)
            let p1 = SIMD3<Float>(innerCorners[j].x, innerCorners[j].y, -d)
            let base = UInt16(vertices.count)
            vertices.append(HoleVertex(position: center, normal: upN))
            vertices.append(HoleVertex(position: p0, normal: upN))
            vertices.append(HoleVertex(position: p1, normal: upN))
            indices.append(contentsOf: [base, base+1, base+2])
        }

        return HoleMesh(vertices: vertices, indices: indices)
    }
}

