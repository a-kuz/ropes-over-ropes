import simd

enum TopologySampler {
    static func sampleRope(engine: TopologyEngine, ropeIndex: Int, count: Int, lift: Float, dragLift: Float) -> [SIMD3<Float>] {
        let rope = engine.ropes[ropeIndex]
        if !rope.active { return Array(repeating: SIMD3<Float>(0, 0, 0), count: count) }

        let nodes = rope.nodes
        if nodes.count < 2 { return Array(repeating: SIMD3<Float>(0, 0, 0), count: count) }

        var poly: [SIMD3<Float>] = []
        poly.reserveCapacity(nodes.count * 3)

        for nodeIndex in nodes.indices {
            let node = nodes[nodeIndex]
            if case .crossing(let crossingId) = node {
                if let shaped = shapeCrossing(engine: engine, ropeIndex: ropeIndex, nodeIndex: nodeIndex, crossingId: crossingId, lift: lift, dragLift: dragLift) {
                    poly.append(contentsOf: shaped)
                    continue
                }
            }

            let xy = engine.position(of: node)
            let z = baseZ(engine: engine, ropeIndex: ropeIndex, node: node, lift: lift, dragLift: dragLift)
            poly.append(SIMD3<Float>(xy.x, xy.y, z))
        }

        let lengths = cumulativeLengths(poly: poly)
        let total = lengths.last ?? 0
        if total <= 1e-6 { return Array(repeating: poly.first ?? .zero, count: count) }

        var out: [SIMD3<Float>] = []
        out.reserveCapacity(count)

        for i in 0..<count {
            let t = Float(i) / Float(max(1, count - 1))
            let d = total * t
            out.append(sample(poly: poly, cum: lengths, dist: d))
        }

        return out
    }

    private static func baseZ(engine: TopologyEngine, ropeIndex: Int, node: TopologyNode, lift: Float, dragLift: Float) -> Float {
        switch node {
        case .floating:
            return dragLift
        case .crossing(let crossingId):
            if let crossing = engine.crossings[crossingId], crossing.ropeOver == ropeIndex {
                return lift
            }
            return 0
        default:
            return 0
        }
    }

    private static func shapeCrossing(engine: TopologyEngine, ropeIndex: Int, nodeIndex: Int, crossingId: Int, lift: Float, dragLift: Float) -> [SIMD3<Float>]? {
        let nodes = engine.ropes[ropeIndex].nodes
        if nodeIndex == 0 || nodeIndex >= nodes.count - 1 { return nil }
        guard let crossing = engine.crossings[crossingId] else { return nil }

        let prevXY = engine.position(of: nodes[nodeIndex - 1])
        let currXY = engine.position(of: nodes[nodeIndex])
        let nextXY = engine.position(of: nodes[nodeIndex + 1])

        let dir = normalize2(nextXY - prevXY)
        if simd_length_squared(dir) < 1e-8 { return nil }

        let window: Float = 0.12
        let entryXY = currXY - dir * window
        let exitXY = currXY + dir * window

        let isOver = crossing.ropeOver == ropeIndex

        if isOver {
            let z1 = lift * 0.45
            let z2 = lift
            let z3 = lift * 0.45
            return [
                SIMD3<Float>(entryXY.x, entryXY.y, z1),
                SIMD3<Float>(currXY.x, currXY.y, z2),
                SIMD3<Float>(exitXY.x, exitXY.y, z3)
            ]
        }

        let sideDir = normalize2(SIMD2<Float>(-dir.y, dir.x))
        let base = Float(crossing.handedness)
        let sign = (ropeIndex == crossing.ropeA) ? base : -base
        let sideScale = holeSideOffset(lift: lift) * sign
        let side = sideDir * sideScale

        let entry2 = entryXY + side
        let mid2 = currXY + side * 1.25
        let exit2 = exitXY + side

        let z0 = baseZ(engine: engine, ropeIndex: ropeIndex, node: nodes[nodeIndex], lift: lift, dragLift: dragLift)
        return [
            SIMD3<Float>(entry2.x, entry2.y, z0),
            SIMD3<Float>(mid2.x, mid2.y, z0),
            SIMD3<Float>(exit2.x, exit2.y, z0)
        ]
    }

    private static func holeSideOffset(lift: Float) -> Float {
        max(0.05, min(0.10, lift * 0.55))
    }

    private static func normalize2(_ v: SIMD2<Float>) -> SIMD2<Float> {
        let l2 = simd_length_squared(v)
        if l2 < 1e-12 { return .zero }
        return v / sqrt(l2)
    }

    private static func cumulativeLengths(poly: [SIMD3<Float>]) -> [Float] {
        var cum: [Float] = [0]
        cum.reserveCapacity(poly.count)
        for i in 1..<poly.count {
            let a = poly[i - 1]
            let b = poly[i]
            cum.append(cum[i - 1] + simd_length(b - a))
        }
        return cum
    }

    private static func sample(poly: [SIMD3<Float>], cum: [Float], dist: Float) -> SIMD3<Float> {
        var lo = 0
        var hi = cum.count - 1
        while lo + 1 < hi {
            let mid = (lo + hi) / 2
            if cum[mid] <= dist {
                lo = mid
            } else {
                hi = mid
            }
        }

        let a = poly[lo]
        let b = poly[min(lo + 1, poly.count - 1)]
        let d0 = cum[lo]
        let d1 = cum[min(lo + 1, cum.count - 1)]
        let span = max(1e-6, d1 - d0)
        let t = (dist - d0) / span
        return a + (b - a) * t
    }
}

