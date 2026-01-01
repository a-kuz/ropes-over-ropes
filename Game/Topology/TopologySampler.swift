import simd

enum TopologySampler {
    static func sampleRope(engine: TopologyEngine, ropeIndex: Int, count: Int, lift: Float, dragLift: Float) -> [SIMD3<Float>] {
        let rope = engine.ropes[ropeIndex]
        if !rope.active { return Array(repeating: SIMD3<Float>(0, 0, 0), count: count) }

        let nodes = rope.nodes
        if nodes.count < 2 { return Array(repeating: SIMD3<Float>(0, 0, 0), count: count) }

        var poly: [SIMD3<Float>] = []
        poly.reserveCapacity(nodes.count * 3)

        var nodeIndex = 0
        while nodeIndex < nodes.count {
            let node = nodes[nodeIndex]

            if case .crossing(let crossingIdA) = node,
               nodeIndex + 1 < nodes.count,
               case .crossing(let crossingIdB) = nodes[nodeIndex + 1],
               let crossingA = engine.crossings[crossingIdA],
               let crossingB = engine.crossings[crossingIdB],
               isLoopPair(engine: engine, crossingA: crossingA, crossingB: crossingB, ropeIndex: ropeIndex) {

                let pointA = crossingA.position
                let pointB = crossingB.position
                let span = pointB - pointA
                let spanLen2 = simd_length_squared(span)
                if spanLen2 > 1e-10 {
                    let direction = span / sqrt(spanLen2)
                    let normal = SIMD2<Float>(-direction.y, direction.x)
                    let sign = loopSideSign(crossing: crossingA, ropeIndex: ropeIndex)
                    let bulge = loopBulge(spanLength: sqrt(spanLen2))

                    let startZ = baseZ(engine: engine, ropeIndex: ropeIndex, node: .crossing(crossingIdA), lift: lift, dragLift: dragLift)
                    let endZ = baseZ(engine: engine, ropeIndex: ropeIndex, node: .crossing(crossingIdB), lift: lift, dragLift: dragLift)

                    poly.append(SIMD3<Float>(pointA.x, pointA.y, startZ))

                    let arcPointCount = 7
                    for arcIndex in 1..<(arcPointCount - 1) {
                        let tValue = Float(arcIndex) / Float(arcPointCount - 1)
                        let along = pointA + span * tValue
                        let sValue = sin(tValue * Float.pi)
                        let offset = normal * (bulge * sign * sValue)
                        let pValue = along + offset
                        let zValue = startZ + (endZ - startZ) * tValue
                        poly.append(SIMD3<Float>(pValue.x, pValue.y, zValue))
                    }

                    poly.append(SIMD3<Float>(pointB.x, pointB.y, endZ))

                    nodeIndex += 2
                    continue
                }
            }

            if case .crossing(let crossingId) = node,
               let shaped = shapeCrossing(engine: engine, ropeIndex: ropeIndex, nodeIndex: nodeIndex, crossingId: crossingId, lift: lift, dragLift: dragLift) {
                poly.append(contentsOf: shaped)
                nodeIndex += 1
                continue
            }

            let positionXY = engine.position(of: node)
            let positionZ = baseZ(engine: engine, ropeIndex: ropeIndex, node: node, lift: lift, dragLift: dragLift)
            poly.append(SIMD3<Float>(positionXY.x, positionXY.y, positionZ))
            nodeIndex += 1
        }

        let lengths = cumulativeLengths(poly: poly)
        let total = lengths.last ?? 0
        if total <= 1e-6 { return Array(repeating: poly.first ?? .zero, count: count) }

        var out: [SIMD3<Float>] = []
        out.reserveCapacity(count)

        for sampleIndex in 0..<count {
            let tValue = Float(sampleIndex) / Float(max(1, count - 1))
            let distance = total * tValue
            out.append(sample(poly: poly, cum: lengths, dist: distance))
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

        let window: Float = 0.11
        let entryXY = currXY - dir * window
        let exitXY = currXY + dir * window

        let isOver = crossing.ropeOver == ropeIndex

        if isOver {
            let z1 = lift * 0.30
            let z2 = lift
            let z3 = lift * 0.30
            return [
                SIMD3<Float>(entryXY.x, entryXY.y, z1),
                SIMD3<Float>(currXY.x, currXY.y, z2),
                SIMD3<Float>(exitXY.x, exitXY.y, z3)
            ]
        }
        let z = max(0, lift * 0.06)
        return [
            SIMD3<Float>(entryXY.x, entryXY.y, z),
            SIMD3<Float>(currXY.x, currXY.y, 0),
            SIMD3<Float>(exitXY.x, exitXY.y, z)
        ]
    }

    private static func isLoopPair(engine: TopologyEngine, crossingA: TopologyCrossing, crossingB: TopologyCrossing, ropeIndex: Int) -> Bool {
        if crossingA.id == crossingB.id { return false }
        if crossingA.ropeA != ropeIndex && crossingA.ropeB != ropeIndex { return false }
        if crossingB.ropeA != ropeIndex && crossingB.ropeB != ropeIndex { return false }
        if crossingA.ropeA != crossingB.ropeA || crossingA.ropeB != crossingB.ropeB { return false }

        var count = 0
        for (_, crossing) in engine.crossings {
            if crossing.ropeA == crossingA.ropeA && crossing.ropeB == crossingA.ropeB {
                count += 1
                if count >= 2 { return true }
            }
        }
        return false
    }

    private static func loopSideSign(crossing: TopologyCrossing, ropeIndex: Int) -> Float {
        let base = Float(crossing.handedness)
        return (ropeIndex == crossing.ropeA) ? base : -base
    }

    private static func loopBulge(spanLength: Float) -> Float {
        let scaled = spanLength * 0.35
        return max(0.08, min(0.22, scaled))
    }

    private static func normalize2(_ vector: SIMD2<Float>) -> SIMD2<Float> {
        let lengthSquared = simd_length_squared(vector)
        if lengthSquared < 1e-12 { return .zero }
        return vector / sqrt(lengthSquared)
    }

    private static func cumulativeLengths(poly: [SIMD3<Float>]) -> [Float] {
        var cum: [Float] = [0]
        cum.reserveCapacity(poly.count)
        for pointIndex in 1..<poly.count {
            let pointA = poly[pointIndex - 1]
            let pointB = poly[pointIndex]
            cum.append(cum[pointIndex - 1] + simd_length(pointB - pointA))
        }
        return cum
    }

    private static func sample(poly: [SIMD3<Float>], cum: [Float], dist: Float) -> SIMD3<Float> {
        var lowerIndex = 0
        var upperIndex = cum.count - 1
        while lowerIndex + 1 < upperIndex {
            let midIndex = (lowerIndex + upperIndex) / 2
            if cum[midIndex] <= dist {
                lowerIndex = midIndex
            } else {
                upperIndex = midIndex
            }
        }

        let pointA = poly[lowerIndex]
        let pointB = poly[min(lowerIndex + 1, poly.count - 1)]
        let startDist = cum[lowerIndex]
        let endDist = cum[min(lowerIndex + 1, cum.count - 1)]
        let segmentSpan = max(1e-6, endDist - startDist)
        let tValue = (dist - startDist) / segmentSpan
        return pointA + (pointB - pointA) * tValue
    }
}
