import simd

final class TopologyEngine {
    private(set) var ropes: [TopologyRope]
    private(set) var crossings: [Int: TopologyCrossing] = [:]
    private var nextCrossingId: Int = 1

    var holePositions: [SIMD2<Float>]
    var floatingPositions: [Int: SIMD2<Float>] = [:]

    init(holePositions: [SIMD2<Float>], ropes: [TopologyRope]) {
        self.holePositions = holePositions
        self.ropes = ropes
        buildInitialCrossings()
    }

    func position(of node: TopologyNode) -> SIMD2<Float> {
        switch node {
        case .hole(let index):
            return holePositions[index]
        case .crossing(let crossingId):
            return crossings[crossingId]?.position ?? .zero
        case .floating(let ropeIndex):
            return floatingPositions[ropeIndex] ?? .zero
        }
    }

    func setFloating(ropeIndex: Int, position: SIMD2<Float>) {
        floatingPositions[ropeIndex] = position
    }

    func beginDrag(ropeIndex: Int, endIndex: Int, floatingPosition: SIMD2<Float>) {
        let node = TopologyNode.floating(ropeIndex)
        if endIndex == 0 {
            ropes[ropeIndex].nodes[0] = node
        } else {
            let last = ropes[ropeIndex].nodes.count - 1
            ropes[ropeIndex].nodes[last] = node
        }
        floatingPositions[ropeIndex] = floatingPosition
    }

    func endDrag(ropeIndex: Int, endIndex: Int, holeIndex: Int) {
        floatingPositions[ropeIndex] = nil
        if endIndex == 0 {
            ropes[ropeIndex].nodes[0] = .hole(holeIndex)
        } else {
            let last = ropes[ropeIndex].nodes.count - 1
            ropes[ropeIndex].nodes[last] = .hole(holeIndex)
        }
    }

    func deactivateRope(ropeIndex: Int) {
        if ropeIndex < 0 || ropeIndex >= ropes.count { return }
        ropes[ropeIndex].active = false
        ropes[ropeIndex].nodes = []

        var toRemove: [Int] = []
        for (id, crossing) in crossings {
            if crossing.ropeA == ropeIndex || crossing.ropeB == ropeIndex {
                toRemove.append(id)
            }
        }
        for id in toRemove {
            crossings[id] = nil
        }
    }

    func processDragSegment(ropeIndex: Int, from: SIMD2<Float>, to: SIMD2<Float>) {
        var currentFrom = from
        let dir = to - from
        if simd_length_squared(dir) < 1e-8 { return }

        for _ in 0..<16 {
            guard let hit = earliestIntersection(ropeIndex: ropeIndex, from: currentFrom, to: to) else { break }

            let crossingId = nextCrossingId
            nextCrossingId += 1

            let crossing = TopologyCrossing(
                id: crossingId,
                ropeA: ropeIndex,
                ropeB: hit.otherRopeIndex,
                position: hit.point,
                ropeOver: ropeIndex
            )
            crossings[crossingId] = crossing

            insertCrossing(ropeIndex: hit.otherRopeIndex, segmentIndex: hit.otherSegmentIndex, crossingId: crossingId)
            insertCrossingBeforeFloating(ropeIndex: ropeIndex, crossingId: crossingId)

            let n = simd_normalize(to - currentFrom)
            currentFrom = hit.point + n * 1e-3
        }
    }

    func relaxCrossingPositions(iterations: Int = 6, alpha: Float = 0.45) {
        if crossings.isEmpty { return }
        if ropes.count < 2 { return }
        let clampedAlpha = max(0, min(1, alpha))

        for _ in 0..<max(1, iterations) {
            for (crossingId, crossing) in crossings {
                let aIndex = crossing.ropeA
                let bIndex = crossing.ropeB
                if aIndex < 0 || aIndex >= ropes.count { continue }
                if bIndex < 0 || bIndex >= ropes.count { continue }
                if !ropes[aIndex].active || !ropes[bIndex].active { continue }

                guard let aNodeIndex = ropes[aIndex].nodes.firstIndex(of: .crossing(crossingId)) else { continue }
                guard let bNodeIndex = ropes[bIndex].nodes.firstIndex(of: .crossing(crossingId)) else { continue }
                if aNodeIndex == 0 || aNodeIndex >= ropes[aIndex].nodes.count - 1 { continue }
                if bNodeIndex == 0 || bNodeIndex >= ropes[bIndex].nodes.count - 1 { continue }

                let a0 = position(of: ropes[aIndex].nodes[aNodeIndex - 1])
                let a1 = position(of: ropes[aIndex].nodes[aNodeIndex + 1])
                let b0 = position(of: ropes[bIndex].nodes[bNodeIndex - 1])
                let b1 = position(of: ropes[bIndex].nodes[bNodeIndex + 1])

                if let hit = SegmentIntersection.intersect(a0: a0, a1: a1, b0: b0, b1: b1) {
                    let newPos = hit.p
                    let blended = crossing.position + (newPos - crossing.position) * clampedAlpha
                    crossings[crossingId] = TopologyCrossing(
                        id: crossing.id,
                        ropeA: crossing.ropeA,
                        ropeB: crossing.ropeB,
                        position: blended,
                        ropeOver: crossing.ropeOver
                    )
                }
            }
        }
    }

    private func insertCrossing(ropeIndex: Int, segmentIndex: Int, crossingId: Int) {
        let insertAt = segmentIndex + 1
        let node = TopologyNode.crossing(crossingId)
        ropes[ropeIndex].nodes.insert(node, at: insertAt)
    }

    private func insertCrossingBeforeFloating(ropeIndex: Int, crossingId: Int) {
        let node = TopologyNode.crossing(crossingId)
        let last = ropes[ropeIndex].nodes.count - 1
        ropes[ropeIndex].nodes.insert(node, at: max(1, last))
    }

    private func earliestIntersection(ropeIndex: Int, from: SIMD2<Float>, to: SIMD2<Float>) -> (t: Float, point: SIMD2<Float>, otherRopeIndex: Int, otherSegmentIndex: Int)? {
        var best: (t: Float, point: SIMD2<Float>, otherRopeIndex: Int, otherSegmentIndex: Int)?

        for otherRopeIndex in ropes.indices where otherRopeIndex != ropeIndex {
            if !ropes[otherRopeIndex].active { continue }
            let otherNodes = ropes[otherRopeIndex].nodes
            if otherNodes.count < 2 { continue }

            for segmentIndex in 0..<(otherNodes.count - 1) {
                let b0 = position(of: otherNodes[segmentIndex])
                let b1 = position(of: otherNodes[segmentIndex + 1])
                if let hit = SegmentIntersection.intersect(a0: from, a1: to, b0: b0, b1: b1) {
                    if best == nil || hit.t < best!.t {
                        best = (hit.t, hit.p, otherRopeIndex, segmentIndex)
                    }
                }
            }
        }

        return best
    }

    private func buildInitialCrossings() {
        var perRope: [[(param: Float, crossingId: Int)]] = Array(repeating: [], count: ropes.count)

        for aIndex in ropes.indices {
            if !ropes[aIndex].active { continue }
            if ropes[aIndex].nodes.count != 2 { continue }
            let a0 = position(of: ropes[aIndex].nodes[0])
            let a1 = position(of: ropes[aIndex].nodes[1])

            for bIndex in (aIndex + 1)..<ropes.count {
                if !ropes[bIndex].active { continue }
                if ropes[bIndex].nodes.count != 2 { continue }
                let b0 = position(of: ropes[bIndex].nodes[0])
                let b1 = position(of: ropes[bIndex].nodes[1])

                if let hit = SegmentIntersection.intersect(a0: a0, a1: a1, b0: b0, b1: b1) {
                    let crossingId = nextCrossingId
                    nextCrossingId += 1

                    let ropeOver = max(aIndex, bIndex)
                    crossings[crossingId] = TopologyCrossing(
                        id: crossingId,
                        ropeA: aIndex,
                        ropeB: bIndex,
                        position: hit.p,
                        ropeOver: ropeOver
                    )
                    perRope[aIndex].append((hit.t, crossingId))
                    perRope[bIndex].append((hit.u, crossingId))
                }
            }
        }

        for ropeIndex in ropes.indices {
            let inserts = perRope[ropeIndex].sorted { $0.param < $1.param }
            if inserts.isEmpty { continue }
            var nodes: [TopologyNode] = []
            nodes.reserveCapacity(inserts.count + 2)
            nodes.append(ropes[ropeIndex].nodes[0])
            for insert in inserts {
                nodes.append(.crossing(insert.crossingId))
            }
            nodes.append(ropes[ropeIndex].nodes[1])
            ropes[ropeIndex].nodes = nodes
        }
    }
}

