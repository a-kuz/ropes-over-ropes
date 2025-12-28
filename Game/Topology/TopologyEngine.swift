import simd

final class TopologyEngine {
    private(set) var ropes: [TopologyRope]
    private(set) var crossings: [Int: TopologyCrossing] = [:]
    private var nextCrossingId: Int = 1
    private(set) var needsRelaxation: Bool = true

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
            return holePositions[safe: index] ?? .zero
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
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        guard ropes[ropeIndex].nodes.count >= 2 else { return }
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
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        guard ropes[ropeIndex].nodes.count >= 2 else { return }
        floatingPositions[ropeIndex] = nil
        if endIndex == 0 {
            ropes[ropeIndex].nodes[0] = .hole(holeIndex)
        } else {
            let last = ropes[ropeIndex].nodes.count - 1
            ropes[ropeIndex].nodes[last] = .hole(holeIndex)
        }
        normalizeCrossings()
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

            if let existing = findCloseCrossing(ropeA: ropeIndex, ropeB: hit.otherRopeIndex, near: hit.point),
               canUndoCrossing(ropeIndex: ropeIndex, otherRopeIndex: hit.otherRopeIndex, crossingId: existing) {
                removeCrossing(crossingId: existing)
                let n = simd_normalize(to - currentFrom)
                currentFrom = hit.point + n * 1e-3
                continue
            }

            let crossingId = nextCrossingId
            nextCrossingId += 1
            let aDir = normalize2(to - currentFrom)
            let otherNodes = ropes[hit.otherRopeIndex].nodes
            let b0 = position(of: otherNodes[hit.otherSegmentIndex])
            let b1 = position(of: otherNodes[hit.otherSegmentIndex + 1])
            let bDir = normalize2(b1 - b0)
            let handedness = cross2(aDir, bDir) >= 0 ? 1 : -1

            let crossing = TopologyCrossing(
                id: crossingId,
                ropeA: ropeIndex,
                ropeB: hit.otherRopeIndex,
                position: hit.point,
                ropeOver: ropeIndex,
                handedness: handedness
            )
            crossings[crossingId] = crossing
            needsRelaxation = true

            insertCrossing(ropeIndex: hit.otherRopeIndex, segmentIndex: hit.otherSegmentIndex, crossingId: crossingId)
            insertCrossingBeforeFloating(ropeIndex: ropeIndex, crossingId: crossingId)

            let n = simd_normalize(to - currentFrom)
            currentFrom = hit.point + n * 1e-3
        }

        if needsRelaxation {
            normalizeCrossings()
        }
    }

    private func normalizeCrossings() {
        if crossings.isEmpty {
            needsRelaxation = false
            return
        }

        var toRemove: [Int] = []
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
                crossings[crossingId] = TopologyCrossing(
                    id: crossing.id,
                    ropeA: crossing.ropeA,
                    ropeB: crossing.ropeB,
                    position: hit.p,
                    ropeOver: crossing.ropeOver,
                    handedness: crossing.handedness
                )
            } else {
                toRemove.append(crossingId)
            }
        }

        for id in toRemove {
            removeCrossing(crossingId: id)
        }

        needsRelaxation = false
    }

    func relaxCrossingPositions(iterations: Int = 12, alpha: Float = 0.75) {
        normalizeCrossings()
        _ = iterations
        _ = alpha
        return
    }

    private func removeCrossing(crossingId: Int) {
        guard let crossing = crossings[crossingId] else { return }
        removeCrossingNode(ropeIndex: crossing.ropeA, crossingId: crossingId)
        removeCrossingNode(ropeIndex: crossing.ropeB, crossingId: crossingId)
        crossings[crossingId] = nil
        needsRelaxation = true
    }

    private func removeCrossingNode(ropeIndex: Int, crossingId: Int) {
        if ropeIndex < 0 || ropeIndex >= ropes.count { return }
        if !ropes[ropeIndex].active { return }
        ropes[ropeIndex].nodes.removeAll { node in
            if case .crossing(let id) = node { return id == crossingId }
            return false
        }
        if ropes[ropeIndex].nodes.count < 2 {
            ropes[ropeIndex].nodes = []
            ropes[ropeIndex].active = false
        }
    }

    private func findCloseCrossing(ropeA: Int, ropeB: Int, near point: SIMD2<Float>) -> Int? {
        let threshold: Float = 0.06
        let thr2 = threshold * threshold
        var bestId: Int?
        var bestD2: Float = .greatestFiniteMagnitude
        for (id, crossing) in crossings {
            if (crossing.ropeA == ropeA && crossing.ropeB == ropeB) || (crossing.ropeA == ropeB && crossing.ropeB == ropeA) {
                let d2 = simd_length_squared(crossing.position - point)
                if d2 < thr2 && d2 < bestD2 {
                    bestD2 = d2
                    bestId = id
                }
            }
        }
        return bestId
    }

    private func canUndoCrossing(ropeIndex: Int, otherRopeIndex: Int, crossingId: Int) -> Bool {
        guard let crossing = crossings[crossingId] else { return false }
        if crossing.ropeOver != ropeIndex { return false }
        if ropeIndex < 0 || ropeIndex >= ropes.count { return false }
        if !ropes[ropeIndex].active { return false }
        let nodes = ropes[ropeIndex].nodes
        if nodes.count < 2 { return false }

        let lastCrossing = nodes.reversed().first { node in
            if case .crossing = node { return true }
            return false
        }
        if case .crossing(let id) = lastCrossing {
            if id != crossingId { return false }
        }
        if (crossing.ropeA == otherRopeIndex || crossing.ropeB == otherRopeIndex) == false { return false }
        return true
    }

    func isEndTop(ropeIndex: Int, endIndex: Int) -> Bool {
        if ropeIndex < 0 || ropeIndex >= ropes.count { return false }
        if !ropes[ropeIndex].active { return false }
        let nodes = ropes[ropeIndex].nodes
        if nodes.count < 2 { return false }

        if endIndex == 0 {
            for node in nodes {
                if case .crossing(let crossingId) = node, let crossing = crossings[crossingId] {
                    return crossing.ropeOver == ropeIndex
                }
            }
            return true
        }

        for node in nodes.reversed() {
            if case .crossing(let crossingId) = node, let crossing = crossings[crossingId] {
                return crossing.ropeOver == ropeIndex
            }
        }
        return true
    }

    private func insertCrossing(ropeIndex: Int, segmentIndex: Int, crossingId: Int) {
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        guard ropes[ropeIndex].nodes.count >= 2 else { return }
        let insertAt = segmentIndex + 1
        guard insertAt >= 1 && insertAt <= ropes[ropeIndex].nodes.count - 1 else { return }
        let node = TopologyNode.crossing(crossingId)
        ropes[ropeIndex].nodes.insert(node, at: insertAt)
    }

    private func insertCrossingBeforeFloating(ropeIndex: Int, crossingId: Int) {
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        guard ropes[ropeIndex].nodes.count >= 2 else { return }
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
                    let aDir = normalize2(a1 - a0)
                    let bDir = normalize2(b1 - b0)
                    let handedness = cross2(aDir, bDir) >= 0 ? 1 : -1
                    crossings[crossingId] = TopologyCrossing(
                        id: crossingId,
                        ropeA: aIndex,
                        ropeB: bIndex,
                        position: hit.p,
                        ropeOver: ropeOver,
                        handedness: handedness
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
        normalizeCrossings()
    }

    private func normalize2(_ v: SIMD2<Float>) -> SIMD2<Float> {
        let l2 = simd_length_squared(v)
        if l2 < 1e-12 { return .zero }
        return v / sqrt(l2)
    }

    private func cross2(_ a: SIMD2<Float>, _ b: SIMD2<Float>) -> Float {
        a.x * b.y - a.y * b.x
    }
}

