import simd
import os.log

struct TopologySnapshot {
    var ropes: [TopologyRope]
    var crossings: [Int: TopologyCrossing]
    var nextCrossingId: Int
    var floatingPositions: [Int: SIMD2<Float>]
}

final class TopologyEngine {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "TopologyEngine")
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

    func snapshot() -> TopologySnapshot {
        TopologySnapshot(
            ropes: ropes,
            crossings: crossings,
            nextCrossingId: nextCrossingId,
            floatingPositions: floatingPositions
        )
    }

    func restore(_ snapshot: TopologySnapshot) {
        ropes = snapshot.ropes
        crossings = snapshot.crossings
        nextCrossingId = snapshot.nextCrossingId
        floatingPositions = snapshot.floatingPositions
        needsRelaxation = true
    }

    func beginDrag(ropeIndex: Int, endIndex: Int, floatingPosition: SIMD2<Float>) {
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        guard ropes[ropeIndex].nodes.count >= 2 else { return }
        Self.logger.info("🎯 BEGIN DRAG: rope=\(ropeIndex) end=\(endIndex) pos=(\(floatingPosition.x), \(floatingPosition.y))")
        logState(label: "Before beginDrag")
        let node = TopologyNode.floating(ropeIndex)
        if endIndex == 0 {
            ropes[ropeIndex].nodes[0] = node
        } else {
            let last = ropes[ropeIndex].nodes.count - 1
            ropes[ropeIndex].nodes[last] = node
        }
        floatingPositions[ropeIndex] = floatingPosition
        logState(label: "After beginDrag")
    }

    func endDrag(ropeIndex: Int, endIndex: Int, holeIndex: Int) {
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        guard ropes[ropeIndex].nodes.count >= 2 else { return }
        Self.logger.info("🏁 END DRAG: rope=\(ropeIndex) end=\(endIndex) hole=\(holeIndex)")
        logState(label: "Before endDrag")
        floatingPositions[ropeIndex] = nil
        if endIndex == 0 {
            ropes[ropeIndex].nodes[0] = .hole(holeIndex)
        } else {
            let last = ropes[ropeIndex].nodes.count - 1
            ropes[ropeIndex].nodes[last] = .hole(holeIndex)
        }
        normalizeCrossings()
        logState(label: "After endDrag")
    }

    func applyCanonicalMove(ropeIndex: Int, endIndex: Int, from: SIMD2<Float>, to: SIMD2<Float>) {
        if ropeIndex < 0 || ropeIndex >= ropes.count { return }
        if !ropes[ropeIndex].active { return }
        if ropes[ropeIndex].nodes.count < 2 { return }

        let dir = to - from
        if simd_length_squared(dir) < 1e-8 { return }

        let hits = allIntersectionsForMove(ropeIndex: ropeIndex, from: from, to: to)
        if hits.isEmpty { return }

        let moveDir = normalize2(to - from)
        for hit in hits {
            if let existing = findCloseCrossing(ropeA: ropeIndex, ropeB: hit.otherRopeIndex, near: hit.point),
               canUndoCrossing(ropeIndex: ropeIndex, endIndex: endIndex, otherRopeIndex: hit.otherRopeIndex, crossingId: existing) {
                removeCrossing(crossingId: existing)
                continue
            }

            let crossingId = nextCrossingId
            nextCrossingId += 1

            let otherNodes = ropes[hit.otherRopeIndex].nodes
            if otherNodes.count < 2 { continue }
            let bSeg = nearestSegmentIndex(ropeIndex: hit.otherRopeIndex, near: hit.point)
            let b0 = position(of: otherNodes[bSeg])
            let b1 = position(of: otherNodes[min(bSeg + 1, otherNodes.count - 1)])
            let bDir = normalize2(b1 - b0)
            let handedness = cross2(moveDir, bDir) >= 0 ? 1 : -1

            let low = min(ropeIndex, hit.otherRopeIndex)
            let high = max(ropeIndex, hit.otherRopeIndex)
            let crossing = TopologyCrossing(
                id: crossingId,
                ropeA: low,
                ropeB: high,
                position: hit.point,
                ropeOver: ropeIndex,
                handedness: handedness
            )
            crossings[crossingId] = crossing
            needsRelaxation = true

            insertCrossing(ropeIndex: hit.otherRopeIndex, segmentIndex: bSeg, crossingId: crossingId)
            insertCrossingNearEnd(ropeIndex: ropeIndex, endIndex: endIndex, crossingId: crossingId)
        }

        if needsRelaxation {
            normalizeCrossings()
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

    func processDragSegment(ropeIndex: Int, from: SIMD2<Float>, to toPoint: SIMD2<Float>) {
        var currentFrom = from
        let dir = toPoint - from
        if simd_length_squared(dir) < 1e-8 { return }

        Self.logger.info("🔄 PROCESS DRAG SEGMENT: rope=\(ropeIndex) from=(\(from.x), \(from.y)) to=(\(toPoint.x), \(toPoint.y))")
        let endIndex = draggingEndIndex(ropeIndex: ropeIndex)
        for _ in 0..<16 {
            guard let hit = earliestIntersection(ropeIndex: ropeIndex, from: currentFrom, to: toPoint) else { break }

            if let existing = findCloseCrossing(ropeA: ropeIndex, ropeB: hit.otherRopeIndex, near: hit.point),
               canUndoCrossing(ropeIndex: ropeIndex, endIndex: endIndex, otherRopeIndex: hit.otherRopeIndex, crossingId: existing) {
                Self.logger.info("  ❌ UNDO crossing[\(existing)] at (\(hit.point.x), \(hit.point.y))")
                removeCrossing(crossingId: existing)
                let direction = simd_normalize(toPoint - currentFrom)
                currentFrom = hit.point + direction * 1e-3
                continue
            }

            let low = min(ropeIndex, hit.otherRopeIndex)
            let high = max(ropeIndex, hit.otherRopeIndex)
            if ropeIndex == high {
                let direction = simd_normalize(toPoint - currentFrom)
                currentFrom = hit.point + direction * 1e-3
                continue
            }

            let crossingId = nextCrossingId
            nextCrossingId += 1
            let aDir = normalize2(toPoint - currentFrom)
            let otherNodes = ropes[hit.otherRopeIndex].nodes
            let b0 = position(of: otherNodes[hit.otherSegmentIndex])
            let b1 = position(of: otherNodes[hit.otherSegmentIndex + 1])
            let bDir = normalize2(b1 - b0)
            let handedness = cross2(aDir, bDir) >= 0 ? 1 : -1

            let ropeOver = nextRopeOverForPair(low: low, high: high)
            let crossing = TopologyCrossing(
                id: crossingId,
                ropeA: low,
                ropeB: high,
                position: hit.point,
                ropeOver: ropeOver,
                handedness: handedness
            )
            crossings[crossingId] = crossing
            needsRelaxation = true

            Self.logger.info("  ✅ ADD crossing[\(crossingId)] at (\(hit.point.x), \(hit.point.y)) ropeA=\(low) ropeB=\(high) over=\(ropeOver)")
            insertCrossing(ropeIndex: hit.otherRopeIndex, segmentIndex: hit.otherSegmentIndex, crossingId: crossingId)
            insertCrossingNearEnd(ropeIndex: ropeIndex, endIndex: endIndex, crossingId: crossingId)

            let direction = simd_normalize(toPoint - currentFrom)
            currentFrom = hit.point + direction * 1e-3
        }

        if needsRelaxation {
            normalizeCrossings()
        }
    }

    private func nextRopeOverForPair(low: Int, high: Int) -> Int {
        var lastId: Int = -1
        var lastOver: Int = high
        for (_, crossing) in crossings {
            if crossing.ropeA != low || crossing.ropeB != high { continue }
            if crossing.id > lastId {
                lastId = crossing.id
                lastOver = crossing.ropeOver
            }
        }
        return (lastOver == high) ? low : high
    }

    private func draggingEndIndex(ropeIndex: Int) -> Int {
        if ropeIndex < 0 || ropeIndex >= ropes.count { return 1 }
        let nodes = ropes[ropeIndex].nodes
        if nodes.isEmpty { return 1 }
        if case .floating = nodes.first { return 0 }
        if case .floating = nodes.last { return 1 }
        return 1
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
                let newPos = closestMidpointBetweenSegments(
                    segmentAStart: a0,
                    segmentAEnd: a1,
                    segmentBStart: b0,
                    segmentBEnd: b1
                )
                crossings[crossingId] = TopologyCrossing(
                    id: crossing.id,
                    ropeA: crossing.ropeA,
                    ropeB: crossing.ropeB,
                    position: newPos,
                    ropeOver: crossing.ropeOver,
                    handedness: crossing.handedness
                )
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

    private func canUndoCrossing(ropeIndex: Int, endIndex: Int, otherRopeIndex: Int, crossingId: Int) -> Bool {
        guard let crossing = crossings[crossingId] else { return false }
        if crossing.ropeOver != ropeIndex { return false }
        if ropeIndex < 0 || ropeIndex >= ropes.count { return false }
        if !ropes[ropeIndex].active { return false }
        let nodes = ropes[ropeIndex].nodes
        if nodes.count < 2 { return false }

        if endIndex == 0 {
            for node in nodes {
                if case .crossing(let id) = node {
                    return id == crossingId && (crossing.ropeA == otherRopeIndex || crossing.ropeB == otherRopeIndex)
                }
            }
            return false
        } else {
            for node in nodes.reversed() {
                if case .crossing(let id) = node {
                    return id == crossingId && (crossing.ropeA == otherRopeIndex || crossing.ropeB == otherRopeIndex)
                }
            }
            return false
        }
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

    private func insertCrossingNearEnd(ropeIndex: Int, endIndex: Int, crossingId: Int) {
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        guard ropes[ropeIndex].nodes.count >= 2 else { return }
        let node = TopologyNode.crossing(crossingId)
        if endIndex == 0 {
            ropes[ropeIndex].nodes.insert(node, at: 1)
        } else {
            let last = ropes[ropeIndex].nodes.count - 1
            ropes[ropeIndex].nodes.insert(node, at: max(1, last))
        }
    }

    private struct IntersectionHit {
        var t: Float
        var point: SIMD2<Float>
        var otherRopeIndex: Int
        var otherSegmentIndex: Int
    }

    private func allIntersectionsForMove(ropeIndex: Int, from: SIMD2<Float>, to toPoint: SIMD2<Float>) -> [IntersectionHit] {
        var hits: [IntersectionHit] = []
        hits.reserveCapacity(8)

        for otherRopeIndex in ropes.indices where otherRopeIndex != ropeIndex {
            if !ropes[otherRopeIndex].active { continue }
            let otherNodes = ropes[otherRopeIndex].nodes
            if otherNodes.count < 2 { continue }

            for segmentIndex in 0..<(otherNodes.count - 1) {
                let segmentStart = position(of: otherNodes[segmentIndex])
                let segmentEnd = position(of: otherNodes[segmentIndex + 1])
                if let hit = SegmentIntersection.intersect(a0: from, a1: toPoint, b0: segmentStart, b1: segmentEnd) {
                    hits.append(IntersectionHit(t: hit.t, point: hit.p, otherRopeIndex: otherRopeIndex, otherSegmentIndex: segmentIndex))
                }
            }
        }

        hits.sort { $0.t < $1.t }
        return hits
    }

    private func earliestIntersection(ropeIndex: Int, from: SIMD2<Float>, to toPoint: SIMD2<Float>) -> IntersectionHit? {
        var best: IntersectionHit?

        for otherRopeIndex in ropes.indices where otherRopeIndex != ropeIndex {
            if !ropes[otherRopeIndex].active { continue }
            let otherNodes = ropes[otherRopeIndex].nodes
            if otherNodes.count < 2 { continue }

            for segmentIndex in 0..<(otherNodes.count - 1) {
                let segmentStart = position(of: otherNodes[segmentIndex])
                let segmentEnd = position(of: otherNodes[segmentIndex + 1])
                if let hit = SegmentIntersection.intersect(a0: from, a1: toPoint, b0: segmentStart, b1: segmentEnd) {
                    if best == nil || hit.t < best!.t {
                        best = IntersectionHit(t: hit.t, point: hit.p, otherRopeIndex: otherRopeIndex, otherSegmentIndex: segmentIndex)
                    }
                }
            }
        }

        return best
    }

    private func nearestSegmentIndex(ropeIndex: Int, near point: SIMD2<Float>) -> Int {
        if ropeIndex < 0 || ropeIndex >= ropes.count { return 0 }
        let nodes = ropes[ropeIndex].nodes
        if nodes.count < 2 { return 0 }

        var bestIndex = 0
        var bestD2: Float = .greatestFiniteMagnitude
        for segmentIndex in 0..<(nodes.count - 1) {
            let a = position(of: nodes[segmentIndex])
            let b = position(of: nodes[segmentIndex + 1])
            let projected = closestPointOnSegment(point: point, segmentStart: a, segmentEnd: b)
            let d2 = simd_length_squared(projected - point)
            if d2 < bestD2 {
                bestD2 = d2
                bestIndex = segmentIndex
            }
        }
        return bestIndex
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

    private func closestPointOnSegment(point: SIMD2<Float>, segmentStart: SIMD2<Float>, segmentEnd: SIMD2<Float>) -> SIMD2<Float> {
        let segment = segmentEnd - segmentStart
        let len2 = simd_length_squared(segment)
        if len2 < 1e-12 { return segmentStart }
        let t = max(0, min(1, simd_dot(point - segmentStart, segment) / len2))
        return segmentStart + segment * t
    }

    private func closestMidpointBetweenSegments(
        segmentAStart: SIMD2<Float>,
        segmentAEnd: SIMD2<Float>,
        segmentBStart: SIMD2<Float>,
        segmentBEnd: SIMD2<Float>
    ) -> SIMD2<Float> {
        let candidate1 = closestPair(point: segmentAStart, segmentStart: segmentBStart, segmentEnd: segmentBEnd)
        let candidate2 = closestPair(point: segmentAEnd, segmentStart: segmentBStart, segmentEnd: segmentBEnd)
        let candidate3 = closestPair(point: segmentBStart, segmentStart: segmentAStart, segmentEnd: segmentAEnd, swap: true)
        let candidate4 = closestPair(point: segmentBEnd, segmentStart: segmentAStart, segmentEnd: segmentAEnd, swap: true)

        let best = [candidate1, candidate2, candidate3, candidate4].min { $0.dist2 < $1.dist2 }!
        return (best.pointOnA + best.pointOnB) * 0.5
    }

    private func closestPair(
        point: SIMD2<Float>,
        segmentStart: SIMD2<Float>,
        segmentEnd: SIMD2<Float>,
        swap: Bool = false
    ) -> (pointOnA: SIMD2<Float>, pointOnB: SIMD2<Float>, dist2: Float) {
        let projected = closestPointOnSegment(point: point, segmentStart: segmentStart, segmentEnd: segmentEnd)
        let d2 = simd_length_squared(projected - point)
        if swap {
            return (projected, point, d2)
        }
        return (point, projected, d2)
    }

    private func closestPointOnSegment(point: SIMD2<Float>, segmentStart: SIMD2<Float>, segmentEnd: SIMD2<Float>) -> SIMD2<Float> {
        let segment = segmentEnd - segmentStart
        let len2 = simd_length_squared(segment)
        if len2 < 1e-12 { return segmentStart }
        let t = max(0, min(1, simd_dot(point - segmentStart, segment) / len2))
        return segmentStart + segment * t
    }

    private func logState(label: String) {
        Self.logger.info("📊 \(label):")
        Self.logger.info("  Crossings: \(self.crossings.count)")
        for (id, crossing) in crossings {
            Self.logger.info("    [\(id)]: ropeA=\(crossing.ropeA) ropeB=\(crossing.ropeB) over=\(crossing.ropeOver) pos=(\(crossing.position.x), \(crossing.position.y))")
        }
        for ropeIndex in ropes.indices where ropes[ropeIndex].active {
            let nodes = ropes[ropeIndex].nodes.map { node -> String in
                switch node {
                case .hole(let h): return "H\(h)"
                case .crossing(let c): return "X\(c)"
                case .floating: return "F"
                }
            }.joined(separator: " → ")
            Self.logger.info("  Rope[\(ropeIndex)]: \(nodes)")
        }
    }
}

