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

    func processCanonicalMove(ropeIndex: Int, endIndex: Int, from: SIMD2<Float>, to: SIMD2<Float>) {
        let dir = to - from
        if simd_length_squared(dir) < 1e-8 { return }

        Self.logger.info("🔄 CANONICAL MOVE: rope=\(ropeIndex) end=\(endIndex) from=(\(from.x), \(from.y)) to=(\(to.x), \(to.y))")
        
        let movingEndIsTop = isEndTop(ropeIndex: ropeIndex, endIndex: endIndex)
        Self.logger.info("  📍 Moving end is \(movingEndIsTop ? "TOP" : "BOTTOM")")

        var hits: [(t: Float, point: SIMD2<Float>, otherRopeIndex: Int, otherSegmentIndex: Int)] = []
        
        for otherRopeIndex in ropes.indices where otherRopeIndex != ropeIndex {
            if !ropes[otherRopeIndex].active { continue }
            let otherNodes = ropes[otherRopeIndex].nodes
            if otherNodes.count < 2 { continue }

            for segmentIndex in 0..<(otherNodes.count - 1) {
                let segmentStart = position(of: otherNodes[segmentIndex])
                let segmentEnd = position(of: otherNodes[segmentIndex + 1])
                if let intersection = SegmentIntersection.intersect(a0: from, a1: to, b0: segmentStart, b1: segmentEnd) {
                    hits.append((intersection.t, intersection.p, otherRopeIndex, segmentIndex))
                }
            }
        }

        hits.sort { $0.t < $1.t }

        for hit in hits {
            if movingEndIsTop {
                let hookPairs = findHookPairs()
                if let hook = hookPairs.first(where: { 
                    ($0.ropeA == ropeIndex || $0.ropeB == ropeIndex) &&
                    ($0.ropeA == hit.otherRopeIndex || $0.ropeB == hit.otherRopeIndex)
                }) {
                    let crossingToRemove: Int
                    if let cA = crossings[hook.crossingIdA], cA.ropeOver == ropeIndex {
                        crossingToRemove = hook.crossingIdA
                    } else {
                        crossingToRemove = hook.crossingIdB
                    }
                    Self.logger.info("  ❌ UNDO hook crossing[\(crossingToRemove)] - top end crossed back")
                    removeCrossing(crossingId: crossingToRemove)
                    continue
                }
            }
            
            let existingInSegment = findCrossingInSegment(
                ropeIndex: hit.otherRopeIndex,
                segmentIndex: hit.otherSegmentIndex,
                withRope: ropeIndex
            )
            
            if let existing = existingInSegment,
               let existingCrossing = crossings[existing] {
                
                if existingCrossing.ropeOver == ropeIndex {
                    Self.logger.info("  ❌ UNDO crossing[\(existing)] at (\(hit.point.x), \(hit.point.y)) - moving rope was over, undoing")
                    removeCrossing(crossingId: existing)
                    continue
                } else {
                    let distanceToExisting = simd_length(hit.point - existingCrossing.position)
                    let threshold: Float = 0.15
                    
                    if distanceToExisting < threshold {
                        Self.logger.info("  ⏭️ SKIP crossing[\(existing)] at (\(hit.point.x), \(hit.point.y)) - moving rope was under, too close")
                        continue
                    } else {
                        Self.logger.info("  🔄 CREATE NEW crossing near existing[\(existing)] at (\(hit.point.x), \(hit.point.y)) - distance=\(distanceToExisting), loop")
                    }
                }
            }

            let low = min(ropeIndex, hit.otherRopeIndex)
            let high = max(ropeIndex, hit.otherRopeIndex)

            let crossingId = nextCrossingId
            nextCrossingId += 1
            
            var existingCount = 0
            for (_, existing) in crossings {
                if existing.ropeA == low && existing.ropeB == high {
                    existingCount += 1
                }
            }
            
            let ropeOver: Int
            if existingCount % 2 == 0 {
                ropeOver = hit.otherRopeIndex
            } else {
                ropeOver = ropeIndex
            }
            
            let aDir = normalize2(to - from)
            let otherNodes = ropes[hit.otherRopeIndex].nodes
            let b0 = position(of: otherNodes[hit.otherSegmentIndex])
            let b1 = position(of: otherNodes[hit.otherSegmentIndex + 1])
            let bDir = normalize2(b1 - b0)
            let handedness = cross2(aDir, bDir) >= 0 ? 1 : -1

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

            Self.logger.info("  ✅ CREATE crossing[\(crossingId)] at (\(hit.point.x), \(hit.point.y)) ropeA=\(low) ropeB=\(high) over=\(ropeIndex)")
            insertCrossing(ropeIndex: hit.otherRopeIndex, segmentIndex: hit.otherSegmentIndex, crossingId: crossingId)
            insertCrossingNearEnd(ropeIndex: ropeIndex, endIndex: endIndex, crossingId: crossingId)
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

            let aPrev = anchorPosition(in: ropes[aIndex].nodes, from: aNodeIndex, direction: -1)
            let aNext = anchorPosition(in: ropes[aIndex].nodes, from: aNodeIndex, direction: 1)
            let bPrev = anchorPosition(in: ropes[bIndex].nodes, from: bNodeIndex, direction: -1)
            let bNext = anchorPosition(in: ropes[bIndex].nodes, from: bNodeIndex, direction: 1)

            if let hit = SegmentIntersection.intersect(a0: aPrev, a1: aNext, b0: bPrev, b1: bNext) {
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
                    segmentAStart: aPrev,
                    segmentAEnd: aNext,
                    segmentBStart: bPrev,
                    segmentBEnd: bNext
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

        needsRelaxation = false
    }
    
    private func anchorPosition(in nodes: [TopologyNode], from index: Int, direction: Int) -> SIMD2<Float> {
        var i = index + direction
        while i >= 0 && i < nodes.count {
            let node = nodes[i]
            switch node {
            case .hole, .floating:
                return position(of: node)
            case .crossing:
                i += direction
            }
        }
        return .zero
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

    private func findCrossingInSegment(ropeIndex: Int, segmentIndex: Int, withRope otherRopeIndex: Int) -> Int? {
        guard ropes.indices.contains(ropeIndex) else { return nil }
        guard ropes[ropeIndex].active else { return nil }
        let nodes = ropes[ropeIndex].nodes
        guard segmentIndex >= 0 && segmentIndex < nodes.count - 1 else { return nil }
        
        for nodeIndex in segmentIndex...(segmentIndex + 1) {
            if case .crossing(let crossingId) = nodes[nodeIndex],
               let crossing = crossings[crossingId],
               (crossing.ropeA == otherRopeIndex || crossing.ropeB == otherRopeIndex) {
                return crossingId
            }
        }
        return nil
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

    func findHookPairs() -> [HookPair] {
        var result: [HookPair] = []
        var processedPairs = Set<String>()
        
        for ropeIdx in ropes.indices {
            let rope = ropes[ropeIdx]
            if !rope.active { continue }
            
            var crossingsOnRope: [(nodeIndex: Int, crossingId: Int, crossing: TopologyCrossing)] = []
            for (nodeIdx, node) in rope.nodes.enumerated() {
                if case .crossing(let cid) = node, let c = crossings[cid] {
                    crossingsOnRope.append((nodeIdx, cid, c))
                }
            }
            
            for i in 0..<crossingsOnRope.count {
                for j in (i+1)..<crossingsOnRope.count {
                    let cA = crossingsOnRope[i]
                    let cB = crossingsOnRope[j]
                    
                    guard cA.crossing.ropeA == cB.crossing.ropeA && cA.crossing.ropeB == cB.crossing.ropeB else { continue }
                    
                    let aOver = cA.crossing.ropeOver == ropeIdx
                    let bOver = cB.crossing.ropeOver == ropeIdx
                    guard aOver != bOver else { continue }
                    
                    let pairKey = "\(min(cA.crossingId, cB.crossingId))-\(max(cA.crossingId, cB.crossingId))"
                    guard !processedPairs.contains(pairKey) else { continue }
                    processedPairs.insert(pairKey)
                    
                    let ropeA = cA.crossing.ropeA
                    let ropeB = cA.crossing.ropeB
                    
                    let ropeAUpperEnd = findUpperEnd(ropeIndex: ropeA, crossingA: cA, crossingB: cB)
                    let ropeBUpperEnd = findUpperEnd(ropeIndex: ropeB, crossingA: cA, crossingB: cB)
                    
                    result.append(HookPair(
                        crossingIdA: cA.crossingId,
                        crossingIdB: cB.crossingId,
                        ropeA: ropeA,
                        ropeB: ropeB,
                        ropeAUpperEnd: ropeAUpperEnd,
                        ropeBUpperEnd: ropeBUpperEnd
                    ))
                }
            }
        }
        return result
    }
    
    private func findUpperEnd(ropeIndex: Int, crossingA: (nodeIndex: Int, crossingId: Int, crossing: TopologyCrossing), crossingB: (nodeIndex: Int, crossingId: Int, crossing: TopologyCrossing)) -> Int {
        guard ropes.indices.contains(ropeIndex) else { return 1 }
        let nodes = ropes[ropeIndex].nodes
        
        var firstCrossingId: Int?
        var lastCrossingId: Int?
        
        for node in nodes {
            if case .crossing(let cid) = node {
                if cid == crossingA.crossingId || cid == crossingB.crossingId {
                    if firstCrossingId == nil {
                        firstCrossingId = cid
                    }
                    lastCrossingId = cid
                }
            }
        }
        
        guard let firstId = firstCrossingId, let lastId = lastCrossingId else { return 1 }
        
        let firstCrossing = (firstId == crossingA.crossingId) ? crossingA.crossing : crossingB.crossing
        let lastCrossing = (lastId == crossingA.crossingId) ? crossingA.crossing : crossingB.crossing
        
        let startIsOver = firstCrossing.ropeOver == ropeIndex
        let endIsOver = lastCrossing.ropeOver == ropeIndex
        
        if startIsOver { return 0 }
        if endIsOver { return 1 }
        return 1
    }
    
    func isEndTop(ropeIndex: Int, endIndex: Int) -> Bool {
        let hookPairs = findHookPairs()
        
        for hook in hookPairs {
            if hook.ropeA == ropeIndex {
                return hook.ropeAUpperEnd == endIndex
            }
            if hook.ropeB == ropeIndex {
                return hook.ropeBUpperEnd == endIndex
            }
        }
        
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

