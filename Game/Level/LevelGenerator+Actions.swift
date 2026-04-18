import Foundation
import simd

extension LevelGenerator {

    // MARK: - Tangle generation

    static func buildActions(
        ropes: [LevelDefinition.Rope],
        holes: [LevelDefinition.Vec2],
        totalDrags: Int,
        shortCount: Int,
        swapPercent: Int = 0
    ) -> [LevelDefinition.Action] {
        var actions: [LevelDefinition.Action] = []
        let holeSimd = holes.map { $0.simd }

        var shortCenter: SIMD2<Float> = .zero
        if shortCount > 0 {
            var sum = SIMD2<Float>.zero
            var cnt: Float = 0
            for i in 0..<shortCount {
                sum += holeSimd[ropes[i].startHole]
                sum += holeSimd[ropes[i].endHole]
                cnt += 2
            }
            shortCenter = sum / cnt
        }
        let shortRadius: Float = {
            guard shortCount > 0 else { return 0 }
            var maxD: Float = 0
            for i in 0..<shortCount {
                maxD = max(maxD, simd_length(holeSimd[ropes[i].startHole] - shortCenter))
                maxD = max(maxD, simd_length(holeSimd[ropes[i].endHole] - shortCenter))
            }
            return maxD + 0.35
        }()

        for i in 0..<ropes.count {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: ropes[i].startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: ropes[i].endHole))
        }

        var endpoints = ropes.map { ($0.startHole, $0.endHole) }
        var usedHoles = Set(endpoints.flatMap { [$0.0, $0.1] })

        @discardableResult
        func tryDrag(ropeIdx: Int, endIdx: Int, targetRopeIdx: Int, unconstrained: Bool = false) -> Bool {
            let currentHole = endIdx == 0 ? endpoints[ropeIdx].0 : endpoints[ropeIdx].1
            let anchorHole = endIdx == 0 ? endpoints[ropeIdx].1 : endpoints[ropeIdx].0
            let anchorPos = holeSimd[anchorHole]
            let tS = holeSimd[endpoints[targetRopeIdx].0]
            let tE = holeSimd[endpoints[targetRopeIdx].1]
            let isShort = !unconstrained && ropeIdx < shortCount

            let otherUsed = Set(
                (0..<ropes.count).filter { $0 != ropeIdx }
                    .flatMap { [endpoints[$0].0, endpoints[$0].1] }
            )

            // Compute centroid of all current endpoints to spread ropes out
            var cx: Float = 0, cy: Float = 0
            for ep in endpoints { cx += holeSimd[ep.0].x + holeSimd[ep.1].x; cy += holeSimd[ep.0].y + holeSimd[ep.1].y }
            let epCount = Float(endpoints.count * 2)
            let centroid = SIMD2<Float>(cx / epCount, cy / epCount)

            var bestHole: Int?
            var bestScore: Float = -.greatestFiniteMagnitude

            for candidate in 0..<holes.count {
                if candidate == currentHole || candidate == anchorHole { continue }
                if otherUsed.contains(candidate) { continue }
                let cPos = holeSimd[candidate]
                if isShort && simd_length(cPos - shortCenter) > shortRadius { continue }
                if segmentsCross(anchorPos, cPos, tS, tE) {
                    // Prefer holes far from the centroid (spreads ropes across the board)
                    let distFromCentroid = simd_length(cPos - centroid)
                    // But also ensure crossing is meaningful (not too far from target rope)
                    let distFromTarget = simd_length(cPos - (tS + tE) * 0.5)
                    let score = distFromCentroid - distFromTarget * 0.3
                    if score > bestScore {
                        bestScore = score
                        bestHole = candidate
                    }
                }
            }

            guard let targetHole = bestHole else { return false }

            // Check that this drag does not reduce total crossing count
            let crossingsBefore = totalCrossingPairs(endpoints: endpoints, holes: holeSimd)
            var testEndpoints = endpoints
            if endIdx == 0 { testEndpoints[ropeIdx].0 = targetHole } else { testEndpoints[ropeIdx].1 = targetHole }
            let crossingsAfter = totalCrossingPairs(endpoints: testEndpoints, holes: holeSimd)
            if crossingsAfter < crossingsBefore { return false }

            usedHoles.remove(currentHole)
            usedHoles.insert(targetHole)
            if endIdx == 0 {
                endpoints[ropeIdx].0 = targetHole
            } else {
                endpoints[ropeIdx].1 = targetHole
            }
            actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: endIdx, holeIndex: targetHole))
            return true
        }

        // Swap: exchange two rope ends (A goes to B's hole, B goes to A's hole)
        func trySwap(ropeA: Int, endA: Int, ropeB: Int, endB: Int) -> Bool {
            guard ropeA != ropeB else { return false }
            let holeA = endA == 0 ? endpoints[ropeA].0 : endpoints[ropeA].1
            let holeB = endB == 0 ? endpoints[ropeB].0 : endpoints[ropeB].1
            guard holeA != holeB else { return false }
            // Check that swap does not reduce crossings
            let crossingsBefore = totalCrossingPairs(endpoints: endpoints, holes: holeSimd)
            var testEndpoints = endpoints
            if endA == 0 { testEndpoints[ropeA].0 = holeB } else { testEndpoints[ropeA].1 = holeB }
            if endB == 0 { testEndpoints[ropeB].0 = holeA } else { testEndpoints[ropeB].1 = holeA }
            let crossingsAfter = totalCrossingPairs(endpoints: testEndpoints, holes: holeSimd)
            if crossingsAfter < crossingsBefore { return false }
            // Swap endpoints
            if endA == 0 { endpoints[ropeA].0 = holeB } else { endpoints[ropeA].1 = holeB }
            if endB == 0 { endpoints[ropeB].0 = holeA } else { endpoints[ropeB].1 = holeA }
            actions.append(.init(type: "swap", ropeIndex: ropeA, endIndex: endA, holeIndex: holeB,
                                 ropeIndex2: ropeB, endIndex2: endB))
            return true
        }

        if shortCount >= 2 {
            for d in 0..<shortCount {
                let ropeIdx = d
                for targetIdx in 0..<shortCount where targetIdx != ropeIdx {
                    let endIdx = d % 2
                    if tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: targetIdx) { break }
                    if tryDrag(ropeIdx: ropeIdx, endIdx: 1 - endIdx, targetRopeIdx: targetIdx) { break }
                }
            }
        }

        for d in 0..<totalDrags {
            let ropeIdx = d % ropes.count
            let targetRopeIdx = (ropeIdx + 1 + d / ropes.count) % ropes.count
            if targetRopeIdx == ropeIdx { continue }
            let endIdx = d / ropes.count % 2

            // Decide swap vs drag based on swapPercent (deterministic per d)
            let useSwap = swapPercent > 0 && ((d * 97 + 13) % 100) < swapPercent
            if useSwap {
                let targetEnd = (d / ropes.count + 1) % 2
                if trySwap(ropeA: ropeIdx, endA: endIdx, ropeB: targetRopeIdx, endB: targetEnd) {
                    continue
                }
                // Fallback to drag if swap failed
            }
            if !tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: targetRopeIdx) {
                tryDrag(ropeIdx: ropeIdx, endIdx: 1 - endIdx, targetRopeIdx: targetRopeIdx)
            }
        }

        for attempt in 0..<ropes.count * 8 {
            let crossings = ropeCrossings(endpoints: endpoints, holes: holeSimd)
            let isolated = (0..<ropes.count).filter { crossings[$0] == 0 }
            if isolated.isEmpty { break }

            let ropeIdx = isolated[attempt % isolated.count]
            var fixed = false
            for other in 0..<ropes.count where other != ropeIdx && !fixed {
                for endIdx in 0...1 where !fixed {
                    fixed = tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: other, unconstrained: true)
                }
            }
            if !fixed {
                for other in 0..<ropes.count where other != ropeIdx && !fixed {
                    for endIdx in 0...1 where !fixed {
                        fixed = tryDrag(ropeIdx: other, endIdx: endIdx, targetRopeIdx: ropeIdx, unconstrained: true)
                    }
                }
            }
            if !fixed {
                let allUsed = Set(endpoints.flatMap { [$0.0, $0.1] })
                let freeHoles = (0..<holes.count).filter { !allUsed.contains($0) }
                for other in 0..<ropes.count where other != ropeIdx && !fixed {
                    let oS = holeSimd[endpoints[other].0]
                    let oE = holeSimd[endpoints[other].1]
                    for h0 in freeHoles where !fixed {
                        for h1 in freeHoles where h1 != h0 && !fixed {
                            if segmentsCross(holeSimd[h0], holeSimd[h1], oS, oE) {
                                let old0 = endpoints[ropeIdx].0
                                let old1 = endpoints[ropeIdx].1
                                usedHoles.remove(old0)
                                usedHoles.remove(old1)
                                endpoints[ropeIdx].0 = h0
                                endpoints[ropeIdx].1 = h1
                                usedHoles.insert(h0)
                                usedHoles.insert(h1)
                                actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: 0, holeIndex: h0))
                                actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: 1, holeIndex: h1))
                                fixed = true
                            }
                        }
                    }
                }
            }
        }

        return actions
    }

    // MARK: - Geometry helpers

    static func totalCrossingPairs(endpoints: [(Int, Int)], holes: [SIMD2<Float>]) -> Int {
        let n = endpoints.count
        var total = 0
        for i in 0..<n {
            let a0 = holes[endpoints[i].0]
            let a1 = holes[endpoints[i].1]
            for j in (i+1)..<n {
                let b0 = holes[endpoints[j].0]
                let b1 = holes[endpoints[j].1]
                if segmentsCross(a0, a1, b0, b1) { total += 1 }
            }
        }
        return total
    }

    static func ropeCrossings(endpoints: [(Int, Int)], holes: [SIMD2<Float>]) -> [Int] {
        let n = endpoints.count
        var counts = Array(repeating: 0, count: n)
        for i in 0..<n {
            let a0 = holes[endpoints[i].0]
            let a1 = holes[endpoints[i].1]
            for j in (i+1)..<n {
                let b0 = holes[endpoints[j].0]
                let b1 = holes[endpoints[j].1]
                if segmentsCross(a0, a1, b0, b1) {
                    counts[i] += 1
                    counts[j] += 1
                }
            }
        }
        return counts
    }

    static func segmentsCross(
        _ a0: SIMD2<Float>, _ a1: SIMD2<Float>,
        _ b0: SIMD2<Float>, _ b1: SIMD2<Float>
    ) -> Bool {
        let d1 = a1 - a0
        let d2 = b1 - b0
        let cross = d1.x * d2.y - d1.y * d2.x
        if abs(cross) < 1e-9 { return false }
        let d = b0 - a0
        let t = (d.x * d2.y - d.y * d2.x) / cross
        let u = (d.x * d1.y - d.y * d1.x) / cross
        return t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99
    }
}
