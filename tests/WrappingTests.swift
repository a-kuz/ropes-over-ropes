import XCTest
import simd

/// Tests for the braid tangle generator.
///
/// KEY INSIGHT: the braid algorithm creates 3D physical entanglement through drag
/// arcs that cross other ropes. The final hole-to-hole STRAIGHT LINES may not cross
/// (the ropes fan out after braiding), but the PHYSICAL PARTICLES will be interleaved.
/// So we test drag-time crossings, not final-endpoint crossings.
final class WrappingTests: XCTestCase {

    // MARK: - Which levels use braid

    func testWrappingLevelSelection() {
        let wrapping = (1...30).filter { $0 >= 5 && ($0 - 5) % 3 == 0 }
        XCTAssertEqual(wrapping, [5, 8, 11, 14, 17, 20, 23, 26, 29])
    }

    // MARK: - Braid involves multiple ropes (not just rope 0)

    func testBraidMultipleRopesParticipate() {
        for levelId in [5, 8, 11, 14, 17, 20] {
            let level = LevelGenerator.generate(levelId: levelId)
            guard let actions = level.actions else { XCTFail("L\(levelId): no actions"); continue }

            let drags = actions.filter { $0.type == "drag" }
            let draggedRopes = Set(drags.map { $0.ropeIndex })

            print("[BRAID L\(levelId)] ropes=\(level.ropes.count) drags=\(drags.count) ropes dragged=\(draggedRopes.sorted())")

            // Braid must involve at least 2 different ropes (not single-rope wrapping)
            XCTAssertGreaterThanOrEqual(draggedRopes.count, 2,
                "Level \(levelId): braid must drag at least 2 different ropes, got \(draggedRopes)")

            // At least half the ropes should participate — limited by available holes
            let minRopesExpected = max(2, level.ropes.count / 2)
            XCTAssertGreaterThanOrEqual(draggedRopes.count, minRopesExpected,
                "Level \(levelId): expected >= \(minRopesExpected) ropes dragged, got \(draggedRopes.count)")
        }
    }

    // MARK: - Every drag arc crosses at least one other rope (real braid step)

    func testEachDragCrossesAnotherRope() {
        for levelId in [5, 8, 11, 14] {
            let level = LevelGenerator.generate(levelId: levelId)
            guard let actions = level.actions else { continue }

            // Replay actions, tracking current endpoint positions
            var endpoints = level.ropes.map { ($0.startHole, $0.endHole) }
            let pinCount = level.ropes.count * 2
            let drags = actions.suffix(from: pinCount).filter { $0.type == "drag" }

            var validDrags = 0
            for (di, drag) in drags.enumerated() {
                let ri = drag.ropeIndex
                let fromHole = drag.endIndex == 0 ? endpoints[ri].0 : endpoints[ri].1
                let fromPos  = level.holes[fromHole].simd
                let toPos    = level.holes[drag.holeIndex].simd

                // Count how many OTHER ropes this drag arc crosses
                var crossCount = 0
                for other in 0..<level.ropes.count where other != ri {
                    let tS = level.holes[endpoints[other].0].simd
                    let tE = level.holes[endpoints[other].1].simd
                    if segmentsCross(fromPos, toPos, tS, tE) { crossCount += 1 }
                }

                print("[BRAID L\(levelId)] drag\(di) rope\(ri): hole\(fromHole)→hole\(drag.holeIndex) crosses \(crossCount) others")
                if crossCount >= 1 { validDrags += 1 }

                // Update endpoint
                if drag.endIndex == 0 { endpoints[ri].0 = drag.holeIndex }
                else                  { endpoints[ri].1 = drag.holeIndex }
            }

            // Most drags should cross at least one other rope
            let minValid = max(1, drags.count * 2 / 3)
            XCTAssertGreaterThanOrEqual(validDrags, minValid,
                "Level \(levelId): expected >= \(minValid)/\(drags.count) crossing drags, got \(validDrags)")
        }
    }

    // MARK: - Stress: all braid levels produce valid braid structure

    func testAllBraidLevelsHaveValidStructure() {
        let braidLevels = (5...100).filter { ($0 - 5) % 3 == 0 }
        var failures: [String] = []

        for levelId in braidLevels {
            let level = LevelGenerator.generate(levelId: levelId)
            guard let actions = level.actions else { failures.append("L\(levelId):no-actions"); continue }

            let drags = actions.filter { $0.type == "drag" }
            guard drags.count >= 2 else { failures.append("L\(levelId):drags=\(drags.count)"); continue }

            let draggedRopes = Set(drags.map { $0.ropeIndex })
            if draggedRopes.count < 2 { failures.append("L\(levelId):ropes=\(draggedRopes.count)") }
        }

        if !failures.isEmpty { print("[STRESS] failures: \(failures)") }
        XCTAssertTrue(failures.isEmpty,
            "Braid levels with invalid structure: \(failures.prefix(10))")
    }

    // MARK: - Physics: braid levels are NOT immediately solved after init

    /// The real correctness test: run the full physics simulation and verify
    /// that at least one rope has 2D crossings with another (win-check would fail).
    /// This is the same check Renderer+WinCheck.swift uses: segmentsCross2D on particle positions.
    func testBraidLevelsArePhysicallyTangled() {
        for levelId in [5, 8, 14] {
            let level = LevelGenerator.generate(levelId: levelId)

            let holePositions  = level.holes.map { SIMD2<Float>($0.xPosition, $0.yPosition) }
            let holeElevations = level.holes.map { $0.zPosition }

            let sim = VerletSimulator(
                holePositions: holePositions,
                holeElevations: holeElevations,
                holeRadius: level.holeRadius,
                boards: level.boards ?? []
            )
            sim.gravity = -14.79
            sim.damping = 0.809
            sim.constraintIterations = 7
            sim.particleCount = level.particlesPerRope
            sim.settleSteps = 19    // must match game default, NOT 300!
            sim.liftHeight = 0.3
            sim.ropeTension = 0.5

            let ropeConfigs = level.ropes.map {
                VerletSimulator.RopeConfig(startHole: $0.startHole, endHole: $0.endHole,
                                           radius: $0.radius, crossSection: $0.crossSection)
            }
            let simActions: [VerletSimulator.LevelAction] = (level.actions ?? []).compactMap {
                guard let t = VerletSimulator.LevelAction.ActionType(rawValue: $0.type) else { return nil }
                return .init(type: t, ropeIndex: $0.ropeIndex, endIndex: $0.endIndex, holeIndex: $0.holeIndex)
            }
            sim.initializeLevel(ropeConfigs: ropeConfigs, actions: simActions)

            let skip = 3
            let activeBands = sim.bands.indices.filter { sim.bands[$0].active }
            var tangledRopes = 0

            for i in activeBands {
                let bandA = sim.bands[i]
                let nA = bandA.positions.count
                var hasCrossing = false
                for j in activeBands where j != i && !hasCrossing {
                    let bandB = sim.bands[j]
                    let nB = bandB.positions.count
                    for si in skip..<max(skip, nA - 1 - skip) where !hasCrossing {
                        let a0 = SIMD2<Float>(bandA.positions[si].x, bandA.positions[si].y)
                        let a1 = SIMD2<Float>(bandA.positions[si+1].x, bandA.positions[si+1].y)
                        for sj in skip..<max(skip, nB - 1 - skip) where !hasCrossing {
                            let b0 = SIMD2<Float>(bandB.positions[sj].x, bandB.positions[sj].y)
                            let b1 = SIMD2<Float>(bandB.positions[sj+1].x, bandB.positions[sj+1].y)
                            if segmentsCross(a0, a1, b0, b1) { hasCrossing = true }
                        }
                    }
                }
                if hasCrossing { tangledRopes += 1 }
            }

            print("[PHYSICS L\(levelId)] active bands=\(activeBands.count) tangled ropes=\(tangledRopes)")

            XCTAssertGreaterThanOrEqual(tangledRopes, 2,
                "Level \(levelId): after braid simulation, at least 2 ropes should be physically tangled (have 2D crossings). Got \(tangledRopes). Braid may not be creating real entanglement.")
        }
    }

    // MARK: - Multiple crossings per pair (braid = same 2 ropes cross many times)

    func testBraidPairsHaveMultipleCrossings() {
        for levelId in [5, 8, 14] {
            let level = LevelGenerator.generate(levelId: levelId)

            let holePositions  = level.holes.map { SIMD2<Float>($0.xPosition, $0.yPosition) }
            let holeElevations = level.holes.map { $0.zPosition }

            let sim = VerletSimulator(
                holePositions: holePositions,
                holeElevations: holeElevations,
                holeRadius: level.holeRadius,
                boards: level.boards ?? []
            )
            sim.gravity = -5.0
            sim.damping = 0.97
            sim.constraintIterations = 8
            sim.particleCount = level.particlesPerRope
            sim.settleSteps = 300
            sim.liftHeight = 0.3
            sim.ropeTension = 0.98

            let ropeConfigs = level.ropes.map {
                VerletSimulator.RopeConfig(startHole: $0.startHole, endHole: $0.endHole,
                                           radius: $0.radius, crossSection: $0.crossSection)
            }
            let simActions: [VerletSimulator.LevelAction] = (level.actions ?? []).compactMap {
                guard let t = VerletSimulator.LevelAction.ActionType(rawValue: $0.type) else { return nil }
                return .init(type: t, ropeIndex: $0.ropeIndex, endIndex: $0.endIndex, holeIndex: $0.holeIndex)
            }
            sim.initializeLevel(ropeConfigs: ropeConfigs, actions: simActions)

            // Count crossings PER PAIR + total crossing pairs
            let skip = 3
            let activeBands = sim.bands.indices.filter { sim.bands[$0].active }
            var totalCrossingPairs = 0
            var maxPairCrossings = 0

            for i in 0..<activeBands.count {
                let bi = activeBands[i]
                let bandA = sim.bands[bi]
                let nA = bandA.positions.count
                for j in (i+1)..<activeBands.count {
                    let bj = activeBands[j]
                    let bandB = sim.bands[bj]
                    let nB = bandB.positions.count
                    var pairCrossings = 0
                    for si in skip..<max(skip, nA - 1 - skip) {
                        let a0 = SIMD2<Float>(bandA.positions[si].x, bandA.positions[si].y)
                        let a1 = SIMD2<Float>(bandA.positions[si+1].x, bandA.positions[si+1].y)
                        for sj in skip..<max(skip, nB - 1 - skip) {
                            let b0 = SIMD2<Float>(bandB.positions[sj].x, bandB.positions[sj].y)
                            let b1 = SIMD2<Float>(bandB.positions[sj+1].x, bandB.positions[sj+1].y)
                            if segmentsCross(a0, a1, b0, b1) { pairCrossings += 1 }
                        }
                    }
                    if pairCrossings > 0 { totalCrossingPairs += 1 }
                    if pairCrossings > maxPairCrossings { maxPairCrossings = pairCrossings }
                }
            }

            print("[PAIR-CROSSINGS L\(levelId)] maxPair=\(maxPairCrossings) totalPairs=\(totalCrossingPairs) ropes=\(activeBands.count)")

            XCTAssertGreaterThanOrEqual(maxPairCrossings, 2,
                "Level \(levelId): braid must have ≥2 crossings between a rope pair (got \(maxPairCrossings)). Not a real braid.")
        }
    }

    // MARK: - Diagnostic dump

    func testDumpWrappingLevel5() {
        let level = LevelGenerator.generate(levelId: 5)
        print("=== DUMP Level 5 (braid) ===")
        print("holes=\(level.holes.count) ropes=\(level.ropes.count)")

        for (i, rope) in level.ropes.enumerated() {
            let s = level.holes[rope.startHole].simd
            let e = level.holes[rope.endHole].simd
            print("  rope\(i): \(fmt(s)) → \(fmt(e))")
        }

        guard let actions = level.actions else { XCTFail("No actions"); return }

        var endpoints = level.ropes.map { ($0.startHole, $0.endHole) }
        let pinCount = level.ropes.count * 2
        let drags = actions.suffix(from: pinCount).filter { $0.type == "drag" }

        print("drags (\(drags.count)):")
        for (di, drag) in drags.enumerated() {
            let ri = drag.ropeIndex
            let fromHole = drag.endIndex == 0 ? endpoints[ri].0 : endpoints[ri].1
            let fromPos  = level.holes[fromHole].simd
            let toPos    = level.holes[drag.holeIndex].simd
            var crossed: [Int] = []
            for other in 0..<level.ropes.count where other != ri {
                let tS = level.holes[endpoints[other].0].simd
                let tE = level.holes[endpoints[other].1].simd
                if segmentsCross(fromPos, toPos, tS, tE) { crossed.append(other) }
            }
            print("  [\(di)] rope\(ri) \(fmt(fromPos))→\(fmt(toPos))  crosses:\(crossed)")
            if drag.endIndex == 0 { endpoints[ri].0 = drag.holeIndex }
            else                  { endpoints[ri].1 = drag.holeIndex }
        }

        // Final endpoint crossings (informational only — braid may have 0)
        let final2D = count2DCrossings(endpoints: endpoints, holes: level.holes)
        print("final endpoint 2D crossings: \(final2D.total) (may be 0 — braid topology is 3D)")
    }

    // MARK: - Storyboard reproduction: exact 2-rope X braid

    func testStoryboardBraid() {
        // Use level 1's hole layout (same as storyboard)
        let level1 = LevelGenerator.generate(levelId: 1)
        let holes = level1.holes
        print("[STORYBOARD] holes=\(holes.count)")
        for (i, h) in holes.enumerated() {
            print("  hole\(i) = (\(String(format:"%.2f",h.xPosition)), \(String(format:"%.2f",h.yPosition)))")
        }

        // Find holes nearest to corners for X formation
        let holeSimd = holes.map { SIMD2<Float>($0.xPosition, $0.yPosition) }
        func nearest(_ target: SIMD2<Float>) -> Int {
            holeSimd.indices.min(by: { simd_length(holeSimd[$0] - target) < simd_length(holeSimd[$1] - target) })!
        }
        let ul = nearest(SIMD2<Float>(-0.8, 0.6))  // upper-left
        let lr = nearest(SIMD2<Float>(0.8, -0.6))   // lower-right
        let ur = nearest(SIMD2<Float>(0.8, 0.6))    // upper-right
        let ll = nearest(SIMD2<Float>(-0.8, -0.6))  // lower-left
        print("  X formation: rope0 \(ul)→\(lr), rope1 \(ur)→\(ll)")

        // Find holes nearest to drag targets (center area, alternating sides)
        let h_lc = nearest(SIMD2<Float>(-0.4, -0.2))  // left-center
        let h_rc = nearest(SIMD2<Float>(0.4, -0.2))   // right-center
        let h_lu = nearest(SIMD2<Float>(-0.4, 0.2))   // left-upper
        let h_ru = nearest(SIMD2<Float>(0.4, 0.2))    // right-upper
        let h_c  = nearest(SIMD2<Float>(0.0, -0.2))   // center
        let h_cu = nearest(SIMD2<Float>(0.0, 0.2))    // center-upper
        print("  targets: lc=\(h_lc) rc=\(h_rc) lu=\(h_lu) ru=\(h_ru) c=\(h_c) cu=\(h_cu)")

        let col = LevelDefinition.Color(redChannel: 0.9, greenChannel: 0.3, blueChannel: 0.5)
        let ropes = [
            LevelDefinition.Rope(startHole: ul, endHole: lr, color: col, radius: 0.055),
            LevelDefinition.Rope(startHole: ur, endHole: ll, color: col, radius: 0.055),
        ]

        // Drags: alternating rope0/rope1, each end crosses the other rope.
        let dragSequence: [(rope: Int, hole: Int)] = [
            (0, h_lc),  // rope0 end crosses rope1
            (1, h_rc),  // rope1 end crosses rope0
            (0, h_rc),  // rope0 back, crosses rope1
            (1, h_lc),  // rope1 back, crosses rope0
            (0, h_lu),  // rope0 to upper-left, crosses rope1
            (1, h_ru),  // rope1 to upper-right, crosses rope0
            (0, h_rc),  // rope0 back to right
            (1, h_lc),  // rope1 back to left
            (0, h_lu),  // rope0 upper-left again
            (1, h_ru),  // rope1 upper-right again
            (0, h_c),   // rope0 center
            (1, h_cu),  // rope1 center-upper
        ]

        var actions = [LevelDefinition.Action]()
        actions.append(.init(type: "pin", ropeIndex: 0, endIndex: 0, holeIndex: ul))
        actions.append(.init(type: "pin", ropeIndex: 0, endIndex: 1, holeIndex: lr))
        actions.append(.init(type: "pin", ropeIndex: 1, endIndex: 0, holeIndex: ur))
        actions.append(.init(type: "pin", ropeIndex: 1, endIndex: 1, holeIndex: ll))
        for d in dragSequence {
            actions.append(.init(type: "drag", ropeIndex: d.rope, endIndex: 1, holeIndex: d.hole))
        }

        let level = LevelDefinition(
            mode: nil,
            id: 9999,
            holeRadius: 0.08,
            particlesPerRope: 70,
            holes: holes,
            ropes: ropes,
            hooks: nil,
            actions: actions,
            boards: nil,
            weights: nil,
            targets: nil,
            rails: nil,
            carts: nil,
            stations: nil
        )

        // Run physics
        let holePositions  = level.holes.map { SIMD2<Float>($0.xPosition, $0.yPosition) }
        let holeElevations = level.holes.map { $0.zPosition }

        let sim = VerletSimulator(
            holePositions: holePositions,
            holeElevations: holeElevations,
            holeRadius: level.holeRadius,
            boards: []
        )
        sim.gravity = -5.0
        sim.damping = 0.97
        sim.constraintIterations = 8
        sim.particleCount = level.particlesPerRope
        sim.settleSteps = 300
        sim.liftHeight = 0.3
        sim.ropeTension = 0.98

        let ropeConfigs = level.ropes.map {
            VerletSimulator.RopeConfig(startHole: $0.startHole, endHole: $0.endHole,
                                       radius: $0.radius, crossSection: $0.crossSection)
        }
        let simActions: [VerletSimulator.LevelAction] = (level.actions ?? []).compactMap {
            guard let t = VerletSimulator.LevelAction.ActionType(rawValue: $0.type) else { return nil }
            return .init(type: t, ropeIndex: $0.ropeIndex, endIndex: $0.endIndex, holeIndex: $0.holeIndex)
        }
        sim.initializeLevel(ropeConfigs: ropeConfigs, actions: simActions)

        // Count physical crossings between the pair
        let skip = 3
        let band0 = sim.bands[0]
        let band1 = sim.bands[1]
        let n0 = band0.positions.count
        let n1 = band1.positions.count
        var pairCrossings = 0

        for si in skip..<max(skip, n0 - 1 - skip) {
            let a0 = SIMD2<Float>(band0.positions[si].x, band0.positions[si].y)
            let a1 = SIMD2<Float>(band0.positions[si+1].x, band0.positions[si+1].y)
            for sj in skip..<max(skip, n1 - 1 - skip) {
                let b0 = SIMD2<Float>(band1.positions[sj].x, band1.positions[sj].y)
                let b1 = SIMD2<Float>(band1.positions[sj+1].x, band1.positions[sj+1].y)
                if segmentsCross(a0, a1, b0, b1) { pairCrossings += 1 }
            }
        }

        print("[STORYBOARD] pairCrossings=\(pairCrossings) particles=\(n0),\(n1)")

        // Dump particle Z at crossings for debug
        for si in stride(from: 0, to: n0, by: 4) {
            let p = band0.positions[si]
            print("  band0[\(si)] = (\(String(format:"%.2f",p.x)), \(String(format:"%.2f",p.y)), \(String(format:"%.3f",p.z)))")
        }
        for si in stride(from: 0, to: n1, by: 4) {
            let p = band1.positions[si]
            print("  band1[\(si)] = (\(String(format:"%.2f",p.x)), \(String(format:"%.2f",p.y)), \(String(format:"%.3f",p.z)))")
        }

        XCTAssertGreaterThanOrEqual(pairCrossings, 2,
            "Storyboard braid: expected ≥2 physical crossings, got \(pairCrossings)")
    }

    // MARK: - Helpers

    private func finalEndpoints(level: LevelDefinition) -> [(Int, Int)] {
        var endpoints = level.ropes.map { ($0.startHole, $0.endHole) }
        guard let actions = level.actions else { return endpoints }

        let pinCount = level.ropes.count * 2
        for action in actions.suffix(from: pinCount) {
            guard action.type == "drag" else { continue }
            if action.endIndex == 0 {
                endpoints[action.ropeIndex].0 = action.holeIndex
            } else {
                endpoints[action.ropeIndex].1 = action.holeIndex
            }
        }
        return endpoints
    }

    struct CrossingResult {
        let total: Int
        let pairs: [(Int, Int)]
    }

    private func count2DCrossings(endpoints: [(Int, Int)], holes: [LevelDefinition.Vec2]) -> CrossingResult {
        var pairs: [(Int, Int)] = []
        for i in 0..<endpoints.count {
            for j in (i+1)..<endpoints.count {
                if segmentsCross(
                    holes[endpoints[i].0].simd,
                    holes[endpoints[i].1].simd,
                    holes[endpoints[j].0].simd,
                    holes[endpoints[j].1].simd
                ) {
                    pairs.append((i, j))
                }
            }
        }
        return CrossingResult(total: pairs.count, pairs: pairs)
    }

    private func segmentsCross(
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

    private func fmt(_ v: SIMD2<Float>) -> String {
        String(format: "%.2f,%.2f", v.x, v.y)
    }
}
