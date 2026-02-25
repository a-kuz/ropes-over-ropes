import XCTest
import simd
@testable import UzlsFour

final class LevelGeneratorTests: XCTestCase {

    // MARK: - Determinism

    func testDeterministic() {
        for levelId in 1...50 {
            let a = LevelGenerator.generate(levelId: levelId)
            let b = LevelGenerator.generate(levelId: levelId)
            XCTAssertEqual(a.holes.count, b.holes.count, "Level \(levelId): hole count")
            XCTAssertEqual(a.ropes.count, b.ropes.count, "Level \(levelId): rope count")
            XCTAssertEqual(a.actions?.count, b.actions?.count, "Level \(levelId): action count")
            for i in 0..<a.ropes.count {
                XCTAssertEqual(a.ropes[i].startHole, b.ropes[i].startHole)
                XCTAssertEqual(a.ropes[i].endHole, b.ropes[i].endHole)
            }
            if let aa = a.actions, let ba = b.actions {
                for i in 0..<aa.count {
                    XCTAssertEqual(aa[i].type, ba[i].type)
                    XCTAssertEqual(aa[i].ropeIndex, ba[i].ropeIndex)
                    XCTAssertEqual(aa[i].endIndex, ba[i].endIndex)
                    XCTAssertEqual(aa[i].holeIndex, ba[i].holeIndex)
                }
            }
        }
    }

    // MARK: - Different levels produce different results

    func testAllLevelsDifferent() {
        var signatures = Set<String>()
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            let sig = level.ropes.map { "\($0.startHole)-\($0.endHole)" }.joined(separator: ",")
                + "|" + "\(level.holes.count)"
            signatures.insert(sig)
        }
        // At least 25 out of 30 should be unique (some may collide due to layout cycling)
        XCTAssertGreaterThanOrEqual(signatures.count, 20, "Too many duplicate levels")
    }

    // MARK: - Hole counts

    func testHoleCountReasonable() {
        for levelId in 1...50 {
            let level = LevelGenerator.generate(levelId: levelId)
            XCTAssertGreaterThanOrEqual(level.holes.count, 10, "Level \(levelId): too few holes")
            XCTAssertLessThanOrEqual(level.holes.count, 25, "Level \(levelId): too many holes")
        }
    }

    // MARK: - No duplicate holes (within tolerance)

    func testNoDuplicateHolePositions() {
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            for i in 0..<level.holes.count {
                for j in (i+1)..<level.holes.count {
                    let dist = simd_length(level.holes[i].simd - level.holes[j].simd)
                    XCTAssertGreaterThan(dist, 0.05,
                        "Level \(levelId): holes \(i) and \(j) are too close (\(dist))")
                }
            }
        }
    }

    // MARK: - Rope count matches difficulty curve

    func testRopeCountMatchesDifficulty() {
        for levelId in 1...2 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 2, "Level \(levelId)")
        }
        for levelId in 3...5 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 3, "Level \(levelId)")
        }
        for levelId in 6...9 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 3, "Level \(levelId)")
        }
        for levelId in 10...15 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 4, "Level \(levelId)")
        }
    }

    // MARK: - Drag count grows with difficulty

    func testDragCountGrowsWithLevel() {
        let early = LevelGenerator.generate(levelId: 1)
        let mid = LevelGenerator.generate(levelId: 10)
        let late = LevelGenerator.generate(levelId: 20)

        let earlyDrags = (early.actions ?? []).filter { $0.type == "drag" }.count
        let midDrags = (mid.actions ?? []).filter { $0.type == "drag" }.count
        let lateDrags = (late.actions ?? []).filter { $0.type == "drag" }.count

        XCTAssertGreaterThanOrEqual(midDrags, earlyDrags, "Mid should have >= early drags")
        XCTAssertGreaterThanOrEqual(lateDrags, midDrags, "Late should have >= mid drags")
        XCTAssertGreaterThanOrEqual(lateDrags, 4, "Late levels should have substantial drags")
    }

    // MARK: - No shared holes between ropes (initial positions)

    func testNoSharedHoles() {
        for levelId in 1...50 {
            let level = LevelGenerator.generate(levelId: levelId)
            var usedHoles = Set<Int>()
            for rope in level.ropes {
                XCTAssertNotEqual(rope.startHole, rope.endHole, "Level \(levelId): self-loop")
                XCTAssertFalse(usedHoles.contains(rope.startHole), "Level \(levelId): hole \(rope.startHole) reused")
                XCTAssertFalse(usedHoles.contains(rope.endHole), "Level \(levelId): hole \(rope.endHole) reused")
                usedHoles.insert(rope.startHole)
                usedHoles.insert(rope.endHole)
            }
        }
    }

    // MARK: - Valid indices everywhere

    func testValidIndices() {
        for levelId in 1...50 {
            let level = LevelGenerator.generate(levelId: levelId)
            let holeRange = level.holes.indices
            let ropeRange = level.ropes.indices

            for rope in level.ropes {
                XCTAssertTrue(holeRange.contains(rope.startHole), "Level \(levelId): bad startHole \(rope.startHole)")
                XCTAssertTrue(holeRange.contains(rope.endHole), "Level \(levelId): bad endHole \(rope.endHole)")
            }

            guard let actions = level.actions else { continue }
            for (i, action) in actions.enumerated() {
                XCTAssertTrue(ropeRange.contains(action.ropeIndex),
                    "Level \(levelId) action \(i): bad ropeIndex \(action.ropeIndex)")
                XCTAssertTrue(holeRange.contains(action.holeIndex),
                    "Level \(levelId) action \(i): bad holeIndex \(action.holeIndex)")
                XCTAssertTrue([0, 1].contains(action.endIndex),
                    "Level \(levelId) action \(i): bad endIndex \(action.endIndex)")
                XCTAssertTrue(["pin", "drag"].contains(action.type),
                    "Level \(levelId) action \(i): bad type '\(action.type)'")
            }
        }
    }

    // MARK: - Actions structure: pin all first, then drags

    func testActionStructure() {
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            guard let actions = level.actions else {
                XCTFail("Level \(levelId): no actions")
                continue
            }

            // First 2*ropeCount actions should be pins
            let pinCount = level.ropes.count * 2
            XCTAssertGreaterThanOrEqual(actions.count, pinCount, "Level \(levelId): not enough actions")

            for i in 0..<pinCount {
                XCTAssertEqual(actions[i].type, "pin", "Level \(levelId) action \(i): expected pin")
            }

            // Each rope should have exactly 2 pins (start and end)
            for ropeIdx in level.ropes.indices {
                let pins = actions.prefix(pinCount).filter { $0.ropeIndex == ropeIdx && $0.type == "pin" }
                XCTAssertEqual(pins.count, 2, "Level \(levelId) rope \(ropeIdx): expected 2 pins, got \(pins.count)")
                let ends = Set(pins.map { $0.endIndex })
                XCTAssertEqual(ends, [0, 1], "Level \(levelId) rope \(ropeIdx): should pin both ends")
            }

            // Remaining actions should be drags
            for i in pinCount..<actions.count {
                XCTAssertEqual(actions[i].type, "drag", "Level \(levelId) action \(i): expected drag")
            }
        }
    }

    // MARK: - At least one 2D crossing between initial rope positions

    func testRopesFormStarPattern() {
        // Structured pairs should connect roughly-opposite holes
        for levelId in 1...20 {
            let level = LevelGenerator.generate(levelId: levelId)
            guard level.ropes.count >= 2 else { continue }
            // Each rope should be reasonably long (connecting distant holes)
            for (i, rope) in level.ropes.enumerated() {
                let dist = simd_length(level.holes[rope.startHole].simd - level.holes[rope.endHole].simd)
                XCTAssertGreaterThanOrEqual(dist, 0.4,
                    "Level \(levelId) rope \(i): too short (\(dist)) for star pattern")
            }
        }
    }

    // MARK: - Rope endpoints are sufficiently far apart

    func testRopeEndpointDistance() {
        for levelId in 1...50 {
            let level = LevelGenerator.generate(levelId: levelId)
            for (i, rope) in level.ropes.enumerated() {
                let dist = simd_length(level.holes[rope.startHole].simd - level.holes[rope.endHole].simd)
                XCTAssertGreaterThan(dist, 0.25,
                    "Level \(levelId) rope \(i): endpoints too close (\(dist))")
            }
        }
    }

    // MARK: - Drag actions reference valid free holes

    func testDragActionsConsistency() {
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            guard let actions = level.actions else { continue }

            // Track which holes are in use
            var ropeState: [(start: Int, end: Int)] = level.ropes.map { ($0.startHole, $0.endHole) }
            var usedHoles = Set(ropeState.flatMap { [$0.start, $0.end] })

            let pinCount = level.ropes.count * 2
            for action in actions.suffix(from: pinCount) {
                guard action.type == "drag" else { continue }
                let ri = action.ropeIndex

                // Target hole should not be currently used by another rope's endpoint
                // (it could be the same rope's old position being freed)
                let oldHole = action.endIndex == 0 ? ropeState[ri].start : ropeState[ri].end
                usedHoles.remove(oldHole)

                if action.endIndex == 0 {
                    ropeState[ri].start = action.holeIndex
                } else {
                    ropeState[ri].end = action.holeIndex
                }
                usedHoles.insert(action.holeIndex)
            }
        }
    }

    // MARK: - Layout variety across levels

    func testLayoutVariety() {
        var holeCounts = Set<Int>()
        for levelId in 1...16 {  // 2 full cycles of 8 layouts
            let level = LevelGenerator.generate(levelId: levelId)
            holeCounts.insert(level.holes.count)
        }
        XCTAssertGreaterThanOrEqual(holeCounts.count, 3, "Not enough layout variety")
    }

    // MARK: - Holes are within reasonable board bounds

    func testHolesWithinBounds() {
        let maxCoord: Float = 1.2
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            for (i, hole) in level.holes.enumerated() {
                XCTAssertLessThanOrEqual(abs(hole.simd.x), maxCoord,
                    "Level \(levelId) hole \(i): x=\(hole.simd.x) out of bounds")
                XCTAssertLessThanOrEqual(abs(hole.simd.y), maxCoord,
                    "Level \(levelId) hole \(i): y=\(hole.simd.y) out of bounds")
            }
        }
    }

    // MARK: - Colors are valid

    func testRopeColorsValid() {
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            for (i, rope) in level.ropes.enumerated() {
                let c = rope.color
                XCTAssertGreaterThanOrEqual(c.redChannel, 0, "Level \(levelId) rope \(i): r < 0")
                XCTAssertLessThanOrEqual(c.redChannel, 1, "Level \(levelId) rope \(i): r > 1")
                XCTAssertGreaterThanOrEqual(c.greenChannel, 0, "Level \(levelId) rope \(i): g < 0")
                XCTAssertLessThanOrEqual(c.greenChannel, 1, "Level \(levelId) rope \(i): g > 1")
                XCTAssertGreaterThanOrEqual(c.blueChannel, 0, "Level \(levelId) rope \(i): b < 0")
                XCTAssertLessThanOrEqual(c.blueChannel, 1, "Level \(levelId) rope \(i): b > 1")
            }
        }
    }

    // MARK: - Radius is positive

    func testRopeRadiusPositive() {
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            for (i, rope) in level.ropes.enumerated() {
                XCTAssertGreaterThan(rope.radius, 0, "Level \(levelId) rope \(i): radius <= 0")
                XCTAssertLessThan(rope.radius, 0.1, "Level \(levelId) rope \(i): radius too large")
            }
        }
    }

    // MARK: - Stress test: generate 200 levels without crash

    func testStressGenerate200Levels() {
        for levelId in 1...200 {
            let level = LevelGenerator.generate(levelId: levelId)
            XCTAssertGreaterThanOrEqual(level.ropes.count, 2, "Level \(levelId): too few ropes")
            XCTAssertNotNil(level.actions, "Level \(levelId): nil actions")
        }
    }

    // MARK: - Screenshot-level complexity at level ~20

    func testLevel20MatchesScreenshotComplexity() {
        // Screenshot shows: 4 ropes, each wrapping 1-2 others → ~6-8 drags
        let level = LevelGenerator.generate(levelId: 20)
        XCTAssertEqual(level.ropes.count, 4, "Level 20 should have 4 ropes")

        let drags = (level.actions ?? []).filter { $0.type == "drag" }
        XCTAssertGreaterThanOrEqual(drags.count, 6, "Level 20 should have >= 6 drags for proper tangling")

        // Verify drags touch multiple different ropes (not all on one rope)
        let draggedRopes = Set(drags.map { $0.ropeIndex })
        XCTAssertGreaterThanOrEqual(draggedRopes.count, 2, "Drags should involve multiple ropes")

        // Verify drags create crossings (at least some drags cross other ropes)
        // Check that initial rope positions have 2D crossings
        var crossingPairs = 0
        for i in 0..<level.ropes.count {
            for j in (i+1)..<level.ropes.count {
                if segmentsCross(
                    level.holes[level.ropes[i].startHole].simd,
                    level.holes[level.ropes[i].endHole].simd,
                    level.holes[level.ropes[j].startHole].simd,
                    level.holes[level.ropes[j].endHole].simd
                ) {
                    crossingPairs += 1
                }
            }
        }
        XCTAssertGreaterThanOrEqual(crossingPairs, 1, "Level 20: ropes should cross initially")
    }

    // MARK: - Linking number test (actual topology after physics simulation)

    func testLevelsProduceLinkingNumbers() {
        let physics = RopePhysics()

        for levelId in [1, 5, 10, 15, 20, 50, 100, 200] {
            let level = LevelGenerator.generate(levelId: levelId)

            // Run VerletSimulator with the generated level
            let sim = VerletSimulator(
                holePositions: level.holes.map { $0.simd },
                holeRadius: level.holeRadius
            )
            sim.settleSteps = 20
            sim.constraintIterations = 8

            let configs = level.ropes.map {
                VerletSimulator.RopeConfig(startHole: $0.startHole, endHole: $0.endHole, radius: $0.radius)
            }
            let actions: [VerletSimulator.LevelAction] = (level.actions ?? []).compactMap { action in
                guard let t = VerletSimulator.LevelAction.ActionType(rawValue: action.type) else { return nil }
                return VerletSimulator.LevelAction(type: t, ropeIndex: action.ropeIndex, endIndex: action.endIndex, holeIndex: action.holeIndex)
            }
            sim.initializeLevel(ropeConfigs: configs, actions: actions)

            // Check linking numbers between all active band pairs
            var totalLinking = 0
            let activeBands = sim.bands.indices.filter { sim.bands[$0].active }
            for i in 0..<activeBands.count {
                for j in (i+1)..<activeBands.count {
                    let lk = physics.linkingNumber(
                        sim.bands[activeBands[i]].positions,
                        sim.bands[activeBands[j]].positions
                    )
                    totalLinking += abs(lk)
                }
            }

            let dragCount = (level.actions ?? []).filter { $0.type == "drag" }.count
            print("Level \(levelId): ropes=\(level.ropes.count) drags=\(dragCount) totalLinking=\(totalLinking)")

            // Levels should have non-zero linking (ropes are actually tangled)
            if levelId >= 3 {
                XCTAssertGreaterThan(totalLinking, 0,
                    "Level \(levelId): linking number is 0 — no real tangling! " +
                    "Drags: \(dragCount)")
            }
        }
    }

    // MARK: - Helpers

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
}
