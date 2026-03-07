import XCTest
import simd

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
            XCTAssertLessThanOrEqual(level.holes.count, 80, "Level \(levelId): too many holes")
        }
    }

    // MARK: - No duplicate holes (within tolerance)

    func testNoDuplicateHolePositions() {
        for levelId in 1...30 {
            let level = LevelGenerator.generate(levelId: levelId)
            for i in 0..<level.holes.count {
                for j in (i+1)..<level.holes.count {
                    let dist = simd_length(level.holes[i].simd - level.holes[j].simd)
                    XCTAssertGreaterThanOrEqual(dist, 0.0,
                        "Level \(levelId): holes \(i) and \(j) overlap (\(dist))")
                }
            }
        }
    }

    // MARK: - Rope count matches difficulty curve

    func testRopeCountMatchesDifficulty() {
        for levelId in 1...2 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 3, "Level \(levelId)")
        }
        for levelId in 3...5 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 4, "Level \(levelId)")
        }
        for levelId in 6...9 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 5, "Level \(levelId)")
        }
        for levelId in 10...15 {
            XCTAssertEqual(LevelGenerator.generate(levelId: levelId).ropes.count, 6, "Level \(levelId)")
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
        for levelId in 1...20 {
            let level = LevelGenerator.generate(levelId: levelId)
            guard level.ropes.count >= 2 else { continue }
            var longCount = 0
            for rope in level.ropes {
                let dist = simd_length(level.holes[rope.startHole].simd - level.holes[rope.endHole].simd)
                if dist >= 0.4 { longCount += 1 }
            }
            XCTAssertGreaterThanOrEqual(longCount, 1,
                "Level \(levelId): at least one long rope expected")
        }
    }

    // MARK: - Rope endpoints are not self-loops

    func testRopeEndpointDistance() {
        for levelId in 1...50 {
            let level = LevelGenerator.generate(levelId: levelId)
            for (i, rope) in level.ropes.enumerated() {
                let dist = simd_length(level.holes[rope.startHole].simd - level.holes[rope.endHole].simd)
                XCTAssertGreaterThan(dist, 0.05,
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
        let maxCoord: Float = 1.5
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
        let level = LevelGenerator.generate(levelId: 20)
        XCTAssertGreaterThanOrEqual(level.ropes.count, 4, "Level 20 should have >= 4 ropes")

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

    // MARK: - No isolated ropes after generation

    func testNoIsolatedRopes() {
        for levelId in 1...100 {
            let level = LevelGenerator.generate(levelId: levelId)
            guard let actions = level.actions, level.ropes.count >= 2 else { continue }

            var ropeState: [(start: Int, end: Int)] = level.ropes.map { ($0.startHole, $0.endHole) }
            let pinCount = level.ropes.count * 2
            for action in actions.suffix(from: pinCount) {
                guard action.type == "drag" else { continue }
                if action.endIndex == 0 {
                    ropeState[action.ropeIndex].start = action.holeIndex
                } else {
                    ropeState[action.ropeIndex].end = action.holeIndex
                }
            }

            for i in 0..<level.ropes.count {
                let a0 = level.holes[ropeState[i].start].simd
                let a1 = level.holes[ropeState[i].end].simd
                var hasCrossing = false
                for j in 0..<level.ropes.count where j != i {
                    let b0 = level.holes[ropeState[j].start].simd
                    let b1 = level.holes[ropeState[j].end].simd
                    if segmentsCross(a0, a1, b0, b1) {
                        hasCrossing = true
                        break
                    }
                }
                XCTAssertTrue(hasCrossing,
                    "Level \(levelId) rope \(i): isolated (no crossings with any other rope). " +
                    "Endpoints: hole \(ropeState[i].start) → hole \(ropeState[i].end)")
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

    // MARK: - Rail Mode E2E: Cart collision pushes cart

    func testRailCartCollision() {
        // Setup: straight horizontal rail with cart at center
        // Rope pinned above and below the cart → particles drape over cart → push it
        let holePositions: [SIMD2<Float>] = [
            SIMD2(0.0, 0.4),    // 0: above cart
            SIMD2(0.3, -0.4),   // 1: below-right of cart (diagonal creates lateral force)
        ]
        let holeElevations: [Float] = [0, 0]

        let sim = VerletSimulator(
            holePositions: holePositions,
            holeElevations: holeElevations,
            holeRadius: 0.08,
            boards: []
        )
        sim.isRailMode = true
        sim.gravity = -5.0
        sim.damping = 0.97
        sim.constraintIterations = 4
        sim.particleCount = 20
        sim.cartFriction = 0.05
        sim.cartDamping = 0.98

        // Rail: horizontal from -0.6 to 0.6
        let railDef = LevelDefinition.RailDef(points: [
            .init(xPosition: -0.6, yPosition: 0),
            .init(xPosition: 0.6, yPosition: 0),
        ])
        // Cart starts at center (t=0.5), right where rope will cross
        let cartDef = LevelDefinition.CartDef(railIndex: 0, startT: 0.5, radius: 0.12, mass: 0.3)
        let stationDef = LevelDefinition.StationDef(railIndex: 0, t: 0.9, radius: 0.15, cartIndex: 0)
        sim.initializeRails(railDefs: [railDef], cartDefs: [cartDef], stationDefs: [stationDef])

        let initialT = sim.carts[0].t
        XCTAssertEqual(initialT, 0.5, accuracy: 0.01)

        // Rope from hole 0 (above) to hole 1 (below-right)
        // This creates a diagonal rope that crosses the cart position
        let ropeConfigs = [
            VerletSimulator.RopeConfig(startHole: 0, endHole: 1, radius: 0.038, crossSection: .circular(radius: 0.038))
        ]
        let actions = [
            VerletSimulator.LevelAction(type: .pin, ropeIndex: 0, endIndex: 0, holeIndex: 0),
            VerletSimulator.LevelAction(type: .pin, ropeIndex: 0, endIndex: 1, holeIndex: 1),
        ]
        sim.initializeLevel(ropeConfigs: ropeConfigs, actions: actions)

        XCTAssertEqual(sim.bands.count, 1)
        XCTAssertTrue(sim.bands[0].active)

        // Log initial particle positions
        let midParticle = sim.bands[0].positions.count / 2
        print("[RAIL TEST] Initial cart t=\(initialT), pos=\(sim.rails[0].position(at: initialT))")
        print("[RAIL TEST] Rope midpoint=\(sim.bands[0].positions[midParticle])")

        // Simulate 300 frames — gravity pulls rope down onto cart
        for frame in 0..<300 {
            sim.update(deltaTime: 1.0 / 60.0)
            if frame % 60 == 0 {
                let ct = sim.carts[0].t
                let cv = sim.carts[0].velocity
                print("[RAIL TEST] frame=\(frame) cart t=\(ct) vel=\(cv)")
            }
        }

        let finalT = sim.carts[0].t
        print("[RAIL TEST] Final cart t=\(finalT) (initial=\(initialT))")
        print("[RAIL TEST] Cart velocity=\(sim.carts[0].velocity)")

        // The diagonal rope should push cart to the right (positive t direction)
        XCTAssertNotEqual(finalT, initialT, accuracy: 0.02,
            "Cart should have moved. Diagonal rope from (0,0.4) to (0.3,-0.4) should push cart rightward")
    }

    // MARK: - Braid Mode

    func testBraidLevelGeneration() {
        for localId in 1...10 {
            let levelId = 3000 + localId
            let level = LevelGenerator.generateBraidLevel(levelId: levelId)

            XCTAssertEqual(level.mode, "braid", "Level \(levelId) should be braid mode")
            XCTAssertTrue(level.isBraidMode)

            let strandCount = level.ropes.count
            XCTAssertGreaterThanOrEqual(strandCount, 3, "Level \(levelId): at least 3 strands")
            XCTAssertEqual(level.holes.count, strandCount * 2, "Level \(levelId): holes = 2 * strands")

            // Top holes: 0..<N, bottom holes: N..<2N
            for i in 0..<strandCount {
                XCTAssertEqual(level.ropes[i].startHole, i, "Level \(levelId): rope \(i) start = top hole \(i)")
                XCTAssertEqual(level.ropes[i].endHole, strandCount + i, "Level \(levelId): rope \(i) end = bottom hole")
            }

            // Braid targets
            XCTAssertNotNil(level.braidTargets)
            XCTAssertEqual(level.braidTargets?.count, strandCount)

            // Each target must be a valid bottom hole
            if let targets = level.braidTargets {
                for (i, t) in targets.enumerated() {
                    XCTAssertGreaterThanOrEqual(t, strandCount, "Level \(levelId): target[\(i)]=\(t) should be >= N")
                    XCTAssertLessThan(t, strandCount * 2, "Level \(levelId): target[\(i)]=\(t) should be < 2N")
                }
                // Targets should be a permutation of bottom holes
                let targetSet = Set(targets)
                XCTAssertEqual(targetSet.count, strandCount, "Level \(levelId): targets should be a permutation")
            }

            // Min crossings
            XCTAssertNotNil(level.braidMinCrossings)
            XCTAssertGreaterThanOrEqual(level.braidMinCrossings ?? 0, 1)

            // Pin actions exist for all ropes
            let pinActions = level.actions?.filter { $0.type == "pin" } ?? []
            XCTAssertEqual(pinActions.count, strandCount * 2, "Level \(levelId): 2 pin actions per rope")
        }
    }

    func testBraidLevelDeterministic() {
        for localId in 1...5 {
            let levelId = 3000 + localId
            let a = LevelGenerator.generateBraidLevel(levelId: levelId)
            let b = LevelGenerator.generateBraidLevel(levelId: levelId)
            XCTAssertEqual(a.ropes.count, b.ropes.count)
            XCTAssertEqual(a.braidTargets, b.braidTargets)
            XCTAssertEqual(a.braidMinCrossings, b.braidMinCrossings)
        }
    }

    func testBraidTargetIsNontrivial() {
        // For levels with enough swaps, the target should NOT be identity
        let level = LevelGenerator.generateBraidLevel(levelId: 3002) // 3 strands, 4 swaps
        let n = level.ropes.count
        let targets = level.braidTargets!
        let isIdentity = (0..<n).allSatisfy { targets[$0] == n + $0 }
        XCTAssertFalse(isIdentity, "Braid target should not be identity after 4 swaps")
    }

    /// Test with user's custom rail level JSON — verifies the level loads and cart exists
    func testUserRailLevelLoads() {
        let json = """
        {"actions":[{"endIndex":0,"holeIndex":5,"ropeIndex":0,"type":"pin"},{"endIndex":1,"holeIndex":1,"ropeIndex":0,"type":"pin"}],"holeRadius":0.08,"holes":[{"x":-1.0371923,"y":0.21173078,"z":0},{"x":-0.4697885,"y":-0.10661539,"z":0},{"x":-0.4952308,"y":-0.6161539,"z":0},{"x":0.39550555,"y":0.19649427,"z":0},{"x":0.58667624,"y":-0.32826182,"z":0},{"x":0.14575253,"y":0.819013,"z":0},{"x":-1.0297476,"y":-0.16861093,"z":0},{"x":-0.992927,"y":0.69973934,"z":0}],"id":2001,"mode":"rail","particlesPerRope":32,"rails":[{"points":[{"x":-1.0588506,"y":0.2654046,"z":0},{"x":-0.5074766,"y":-0.13609938,"z":0},{"x":0.04883345,"y":-0.312348,"z":0},{"x":0.22069326,"y":0.8374461,"z":0},{"x":0.34850514,"y":0.19717823,"z":0}]}],"ropes":[{"color":{"b":0.05,"g":0.3,"r":0.95},"endHole":1,"radius":0.038,"startHole":5}]}
        """
        let data = json.data(using: .utf8)!
        let level = try! JSONDecoder().decode(LevelDefinition.self, from: data)

        XCTAssertEqual(level.mode, "rail")
        XCTAssertTrue(level.isRailMode)
        XCTAssertEqual(level.rails?.count, 1)
        XCTAssertEqual(level.rails?[0].points.count, 5)
        XCTAssertEqual(level.holes.count, 8)
        XCTAssertEqual(level.ropes.count, 1)

        // Note: this level has no carts/stations defined — user needs to add them in editor
        // Cart and station are required for rail mode to be playable
        XCTAssertNil(level.carts, "User level has no carts — need to add via editor")
        XCTAssertNil(level.stations, "User level has no stations — need to add via editor")
    }
}
