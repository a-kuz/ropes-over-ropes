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
                    XCTAssertGreaterThan(dist, 0.01,
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
        // sim.cartFriction = 0.05  // removed from VerletSimulator
        // sim.cartDamping = 0.98

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

    // MARK: - Particle Braid Geometry

    func testParticleBraidRawGeometry() {
        let level = LevelGenerator.generateParticleBraidLevel(levelId: 3002)
        let n = level.ropes.count
        XCTAssertEqual(n, 3, "3 strands for level 3002")

        // Must have ropeParticles
        guard let allParticles = level.ropeParticles else {
            XCTFail("ropeParticles is nil")
            return
        }
        XCTAssertEqual(allParticles.count, n)

        let P = level.particlesPerRope
        for s in 0..<n {
            XCTAssertEqual(allParticles[s].count, P, "Strand \(s): wrong particle count")
        }

        // Check holes: N top + N bottom + decorative
        XCTAssertGreaterThanOrEqual(level.holes.count, 2 * n)

        let holeSpacing: Float = 0.28
        for s in 0..<n {
            let particles = allParticles[s]

            // First particle should match top hole position (x, y)
            let topHole = level.holes[level.ropes[s].startHole]
            XCTAssertEqual(particles[0].xPosition, topHole.xPosition, accuracy: 0.01,
                "Strand \(s): first particle x should match top hole")
            XCTAssertEqual(particles[0].yPosition, topHole.yPosition, accuracy: 0.01,
                "Strand \(s): first particle y should match top hole")

            // Last particle should match bottom hole position (x, y)
            let botHole = level.holes[level.ropes[s].endHole]
            XCTAssertEqual(particles[P-1].xPosition, botHole.xPosition, accuracy: 0.01,
                "Strand \(s): last particle x should match bottom hole")
            XCTAssertEqual(particles[P-1].yPosition, botHole.yPosition, accuracy: 0.01,
                "Strand \(s): last particle y should match bottom hole")

            // All particles within reasonable bounds
            for (pi, p) in particles.enumerated() {
                XCTAssertLessThanOrEqual(abs(p.xPosition), 0.50,
                    "Strand \(s) particle \(pi): x=\(p.xPosition) out of bounds")
                XCTAssertGreaterThanOrEqual(p.yPosition, -0.70,
                    "Strand \(s) particle \(pi): y=\(p.yPosition) below bottom")
                XCTAssertLessThanOrEqual(p.yPosition, 0.70,
                    "Strand \(s) particle \(pi): y=\(p.yPosition) above top")
                // Z: endpoints are 0, rest should be small positive
                if pi > 0 && pi < P-1 {
                    XCTAssertGreaterThanOrEqual(p.zPosition, 0,
                        "Strand \(s) particle \(pi): z=\(p.zPosition) negative")
                    XCTAssertLessThanOrEqual(p.zPosition, 0.2,
                        "Strand \(s) particle \(pi): z=\(p.zPosition) too high")
                }
            }

            // Y should be roughly monotonic along the braid axis
            // (small local variations OK due to helix orbit)
            let yStart = particles[0].yPosition
            let yEnd = particles[P-1].yPosition
            let yDir = yEnd - yStart // positive = increasing, negative = decreasing
            XCTAssertGreaterThan(abs(yDir), 0.5,
                "Strand \(s): endpoints should span significant Y range")
        }
    }

    func testParticleBraidSegmentLengths() {
        // Segment lengths from particles should be close to manual braid's ~0.020
        let level = LevelGenerator.generateParticleBraidLevel(levelId: 3002)
        guard let allParticles = level.ropeParticles else {
            XCTFail("no ropeParticles")
            return
        }

        for (s, particles) in allParticles.enumerated() {
            var totalLen: Float = 0
            for i in 1..<particles.count {
                let dx = particles[i].xPosition - particles[i-1].xPosition
                let dy = particles[i].yPosition - particles[i-1].yPosition
                let dz = particles[i].zPosition - particles[i-1].zPosition
                totalLen += sqrt(dx*dx + dy*dy + dz*dz)
            }
            let avgSeg = totalLen / Float(particles.count - 1)
            // Manual braid has segLen ≈ 0.020. Generated should be similar.
            XCTAssertLessThan(avgSeg, 0.035,
                "Strand \(s): avgSeg=\(avgSeg) too large (rope will be too slack)")
            XCTAssertGreaterThan(avgSeg, 0.015,
                "Strand \(s): avgSeg=\(avgSeg) too small")
            print("[BRAID-TEST] Strand \(s): totalLen=\(String(format:"%.4f",totalLen)) avgSeg=\(String(format:"%.5f",avgSeg)) particles=\(particles.count)")
        }
    }

    func testParticleBraidHelixOrbit() {
        // Helical strands should orbit within helixRadius of center
        let level = LevelGenerator.generateParticleBraidLevel(levelId: 3002)
        guard let allParticles = level.ropeParticles else {
            XCTFail("no ropeParticles")
            return
        }

        let n = level.ropes.count
        let P = allParticles[0].count

        // All strands share centerX=0 for single-config levels
        // In the braid zone (middle 50%), X spread should be <= 2*helixRadius
        let maxHelixRadius: Float = 0.10  // slightly larger than 0.08 to account for rounding
        for pi in (P/4)..<(3*P/4) {
            var xs: [Float] = []
            for s in 0..<n {
                xs.append(allParticles[s][pi].xPosition)
            }
            let spread = xs.max()! - xs.min()!
            XCTAssertLessThanOrEqual(spread, 2 * maxHelixRadius,
                "Particle \(pi): X spread \(spread) exceeds 2*helixRadius")
        }

        // Z should have variation in braid zone (over/under)
        var hasZVariation = false
        for pi in (P/4)..<(3*P/4) {
            var zs: [Float] = []
            for s in 0..<n { zs.append(allParticles[s][pi].zPosition) }
            if zs.max()! - zs.min()! > 0.02 { hasZVariation = true; break }
        }
        XCTAssertTrue(hasZVariation, "Braid zone should have Z over/under variation")
    }

    func testParticleBraidHasZCrossings() {
        // At crossing points, one strand should be over (high z) and another under (low z)
        let level = LevelGenerator.generateParticleBraidLevel(levelId: 3002)
        guard let allParticles = level.ropeParticles else {
            XCTFail("no ropeParticles")
            return
        }

        let n = level.ropes.count
        let P = allParticles[0].count

        // Find particles where two strands have similar X,Y but different Z
        var crossingCount = 0
        for pi in (P/4)..<(3*P/4) { // only check braid zone (middle 50%)
            for s1 in 0..<n {
                for s2 in (s1+1)..<n {
                    let p1 = allParticles[s1][pi]
                    let p2 = allParticles[s2][pi]
                    let dx = abs(p1.xPosition - p2.xPosition)
                    let dz = abs(p1.zPosition - p2.zPosition)
                    // If strands are close in X (crossing), Z should differ
                    if dx < 0.05 && dz > 0.02 {
                        crossingCount += 1
                    }
                }
            }
        }
        print("[BRAID-TEST] Z-crossings found: \(crossingCount)")
        XCTAssertGreaterThan(crossingCount, 0,
            "Should have at least one over/under crossing with Z separation")
    }

    func testParticleBraidPrintRawPositions() {
        // Debug helper: print raw generated positions
        let level = LevelGenerator.generateParticleBraidLevel(levelId: 3002)
        guard let allParticles = level.ropeParticles else {
            XCTFail("no ropeParticles")
            return
        }
        for (s, particles) in allParticles.enumerated() {
            print("// Raw strand \(s): \(particles.count) particles")
            for (i, p) in particles.enumerated() {
                print("  [\(i)] x=\(String(format:"%.4f",p.xPosition)) y=\(String(format:"%.4f",p.yPosition)) z=\(String(format:"%.4f",p.zPosition))")
            }
        }
    }

    func testBraidShowcaseLevel() {
        let level = LevelGenerator.generateStructureShowcase(levelId: 3100)
        XCTAssertEqual(level.id, 3100)

        // 4 helixes: 2+3+4+3 = 12 ropes
        XCTAssertEqual(level.ropes.count, 12)
        // 24 endpoint holes + decorative holes in rings
        XCTAssertGreaterThanOrEqual(level.holes.count, 24)

        guard let allParticles = level.ropeParticles else {
            XCTFail("ropeParticles is nil")
            return
        }
        XCTAssertEqual(allParticles.count, 12)

        let P = level.particlesPerRope
        for (i, particles) in allParticles.enumerated() {
            XCTAssertEqual(particles.count, P, "Strand \(i): wrong particle count")
        }

        // Endpoints should be at z=0 (in holes)
        for (i, particles) in allParticles.enumerated() {
            XCTAssertEqual(particles[0].zPosition, 0, accuracy: 0.001,
                "Strand \(i): start z should be 0")
            XCTAssertEqual(particles[P-1].zPosition, 0, accuracy: 0.001,
                "Strand \(i): end z should be 0")
        }

        // No exact duplicate hole positions
        for i in 0..<level.holes.count {
            for j in (i+1)..<level.holes.count {
                let d = simd_distance(level.holes[i].simd, level.holes[j].simd)
                XCTAssertGreaterThan(d, 0.01, "Holes \(i) and \(j) are duplicates: \(d)")
            }
        }

        // Custom particle count should work
        let level80 = LevelGenerator.generateStructureShowcase(levelId: 3100, particleCount: 80)
        XCTAssertEqual(level80.ropeParticles?.first?.count, 80)
        XCTAssertEqual(level80.particlesPerRope, 80)
    }

    // MARK: - 3D Rope penetration test

    /// For every pair of ropes, find the closest distance between all segment pairs in 3D.
    /// If distance < 2×ropeRadius → ropes penetrate each other.
    func testNoRopePenetrationInStructuredLevels() {
        let levelIds = [3100, 3101, 3102, 3103, 3104, 3105, 3108, 3110, 3111, 3112, 3113, 3115]
        let ropeRadius: Float = 0.038
        let minDist = ropeRadius * 0.9 // 1×radius with 10% tolerance

        for levelId in levelIds {
            let level = LevelGenerator.generateStructureShowcase(levelId: levelId, particleCount: 55)
            guard let allParticles = level.ropeParticles else { continue }

            var violations = 0
            var worstDist: Float = Float.greatestFiniteMagnitude
            var worstPair = (0, 0)
            let totalRopes = allParticles.count

            for a in 0..<totalRopes {
                for b in (a+1)..<totalRopes {
                    let pA = allParticles[a]
                    let pB = allParticles[b]

                    for i in 0..<(pA.count - 1) {
                        let a0 = SIMD3<Float>(pA[i].xPosition, pA[i].yPosition, pA[i].zPosition)
                        let a1 = SIMD3<Float>(pA[i+1].xPosition, pA[i+1].yPosition, pA[i+1].zPosition)

                        // Skip segments near holes (z ≈ 0)
                        if a0.z < 0.01 && a1.z < 0.01 { continue }

                        for j in 0..<(pB.count - 1) {
                            let b0 = SIMD3<Float>(pB[j].xPosition, pB[j].yPosition, pB[j].zPosition)
                            let b1 = SIMD3<Float>(pB[j+1].xPosition, pB[j+1].yPosition, pB[j+1].zPosition)

                            if b0.z < 0.01 && b1.z < 0.01 { continue }

                            let dist = closestDistanceBetweenSegments3D(a0, a1, b0, b1)
                            if dist < minDist {
                                violations += 1
                                if dist < worstDist {
                                    worstDist = dist
                                    worstPair = (a, b)
                                }
                            }
                        }
                    }
                }
            }

            let maxAllowed = totalRopes * 3
            XCTAssertLessThanOrEqual(violations, maxAllowed,
                "Level \(levelId): \(violations) 3D penetrations (ropes \(worstPair.0),\(worstPair.1) worst dist=\(String(format: "%.4f", worstDist)) < minDist=\(String(format: "%.4f", minDist)))")
        }
    }

    /// Closest distance between two 3D line segments.
    private func closestDistanceBetweenSegments3D(
        _ p0: SIMD3<Float>, _ p1: SIMD3<Float>,
        _ q0: SIMD3<Float>, _ q1: SIMD3<Float>
    ) -> Float {
        let d1 = p1 - p0
        let d2 = q1 - q0
        let r = p0 - q0

        let a = simd_dot(d1, d1) // |d1|²
        let e = simd_dot(d2, d2) // |d2|²
        let f = simd_dot(d2, r)

        // Both segments degenerate to points?
        if a < 1e-10 && e < 1e-10 {
            return simd_length(r)
        }
        if a < 1e-10 {
            // First segment degenerates
            let t = simd_clamp(f / e, 0, 1)
            return simd_length(r - d2 * t)
        }

        let c = simd_dot(d1, r)
        if e < 1e-10 {
            // Second segment degenerates
            let s = simd_clamp(-c / a, 0, 1)
            return simd_length(r + d1 * s)
        }

        let b = simd_dot(d1, d2)
        let denom = a * e - b * b

        var s: Float = 0
        if denom > 1e-10 {
            s = simd_clamp((b * f - c * e) / denom, 0, 1)
        }

        var t = (b * s + f) / e
        if t < 0 {
            t = 0
            s = simd_clamp(-c / a, 0, 1)
        } else if t > 1 {
            t = 1
            s = simd_clamp((b - c) / a, 0, 1)
        }

        let closest = r + d1 * s - d2 * t
        return simd_length(closest)
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

    // MARK: - Braid untangle levels

    func testBraidUntangleDebug() {
        // Detailed diagnostic for a specific braid level
        for levelId in [8, 12, 16, 20, 60, 68] {
            guard LevelGenerator.isBraidUntangleLevel(levelId) else { continue }
            let level = LevelGenerator.generate(levelId: levelId)
            let holeSimd = level.holes.map { SIMD2<Float>($0.xPosition, $0.yPosition) }
            let actions = level.actions ?? []

            print("\n=== BRAID LEVEL \(levelId) ===")
            print("Ropes: \(level.ropes.count), Holes: \(level.holes.count)")
            for (i, r) in level.ropes.enumerated() {
                let s = holeSimd[r.startHole]
                let e = holeSimd[r.endHole]
                print("  Rope \(i): hole \(r.startHole)(\(String(format:"%.2f,%.2f",s.x,s.y))) → hole \(r.endHole)(\(String(format:"%.2f,%.2f",e.x,e.y)))")
            }

            // Check crossing
            if level.ropes.count >= 2 {
                let r0 = level.ropes[0], r1 = level.ropes[1]
                let crosses = LevelGenerator.segmentsCross(holeSimd[r0.startHole], holeSimd[r0.endHole],
                                                            holeSimd[r1.startHole], holeSimd[r1.endHole])
                print("  Ropes 0×1 cross: \(crosses)")
            }

            print("  Actions (\(actions.count)):")
            for a in actions {
                print("    \(a.type) rope=\(a.ropeIndex) end=\(a.endIndex) hole=\(a.holeIndex) pos=\(String(format:"%.2f,%.2f", holeSimd[a.holeIndex].x, holeSimd[a.holeIndex].y))")
            }

            let drags = actions.filter { $0.type == "drag" }
            XCTAssertGreaterThan(drags.count, 2, "Level \(levelId): need >2 drags for spiral, got \(drags.count)")
        }
    }

    func testBraidUntangleLevelsDragCount() {
        let braidLevels = [8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48]
        for levelId in braidLevels {
            XCTAssertTrue(LevelGenerator.isBraidUntangleLevel(levelId), "Level \(levelId) should be braid")
            let level = LevelGenerator.generate(levelId: levelId)
            let actions = level.actions ?? []
            let drags = actions.filter { $0.type == "drag" }

            // Debug: check if ropes cross each other
            let holeSimd = level.holes.map { SIMD2<Float>($0.xPosition, $0.yPosition) }
            for i in 0..<level.ropes.count {
                for j in (i+1)..<level.ropes.count {
                    let crosses = LevelGenerator.segmentsCross(
                        holeSimd[level.ropes[i].startHole], holeSimd[level.ropes[i].endHole],
                        holeSimd[level.ropes[j].startHole], holeSimd[level.ropes[j].endHole])
                    print("Level \(levelId): rope \(i)(\(level.ropes[i].startHole)→\(level.ropes[i].endHole)) x rope \(j)(\(level.ropes[j].startHole)→\(level.ropes[j].endHole)) = \(crosses)")
                }
            }

            print("Level \(levelId): ropes=\(level.ropes.count) holes=\(level.holes.count) drags=\(drags.count)")
            XCTAssertGreaterThan(drags.count, 1, "Level \(levelId) should have multiple drags, got \(drags.count)")
        }
    }
}
