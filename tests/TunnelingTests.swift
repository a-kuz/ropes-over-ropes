import XCTest
import simd

/// Test that two crossed ropes don't tunnel through each other under high tension.
final class TunnelingTests: XCTestCase {

    /// Create a simulator with two short ropes pinned in an X pattern.
    /// Rope 0: hole 0 (-0.8, 0.5) → hole 1 (0.8, -0.5)  diagonal \
    /// Rope 1: hole 2 (-0.5, -0.8) → hole 3 (0.5, 0.8)   diagonal /
    /// They cross in the center.
    /// Blue rope pinned first (goes under), red rope pinned second (goes over).
    /// Then drag blue (lower) end across red to test tunneling.
    private func makeXSetup() -> VerletSimulator {
        let holes: [SIMD2<Float>] = [
            SIMD2<Float>(-0.4,  0.25),  // 0 — red start
            SIMD2<Float>( 0.4, -0.25),  // 1 — red end
            SIMD2<Float>(-0.25, -0.4),  // 2 — blue start
            SIMD2<Float>( 0.25,  0.4),  // 3 — blue end (will be dragged)
        ]
        let elevations: [Float] = Array(repeating: 0, count: holes.count)
        let sim = VerletSimulator(holePositions: holes, holeElevations: elevations, holeRadius: 0.08, boards: [])
        sim.particleCount = 15
        sim.constraintIterations = 8
        sim.gravity = -5.0
        sim.damping = 0.97
        sim.ropeTension = 0.98
        let ropeRadius: Float = 0.04
        sim.liftHeight = ropeRadius * 2 + 0.01  // just above diameter

        let configs: [VerletSimulator.RopeConfig] = [
            VerletSimulator.RopeConfig(startHole: 0, endHole: 1, radius: 0.04), // 0: red
            VerletSimulator.RopeConfig(startHole: 2, endHole: 3, radius: 0.04), // 1: blue
        ]
        // Blue pinned first → settles underneath. Red pinned second → settles on top.
        let actions: [VerletSimulator.LevelAction] = [
            VerletSimulator.LevelAction(type: .pin, ropeIndex: 1, endIndex: 0, holeIndex: 2),
            VerletSimulator.LevelAction(type: .pin, ropeIndex: 1, endIndex: 1, holeIndex: 3),
            VerletSimulator.LevelAction(type: .pin, ropeIndex: 0, endIndex: 0, holeIndex: 0),
            VerletSimulator.LevelAction(type: .pin, ropeIndex: 0, endIndex: 1, holeIndex: 1),
        ]
        sim.initializeLevel(ropeConfigs: configs, actions: actions)
        sim.doSteps(300, collide: true)
        return sim
    }

    /// Count 2D crossings between rope i and rope j (same logic as win check).
    private func countCrossings(_ sim: VerletSimulator, _ i: Int, _ j: Int) -> Int {
        let posA = sim.bands[i].positions
        let posB = sim.bands[j].positions
        let skip = 3
        let nA = posA.count
        let nB = posB.count
        let startA = skip, endA = max(startA, nA - 1 - skip)
        let startB = skip, endB = max(startB, nB - 1 - skip)
        var count = 0
        for si in startA..<endA {
            let a0 = SIMD2<Float>(posA[si].x, posA[si].y)
            let a1 = SIMD2<Float>(posA[si + 1].x, posA[si + 1].y)
            for sj in startB..<endB {
                let b0 = SIMD2<Float>(posB[sj].x, posB[sj].y)
                let b1 = SIMD2<Float>(posB[sj + 1].x, posB[sj + 1].y)
                if segmentsCross(a0, a1, b0, b1) {
                    count += 1
                }
            }
        }
        return count
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
        return t > 1e-6 && t < (1 - 1e-6) && u > 1e-6 && u < (1 - 1e-6)
    }

    /// Verify that after setup, the two ropes actually cross.
    func testInitialCrossing() {
        let sim = makeXSetup()
        let crossings = countCrossings(sim, 0, 1)
        XCTAssertGreaterThan(crossings, 0, "Ropes should cross after init")
    }

    /// Drag one end of rope 1 far away (5x stretch), simulating high tension.
    /// The crossing count should remain > 0 throughout — if it drops to 0,
    /// the rope tunneled through the other.
    func testNoTunnelingUnderStretch() {
        let sim = makeXSetup()

        let initialCrossings = countCrossings(sim, 0, 1)
        XCTAssertGreaterThan(initialCrossings, 0, "Precondition: ropes must cross")

        // Unpin rope 1 end (endIndex=1) and drag it far away
        let bandIdx = 1
        let endIdx = 1
        let startPos = sim.holePositions[3] // (0.5, 0.8)
        sim.beginDrag(bandIndex: bandIdx, endIndex: endIdx, worldPosition: startPos)

        // Drag incrementally towards far corner over many steps
        let target = SIMD2<Float>(2.0, -2.0)  // far away — big stretch
        let dragSteps = 200
        var tunneledAtStep: Int? = nil

        for step in 0..<dragSteps {
            let t = Float(step + 1) / Float(dragSteps)
            let pos = startPos + (target - startPos) * t
            sim.updateDrag(worldPosition: pos)
            sim.doSteps(4, collide: true)

            let crossings = countCrossings(sim, 0, 1)
            if crossings == 0 && tunneledAtStep == nil {
                tunneledAtStep = step
                // Don't break — continue to see if it stays at 0
            }
        }

        if let step = tunneledAtStep {
            let finalCrossings = countCrossings(sim, 0, 1)
            XCTFail("Rope tunneled through at drag step \(step)/\(dragSteps). Final crossings: \(finalCrossings)")
        }
    }

    /// Same test but drag even more aggressively (instant teleport to far position).
    func testNoTunnelingInstantStretch() {
        let sim = makeXSetup()
        XCTAssertGreaterThan(countCrossings(sim, 0, 1), 0, "Precondition: ropes must cross")

        // Unpin and teleport rope 1's end very far
        sim.beginDrag(bandIndex: 1, endIndex: 1, worldPosition: sim.holePositions[3])
        sim.updateDrag(worldPosition: SIMD2<Float>(3.0, -3.0))

        // Run physics for a while
        sim.doSteps(500, collide: true)

        let crossings = countCrossings(sim, 0, 1)
        XCTAssertGreaterThan(crossings, 0, "Rope should not tunnel through even with instant large stretch")
    }

    /// Stress test: fast drag with fewer constraint iterations (more likely to tunnel).
    func testNoTunnelingLowIterations() {
        let sim = makeXSetup()
        sim.constraintIterations = 2  // weak solver
        XCTAssertGreaterThan(countCrossings(sim, 0, 1), 0, "Precondition: ropes must cross")

        sim.beginDrag(bandIndex: 1, endIndex: 1, worldPosition: sim.holePositions[3])

        // Very fast drag
        let target = SIMD2<Float>(3.0, -3.0)
        let startPos = sim.holePositions[3]
        let dragSteps = 30  // fast — few steps, big jumps
        var tunneledAtStep: Int? = nil

        for step in 0..<dragSteps {
            let t = Float(step + 1) / Float(dragSteps)
            let pos = startPos + (target - startPos) * t
            sim.updateDrag(worldPosition: pos)
            sim.doSteps(2, collide: true)

            if countCrossings(sim, 0, 1) == 0 && tunneledAtStep == nil {
                tunneledAtStep = step
            }
        }

        if let step = tunneledAtStep {
            XCTFail("Tunneled at step \(step)/\(dragSteps) with low iterations")
        }
    }

    /// Exact reproduction of the visual tunneling test:
    /// Blue rope is UNDER red. Drag blue end (hole 3) far across red rope.
    /// Should FAIL if tunneling occurs.
    func testNoTunnelingDragAcross() {
        let sim = makeXSetup()
        let initialCrossings = countCrossings(sim, 0, 1)
        XCTAssertGreaterThan(initialCrossings, 0, "Precondition: ropes must cross initially")

        let startPos = sim.holePositions[3]  // blue end at (0.25, 0.4)
        sim.beginDrag(bandIndex: 1, endIndex: 1, worldPosition: startPos)

        let target = SIMD2<Float>(-30.0, -30.0)
        let totalFrames = 6000       // same as autoDragTotalSteps
        let dt: Float = 1.0 / 120.0 // same as game frame rate
        var tunneledAtFrame: Int? = nil
        var prevCrossings = initialCrossings
        var logLines: [String] = []

        func logState(frame: Int, c: Int, pos: SIMD2<Float>, label: String) {
            let posA = sim.bands[0].positions
            let posB = sim.bands[1].positions
            let skip = 3
            // Log ALL crossing pairs and near pairs
            var pairInfo = ""
            for si in skip..<max(skip, posA.count-1-skip) {
                let a0 = posA[si], a1 = posA[si+1]
                let a02 = SIMD2<Float>(a0.x, a0.y), a12 = SIMD2<Float>(a1.x, a1.y)
                for sj in skip..<max(skip, posB.count-1-skip) {
                    let b0 = posB[sj], b1 = posB[sj+1]
                    let b02 = SIMD2<Float>(b0.x, b0.y), b12 = SIMD2<Float>(b1.x, b1.y)
                    let crosses = segmentsCross(a02, a12, b02, b12)
                    let mA = (a0 + a1) * 0.5, mB = (b0 + b1) * 0.5
                    let d2D = simd_length(SIMD2<Float>(mA.x-mB.x, mA.y-mB.y))
                    if crosses || d2D < 0.15 {
                        let zA = (a0.z + a1.z) * 0.5
                        let zB = (b0.z + b1.z) * 0.5
                        pairInfo += " r\(si)/b\(sj):\(crosses ? "X" : "~") zA=\(String(format:"%.3f",zA)) zB=\(String(format:"%.3f",zB)) Δ=\(String(format:"%.3f",zA-zB))"
                    }
                }
            }
            let dragZ = sim.bands[sim.dragInfo?.bandIndex ?? 1].positions.last?.z ?? 0
            logLines.append("[\(label)] f=\(frame) cross=\(c) dragZ=\(String(format:"%.4f",dragZ)) pos=(\(String(format:"%.3f",pos.x)),\(String(format:"%.3f",pos.y)))\(pairInfo)")
        }

        for frame in 0..<totalFrames {
            let t = Float(frame + 1) / Float(totalFrames)
            let pos = startPos + (target - startPos) * t
            sim.updateDrag(worldPosition: pos)
            sim.update(deltaTime: dt)

            let c = countCrossings(sim, 0, 1)
            if frame % 10 == 0 || c != prevCrossings || (frame >= 100 && frame <= 115) {
                logState(frame: frame, c: c, pos: pos, label: c != prevCrossings ? "CHANGE" : "tick")
            }
            if c == 0 && tunneledAtFrame == nil {
                tunneledAtFrame = frame

                // Log crossing segment details at tunneling moment
                let posA = sim.bands[0].positions  // red
                let posB = sim.bands[1].positions  // blue
                let skip = 3
                var log = "=== TUNNELING frame=\(frame) dragPos=(\(String(format:"%.2f",pos.x)),\(String(format:"%.2f",pos.y))) blueEndZ=\(String(format:"%.4f",posB.last?.z ?? 0))\n"

                for si in skip..<(posA.count - 1 - skip) {
                    let a0 = SIMD2<Float>(posA[si].x, posA[si].y)
                    let a1 = SIMD2<Float>(posA[si+1].x, posA[si+1].y)
                    for sj in skip..<(posB.count - 1 - skip) {
                        let b0 = SIMD2<Float>(posB[sj].x, posB[sj].y)
                        let b1 = SIMD2<Float>(posB[sj+1].x, posB[sj+1].y)
                        let midA = (a0 + a1) * 0.5
                        let midB = (b0 + b1) * 0.5
                        if simd_length(midA - midB) < 1.0 {
                            let zA = (posA[si].z + posA[si+1].z) * 0.5
                            let zB = (posB[sj].z + posB[sj+1].z) * 0.5
                            let r = sim.bands[0].radius + sim.bands[1].radius
                            let crosses = segmentsCross(a0, a1, b0, b1)
                            log += "  red[\(si)] blue[\(sj)]: zA=\(String(format:"%.4f",zA)) zB=\(String(format:"%.4f",zB)) zDiff=\(String(format:"%.4f",zA-zB)) sumR=\(String(format:"%.4f",r)) 2Dcross=\(crosses)\n"
                        }
                    }
                }
                log += "  prevCrossings=\(prevCrossings) nowCrossings=\(c)"
                try? log.write(toFile: "/tmp/tunneling_log.txt", atomically: true, encoding: .utf8)
            }
            prevCrossings = c
        }

        let fullLog = logLines.joined(separator: "\n")
        try? fullLog.write(toFile: "/tmp/tunneling_log.txt", atomically: true, encoding: .utf8)

        if let frame = tunneledAtFrame {
            let pct = Float(frame) / Float(totalFrames) * 100
            XCTFail("TUNNELING at frame \(frame)/\(totalFrames) (\(String(format:"%.1f",pct))% of drag)\nLog:\n\(fullLog)")
        }
    }

    /// Simulate fast swipe: big jumps between updateDrag calls with full physics steps.
    func testNoTunnelingFastSwipe() {
        let sim = makeXSetup()
        XCTAssertGreaterThan(countCrossings(sim, 0, 1), 0, "Precondition")

        sim.beginDrag(bandIndex: 1, endIndex: 1, worldPosition: sim.holePositions[3])

        // 5 big jumps (simulating a fast finger swipe at 12fps with 120hz physics)
        let positions: [SIMD2<Float>] = [
            SIMD2<Float>(0.3, 0.4),
            SIMD2<Float>(-0.2, -0.3),
            SIMD2<Float>(-0.8, -1.0),
            SIMD2<Float>(-1.5, -1.5),
            SIMD2<Float>(-2.0, -2.0),
        ]
        for pos in positions {
            sim.updateDrag(worldPosition: pos)
            // 10 physics steps per touch event (120hz / 12fps)
            sim.doSteps(10, collide: true)
        }

        // Settle
        sim.doSteps(200, collide: true)

        let crossings = countCrossings(sim, 0, 1)
        XCTAssertGreaterThan(crossings, 0, "Fast swipe should not cause tunneling")
    }

    /// Extreme: teleport end directly through the other rope in one frame.
    func testNoTunnelingTeleport() {
        let sim = makeXSetup()
        XCTAssertGreaterThan(countCrossings(sim, 0, 1), 0, "Precondition: ropes must cross")

        // Teleport rope 1 end from (0.5, 0.8) to (-1.5, -1.5) in one step
        sim.beginDrag(bandIndex: 1, endIndex: 1, worldPosition: sim.holePositions[3])
        sim.updateDrag(worldPosition: SIMD2<Float>(-1.5, -1.5))
        sim.doSteps(1, collide: true)

        // Settle
        sim.doSteps(200, collide: true)

        let crossings = countCrossings(sim, 0, 1)
        XCTAssertGreaterThan(crossings, 0, "Teleporting end through other rope should not lose crossing")
    }
}
