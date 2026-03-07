import Foundation
import simd

extension Renderer {
    func updateFrameSimulation(deltaTime: Float) {
        // Slow braid replay: compute drag target from ACTUAL particle positions
        switch braidDragPhase {
        case .idle:
            if braidRemainingCrossings > 0, !pendingBraidDrags.isEmpty || braidRemainingCrossings > 0 {
                braidDragTimer += deltaTime
                if braidDragTimer >= braidDragInterval {
                    braidDragTimer = 0
                    // Compute next drag from live particle positions
                    if let action = computeNextBraidDrag() {
                        braidDragAction = action
                        guard let sim = simulator, sim.bands.indices.contains(action.ropeIndex) else { break }
                        let bi = action.ropeIndex
                        let ei = action.endIndex
                        braidDragIdx = ei == 0 ? 0 : sim.bands[bi].positions.count - 1
                        braidDragFromPos = sim.bands[bi].positions[braidDragIdx]
                        braidDragToPos = sim.holePosition3D(action.holeIndex)
                        let fromElev = sim.boardSurfaceZ(x: braidDragFromPos.x, y: braidDragFromPos.y)
                        let toElev = sim.holeSurfaceZ(action.holeIndex)
                        let maxElev = max(fromElev, toElev)
                        braidDragLiftFrom = SIMD3<Float>(braidDragFromPos.x, braidDragFromPos.y, maxElev + sim.liftHeight)
                        braidDragLiftTo = SIMD3<Float>(braidDragToPos.x, braidDragToPos.y, maxElev + sim.liftHeight)
                        if ei == 0 { sim.bands[bi].pinStart = nil } else { sim.bands[bi].pinEnd = nil }
                        braidDragStep = 1
                        braidDragPhase = .lift
                        braidRemainingCrossings -= 1
                    } else {
                        braidRemainingCrossings = 0 // can't find valid target, stop
                    }
                }
            } else if !pendingBraidDrags.isEmpty {
                // Fallback: use pre-computed drags if available
                braidDragTimer += deltaTime
                if braidDragTimer >= braidDragInterval {
                    braidDragTimer = 0
                    let action = pendingBraidDrags.removeFirst()
                    braidDragAction = action
                    guard let sim = simulator, sim.bands.indices.contains(action.ropeIndex) else { break }
                    let bi = action.ropeIndex
                    let ei = action.endIndex
                    braidDragIdx = ei == 0 ? 0 : sim.bands[bi].positions.count - 1
                    braidDragFromPos = sim.bands[bi].positions[braidDragIdx]
                    braidDragToPos = sim.holePosition3D(action.holeIndex)
                    let fromElev = sim.boardSurfaceZ(x: braidDragFromPos.x, y: braidDragFromPos.y)
                    let toElev = sim.holeSurfaceZ(action.holeIndex)
                    let maxElev = max(fromElev, toElev)
                    braidDragLiftFrom = SIMD3<Float>(braidDragFromPos.x, braidDragFromPos.y, maxElev + sim.liftHeight)
                    braidDragLiftTo = SIMD3<Float>(braidDragToPos.x, braidDragToPos.y, maxElev + sim.liftHeight)
                    if ei == 0 { sim.bands[bi].pinStart = nil } else { sim.bands[bi].pinEnd = nil }
                    braidDragStep = 1
                    braidDragPhase = .lift
                }
            }
        case .lift:
            if let sim = simulator, let action = braidDragAction {
                let t = Float(braidDragStep) / 150.0
                sim.bands[action.ropeIndex].positions[braidDragIdx] = braidDragFromPos + (braidDragLiftFrom - braidDragFromPos) * t
                sim.bands[action.ropeIndex].previousPositions[braidDragIdx] = sim.bands[action.ropeIndex].positions[braidDragIdx]
                sim.doSteps(4, collide: true)
                braidDragStep += 1
                if braidDragStep > 150 { braidDragStep = 1; braidDragPhase = .traverse }
            }
        case .traverse:
            if let sim = simulator, let action = braidDragAction {
                let t = Float(braidDragStep) / 150.0
                sim.bands[action.ropeIndex].positions[braidDragIdx] = braidDragLiftFrom + (braidDragLiftTo - braidDragLiftFrom) * t
                sim.bands[action.ropeIndex].previousPositions[braidDragIdx] = sim.bands[action.ropeIndex].positions[braidDragIdx]
                sim.doSteps(4, collide: true)
                braidDragStep += 1
                if braidDragStep > 150 { braidDragStep = 1; braidDragPhase = .lower }
            }
        case .lower:
            if let sim = simulator, let action = braidDragAction {
                let t = Float(braidDragStep) / 150.0
                sim.bands[action.ropeIndex].positions[braidDragIdx] = braidDragLiftTo + (braidDragToPos - braidDragLiftTo) * t
                sim.bands[action.ropeIndex].previousPositions[braidDragIdx] = sim.bands[action.ropeIndex].positions[braidDragIdx]
                sim.doSteps(4, collide: true)
                braidDragStep += 1
                if braidDragStep > 3 { braidDragStep = 0; braidDragPhase = .settle }
            }
        case .settle:
            if let sim = simulator, let action = braidDragAction {
                let bi = action.ropeIndex
                let ei = action.endIndex
                if ei == 0 { sim.bands[bi].pinStart = action.holeIndex } else { sim.bands[bi].pinEnd = action.holeIndex }
                sim.bands[bi].positions[braidDragIdx] = braidDragToPos
                sim.bands[bi].previousPositions[braidDragIdx] = braidDragToPos
                sim.doSteps(sim.settleSteps, collide: true)

                // Update upper/lower: dragged end → upper; other rope's end on same side → lower
                let otherRi = 1 - bi
                braidIsLower[bi][ei] = false
                let targetPos2D = SIMD2<Float>(braidDragToPos.x, braidDragToPos.y)
                let center2D = holePositions.reduce(SIMD2<Float>.zero, +) / Float(max(1, holePositions.count))
                let targetSide = simd_dot(targetPos2D - center2D, SIMD2<Float>(1, 0)) > 0
                for otherEi in 0..<2 {
                    let otherHole = otherEi == 0 ? (sim.bands[otherRi].pinStart ?? -1) : (sim.bands[otherRi].pinEnd ?? -1)
                    guard otherHole >= 0, otherHole < holePositions.count else { continue }
                    let otherSide = simd_dot(holePositions[otherHole] - center2D, SIMD2<Float>(1, 0)) > 0
                    if otherSide == targetSide {
                        braidIsLower[otherRi][otherEi] = true
                    }
                }

                braidDragPhase = .idle
                braidDragAction = nil
            }
        }

        simulator?.update(deltaTime: deltaTime)

        let isDragging = dragState != nil || simulator?.dragInfo != nil || simulator?.hasLowerAnimations == true
        if isDragging, let friction = simulator?.consumeAndResetFriction() {
            frictionSound.update(intensity: friction.intensity, speed: friction.speed)
        } else {
            _ = simulator?.consumeAndResetFriction()
            frictionSound.fadeOut()
        }

        let flowStep = levelFlow.update(deltaTime: deltaTime)
        if flowStep.shouldLoadNextLevel {
            let nextId = currentLevelId + 1
            Self.logger.info("Level \(self.currentLevelId) completed! Loading level \(nextId)...")
            loadLevel(levelId: nextId)
        }
        if flowStep.shouldRunSettleCheck {
            if dragState != nil || simulator?.dragInfo != nil || simulator?.hasLowerAnimations == true {
                levelFlow.scheduleSettleCheck()
            } else {
                PhysicsProfiler.shared.measure(.winCheck) { removeUntangledRopes() }
            }
        }

        PhysicsProfiler.shared.measure(.meshBuild) { updateRopeMesh() }

        // Tension mode: update weight render state and check win
        if isTensionMode {
            updateWeightRenderState()
            if !tensionLevelCompleted, let sim = simulator, sim.allWeightsSettled {
                checkTensionModeComplete()
            }
        }

        // Rail mode: update cart render state and check win
        if isRailMode {
            updateCartRenderState()
            if !railLevelCompleted, let sim = simulator, sim.allCartsSettled {
                checkRailModeComplete()
            }
        }

        // Braid mode: check win after each move settles
        if isBraidMode && !braidLevelCompleted {
            if dragState == nil && simulator?.dragInfo == nil && simulator?.hasLowerAnimations != true {
                checkBraidModeComplete()
            }
        }
    }

    /// Compute the next braid drag using ACTUAL particle positions from the simulator.
    /// Finds a LOWER end, then finds the nearest hole where the drag path crosses
    /// the other rope's actual particle segments in 2D projection.
    func computeNextBraidDrag() -> VerletSimulator.LevelAction? {
        guard let sim = simulator else { return nil }
        guard sim.bands.count >= 2 else { return nil }

        // Find a LOWER end to drag
        var ri = -1, ei = -1
        for tryRi in 0..<2 {
            for tryEi in 0..<2 {
                guard tryRi < braidIsLower.count, tryEi < braidIsLower[tryRi].count else { continue }
                if braidIsLower[tryRi][tryEi] {
                    ri = tryRi; ei = tryEi; break
                }
            }
            if ri >= 0 { break }
        }
        guard ri >= 0 else { return nil }

        let otherRi = 1 - ri
        guard sim.bands.indices.contains(ri), sim.bands.indices.contains(otherRi) else { return nil }

        // Get actual particle positions of the OTHER rope (2D projection)
        let otherBand = sim.bands[otherRi]
        let otherPositions = otherBand.positions // [SIMD3<Float>]

        // Current end position
        let pidx = ei == 0 ? 0 : sim.bands[ri].positions.count - 1
        let currentPos = SIMD2<Float>(sim.bands[ri].positions[pidx].x, sim.bands[ri].positions[pidx].y)
        let currentHole: Int
        if ei == 0 {
            currentHole = sim.bands[ri].pinStart ?? -1
        } else {
            currentHole = sim.bands[ri].pinEnd ?? -1
        }

        // Occupied holes
        var occupied = Set<Int>()
        for j in 0..<min(2, sim.bands.count) {
            if let ps = sim.bands[j].pinStart { occupied.insert(ps) }
            if let pe = sim.bands[j].pinEnd { occupied.insert(pe) }
        }
        if currentHole >= 0 { occupied.insert(currentHole) }

        // Find nearest hole where drag arc crosses the other rope's ACTUAL particles
        var bestHole = -1
        var bestDist: Float = .greatestFiniteMagnitude

        for (hi, hp) in holePositions.enumerated() where !occupied.contains(hi) {
            // Check if line segment (currentPos → hp) crosses any particle segment of otherRope
            var crosses = false
            let skip = 2 // skip segments near the pinned ends
            let maxSeg = max(skip, otherPositions.count - 1 - skip)
            for si in skip..<maxSeg {
                let a = SIMD2<Float>(otherPositions[si].x, otherPositions[si].y)
                let b = SIMD2<Float>(otherPositions[si + 1].x, otherPositions[si + 1].y)
                if segmentsCross2D(currentPos, hp, a, b) {
                    crosses = true; break
                }
            }
            guard crosses else { continue }
            let d = simd_length(hp - currentPos)
            if d < bestDist { bestDist = d; bestHole = hi }
        }

        guard bestHole >= 0 else { return nil }
        return VerletSimulator.LevelAction(type: .drag, ropeIndex: ri, endIndex: ei, holeIndex: bestHole)
    }

    private func segmentsCross2D(_ a0: SIMD2<Float>, _ a1: SIMD2<Float>,
                                  _ b0: SIMD2<Float>, _ b1: SIMD2<Float>) -> Bool {
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
