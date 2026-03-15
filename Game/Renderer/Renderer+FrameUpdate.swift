import Foundation
import simd

extension Renderer {
    func updateFrameSimulation(deltaTime: Float) {
        // Slow init visualization: animate drag actions step by step
        if slowDragPhase != .idle || !pendingInitDrags.isEmpty, let sim = simulator {
            for _ in 0..<3 { advanceSlowInit(sim: sim, dt: deltaTime) }
            updateRopeMesh()
            // When all drags finished, save snapshot and sync rope endpoints
            if slowDragPhase == .idle && pendingInitDrags.isEmpty {
                if let sim = simulator {
                    postInitBandsSnapshot = (levelId: currentLevelId, bands: sim.bands)
                }
                syncRopeEndpointsFromSimulator()
            }
            return
        }

        // Victory replay: reload level and start replay on next frame
        if victoryReplayPending {
            victoryReplayPending = false
            startVictoryReplay()
            return
        }

        // Waiting for last rope suck animation to finish before starting replay
        if victoryWaitingForSuck {
            let anySucking = simulator?.bands.contains { $0.fadeOut > 0 && $0.active } ?? false
            if !anySucking {
                victoryWaitingForSuck = false
                victoryOrbitSavedCamera = camera
                victoryOrbitActive = true
                victoryOrbitTime = 0
                victoryReplayPending = true
            }
        }

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

        // Auto-drag test (level 998)
        if autoDragActive || (currentLevelId == 998 && !autoDragActive && autoDragStep > 0), let sim = simulator {
            if autoDragActive {
                if autoDragStep == 0 {
                    sim.beginDrag(bandIndex: autoDragBand, endIndex: autoDragEndIdx, worldPosition: autoDragStart)
                    autoDragPrevCrossings = -1
                    Self.logger.info("[AUTO-DRAG] started band=\(self.autoDragBand) end=\(self.autoDragEndIdx)")
                }
                autoDragStep += 1
                let t = min(Float(autoDragStep) / Float(autoDragTotalSteps), 1.0)
                let pos = autoDragStart + (autoDragEnd - autoDragStart) * t
                sim.updateDrag(worldPosition: pos)

                if autoDragStep >= autoDragTotalSteps {
                    autoDragActive = false
                    Self.logger.info("[AUTO-DRAG] finished step=\(self.autoDragStep)")
                }
            }

            // Log crossing state every 60 frames and on change
            if currentLevelId == 998, sim.bands.count >= 2 {
                let crossings = autoDragCountCrossings(sim)
                let changed = crossings != autoDragPrevCrossings
                if changed || autoDragStep % 60 == 0 {
                    let n0 = sim.bands[0].positions.count
                    let n1 = sim.bands[1].positions.count
                    // Find nearest segment pairs in 2D
                    var nearLog = ""
                    let skip = 3
                    var minDist: Float = .infinity
                    var minPair = (-1, -1)
                    for si in skip..<max(skip, n0-1-skip) {
                        let a0 = sim.bands[0].positions[si]
                        let a1 = sim.bands[0].positions[si+1]
                        for sj in skip..<max(skip, n1-1-skip) {
                            let b0 = sim.bands[1].positions[sj]
                            let b1 = sim.bands[1].positions[sj+1]
                            let mA = (a0 + a1) * 0.5
                            let mB = (b0 + b1) * 0.5
                            let d = simd_length(SIMD2<Float>(mA.x-mB.x, mA.y-mB.y))
                            if d < minDist {
                                minDist = d
                                minPair = (si, sj)
                            }
                        }
                    }
                    if minPair.0 >= 0 {
                        let si = minPair.0, sj = minPair.1
                        let zA = (sim.bands[0].positions[si].z + sim.bands[0].positions[si+1].z) * 0.5
                        let zB = (sim.bands[1].positions[sj].z + sim.bands[1].positions[sj+1].z) * 0.5
                        nearLog = " nearPair=red[\(si)]blue[\(sj)] zA=\(String(format:"%.4f",zA)) zB=\(String(format:"%.4f",zB)) zDiff=\(String(format:"%.4f",zA-zB))"
                    }
                    let dragPos = autoDragStart + (autoDragEnd - autoDragStart) * min(Float(autoDragStep)/Float(autoDragTotalSteps), 1)
                    Self.logger.warning("[AUTO-DRAG] step=\(self.autoDragStep) cross=\(crossings)\(nearLog) dragZ=\(String(format:"%.4f", sim.bands[self.autoDragBand].positions.last?.z ?? 0)) dragXY=(\(String(format:"%.2f",dragPos.x)),\(String(format:"%.2f",dragPos.y)))")
                    autoDragPrevCrossings = crossings
                }
            }
        }

        if let sim = simulator, dragState != nil {
            let response = max(dragInputResponse, 0)
            let alpha = response > 0 ? min(1.0, deltaTime * response) : 1.0
            lastDragWorld += (dragWorld - lastDragWorld) * alpha
            sim.updateDrag(worldPosition: lastDragWorld)
        }

        if !physicsPaused {
            simulator?.update(deltaTime: deltaTime)
        }

        updateVictoryOrbit(deltaTime: deltaTime)
        updateVictoryReplay()

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
        if flowStep.shouldRunSettleCheck && !victoryReplayActive {
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

    // MARK: - Slow init visualization

    /// Advance slow init drag/swap animation. Each action is broken into phases:
    /// lift (3 steps) → traverse (12 steps) → lower (3 steps) → settle
    /// One physics sub-step per frame so the user can watch.
    func advanceSlowInit(sim: VerletSimulator, dt: Float) {
        let dragSteps = 4

        switch slowDragPhase {
        case .idle:
            guard !pendingInitDrags.isEmpty else { return }
            let action = pendingInitDrags.removeFirst()
            slowDragIsSwap = action.type == .swap

            // Setup band A
            let bi = action.ropeIndex
            let ei = action.endIndex
            slowDragBandIdx = bi
            slowDragParticleIdx = ei == 0 ? 0 : sim.bands[bi].positions.count - 1
            slowDragFromPos = sim.bands[bi].positions[slowDragParticleIdx]

            // Compute lift height above all ropes
            let ropeMaxZ = sim.maxRopeZ()

            if slowDragIsSwap {
                let bi2 = action.ropeIndex2
                let ei2 = action.endIndex2
                slowDragBandIdx2 = bi2
                slowDragParticleIdx2 = ei2 == 0 ? 0 : sim.bands[bi2].positions.count - 1
                slowDragFromPos2 = sim.bands[bi2].positions[slowDragParticleIdx2]

                slowDragToPos = slowDragFromPos2
                slowDragToPos2 = slowDragFromPos

                let boardMax = max(
                    sim.boardSurfaceZ(x: slowDragFromPos.x, y: slowDragFromPos.y),
                    sim.boardSurfaceZ(x: slowDragFromPos2.x, y: slowDragFromPos2.y)
                )
                let clearance = sim.liftHeight + sim.bands[bi].radius * 2
                let highLift = max(ropeMaxZ, boardMax) + clearance
                let lowLift = boardMax + sim.liftHeight * 0.4
                slowDragLiftFrom = SIMD3<Float>(slowDragFromPos.x, slowDragFromPos.y, highLift)
                slowDragLiftTo = SIMD3<Float>(slowDragToPos.x, slowDragToPos.y, highLift)
                slowDragLiftFrom2 = SIMD3<Float>(slowDragFromPos2.x, slowDragFromPos2.y, lowLift)
                slowDragLiftTo2 = SIMD3<Float>(slowDragToPos2.x, slowDragToPos2.y, lowLift)

                if ei == 0 { sim.bands[bi].pinStart = nil } else { sim.bands[bi].pinEnd = nil }
                if ei2 == 0 { sim.bands[bi2].pinStart = nil } else { sim.bands[bi2].pinEnd = nil }
            } else {
                slowDragToPos = sim.holePosition3D(action.holeIndex)
                let boardMax = max(
                    sim.boardSurfaceZ(x: slowDragFromPos.x, y: slowDragFromPos.y),
                    sim.boardSurfaceZ(x: slowDragToPos.x, y: slowDragToPos.y)
                )
                let clearance = sim.liftHeight + sim.bands[bi].radius * 2
                let liftZ = max(ropeMaxZ, boardMax) + clearance
                slowDragLiftFrom = SIMD3<Float>(slowDragFromPos.x, slowDragFromPos.y, liftZ)
                slowDragLiftTo = SIMD3<Float>(slowDragToPos.x, slowDragToPos.y, liftZ)
                if ei == 0 { sim.bands[bi].pinStart = nil } else { sim.bands[bi].pinEnd = nil }
            }

            slowDragStep = 1
            slowDragPhase = .lift

        case .lift:
            let t = Float(slowDragStep) / 6.0
            setSlowPos(sim, t: t, from: slowDragFromPos, to: slowDragLiftFrom, band: slowDragBandIdx, particle: slowDragParticleIdx)
            if slowDragIsSwap {
                setSlowPos(sim, t: t, from: slowDragFromPos2, to: slowDragLiftFrom2, band: slowDragBandIdx2, particle: slowDragParticleIdx2)
            }
            sim.doSteps(dragSteps, collide: true)
            slowDragStep += 1
            if slowDragStep > 6 { slowDragStep = 1; slowDragPhase = .traverse }

        case .traverse:
            let t = Float(slowDragStep) / 24.0
            setSlowPos(sim, t: t, from: slowDragLiftFrom, to: slowDragLiftTo, band: slowDragBandIdx, particle: slowDragParticleIdx)
            if slowDragIsSwap {
                setSlowPos(sim, t: t, from: slowDragLiftFrom2, to: slowDragLiftTo2, band: slowDragBandIdx2, particle: slowDragParticleIdx2)
            }
            sim.doSteps(dragSteps, collide: true)
            slowDragStep += 1
            if slowDragStep > 24 { slowDragStep = 1; slowDragPhase = .lower }

        case .lower:
            let t = Float(slowDragStep) / 6.0
            setSlowPos(sim, t: t, from: slowDragLiftTo, to: slowDragToPos, band: slowDragBandIdx, particle: slowDragParticleIdx)
            if slowDragIsSwap {
                setSlowPos(sim, t: t, from: slowDragLiftTo2, to: slowDragToPos2, band: slowDragBandIdx2, particle: slowDragParticleIdx2)
            }
            sim.doSteps(dragSteps, collide: true)
            slowDragStep += 1
            if slowDragStep > 6 {
                // Re-pin A
                repinSlowDrag(sim, band: slowDragBandIdx, particle: slowDragParticleIdx, toPos: slowDragToPos)
                if slowDragIsSwap {
                    // Re-pin B
                    repinSlowDrag(sim, band: slowDragBandIdx2, particle: slowDragParticleIdx2, toPos: slowDragToPos2)
                }
                slowDragSettleCount = 0
                slowDragPhase = .settle
            }

        case .settle:
            sim.doSteps(1, collide: true)
            slowDragSettleCount += 1
            if slowDragSettleCount >= sim.settleSteps {
                slowDragPhase = .idle
            }
        }
    }

    private func setSlowPos(_ sim: VerletSimulator, t: Float, from: SIMD3<Float>, to: SIMD3<Float>, band: Int, particle: Int) {
        let pos = from + (to - from) * t
        sim.bands[band].positions[particle] = pos
        sim.bands[band].previousPositions[particle] = pos
    }

    private func repinSlowDrag(_ sim: VerletSimulator, band: Int, particle: Int, toPos: SIMD3<Float>) {
        let hole = findHoleIndex(near: toPos, sim: sim)
        if particle == 0 {
            sim.bands[band].pinStart = hole
        } else {
            sim.bands[band].pinEnd = hole
        }
        let hp = sim.holePosition3D(hole)
        sim.bands[band].positions[particle] = hp
        sim.bands[band].previousPositions[particle] = hp
    }

    private func findHoleIndex(near pos: SIMD3<Float>, sim: VerletSimulator) -> Int {
        var bestIdx = 0
        var bestDist: Float = .greatestFiniteMagnitude
        for i in 0..<sim.holePositions.count {
            let hp = sim.holePosition3D(i)
            let d = simd_length(hp - pos)
            if d < bestDist { bestDist = d; bestIdx = i }
        }
        return bestIdx
    }

    /// Sync rope endpoints and holeOccupied from simulator state (after slow init completes)
    func syncRopeEndpointsFromSimulator() {
        guard let sim = simulator else { return }
        let holeCount = holePositions.count
        holeOccupied = Array(repeating: false, count: holeCount)
        for ropeIndex in ropes.indices {
            guard sim.bands.indices.contains(ropeIndex) else { continue }
            let band = sim.bands[ropeIndex]
            if let pinStart = band.pinStart {
                if pinStart >= 0 { ropes[ropeIndex].startHole = pinStart }
            }
            if let pinEnd = band.pinEnd {
                if pinEnd >= 0 { ropes[ropeIndex].endHole = pinEnd }
            }
            let s = ropes[ropeIndex].startHole
            let e = ropes[ropeIndex].endHole
            if holeOccupied.indices.contains(s) { holeOccupied[s] = true }
            if holeOccupied.indices.contains(e) { holeOccupied[e] = true }
        }
    }
}
