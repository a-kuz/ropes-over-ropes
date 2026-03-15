import simd
import Foundation

extension Renderer {
    func removeUntangledRopes() {
        if isTensionMode || isRailMode || isBraidMode { return }
        var removed = true
        while removed {
            removed = false
            for ropeIndex in ropes.indices {
                if ropes[ropeIndex].startHole == -1 { continue }
                guard let sim = simulator, sim.bands.indices.contains(ropeIndex),
                      sim.bands[ropeIndex].fadeOut == 0 else { continue }
                if isRopeUntangled(ropeIndex: ropeIndex) {
                    Self.logger.info("Rope \(ropeIndex) is untangled — fading out")
                    startFadeOut(ropeIndex: ropeIndex)
                    Haptics.medium()
                    SoundPlayer.playRopeVanish()
                    removed = true
                    break
                }
            }
        }

        checkLevelComplete()
    }

    private func checkLevelComplete() {
        let allDone = ropes.allSatisfy { $0.startHole == -1 }
        let allBandsInactive = simulator?.bands.allSatisfy { !$0.active } ?? false
        let occupiedHoles = holeOccupied.enumerated().compactMap { index, occupied in occupied ? String(index) : nil }

        if allBandsInactive && !allDone {
            Self.logger.error("[WIN-DIAG] all bands inactive but ropes not cleared ropes=\(self.winDiagRopeStates()) occupied=[\(occupiedHoles.joined(separator: ","))]")
        }

        if allDone && !occupiedHoles.isEmpty {
            Self.logger.error("[WIN-DIAG] ropes cleared but occupied holes remain occupied=[\(occupiedHoles.joined(separator: ","))] ropes=\(self.winDiagRopeStates())")
        }

        if allDone {
            Self.logger.info("[WIN-DIAG] level complete ropes=\(self.winDiagRopeStates()) occupied=[\(occupiedHoles.joined(separator: ","))]")
            Haptics.success()
            // Show victory UI immediately — replay/orbit start after suck animation finishes
            onLevelComplete?()
            startVictoryOrbit()
        }
    }

    /// Start fade-out animation. Actual deactivation happens when fadeOut reaches 1.0.
    private func startFadeOut(ropeIndex: Int) {
        guard let sim = simulator, sim.bands.indices.contains(ropeIndex) else { return }

        let pinS = sim.bands[ropeIndex].pinStart
        let pinE = sim.bands[ropeIndex].pinEnd
        let holeS = pinS ?? (ropes[ropeIndex].startHole >= 0 ? ropes[ropeIndex].startHole : nil)
        let holeE = pinE ?? (ropes[ropeIndex].endHole >= 0 ? ropes[ropeIndex].endHole : nil)

        let suckTarget: Int
        let suckFromEnd: Int
        if let s = holeS {
            suckTarget = s
            suckFromEnd = 1
        } else if let e = holeE {
            suckTarget = e
            suckFromEnd = 0
        } else {
            suckTarget = 0
            suckFromEnd = 1
        }

        let tailHole = suckFromEnd == 1 ? holeE : holeS

        sim.bands[ropeIndex].suckHole = suckTarget
        sim.bands[ropeIndex].suckTailHole = tailHole
        sim.bands[ropeIndex].suckFromEnd = suckFromEnd
        sim.bands[ropeIndex].suckConsumed = 0
        sim.bands[ropeIndex].suckFrame = 0

        var suckSegs = ContiguousArray<Float>()
        let positions = sim.bands[ropeIndex].positions
        let pc = positions.count
        suckSegs.reserveCapacity(max(0, pc - 1))
        for k in 0..<(pc - 1) {
            suckSegs.append(simd_length(positions[k + 1] - positions[k]))
        }
        sim.bands[ropeIndex].suckSegLengths = suckSegs
        sim.bands[ropeIndex].suckOrigPositions = positions

        // Record vanish moment for replay (skip during active replay to avoid overwriting)
        if !victoryReplayActive {
            replayVanishRecords[ropeIndex] = moveHistory.count
        }

        sim.bands[ropeIndex].fadeOut = 0.001
        sim.bands[ropeIndex].pinStart = nil
        sim.bands[ropeIndex].pinEnd = nil

        let band = sim.bands[ropeIndex]
        let totalArc = band.suckSegLengths.reduce(0, +)
        let estDuration = totalArc / max(VerletSimulator.Band.fadeOutSpeed * band.segmentLength, 1e-9)
        Self.logger.info("[SUCK-START] rope=\(ropeIndex) n=\(band.positions.count) totalArc=\(String(format:"%.3f", totalArc)) estDuration=\(String(format:"%.1f", estDuration))s")

        let sh = ropes[ropeIndex].startHole
        let eh = ropes[ropeIndex].endHole
        Self.logger.warning("[WIN-DIAG] startFadeOut rope=\(ropeIndex) ropeHoles=(\(sh),\(eh)) simPins=(\(pinS.map(String.init) ?? "nil"),\(pinE.map(String.init) ?? "nil")) suckTarget=\(suckTarget) tailHole=\(tailHole.map(String.init) ?? "nil") occupiedBefore=\(self.winDiagOccupiedHoles())")
        if sh >= 0 && sh < holeOccupied.count { holeOccupied[sh] = false }
        if eh >= 0 && eh < holeOccupied.count { holeOccupied[eh] = false }
        ropes[ropeIndex].startHole = -1
        ropes[ropeIndex].endHole = -1
        Self.logger.warning("[WIN-DIAG] startFadeOut cleared rope=\(ropeIndex) occupiedAfter=\(self.winDiagOccupiedHoles()) ropes=\(self.winDiagRopeStates())")
    }

    // MARK: - Victory Camera Orbit + Replay

    func startVictoryOrbit() {
        guard !victoryReplayActive, !victoryReplayPending, !victoryWaitingForSuck else { return }

        if isTensionMode || isRailMode || isBraidMode || moveHistory.isEmpty {
            // No replay — just start orbit (UI already shown)
            victoryOrbitSavedCamera = camera
            victoryOrbitActive = true
            victoryOrbitTime = 0
            return
        }

        // Save moves buffer now (moveHistory will be cleared when level reloads)
        victoryReplayMovesBuffer = moveHistory
        // Wait for last rope suck animation to complete, then start replay
        victoryWaitingForSuck = true
    }

    /// Called at the start of next frame after suck animation completes
    func startVictoryReplay() {
        let moves = victoryReplayMovesBuffer
        victoryReplayMovesBuffer = []
        let vanishRecords = replayVanishRecords

        // Reload level (resets everything including moveHistory and replayVanishRecords)
        loadLevel(levelId: currentLevelId)

        // Set up replay state AFTER loadLevel
        victoryReplayMoves = moves
        victoryReplayVanishAtMove = vanishRecords
        victoryReplayActive = true
        victoryReplayMoveIndex = 0
        victoryReplayPhase = .pause
        victoryReplayStep = 0

        // Start cinematic camera orbit
        victoryOrbitSavedCamera = camera
        victoryOrbitActive = true
        victoryOrbitTime = 0
    }

    func updateVictoryOrbit(deltaTime: Float) {
        guard victoryOrbitActive else { return }
        victoryOrbitTime += deltaTime

        // Camera stays exactly as-is — no rotation, no tilt change, no zoom
        // Non-replay modes: stop after fixed duration (UI already shown)
        if !victoryReplayActive && victoryOrbitTime >= victoryOrbitDuration {
            victoryOrbitActive = false
        }
    }

    func updateVictoryReplay() {
        guard victoryReplayActive, let sim = simulator else { return }
        guard victoryReplayMoveIndex < victoryReplayMoves.count else {
            finishVictoryReplay()
            return
        }

        let move = victoryReplayMoves[victoryReplayMoveIndex]

        switch victoryReplayPhase {
        case .pause:
            victoryReplayStep += 1
            if victoryReplayStep >= replayPauseFrames {
                // Begin this move: unpin the end, compute positions
                guard sim.bands.indices.contains(move.ropeIndex) else {
                    victoryReplayMoveIndex += 1
                    victoryReplayStep = 0
                    victoryReplayPhase = .pause
                    return
                }
                let bi = move.ropeIndex
                let ei = move.endIndex
                let pidx = ei == 0 ? 0 : sim.bands[bi].positions.count - 1
                victoryReplayParticleIdx = pidx

                victoryReplayFromPos = sim.bands[bi].positions[pidx]
                victoryReplayToPos = sim.holePosition3D(move.toHole)

                let fromElev = sim.boardSurfaceZ(x: victoryReplayFromPos.x, y: victoryReplayFromPos.y)
                let toElev = sim.holeSurfaceZ(move.toHole)
                let maxElev = max(fromElev, toElev)
                victoryReplayLiftFrom = SIMD3<Float>(victoryReplayFromPos.x, victoryReplayFromPos.y, maxElev + sim.liftHeight)
                victoryReplayLiftTo = SIMD3<Float>(victoryReplayToPos.x, victoryReplayToPos.y, maxElev + sim.liftHeight)

                // Unpin
                if ei == 0 { sim.bands[bi].pinStart = nil } else { sim.bands[bi].pinEnd = nil }
                // Free original hole
                if holeOccupied.indices.contains(move.fromHole) { holeOccupied[move.fromHole] = false }

                victoryReplayStep = 0
                victoryReplayPhase = .lift
            }

        case .lift:
            victoryReplayStep += 1
            let t = Float(victoryReplayStep) / Float(replayLiftFrames)
            let bi = move.ropeIndex
            let pidx = victoryReplayParticleIdx
            sim.bands[bi].positions[pidx] = victoryReplayFromPos + (victoryReplayLiftFrom - victoryReplayFromPos) * t
            sim.bands[bi].previousPositions[pidx] = sim.bands[bi].positions[pidx]
            sim.doSteps(4, collide: true)
            if victoryReplayStep >= replayLiftFrames {
                victoryReplayStep = 0
                victoryReplayPhase = .traverse
            }

        case .traverse:
            victoryReplayStep += 1
            let t = Float(victoryReplayStep) / Float(replayTraverseFrames)
            let smooth = t * t * (3 - 2 * t) // ease in-out
            let bi = move.ropeIndex
            let pidx = victoryReplayParticleIdx
            sim.bands[bi].positions[pidx] = victoryReplayLiftFrom + (victoryReplayLiftTo - victoryReplayLiftFrom) * smooth
            sim.bands[bi].previousPositions[pidx] = sim.bands[bi].positions[pidx]
            sim.doSteps(4, collide: true)
            if victoryReplayStep >= replayTraverseFrames {
                victoryReplayStep = 0
                victoryReplayPhase = .lower
            }

        case .lower:
            victoryReplayStep += 1
            let t = Float(victoryReplayStep) / Float(replayLowerFrames)
            let bi = move.ropeIndex
            let pidx = victoryReplayParticleIdx
            sim.bands[bi].positions[pidx] = victoryReplayLiftTo + (victoryReplayToPos - victoryReplayLiftTo) * t
            sim.bands[bi].previousPositions[pidx] = sim.bands[bi].positions[pidx]
            sim.doSteps(4, collide: true)
            if victoryReplayStep >= replayLowerFrames {
                victoryReplayStep = 0
                victoryReplayPhase = .settle
            }

        case .settle:
            victoryReplayStep += 1
            let bi = move.ropeIndex
            let ei = move.endIndex
            let pidx = victoryReplayParticleIdx
            // Pin at target
            if ei == 0 { sim.bands[bi].pinStart = move.toHole } else { sim.bands[bi].pinEnd = move.toHole }
            sim.bands[bi].positions[pidx] = victoryReplayToPos
            sim.bands[bi].previousPositions[pidx] = victoryReplayToPos
            sim.doSteps(4, collide: true)
            // Update rope endpoints & hole state
            if ei == 0 { ropes[bi].startHole = move.toHole } else { ropes[bi].endHole = move.toHole }
            if holeOccupied.indices.contains(move.toHole) { holeOccupied[move.toHole] = true }

            if victoryReplayStep >= replaySettleFrames {
                victoryReplayMoveIndex += 1
                // Trigger rope vanish if it happened after this many moves
                for (ropeIdx, vanishAt) in victoryReplayVanishAtMove
                    where vanishAt == victoryReplayMoveIndex {
                    startFadeOut(ropeIndex: ropeIdx)
                }
                victoryReplayStep = 0
                victoryReplayPhase = .pause
            }

        case .done:
            break
        }
    }

    private func finishVictoryReplay() {
        victoryReplayActive = false
        victoryOrbitActive = false
        // onLevelComplete was already called at victory detection — don't call again
    }

    /// A rope is untangled if it has ZERO 2D crossings with any other active rope.
    /// Skip segments near holes to avoid false positives from ropes converging at pins.
    private func isRopeUntangled(ropeIndex: Int) -> Bool {
        guard let sim = simulator else { return false }
        guard sim.bands.indices.contains(ropeIndex) else { return false }
        guard sim.bands[ropeIndex].active else { return false }

        let bandA = sim.bands[ropeIndex]
        let nA = bandA.positions.count
        let skip = 3  // skip near-hole segments

        for otherIndex in sim.bands.indices {
            if otherIndex == ropeIndex { continue }
            guard sim.bands[otherIndex].active && sim.bands[otherIndex].fadeOut == 0 else { continue }

            let bandB = sim.bands[otherIndex]
            let nB = bandB.positions.count

            let startA = skip
            let endA = max(startA, nA - 1 - skip)
            let startB = skip
            let endB = max(startB, nB - 1 - skip)

            for i in startA..<endA {
                let a0 = SIMD2<Float>(bandA.positions[i].x, bandA.positions[i].y)
                let a1 = SIMD2<Float>(bandA.positions[i + 1].x, bandA.positions[i + 1].y)

                for j in startB..<endB {
                    let b0 = SIMD2<Float>(bandB.positions[j].x, bandB.positions[j].y)
                    let b1 = SIMD2<Float>(bandB.positions[j + 1].x, bandB.positions[j + 1].y)

                    if segmentsCross2D(a0, a1, b0, b1) {
                        return false
                    }
                }
            }
        }
        return true
    }

    /// Fast 2D segment intersection test.
    private func segmentsCross2D(
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

    func checkTensionModeComplete() {
        guard isTensionMode, !tensionLevelCompleted else { return }
        guard let sim = simulator, sim.allWeightsSettled else { return }
        tensionLevelCompleted = true
        Self.logger.info("[TENSION] All weights settled — level complete!")
        Haptics.success()
        onLevelComplete?()
        startVictoryOrbit()
    }

    func checkRailModeComplete() {
        guard isRailMode, !railLevelCompleted else { return }
        guard let sim = simulator, sim.allCartsSettled else { return }
        railLevelCompleted = true
        Self.logger.info("[RAIL] All carts at stations — level complete!")
        Haptics.success()
        onLevelComplete?()
        startVictoryOrbit()
    }

    func checkBraidModeComplete() {
        guard isBraidMode, !braidLevelCompleted else { return }
        guard !braidTargets.isEmpty else { return }

        // Check 1: each strand's bottom end is at its target hole
        for ropeIndex in ropes.indices {
            guard ropeIndex < braidTargets.count else { return }
            let targetHole = braidTargets[ropeIndex]
            let currentHole = ropes[ropeIndex].endHole
            if currentHole != targetHole {
                return // Not yet — strand not in target position
            }
        }

        // Check 2: minimum crossing count (prevents trivial direct placement)
        if braidMinCrossings > 0 {
            let crossingCount = countTotalCrossings()
            if crossingCount < braidMinCrossings {
                Self.logger.info("[BRAID] Arrangement correct but only \(crossingCount)/\(self.braidMinCrossings) crossings")
                return
            }
        }

        braidLevelCompleted = true
        Self.logger.info("[BRAID] Braid complete!")
        Haptics.success()
        onLevelComplete?()
        startVictoryOrbit()
    }

    /// Count total 2D crossings between all active rope pairs (for braid mode validation)
    private func countTotalCrossings() -> Int {
        guard let sim = simulator else { return 0 }
        var total = 0
        let skip = 3

        for i in 0..<sim.bands.count {
            guard sim.bands[i].active else { continue }
            let nA = sim.bands[i].positions.count
            let startA = skip
            let endA = max(startA, nA - 1 - skip)

            for j in (i+1)..<sim.bands.count {
                guard sim.bands[j].active else { continue }
                let nB = sim.bands[j].positions.count
                let startB = skip
                let endB = max(startB, nB - 1 - skip)
                var pairCrossings = 0

                for si in startA..<endA {
                    let a0 = SIMD2<Float>(sim.bands[i].positions[si].x, sim.bands[i].positions[si].y)
                    let a1 = SIMD2<Float>(sim.bands[i].positions[si + 1].x, sim.bands[i].positions[si + 1].y)

                    for sj in startB..<endB {
                        let b0 = SIMD2<Float>(sim.bands[j].positions[sj].x, sim.bands[j].positions[sj].y)
                        let b1 = SIMD2<Float>(sim.bands[j].positions[sj + 1].x, sim.bands[j].positions[sj + 1].y)

                        if segmentsCross2D(a0, a1, b0, b1) {
                            pairCrossings += 1
                        }
                    }
                }
                if pairCrossings > 0 { total += 1 }
            }
        }
        return total
    }

    private func winDiagOccupiedHoles() -> String {
        let occupied = holeOccupied.enumerated().compactMap { index, value in value ? String(index) : nil }
        return "[\(occupied.joined(separator: ","))]"
    }

    private func winDiagRopeStates() -> String {
        let sim = simulator
        return ropes.enumerated().map { index, rope in
            let band = sim?.bands[safe: index]
            let active = band?.active == true ? "1" : "0"
            let fadeOut = band.map { String(format: "%.3f", $0.fadeOut) } ?? "nil"
            let pinStart = band?.pinStart.map(String.init) ?? "nil"
            let pinEnd = band?.pinEnd.map(String.init) ?? "nil"
            let suckHole = band?.suckHole.map(String.init) ?? "nil"
            return "r\(index){rope=(\(rope.startHole),\(rope.endHole)) active=\(active) fade=\(fadeOut) pins=(\(pinStart),\(pinEnd)) suck=\(suckHole)}"
        }.joined(separator: " ")
    }
}
