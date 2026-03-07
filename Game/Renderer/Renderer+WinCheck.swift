import simd
import Foundation

extension Renderer {
    func removeUntangledRopes() {
        if isTensionMode || isRailMode { return }  // No rope removal in tension/rail mode
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
            onLevelComplete?()
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
        return t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99
    }

    func checkTensionModeComplete() {
        guard isTensionMode, !tensionLevelCompleted else { return }
        guard let sim = simulator, sim.allWeightsSettled else { return }
        tensionLevelCompleted = true
        Self.logger.info("[TENSION] All weights settled — level complete!")
        Haptics.success()
        onLevelComplete?()
    }

    func checkRailModeComplete() {
        guard isRailMode, !railLevelCompleted else { return }
        guard let sim = simulator, sim.allCartsSettled else { return }
        railLevelCompleted = true
        Self.logger.info("[RAIL] All carts at stations — level complete!")
        Haptics.success()
        onLevelComplete?()
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
