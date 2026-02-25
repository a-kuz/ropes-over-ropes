import simd
import Foundation

extension Renderer {
    func removeUntangledRopes() {
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
                    removed = true
                    break
                }
            }
        }

        checkLevelComplete()
    }

    private func checkLevelComplete() {
        let allDone = ropes.allSatisfy { $0.startHole == -1 }
        if allDone {
            Haptics.success()
            onLevelComplete?()
            // Delay next level so victory animation shows
            nextLevelTimer = 1.2
        }
    }

    /// Start fade-out animation. Actual deactivation happens when fadeOut reaches 1.0.
    private func startFadeOut(ropeIndex: Int) {
        guard let sim = simulator, sim.bands.indices.contains(ropeIndex) else { return }
        sim.bands[ropeIndex].fadeOut = 0.001  // trigger fade

        // Free holes immediately so other ropes can use them
        let startHoleIndex = ropes[ropeIndex].startHole
        let endHoleIndex = ropes[ropeIndex].endHole
        if startHoleIndex >= 0 && startHoleIndex < holeOccupied.count {
            holeOccupied[startHoleIndex] = false
        }
        if endHoleIndex >= 0 && endHoleIndex < holeOccupied.count {
            holeOccupied[endHoleIndex] = false
        }
        ropes[ropeIndex].startHole = -1
        ropes[ropeIndex].endHole = -1
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
}
