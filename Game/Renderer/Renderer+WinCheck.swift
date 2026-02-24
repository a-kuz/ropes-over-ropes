import simd

extension Renderer {
    func removeUntangledRopes() {
        var removed = true
        while removed {
            removed = false
            for ropeIndex in ropes.indices {
                if ropes[ropeIndex].startHole == -1 { continue }
                if isRopeUntangled(ropeIndex: ropeIndex) {
                    deactivateRope(ropeIndex: ropeIndex)
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
            let nextLevelId = currentLevelId + 1
            if LevelLoader.load(levelId: nextLevelId) != nil {
                Self.logger.info("Level \(self.currentLevelId) completed! Loading level \(nextLevelId)...")
                loadLevel(levelId: nextLevelId)
            } else {
                Self.logger.info("Level \(self.currentLevelId) completed! No more levels.")
            }
        }
    }

    private func deactivateRope(ropeIndex: Int) {
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
        if let sim = simulator, sim.bands.indices.contains(ropeIndex) {
            sim.bands[ropeIndex].active = false
            sim.bands[ropeIndex].pinStart = nil
            sim.bands[ropeIndex].pinEnd = nil
        }
    }

    /// A rope is untangled if it's not linked with any other active rope.
    /// Two ropes are linked if they cross in 2D projection and the
    /// over/under pattern alternates (one is sometimes above, sometimes below).
    private func isRopeUntangled(ropeIndex: Int) -> Bool {
        guard let sim = simulator else { return false }
        guard sim.bands.indices.contains(ropeIndex) else { return false }
        guard sim.bands[ropeIndex].active else { return false }

        for otherIndex in sim.bands.indices {
            if otherIndex == ropeIndex { continue }
            guard sim.bands[otherIndex].active, sim.bands[otherIndex].pinStart != nil else { continue }
            if areRopesLinked(sim: sim, a: ropeIndex, b: otherIndex) {
                return false
            }
        }
        return true
    }

    /// Two ropes are linked if their 2D crossings have alternating over/under z.
    private func areRopesLinked(sim: VerletSimulator, a: Int, b: Int) -> Bool {
        let bandA = sim.bands[a]
        let bandB = sim.bands[b]
        let nA = bandA.positions.count
        let nB = bandB.positions.count

        var hasOver = false
        var hasUnder = false

        for i in 0..<(nA - 1) {
            let a0xy = SIMD2<Float>(bandA.positions[i].x, bandA.positions[i].y)
            let a1xy = SIMD2<Float>(bandA.positions[i + 1].x, bandA.positions[i + 1].y)

            for j in 0..<(nB - 1) {
                let b0xy = SIMD2<Float>(bandB.positions[j].x, bandB.positions[j].y)
                let b1xy = SIMD2<Float>(bandB.positions[j + 1].x, bandB.positions[j + 1].y)

                guard let (tA, tB) = segmentIntersection2D(a0xy, a1xy, b0xy, b1xy) else { continue }

                let zA = bandA.positions[i].z * (1 - tA) + bandA.positions[i + 1].z * tA
                let zB = bandB.positions[j].z * (1 - tB) + bandB.positions[j + 1].z * tB

                if zA > zB + 1e-4 {
                    hasOver = true
                } else if zA < zB - 1e-4 {
                    hasUnder = true
                }

                if hasOver && hasUnder { return true }
            }
        }

        return false
    }

    /// Returns (tA, tB) parameters of intersection point, or nil if segments don't cross.
    private func segmentIntersection2D(
        _ a0: SIMD2<Float>, _ a1: SIMD2<Float>,
        _ b0: SIMD2<Float>, _ b1: SIMD2<Float>
    ) -> (Float, Float)? {
        let d1 = a1 - a0
        let d2 = b1 - b0
        let cross = d1.x * d2.y - d1.y * d2.x
        if abs(cross) < 1e-9 { return nil }
        let d = b0 - a0
        let t = (d.x * d2.y - d.y * d2.x) / cross
        let u = (d.x * d1.y - d.y * d1.x) / cross
        guard t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99 else { return nil }
        return (t, u)
    }
}
