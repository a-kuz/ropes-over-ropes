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
        guard let topology else { return }
        let hasNoHooks = topology.hooks.isEmpty
        if hasNoHooks {
            let nextLevelId = currentLevelId + 1
            if LevelLoader.load(levelId: nextLevelId) != nil {
                Self.logger.info("Level \(self.currentLevelId) completed! No hooks remaining. Loading level \(nextLevelId)...")
                loadLevel(levelId: nextLevelId)
            } else {
                Self.logger.info("Level \(self.currentLevelId) completed! No hooks remaining. No more levels available.")
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
        topology?.deactivateRope(ropeIndex: ropeIndex)
    }

    private func isRopeUntangled(ropeIndex: Int) -> Bool {
        guard let topology else { return false }
        guard topology.ropes.indices.contains(ropeIndex) else { return false }
        guard topology.ropes[ropeIndex].active else { return false }

        for (_, hook) in topology.hooks {
            if hook.ropeA == ropeIndex || hook.ropeB == ropeIndex {
                return false
            }
        }

        return true
    }
}
