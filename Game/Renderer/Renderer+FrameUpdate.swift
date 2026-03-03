import Foundation

extension Renderer {
    func updateFrameSimulation(deltaTime: Float) {
        simulator?.update(deltaTime: deltaTime)

        let isDragging = dragState != nil || simulator?.lowerAnimation != nil
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
            if simulator?.lowerAnimation != nil {
                levelFlow.scheduleSettleCheck()
            } else {
                PhysicsProfiler.shared.measure(.winCheck) { removeUntangledRopes() }
            }
        }

        PhysicsProfiler.shared.measure(.meshBuild) { updateRopeMesh() }
    }
}
