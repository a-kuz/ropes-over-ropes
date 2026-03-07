import Foundation

extension Renderer {
    func updateFrameSimulation(deltaTime: Float) {
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
    }
}
