import Foundation

struct RendererLevelFlowStep {
    var shouldLoadNextLevel: Bool
    var shouldRunSettleCheck: Bool
}

final class RendererLevelFlowCoordinator {
    private(set) var settleCheckTimer: Float?
    private(set) var nextLevelTimer: Float?
    let settleCheckDelay: Float

    init(settleCheckDelay: Float) {
        self.settleCheckDelay = settleCheckDelay
    }

    func scheduleSettleCheck() {
        settleCheckTimer = settleCheckDelay
    }

    func cancelSettleCheck() {
        settleCheckTimer = nil
    }

    func clearAll() {
        settleCheckTimer = nil
        nextLevelTimer = nil
    }

    func update(deltaTime: Float) -> RendererLevelFlowStep {
        var shouldLoadNextLevel = false
        var shouldRunSettleCheck = false

        if let timer = nextLevelTimer {
            let remaining = timer - deltaTime
            if remaining <= 0 {
                nextLevelTimer = nil
                shouldLoadNextLevel = true
            } else {
                nextLevelTimer = remaining
            }
        }

        if let timer = settleCheckTimer {
            let remaining = timer - deltaTime
            if remaining <= 0 {
                settleCheckTimer = nil
                shouldRunSettleCheck = true
            } else {
                settleCheckTimer = remaining
            }
        }

        return RendererLevelFlowStep(
            shouldLoadNextLevel: shouldLoadNextLevel,
            shouldRunSettleCheck: shouldRunSettleCheck
        )
    }
}
