import simd

extension VerletSimulator {
    func consumeAndResetFriction() -> (intensity: Float, speed: Float, position: SIMD3<Float>)? {
        guard frictionSampleCount > 0 else { return nil }
        let n = Float(frictionSampleCount)
        let result = (
            intensity: frictionAccumulator / n,
            speed: frictionSpeedAccumulator / n,
            position: frictionPositionAccumulator / n
        )
        frictionAccumulator = 0
        frictionSpeedAccumulator = 0
        frictionPositionAccumulator = .zero
        frictionSampleCount = 0
        return result
    }

    func takeSnapshot() -> Snapshot {
        Snapshot(bands: bands.map { b in
            BandSnapshot(
                positions: b.positions,
                previousPositions: b.previousPositions,
                twistAngles: b.twistAngles,
                previousTwistAngles: b.previousTwistAngles,
                segmentLength: b.segmentLength,
                pinStart: b.pinStart,
                pinEnd: b.pinEnd,
                active: b.active,
                fadeOut: b.fadeOut,
                suckHole: b.suckHole,
                suckTailHole: b.suckTailHole,
                suckFromEnd: b.suckFromEnd,
                suckConsumed: b.suckConsumed,
                suckSegLengths: b.suckSegLengths,
                suckOrigPositions: b.suckOrigPositions
            )
        })
    }

    func restoreSnapshot(_ snapshot: Snapshot) {
        for i in bands.indices where i < snapshot.bands.count {
            let s = snapshot.bands[i]
            bands[i].positions = s.positions
            bands[i].previousPositions = s.previousPositions
            bands[i].twistAngles = s.twistAngles
            bands[i].previousTwistAngles = s.previousTwistAngles
            bands[i].segmentLength = s.segmentLength
            bands[i].pinStart = s.pinStart
            bands[i].pinEnd = s.pinEnd
            bands[i].active = s.active
            bands[i].fadeOut = s.fadeOut
            bands[i].suckHole = s.suckHole
            bands[i].suckTailHole = s.suckTailHole
            bands[i].suckFromEnd = s.suckFromEnd
            bands[i].suckConsumed = s.suckConsumed
            bands[i].suckSegLengths = s.suckSegLengths
            bands[i].suckOrigPositions = s.suckOrigPositions
        }
        dragInfo = nil
        dragStartPos = nil
        dragTargetPos = nil
        lowerAnimations.removeAll()
        currentTension = ropeTension
        wakeUp()
    }
}
