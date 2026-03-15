import simd
import QuartzCore

extension VerletSimulator {
    func beginDrag(bandIndex: Int, endIndex: Int, worldPosition: SIMD2<Float>) {
        guard bands.indices.contains(bandIndex) else { return }
        wakeUp()
        let lowerAnimationKey = LowerAnimationKey(bandIndex: bandIndex, endIndex: endIndex)
        let originalHole: Int
        if endIndex == 0 {
            originalHole = bands[bandIndex].pinStart
                ?? lowerAnimations[lowerAnimationKey]?.targetHole
                ?? 0
            bands[bandIndex].pinStart = nil
        } else {
            originalHole = bands[bandIndex].pinEnd
                ?? lowerAnimations[lowerAnimationKey]?.targetHole
                ?? 0
            bands[bandIndex].pinEnd = nil
        }
        dragInfo = DragInfo(bandIndex: bandIndex, endIndex: endIndex, originalHoleIndex: originalHole)

        lowerAnimations.removeValue(forKey: lowerAnimationKey)

        let elev: Float
        if originalHole >= 0 {
            elev = holeSurfaceZ(originalHole)
        } else {
            elev = boardSurfaceZ(x: worldPosition.x, y: worldPosition.y)
        }
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        let currentPos = bands[bandIndex].positions[idx]
        let liftPos = SIMD3<Float>(worldPosition.x, worldPosition.y, elev + liftHeight)
        bands[bandIndex].previousPositions[idx] = currentPos
        dragStartPos = currentPos
        dragTargetPos = liftPos
        dragPickupElapsed = 0
    }

    func updateDrag(worldPosition: SIMD2<Float>) {
        guard let drag = dragInfo else { return }
        let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
        dragStartPos = bands[drag.bandIndex].positions[idx]
        let surfZ = boardSurfaceZ(x: worldPosition.x, y: worldPosition.y)
        dragTargetPos = SIMD3<Float>(worldPosition.x, worldPosition.y, surfZ + liftHeight)
    }

    func endDrag(targetHoleIndex: Int) {
        guard let drag = dragInfo else { return }

        let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
        let currentPos = bands[drag.bandIndex].positions[idx]
        let holeXY = holePositions[targetHoleIndex]
        let holeElev = holeSurfaceZ(targetHoleIndex)
        let aboveHole = SIMD3<Float>(holeXY.x, holeXY.y, holeElev + liftHeight)
        let dist = simd_length(currentPos - aboveHole)
        let returnDuration = min(max(dist * 0.9, 0.25), 0.8)

        let lowerAnimationKey = LowerAnimationKey(bandIndex: drag.bandIndex, endIndex: drag.endIndex)
        lowerAnimations[lowerAnimationKey] = LowerAnimation(
            bandIndex: drag.bandIndex,
            endIndex: drag.endIndex,
            targetHole: targetHoleIndex,
            startPos: currentPos,
            returnPos: aboveHole,
            returnDuration: returnDuration
        )

        dragInfo = nil
        dragStartPos = nil
        dragTargetPos = nil
        dragPickupElapsed = .greatestFiniteMagnitude
    }

    /// Programmatically move a band end to a different hole with animation (used for braid swap)
    func swapEndToHole(bandIndex: Int, endIndex: Int, holeIndex: Int) {
        guard bands.indices.contains(bandIndex) else { return }
        wakeUp()

        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        let currentPos = bands[bandIndex].positions[idx]
        let holeXY = holePositions[holeIndex]
        let holeElev = holeSurfaceZ(holeIndex)
        let aboveHole = SIMD3<Float>(holeXY.x, holeXY.y, holeElev + liftHeight)
        let dist = simd_length(currentPos - aboveHole)
        let returnDuration = min(max(dist * 0.9, 0.25), 0.8)

        // Remove current pin
        if endIndex == 0 {
            bands[bandIndex].pinStart = nil
        } else {
            bands[bandIndex].pinEnd = nil
        }

        // Lift slightly to clear other strands
        let liftPos = SIMD3<Float>(currentPos.x, currentPos.y, currentPos.z + liftHeight * 0.5)
        bands[bandIndex].positions[idx] = liftPos
        bands[bandIndex].previousPositions[idx] = liftPos

        let key = LowerAnimationKey(bandIndex: bandIndex, endIndex: endIndex)
        lowerAnimations[key] = LowerAnimation(
            bandIndex: bandIndex,
            endIndex: endIndex,
            targetHole: holeIndex,
            startPos: liftPos,
            returnPos: aboveHole,
            returnDuration: returnDuration
        )
    }

    func endpointZ(bandIndex: Int, endIndex: Int) -> Float {
        guard bands.indices.contains(bandIndex) else { return 0 }
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        return bands[bandIndex].positions[idx].z
    }

    struct RopeConfig {
        let startHole: Int
        let endHole: Int
        let radius: Float
        var crossSection: CrossSection? = nil
    }

    struct LevelAction {
        enum ActionType: String { case pin, drag, swap }
        let type: ActionType
        let ropeIndex: Int
        let endIndex: Int
        let holeIndex: Int
        // For swap: second rope/end pair (they exchange holes)
        var ropeIndex2: Int = 0
        var endIndex2: Int = 0
    }

    struct WeightConfig {
        let position: SIMD2<Float>
        let mass: Float
        let radius: Float
        let targetPosition: SIMD2<Float>?
        let targetRadius: Float
    }

    func initializeWeights(_ configs: [WeightConfig]) {
        weights = configs.map { config in
            Weight(
                position: config.position,
                previousPosition: config.position,
                mass: config.mass,
                radius: config.radius,
                targetPosition: config.targetPosition,
                targetRadius: config.targetRadius
            )
        }
    }

    /// Fixed physics parameters used during level initialization (matched to Rust cross/).
    /// These ensure deterministic, collision-correct init regardless of user's runtime settings.
    private struct InitParams {
        static let gravity: Float = -6.3927
        static let damping: Float = 0.9470582
        static let constraintIterations: Int = 11
        static let settleSteps: Int = 5
        static let liftHeight: Float = 0.3
        static let ropeTension: Float = 0.88084733
        static let particleCount: Int = 65
        static let bendCompliance: Float = 0.00015530341
        static let bendVelocityCoupling: Float = 0.65
    }

    func initializeLevel(ropeConfigs: [RopeConfig], actions: [LevelAction], ropeParticles: [[SIMD3<Float>]]? = nil) {
        // Save user's runtime params
        let savedGravity = gravity
        let savedDamping = damping
        let savedConstraintIterations = constraintIterations
        let savedSettleSteps = settleSteps
        let savedLiftHeight = liftHeight
        let savedRopeTension = ropeTension
        let savedParticleCount = particleCount
        let savedBendCompliance = bendCompliance
        let savedBendVelocityCoupling = bendVelocityCoupling

        // Apply fixed init params (matched to Rust) for deterministic generation
        gravity = InitParams.gravity
        damping = InitParams.damping
        constraintIterations = InitParams.constraintIterations
        settleSteps = InitParams.settleSteps
        liftHeight = InitParams.liftHeight
        ropeTension = InitParams.ropeTension
        particleCount = InitParams.particleCount
        bendCompliance = InitParams.bendCompliance
        bendVelocityCoupling = InitParams.bendVelocityCoupling

        defer {
            // Restore user's runtime params after init
            gravity = savedGravity
            damping = savedDamping
            constraintIterations = savedConstraintIterations
            settleSteps = savedSettleSteps
            liftHeight = savedLiftHeight
            ropeTension = savedRopeTension
            particleCount = savedParticleCount
            bendCompliance = savedBendCompliance
            bendVelocityCoupling = savedBendVelocityCoupling
        }

        bands.removeAll()
        currentTension = ropeTension

        // Particle-based initialization: use explicit positions directly
        if let particles = ropeParticles, !particles.isEmpty {
            for (i, config) in ropeConfigs.enumerated() {
                guard i < particles.count, particles[i].count >= 2 else {
                    let _ = addBand(radius: config.radius, crossSection: config.crossSection, particleCount: particleCount)
                    pin(bandIndex: i, startHole: config.startHole, endHole: config.endHole)
                    continue
                }
                let pos = particles[i]
                let n = pos.count
                let bandIndex = addBand(radius: config.radius, crossSection: config.crossSection, particleCount: n)

                // Set positions from particle data
                for j in 0..<n {
                    bands[bandIndex].positions[j] = pos[j]
                }
                // Snap endpoints to hole positions
                bands[bandIndex].positions[0] = pinPosition3D(config.startHole)
                bands[bandIndex].positions[n - 1] = pinPosition3D(config.endHole)
                bands[bandIndex].previousPositions = bands[bandIndex].positions

                // Compute segment length from actual path
                var totalLength: Float = 0
                for j in 1..<n {
                    totalLength += simd_length(bands[bandIndex].positions[j] - bands[bandIndex].positions[j - 1])
                }
                bands[bandIndex].segmentLength = totalLength / Float(max(1, n - 1))
                bands[bandIndex].pinStart = config.startHole
                bands[bandIndex].pinEnd = config.endHole
                bands[bandIndex].active = true
            }
            // Small settle with collision to stabilize
            doSteps(settleSteps, collide: true)
            return
        }

        for config in ropeConfigs {
            let _ = addBand(radius: config.radius, crossSection: config.crossSection, particleCount: particleCount)
        }

        if actions.isEmpty {
            for (i, config) in ropeConfigs.enumerated() {
                pin(bandIndex: i, startHole: config.startHole, endHole: config.endHole)
            }
            return
        }

        Self.logger.info("Replaying \(actions.count) actions, bands.count=\(self.bands.count), weights.count=\(self.weights.count), holePositions.count=\(self.holePositions.count), isTensionMode=\(self.isTensionMode)")

        let initStart = CACurrentMediaTime()
        var pinTime: Double = 0
        var dragTime: Double = 0
        var pinCount = 0
        var dragCount = 0

        for action in actions {
            let t0 = CACurrentMediaTime()
            switch action.type {
            case .pin:
                let resolvedIndex: Int
                if isTensionMode && action.holeIndex >= holePositions.count {
                    // Weight pin: convert to negative index
                    let wi = action.holeIndex - holePositions.count
                    resolvedIndex = Self.weightPinIndex(wi)
                    if weights.indices.contains(wi) {
                        weights[wi].attachedBandEnds.append((bandIndex: action.ropeIndex, endIndex: action.endIndex))
                    }
                } else {
                    resolvedIndex = action.holeIndex
                }
                guard bands.indices.contains(action.ropeIndex) else {
                    Self.logger.error("[PIN] ropeIndex \(action.ropeIndex) out of range (bands.count=\(self.bands.count))")
                    continue
                }
                if action.endIndex == 0 {
                    bands[action.ropeIndex].pinStart = resolvedIndex
                } else {
                    bands[action.ropeIndex].pinEnd = resolvedIndex
                }
                if bands[action.ropeIndex].pinStart != nil && bands[action.ropeIndex].pinEnd != nil {
                    pinAndSettle(bandIndex: action.ropeIndex)
                    pinCount += 1
                    pinTime += CACurrentMediaTime() - t0
                }
            case .drag:
                simulateDrag(bandIndex: action.ropeIndex, endIndex: action.endIndex, toHole: action.holeIndex)
                dragCount += 1
                dragTime += CACurrentMediaTime() - t0
            case .swap:
                simulateSwap(bandA: action.ropeIndex, endA: action.endIndex,
                             bandB: action.ropeIndex2, endB: action.endIndex2)
                dragCount += 1
                dragTime += CACurrentMediaTime() - t0
            }
        }

        let totalMs = (CACurrentMediaTime() - initStart) * 1000
        let pinMs = pinTime * 1000
        let dragMs = dragTime * 1000
        let avgDragMs = dragCount > 0 ? dragMs / Double(dragCount) : 0
        Self.logger.warning("""
            [INIT-PROFILE] pins=\(pinCount) pinTime=\(String(format: "%.1f", pinMs))ms \
            drags=\(dragCount) dragTime=\(String(format: "%.1f", dragMs))ms \
            avgDrag=\(String(format: "%.1f", avgDragMs))ms \
            total=\(String(format: "%.1f", totalMs))ms
            """)
    }

    private func pinAndSettle(bandIndex: Int) {
        guard let startHole = bands[bandIndex].pinStart,
              let endHole = bands[bandIndex].pinEnd else { return }

        let p0 = pinPosition3D(startHole)
        let p1 = pinPosition3D(endHole)
        let n = bands[bandIndex].positions.count

        for i in 0..<n {
            let t = Float(i) / Float(max(1, n - 1))
            let x = p0.x * (1 - t) + p1.x * t
            let y = p0.y * (1 - t) + p1.y * t
            bands[bandIndex].positions[i] = SIMD3<Float>(x, y, liftHeight)
        }
        bands[bandIndex].positions[0] = p0
        bands[bandIndex].positions[n - 1] = p1
        bands[bandIndex].previousPositions = bands[bandIndex].positions

        let dist = simd_length(p1 - p0)
        bands[bandIndex].segmentLength = dist / Float(max(1, n - 1))
        bands[bandIndex].active = true

        let totalLen = bands[bandIndex].segmentLength * Float(n - 1)
        let segL = bands[bandIndex].segmentLength
        Self.logger.warning("[PIN-SETTLE] band=\(bandIndex) n=\(n) dist=\(String(format:"%.4f",dist)) segLen=\(String(format:"%.6f",segL)) totalLen=\(String(format:"%.4f",totalLen))")

        let hasOthers = bands.enumerated().contains(where: { $0.offset != bandIndex && $0.element.active && $0.element.pinStart != nil })
        doSteps(settleSteps, collide: hasOthers)

        var actualLen: Float = 0
        for i in 1..<n {
            actualLen += simd_length(bands[bandIndex].positions[i] - bands[bandIndex].positions[i-1])
        }
        Self.logger.warning("[PIN-SETTLE-AFTER] band=\(bandIndex) actualLen=\(String(format:"%.4f",actualLen)) vs restLen=\(String(format:"%.4f",totalLen))")
    }

    /// Find the max Z of all active band particles (to lift above everything)
    func maxRopeZ() -> Float {
        var maxZ: Float = 0
        for b in bands where b.active && b.fadeOut == 0 {
            for p in b.positions { maxZ = max(maxZ, p.z) }
        }
        return maxZ
    }

    func simulateDrag(bandIndex: Int, endIndex: Int, toHole: Int) {
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        let fromPos = bands[bandIndex].positions[idx]
        let toPos = holePosition3D(toHole)

        if endIndex == 0 {
            bands[bandIndex].pinStart = nil
        } else {
            bands[bandIndex].pinEnd = nil
        }

        // Lift above ALL ropes, not just board surface
        let clearance: Float = liftHeight + bands[bandIndex].radius * 2
        let liftZ = max(maxRopeZ(), boardSurfaceZ(x: fromPos.x, y: fromPos.y)) + clearance
        let liftFrom = SIMD3<Float>(fromPos.x, fromPos.y, liftZ)
        let liftTo = SIMD3<Float>(toPos.x, toPos.y, liftZ)

        let dragSteps = 4
        let liftSteps = 6
        let traverseSteps = 24
        let lowerSteps = 6

        for s in 1...liftSteps {
            let t = Float(s) / Float(liftSteps)
            bands[bandIndex].positions[idx] = fromPos + (liftFrom - fromPos) * t
            bands[bandIndex].previousPositions[idx] = bands[bandIndex].positions[idx]
            doSteps(dragSteps, collide: true)
        }

        for s in 1...traverseSteps {
            let t = Float(s) / Float(traverseSteps)
            bands[bandIndex].positions[idx] = liftFrom + (liftTo - liftFrom) * t
            bands[bandIndex].previousPositions[idx] = bands[bandIndex].positions[idx]
            doSteps(dragSteps, collide: true)
        }

        for s in 1...lowerSteps {
            let t = Float(s) / Float(lowerSteps)
            bands[bandIndex].positions[idx] = liftTo + (toPos - liftTo) * t
            bands[bandIndex].previousPositions[idx] = bands[bandIndex].positions[idx]
            doSteps(dragSteps, collide: true)
        }

        if endIndex == 0 {
            bands[bandIndex].pinStart = toHole
        } else {
            bands[bandIndex].pinEnd = toHole
        }
        bands[bandIndex].positions[idx] = toPos
        bands[bandIndex].previousPositions[idx] = toPos

        doSteps(settleSteps, collide: true)
    }

    /// Swap two rope ends simultaneously — both lift, cross, and land in each other's holes.
    func simulateSwap(bandA: Int, endA: Int, bandB: Int, endB: Int) {
        let idxA = endA == 0 ? 0 : bands[bandA].positions.count - 1
        let idxB = endB == 0 ? 0 : bands[bandB].positions.count - 1
        let fromA = bands[bandA].positions[idxA]
        let fromB = bands[bandB].positions[idxB]

        let toA = fromB
        let toB = fromA

        if endA == 0 { bands[bandA].pinStart = nil } else { bands[bandA].pinEnd = nil }
        if endB == 0 { bands[bandB].pinStart = nil } else { bands[bandB].pinEnd = nil }

        // A lifts above all ropes (over), B lifts just above board (under)
        let ropeMax = maxRopeZ()
        let clearanceA = liftHeight + bands[bandA].radius * 2
        let highLift = max(ropeMax, max(boardSurfaceZ(x: fromA.x, y: fromA.y), boardSurfaceZ(x: fromB.x, y: fromB.y))) + clearanceA
        let lowLift = max(boardSurfaceZ(x: fromA.x, y: fromA.y), boardSurfaceZ(x: fromB.x, y: fromB.y)) + liftHeight * 0.4
        let liftFromA = SIMD3<Float>(fromA.x, fromA.y, highLift)
        let liftFromB = SIMD3<Float>(fromB.x, fromB.y, lowLift)
        let liftToA = SIMD3<Float>(toA.x, toA.y, highLift)
        let liftToB = SIMD3<Float>(toB.x, toB.y, lowLift)

        let dragSteps = 4
        let liftSteps = 6
        let traverseSteps = 24
        let lowerSteps = 6

        for s in 1...liftSteps {
            let t = Float(s) / Float(liftSteps)
            bands[bandA].positions[idxA] = fromA + (liftFromA - fromA) * t
            bands[bandA].previousPositions[idxA] = bands[bandA].positions[idxA]
            bands[bandB].positions[idxB] = fromB + (liftFromB - fromB) * t
            bands[bandB].previousPositions[idxB] = bands[bandB].positions[idxB]
            doSteps(dragSteps, collide: true)
        }

        for s in 1...traverseSteps {
            let t = Float(s) / Float(traverseSteps)
            bands[bandA].positions[idxA] = liftFromA + (liftToA - liftFromA) * t
            bands[bandA].previousPositions[idxA] = bands[bandA].positions[idxA]
            bands[bandB].positions[idxB] = liftFromB + (liftToB - liftFromB) * t
            bands[bandB].previousPositions[idxB] = bands[bandB].positions[idxB]
            doSteps(dragSteps, collide: true)
        }

        for s in 1...lowerSteps {
            let t = Float(s) / Float(lowerSteps)
            bands[bandA].positions[idxA] = liftToA + (toA - liftToA) * t
            bands[bandA].previousPositions[idxA] = bands[bandA].positions[idxA]
            bands[bandB].positions[idxB] = liftToB + (toB - liftToB) * t
            bands[bandB].previousPositions[idxB] = bands[bandB].positions[idxB]
            doSteps(dragSteps, collide: true)
        }

        // Re-pin: A now at B's old hole, B now at A's old hole
        // Find which holes they landed at
        let holeA = nearestHole(to: toA)
        let holeB = nearestHole(to: toB)
        if endA == 0 { bands[bandA].pinStart = holeA } else { bands[bandA].pinEnd = holeA }
        if endB == 0 { bands[bandB].pinStart = holeB } else { bands[bandB].pinEnd = holeB }
        bands[bandA].positions[idxA] = holePosition3D(holeA)
        bands[bandA].previousPositions[idxA] = bands[bandA].positions[idxA]
        bands[bandB].positions[idxB] = holePosition3D(holeB)
        bands[bandB].previousPositions[idxB] = bands[bandB].positions[idxB]

        doSteps(settleSteps, collide: true)
    }

    private func nearestHole(to pos: SIMD3<Float>) -> Int {
        var best = 0
        var bestD: Float = .greatestFiniteMagnitude
        for i in 0..<holePositions.count {
            let hp = holePosition3D(i)
            let d = simd_length(hp - pos)
            if d < bestD { bestD = d; best = i }
        }
        return best
    }
}
