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
        let liftPos = SIMD3<Float>(worldPosition.x, worldPosition.y, elev + liftHeight)
        bands[bandIndex].positions[idx] = liftPos
        bands[bandIndex].previousPositions[idx] = liftPos
        dragStartPos = liftPos
        dragTargetPos = liftPos
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
        enum ActionType: String { case pin, drag }
        let type: ActionType
        let ropeIndex: Int
        let endIndex: Int
        let holeIndex: Int
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

    /// Attach a band end to a weight (uses negative pin index convention)
    func attachBandEndToWeight(bandIndex: Int, endIndex: Int, weightIndex: Int) {
        let pinIdx = Self.weightPinIndex(weightIndex)
        if endIndex == 0 {
            bands[bandIndex].pinStart = pinIdx
        } else {
            bands[bandIndex].pinEnd = pinIdx
        }
        weights[weightIndex].attachedBandEnds.append((bandIndex: bandIndex, endIndex: endIndex))

        // Position the band end at the weight
        let wPos = SIMD3<Float>(weights[weightIndex].position.x, weights[weightIndex].position.y,
                                boardSurfaceZ(x: weights[weightIndex].position.x, y: weights[weightIndex].position.y))
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        bands[bandIndex].positions[idx] = wPos
        bands[bandIndex].previousPositions[idx] = wPos
    }

    func initializeLevel(ropeConfigs: [RopeConfig], actions: [LevelAction], ropeParticles: [[SIMD3<Float>]]? = nil) {
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

    func simulateDrag(bandIndex: Int, endIndex: Int, toHole: Int) {
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        let fromPos = bands[bandIndex].positions[idx]
        let toPos = holePosition3D(toHole)

        if endIndex == 0 {
            bands[bandIndex].pinStart = nil
        } else {
            bands[bandIndex].pinEnd = nil
        }

        let fromElev = boardSurfaceZ(x: fromPos.x, y: fromPos.y)
        let toElev = holeSurfaceZ(toHole)
        let maxElev = max(fromElev, toElev)
        let liftFrom = SIMD3<Float>(fromPos.x, fromPos.y, maxElev + liftHeight)
        let liftTo = SIMD3<Float>(toPos.x, toPos.y, maxElev + liftHeight)

        let dragSteps = 4
        for s in 1...3 {
            let t = Float(s) / 3.0
            bands[bandIndex].positions[idx] = fromPos + (liftFrom - fromPos) * t
            bands[bandIndex].previousPositions[idx] = bands[bandIndex].positions[idx]
            doSteps(dragSteps, collide: true)
        }

        let traverseSteps = 12
        for s in 1...traverseSteps {
            let t = Float(s) / Float(traverseSteps)
            bands[bandIndex].positions[idx] = liftFrom + (liftTo - liftFrom) * t
            bands[bandIndex].previousPositions[idx] = bands[bandIndex].positions[idx]
            doSteps(dragSteps, collide: true)
        }

        for s in 1...3 {
            let t = Float(s) / 3.0
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
}
