import simd
import os.log

final class VerletSimulator {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "VerletSim")

    // MARK: - Band (rope) state

    struct Band {
        var positions: [SIMD3<Float>]
        var previousPositions: [SIMD3<Float>]
        var segmentLength: Float
        var radius: Float
        var pinStart: Int?
        var pinEnd: Int?
        var active: Bool
    }

    // MARK: - Drag state

    struct DragInfo {
        var bandIndex: Int
        var endIndex: Int          // 0 = first particle, 1 = last particle
        var originalHoleIndex: Int
    }

    // MARK: - Properties

    var bands: [Band] = []
    let holePositions: [SIMD2<Float>]
    let holeRadius: Float
    let holeDepth: Float

    // Physics parameters (tuneable)
    var gravity: Float = -5.0
    var damping: Float = 0.97
    var constraintIterations: Int = 8
    var settleSteps: Int = 5
    var liftHeight: Float = 0.30
    var collisionMultiplier: Float = 1.2
    var particleCount: Int = 6

    private let dt: Float = 1.0 / 60.0
    private var accumulator: Float = 0
    private var dragTargetPos: SIMD3<Float>?
    private var dragStartPos: SIMD3<Float>?

    var dragInfo: DragInfo?

    // MARK: - Init

    init(holePositions: [SIMD2<Float>], holeRadius: Float) {
        self.holePositions = holePositions
        self.holeRadius = holeRadius
        self.holeDepth = holeRadius * 1.25
    }

    // MARK: - Band management

    @discardableResult
    func addBand(radius: Float, particleCount: Int? = nil) -> Int {
        let n = particleCount ?? self.particleCount
        let band = Band(
            positions: Array(repeating: .zero, count: n),
            previousPositions: Array(repeating: .zero, count: n),
            segmentLength: 0,
            radius: radius,
            pinStart: nil,
            pinEnd: nil,
            active: false
        )
        bands.append(band)
        return bands.count - 1
    }

    /// Pin both ends and initialize the band as a straight line ABOVE the board,
    /// then settle with collision so it drapes over existing bands.
    func pin(bandIndex: Int, startHole: Int, endHole: Int) {
        guard bands.indices.contains(bandIndex) else { return }

        let p0 = holePosition3D(startHole)
        let p1 = holePosition3D(endHole)
        let n = bands[bandIndex].positions.count

        // Initialize ABOVE the board (being placed from above)
        for i in 0..<n {
            let t = Float(i) / Float(max(1, n - 1))
            let x = p0.x * (1 - t) + p1.x * t
            let y = p0.y * (1 - t) + p1.y * t
            let z: Float = (i == 0 || i == n - 1) ? p0.z : liftHeight
            bands[bandIndex].positions[i] = SIMD3<Float>(x, y, z)
        }
        // Endpoints go to holes
        bands[bandIndex].positions[0] = p0
        bands[bandIndex].positions[n - 1] = p1
        bands[bandIndex].previousPositions = bands[bandIndex].positions

        let dist = simd_length(p1 - p0)
        bands[bandIndex].segmentLength = dist / Float(max(1, n - 1))
        bands[bandIndex].pinStart = startHole
        bands[bandIndex].pinEnd = endHole
        bands[bandIndex].active = true

        // Settle onto board with collision against other bands
        let hasOthers = bands.enumerated().contains(where: { $0.offset != bandIndex && $0.element.active && $0.element.pinStart != nil })
        doSteps(settleSteps, collide: hasOthers)
    }

    // MARK: - Physics step

    /// Main per-frame update. Uses fixed timestep accumulator so physics
    /// runs the correct number of substeps regardless of frame rate.
    /// During drag, the dragged endpoint is interpolated across substeps
    /// to prevent tunneling through other ropes.
    func update(deltaTime: Float) {
        // Cap accumulated time to prevent spiral of death at very low FPS
        accumulator += min(deltaTime, 1.0 / 15.0)

        let stepsNeeded = Int(accumulator / dt)
        guard stepsNeeded > 0 else { return }

        if let drag = dragInfo, let target = dragTargetPos {
            let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
            let startPos = dragStartPos ?? bands[drag.bandIndex].positions[idx]

            for s in 1...stepsNeeded {
                let t = Float(s) / Float(stepsNeeded)
                let interpPos = startPos + (target - startPos) * t
                bands[drag.bandIndex].positions[idx] = interpPos
                bands[drag.bandIndex].previousPositions[idx] = interpPos
                verletStep(collide: true)
            }
            // Next frame interpolates from where we ended up
            dragStartPos = target
        } else {
            for _ in 0..<stepsNeeded {
                verletStep(collide: true)
            }
        }

        accumulator -= Float(stepsNeeded) * dt
    }

    func doSteps(_ n: Int, collide: Bool) {
        for _ in 0..<n {
            verletStep(collide: collide)
        }
    }

    private func verletStep(collide: Bool) {
        let dt2 = dt * dt

        // 1. Verlet position update
        for bi in bands.indices {
            guard bands[bi].active else { continue }
            let n = bands[bi].positions.count
            for i in 1..<(n - 1) {  // skip pinned endpoints
                let pos = bands[bi].positions[i]
                let old = bands[bi].previousPositions[i]
                let vel = (pos - old) * damping
                bands[bi].previousPositions[i] = pos
                bands[bi].positions[i] = pos + vel + SIMD3<Float>(0, 0, gravity * dt2)
            }
        }

        // 2. Constraint iterations
        for _ in 0..<constraintIterations {
            for bi in bands.indices {
                guard bands[bi].active else { continue }
                bandConstraints(bi)
            }
            if collide {
                let active = bands.indices.filter { bands[$0].active }
                for i in 0..<active.count {
                    for j in (i + 1)..<active.count {
                        collideStep(active[i], active[j])
                    }
                }
            }
        }
    }

    private func bandConstraints(_ bi: Int) {
        let n = bands[bi].positions.count
        let segLen = bands[bi].segmentLength
        let R = bands[bi].radius

        // Even-odd (red-black) distance constraints
        for offset in 0...1 {
            var idx = offset
            while idx < n - 1 {
                let diff = bands[bi].positions[idx + 1] - bands[bi].positions[idx]
                let dist = simd_length(diff) + 1e-12
                let corr = diff * ((dist - segLen) / dist * 0.5)
                if idx > 0 {
                    bands[bi].positions[idx] += corr
                }
                if idx + 1 < n - 1 {
                    bands[bi].positions[idx + 1] -= corr
                }
                idx += 2
            }
        }

        // Pin constraints (hard)
        if let startHole = bands[bi].pinStart {
            bands[bi].positions[0] = holePosition3D(startHole)
        }
        if let endHole = bands[bi].pinEnd {
            bands[bi].positions[n - 1] = holePosition3D(endHole)
        }

        // Board: z >= R (rope rests on surface)
        for i in 1..<(n - 1) {
            if bands[bi].positions[i].z < R {
                bands[bi].positions[i].z = R
            }
        }
    }

    /// Capsule-capsule collision: checks closest point between every pair
    /// of segments from two bands, pushes apart if closer than minDist.
    private func collideStep(_ bi: Int, _ bj: Int) {
        let minDist = (bands[bi].radius + bands[bj].radius) * collisionMultiplier
        let minDist2 = minDist * minDist
        let segsI = bands[bi].positions.count - 1
        let segsJ = bands[bj].positions.count - 1

        for i in 0..<segsI {
            let a0 = bands[bi].positions[i]
            let a1 = bands[bi].positions[i + 1]
            let d1 = a1 - a0
            let lenA2 = simd_dot(d1, d1)

            for j in 0..<segsJ {
                let b0 = bands[bj].positions[j]
                let b1 = bands[bj].positions[j + 1]
                let d2 = b1 - b0
                let lenB2 = simd_dot(d2, d2)
                let r = a0 - b0

                let a = lenA2
                let e = lenB2
                let f = simd_dot(d2, r)
                let c = simd_dot(d1, r)
                let b = simd_dot(d1, d2)

                let denom = a * e - b * b
                var s: Float = 0
                var t: Float

                if denom > 1e-12 {
                    s = min(max((b * f - c * e) / denom, 0), 1)
                }
                t = (b * s + f) / max(e, 1e-12)

                if t < 0 {
                    t = 0
                    s = min(max(-c / max(a, 1e-12), 0), 1)
                } else if t > 1 {
                    t = 1
                    s = min(max((b - c) / max(a, 1e-12), 0), 1)
                }

                let closestA = a0 + d1 * s
                let closestB = b0 + d2 * t
                let diff = closestA - closestB
                let dist2 = simd_dot(diff, diff)

                if dist2 < minDist2 && dist2 > 1e-12 {
                    let dist = sqrtf(dist2)
                    let overlap = minDist - dist
                    let normal = diff / dist
                    let corr = normal * (overlap * 0.5)

                    // Distribute correction to segment endpoints by parameter
                    let wA0 = 1 - s
                    let wA1 = s
                    let wB0 = 1 - t
                    let wB1 = t

                    // Skip pinned endpoints (index 0 and last)
                    if i > 0 { bands[bi].positions[i] += corr * wA0 }
                    if i + 1 < bands[bi].positions.count - 1 { bands[bi].positions[i + 1] += corr * wA1 }
                    if j > 0 { bands[bj].positions[j] -= corr * wB0 }
                    if j + 1 < bands[bj].positions.count - 1 { bands[bj].positions[j + 1] -= corr * wB1 }
                }
            }
        }
    }

    // MARK: - Drag

    func beginDrag(bandIndex: Int, endIndex: Int, worldPosition: SIMD2<Float>) {
        guard bands.indices.contains(bandIndex) else { return }
        let originalHole: Int
        if endIndex == 0 {
            originalHole = bands[bandIndex].pinStart ?? 0
            bands[bandIndex].pinStart = nil
        } else {
            originalHole = bands[bandIndex].pinEnd ?? 0
            bands[bandIndex].pinEnd = nil
        }
        dragInfo = DragInfo(bandIndex: bandIndex, endIndex: endIndex, originalHoleIndex: originalHole)

        // Set the dragged endpoint to lift height
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        let liftPos = SIMD3<Float>(worldPosition.x, worldPosition.y, liftHeight)
        bands[bandIndex].positions[idx] = liftPos
        bands[bandIndex].previousPositions[idx] = liftPos
        dragStartPos = liftPos
        dragTargetPos = liftPos
    }

    func updateDrag(worldPosition: SIMD2<Float>) {
        guard let drag = dragInfo else { return }
        let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
        // Save current position as interpolation start, set target
        dragStartPos = bands[drag.bandIndex].positions[idx]
        dragTargetPos = SIMD3<Float>(worldPosition.x, worldPosition.y, liftHeight)
    }

    func endDrag(targetHoleIndex: Int) {
        guard let drag = dragInfo else { return }

        // Re-pin at target hole
        if drag.endIndex == 0 {
            bands[drag.bandIndex].pinStart = targetHoleIndex
        } else {
            bands[drag.bandIndex].pinEnd = targetHoleIndex
        }

        // Set endpoint to hole position — rope will settle naturally via update()
        let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
        bands[drag.bandIndex].positions[idx] = holePosition3D(targetHoleIndex)
        bands[drag.bandIndex].previousPositions[idx] = bands[drag.bandIndex].positions[idx]

        dragInfo = nil
        dragStartPos = nil
        dragTargetPos = nil
    }

    /// For `isEndTopForDrag` — compare z at endpoints
    func endpointZ(bandIndex: Int, endIndex: Int) -> Float {
        guard bands.indices.contains(bandIndex) else { return 0 }
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        return bands[bandIndex].positions[idx].z
    }

    // MARK: - Level initialization from pre-decomposed actions

    struct RopeConfig {
        let startHole: Int
        let endHole: Int
        let radius: Float
    }

    struct LevelAction {
        enum ActionType: String { case pin, drag }
        let type: ActionType
        let ropeIndex: Int
        let endIndex: Int
        let holeIndex: Int
    }

    /// Initialize bands from pre-decomposed actions (generated by tools/decompose_level.py).
    /// Actions are pin + drag sequences that reproduce the crossing topology.
    func initializeLevel(ropeConfigs: [RopeConfig], actions: [LevelAction]) {
        bands.removeAll()

        for config in ropeConfigs {
            addBand(radius: config.radius, particleCount: particleCount)
        }

        if actions.isEmpty {
            // No actions — just pin all ropes at their final positions sequentially
            for (i, config) in ropeConfigs.enumerated() {
                pin(bandIndex: i, startHole: config.startHole, endHole: config.endHole)
            }
            return
        }

        Self.logger.info("Replaying \(actions.count) actions")

        for action in actions {
            switch action.type {
            case .pin:
                if action.endIndex == 0 {
                    bands[action.ropeIndex].pinStart = action.holeIndex
                } else {
                    bands[action.ropeIndex].pinEnd = action.holeIndex
                }
                if bands[action.ropeIndex].pinStart != nil && bands[action.ropeIndex].pinEnd != nil {
                    pinAndSettle(bandIndex: action.ropeIndex)
                }
            case .drag:
                simulateDrag(bandIndex: action.ropeIndex, endIndex: action.endIndex, toHole: action.holeIndex)
            }
        }
    }

    // MARK: - Simulation helpers

    private func pinAndSettle(bandIndex: Int) {
        guard let startHole = bands[bandIndex].pinStart,
              let endHole = bands[bandIndex].pinEnd else { return }

        let p0 = holePosition3D(startHole)
        let p1 = holePosition3D(endHole)
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

        let hasOthers = bands.enumerated().contains(where: { $0.offset != bandIndex && $0.element.active && $0.element.pinStart != nil })
        doSteps(settleSteps, collide: hasOthers)
    }

    private func simulateDrag(bandIndex: Int, endIndex: Int, toHole: Int) {
        let idx = endIndex == 0 ? 0 : bands[bandIndex].positions.count - 1
        let fromPos = bands[bandIndex].positions[idx]
        let toPos = holePosition3D(toHole)

        if endIndex == 0 {
            bands[bandIndex].pinStart = nil
        } else {
            bands[bandIndex].pinEnd = nil
        }

        let liftFrom = SIMD3<Float>(fromPos.x, fromPos.y, liftHeight)
        let liftTo = SIMD3<Float>(toPos.x, toPos.y, liftHeight)

        let dragSteps = 8

        for s in 1...5 {
            let t = Float(s) / 5.0
            let wp = fromPos + (liftFrom - fromPos) * t
            bands[bandIndex].positions[idx] = wp
            bands[bandIndex].previousPositions[idx] = wp
            doSteps(dragSteps, collide: true)
        }

        let traverseSteps = 30
        for s in 1...traverseSteps {
            let t = Float(s) / Float(traverseSteps)
            let wp = liftFrom + (liftTo - liftFrom) * t
            bands[bandIndex].positions[idx] = wp
            bands[bandIndex].previousPositions[idx] = wp
            doSteps(dragSteps, collide: true)
        }

        for s in 1...5 {
            let t = Float(s) / 5.0
            let wp = liftTo + (toPos - liftTo) * t
            bands[bandIndex].positions[idx] = wp
            bands[bandIndex].previousPositions[idx] = wp
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

    // MARK: - Utility

    func holePosition3D(_ holeIndex: Int) -> SIMD3<Float> {
        guard holePositions.indices.contains(holeIndex) else { return .zero }
        let p = holePositions[holeIndex]
        return SIMD3<Float>(p.x, p.y, -holeDepth)
    }

}
