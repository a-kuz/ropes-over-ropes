import simd
import os.log
import QuartzCore

enum CrossSection {
    case circular(radius: Float)
    case rectangular(width: Float, height: Float)

    var collisionRadius: Float {
        switch self {
        case .circular(let r): return r
        case .rectangular(let w, let h): return max(w, h) * 0.5
        }
    }

    var halfWidth: Float {
        switch self {
        case .circular(let r): return r
        case .rectangular(let w, _): return w * 0.5
        }
    }

    var halfHeight: Float {
        switch self {
        case .circular(let r): return r
        case .rectangular(_, let h): return h * 0.5
        }
    }

    var isRectangular: Bool {
        if case .rectangular = self { return true }
        return false
    }

    func effectiveRadius(normal: SIMD3<Float>, d1: SIMD3<Float>, d2: SIMD3<Float>) -> Float {
        switch self {
        case .circular(let r):
            return r
        case .rectangular(let w, let h):
            let hw = w * 0.5
            let hh = h * 0.5
            return abs(simd_dot(normal, d1)) * hw + abs(simd_dot(normal, d2)) * hh
        }
    }
}

struct MaterialFrame {
    var tangent: SIMD3<Float>
    var d1: SIMD3<Float>
    var d2: SIMD3<Float>
}

final class VerletSimulator {
    static let logger = Logger(subsystem: "com.uzls.four", category: "VerletSim")

    // MARK: - Band (rope) state

    struct Band {
        var positions: ContiguousArray<SIMD3<Float>>
        var previousPositions: ContiguousArray<SIMD3<Float>>
        var twistAngles: ContiguousArray<Float>
        var previousTwistAngles: ContiguousArray<Float>
        var segmentLength: Float
        var radius: Float
        var crossSection: CrossSection
        var pinStart: Int?
        var pinEnd: Int?
        var active: Bool
        var fadeOut: Float = 0
        var suckHole: Int?
        var suckTailHole: Int?
        var suckFromEnd: Int = 1
        var suckConsumed: Float = 0
        var suckFrame: Int = 0
        var suckSegLengths: ContiguousArray<Float> = []
        var suckOrigPositions: ContiguousArray<SIMD3<Float>> = []
        static let fadeOutSpeed: Float = 45.0
    }

    // MARK: - Drag state

    struct DragInfo {
        var bandIndex: Int
        var endIndex: Int          // 0 = first particle, 1 = last particle
        var originalHoleIndex: Int
    }

    // MARK: - Properties

    struct BoardDef {
        let centerX: Float
        let centerY: Float
        let width: Float
        let height: Float
        let elevation: Float
    }

    var bands: [Band] = []
    let holePositions: [SIMD2<Float>]
    let holeElevations: [Float]
    let holeRadius: Float
    let holeDepth: Float
    let boards: [BoardDef]

    // Physics parameters (tuneable)
    var gravity: Float = -14.298969268798828
    var damping: Float = 0.92867755889892578
    var constraintIterations: Int = 2 {
        didSet { constraintIterations = max(constraintIterations, 2) }
    }
    var settleSteps: Int = 5
    var liftHeight: Float = 0.30000001192092896
    /// Rope tension: multiplier on rest length. < 1 = taut (shorter rope), 1 = natural length.
    /// 0.95 = rope is 5% shorter than span → pulled tight. Don't go below ~0.85.
    var ropeTension: Float = 0.98000001907348633
    var currentTension: Float = 1.0
    private let tensionSpeed: Float = 0.5  // per second — slow tightening after drag
    /// Rubber friction coefficient for rope-rope collisions (Coulomb model).
    /// Real rubber μ ≈ 1.0–2.0; we use a moderate value to keep PBD stable.
    var frictionCoefficient: Float = 0.8
    var particleCount: Int = 6
    var bendCompliance: Float = 0
    var bendVelocityCoupling: Float = 0.44999998807907104
    var twistStiffness: Float = 0.15
    var twistDamping: Float = 0.4
    var gravityTorqueStrength: Float = 0.8
    var stretchThinning: Float = 0.5
    var squareCrossSection: Bool = false

    private let dt: Float = 1.0 / 120.0  // fixed dt, supports ProMotion 120fps
    private var accumulator: Float = 0
    private var resampleCounter: Int = 0
    private let resampleInterval: Int = 30  // resample every N substeps to avoid fighting solver
    var dragTargetPos: SIMD3<Float>?
    var dragStartPos: SIMD3<Float>?
    var logTimer: Float = 0
    var logEvery: Float = 1.0  // log every N seconds

    var dragInfo: DragInfo?

    // MARK: - Idle sleep
    private let idleTimeout: Float = 3.0
    private var idleTimer: Float = 0
    private(set) var isSleeping: Bool = false

    func wakeUp() {
        idleTimer = 0
        isSleeping = false
    }

    // MARK: - Friction sound feedback

    struct FrictionEvent {
        var intensity: Float
        var relativeSpeed: Float
        var position: SIMD3<Float>
    }

    var frictionAccumulator: Float = 0
    var frictionSpeedAccumulator: Float = 0
    var frictionPositionAccumulator: SIMD3<Float> = .zero
    var frictionSampleCount: Int = 0


    // MARK: - Undo snapshots

    struct BandSnapshot {
        var positions: ContiguousArray<SIMD3<Float>>
        var previousPositions: ContiguousArray<SIMD3<Float>>
        var twistAngles: ContiguousArray<Float>
        var previousTwistAngles: ContiguousArray<Float>
        var segmentLength: Float
        var pinStart: Int?
        var pinEnd: Int?
        var active: Bool
        var fadeOut: Float
        var suckHole: Int?
        var suckTailHole: Int?
        var suckFromEnd: Int
        var suckConsumed: Float
        var suckSegLengths: ContiguousArray<Float>
        var suckOrigPositions: ContiguousArray<SIMD3<Float>>
    }

    struct Snapshot {
        var bands: [BandSnapshot]
    }


    // MARK: - Post-drag lower animation

    struct LowerAnimation {
        let bandIndex: Int
        let endIndex: Int
        let targetHole: Int
        var startPos: SIMD3<Float>
        var timer: Float = 0
        var returnPos: SIMD3<Float>?
        var returnDuration: Float = 0
        static let duration: Float = 0.55
    }

    struct LowerAnimationKey: Hashable {
        let bandIndex: Int
        let endIndex: Int
    }

    var lowerAnimations: [LowerAnimationKey: LowerAnimation] = [:]

    var hasLowerAnimations: Bool {
        !lowerAnimations.isEmpty
    }

    // MARK: - Init

    init(holePositions: [SIMD2<Float>], holeElevations: [Float] = [], holeRadius: Float, boards: [LevelDefinition.Board] = []) {
        self.holePositions = holePositions
        self.holeElevations = holeElevations.isEmpty ? Array(repeating: 0, count: holePositions.count) : holeElevations
        self.holeRadius = holeRadius
        self.holeDepth = holeRadius * 1.25
        self.boards = boards.map { BoardDef(centerX: $0.centerX, centerY: $0.centerY, width: $0.width, height: $0.height, elevation: $0.elevation) }
    }

    // MARK: - Band management

    @discardableResult
    func addBand(radius: Float, crossSection: CrossSection? = nil, particleCount: Int? = nil) -> Int {
        let n = particleCount ?? self.particleCount
        let cs = crossSection ?? .circular(radius: radius)
        let band = Band(
            positions: ContiguousArray(repeating: .zero, count: n),
            previousPositions: ContiguousArray(repeating: .zero, count: n),
            twistAngles: ContiguousArray(repeating: 0, count: n),
            previousTwistAngles: ContiguousArray(repeating: 0, count: n),
            segmentLength: 0,
            radius: radius,
            crossSection: cs,
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

        let totalLen = bands[bandIndex].segmentLength * Float(n - 1)
        Self.logger.warning("[PIN] band=\(bandIndex) n=\(n) dist=\(String(format:"%.4f",dist)) segLen=\(String(format:"%.6f",self.bands[bandIndex].segmentLength)) totalLen=\(String(format:"%.4f",totalLen)) tension=\(String(format:"%.3f",self.currentTension))")

        let hasOthers = bands.enumerated().contains(where: { $0.offset != bandIndex && $0.element.active && $0.element.pinStart != nil })
        doSteps(settleSteps, collide: hasOthers)
    }

    // MARK: - Physics step

    /// Main per-frame update. Uses fixed timestep accumulator so physics
    /// runs the correct number of substeps regardless of frame rate.
    /// During drag, the dragged endpoint is interpolated across substeps
    /// to prevent tunneling through other ropes.
    func update(deltaTime: Float) {
        let clampedDt = min(deltaTime, 1.0 / 15.0)  // spiral-of-death protection

        if dragInfo != nil || hasLowerAnimations {
            idleTimer = 0
            isSleeping = false
        } else {
            idleTimer += clampedDt
        }

        if idleTimer >= idleTimeout && !isSleeping {
            for bi in bands.indices where bands[bi].active && bands[bi].fadeOut == 0 {
                bands[bi].previousPositions = bands[bi].positions
                bands[bi].previousTwistAngles = bands[bi].twistAngles
            }
            isSleeping = true
        }

        if isSleeping {
            for i in bands.indices where bands[i].fadeOut > 0 && bands[i].active {
                isSleeping = false
                idleTimer = 0
                break
            }
            if isSleeping {
                accumulator = 0
                return
            }
        }

        // Advance suck-into-hole animations: slide rope along its arc-length curve into the hole
        for i in bands.indices where bands[i].fadeOut > 0 && bands[i].active {
            guard let hole = bands[i].suckHole else { continue }
            let n = bands[i].positions.count
            let holeXY = holePositions[hole]
            let holeElev = holeSurfaceZ(hole)
            let holeBelow = SIMD3<Float>(holeXY.x, holeXY.y, holeElev - holeDepth)

            let pullSpeed = Band.fadeOutSpeed * bands[i].segmentLength
            bands[i].suckConsumed += pullSpeed * clampedDt

            let fromEnd = bands[i].suckFromEnd
            let suckSegs = bands[i].suckSegLengths
            let R = bands[i].radius

            let origPositions = bands[i].suckOrigPositions

            var arcLen = ContiguousArray<Float>(repeating: 0, count: n)
            if fromEnd == 1 {
                for k in 1..<n {
                    arcLen[k] = arcLen[k - 1] + (k - 1 < suckSegs.count ? suckSegs[k - 1] : bands[i].segmentLength)
                }
            } else {
                for k in stride(from: n - 2, through: 0, by: -1) {
                    arcLen[k] = arcLen[k + 1] + (k < suckSegs.count ? suckSegs[k] : bands[i].segmentLength)
                }
            }

            let totalArc = fromEnd == 1 ? arcLen[n - 1] : arcLen[0]
            let consumed = bands[i].suckConsumed

            for k in 0..<n {
                let myArc = arcLen[k]
                let shifted = myArc - consumed

                if shifted <= 0 {
                    bands[i].positions[k] = holeBelow - SIMD3<Float>(0, 0, -shifted)
                } else {
                    if fromEnd == 1 {
                        var seg = 0
                        var acc: Float = 0
                        while seg < n - 1 {
                            let segL = seg < suckSegs.count ? suckSegs[seg] : bands[i].segmentLength
                            if acc + segL >= shifted { break }
                            acc += segL
                            seg += 1
                        }
                        let segL = seg < suckSegs.count ? suckSegs[seg] : bands[i].segmentLength
                        let t = segL > 1e-9 ? (shifted - acc) / segL : 0
                        let p0 = origPositions[seg]
                        let p1 = seg + 1 < n ? origPositions[seg + 1] : p0
                        bands[i].positions[k] = p0 + (p1 - p0) * min(t, 1)
                    } else {
                        var seg = n - 1
                        var acc: Float = 0
                        while seg > 0 {
                            let segL = (seg - 1) < suckSegs.count ? suckSegs[seg - 1] : bands[i].segmentLength
                            if acc + segL >= shifted { break }
                            acc += segL
                            seg -= 1
                        }
                        let segL = (seg - 1 >= 0 && seg - 1 < suckSegs.count) ? suckSegs[seg - 1] : bands[i].segmentLength
                        let t = segL > 1e-9 ? (shifted - acc) / segL : 0
                        let p0 = origPositions[seg]
                        let p1 = seg - 1 >= 0 ? origPositions[seg - 1] : p0
                        bands[i].positions[k] = p0 + (p1 - p0) * min(t, 1)
                    }

                    let surfZ = boardSurfaceZ(x: bands[i].positions[k].x, y: bands[i].positions[k].y)
                    if bands[i].positions[k].z >= surfZ && bands[i].positions[k].z < surfZ + R {
                        bands[i].positions[k].z = surfZ + R
                    }
                }
                bands[i].previousPositions[k] = bands[i].positions[k]
            }

            if consumed >= totalArc {
                Self.logger.warning("[WIN-DIAG] fadeOutComplete band=\(i) pinStart=\(self.bands[i].pinStart.map(String.init) ?? "nil") pinEnd=\(self.bands[i].pinEnd.map(String.init) ?? "nil") suckHole=\(self.bands[i].suckHole.map(String.init) ?? "nil") activeBefore=\(self.bands[i].active)")
                bands[i].fadeOut = 1
                bands[i].active = false
                bands[i].pinStart = nil
                bands[i].pinEnd = nil
                bands[i].suckHole = nil
            } else {
                var aboveCount = 0
                for k in 0..<n {
                    let surfZ = boardSurfaceZ(x: bands[i].positions[k].x, y: bands[i].positions[k].y)
                    if bands[i].positions[k].z >= surfZ { aboveCount += 1 }
                }
                bands[i].fadeOut = min(1.0 - Float(aboveCount) / Float(n), 0.999)
            }
        }

        accumulator += clampedDt
        let stepsNeeded = Int(accumulator / dt)
        accumulator -= Float(stepsNeeded) * dt
        let n = max(stepsNeeded, 0)
        guard n > 0 else { return }

        // Smooth tension transition
        let targetTension = dragInfo != nil ? 1.0 : ropeTension
        if currentTension != targetTension {
            let diff = targetTension - currentTension
            let step = tensionSpeed * clampedDt
            if abs(diff) <= step {
                currentTension = targetTension
            } else {
                currentTension += diff > 0 ? step : -step
            }
        }

        logTimer += deltaTime
        let shouldLog = logTimer >= logEvery
        if shouldLog { logTimer = 0 }

        if shouldLog {
            Self.logger.info("""
                [FRAME] dt=\(deltaTime, format: .fixed(precision: 4)) \
                substeps=\(n) fixedDt=\(self.dt, format: .fixed(precision: 5)) \
                drag=\(self.dragInfo != nil) particles=\(self.bands.first?.positions.count ?? 0) \
                bands=\(self.bands.filter { $0.active }.count) constIter=\(self.constraintIterations)
                """)
        }

        if let drag = dragInfo, let target = dragTargetPos {
            let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
            let startPos = dragStartPos ?? bands[drag.bandIndex].positions[idx]

            for s in 1...n {
                let t = Float(s) / Float(n)
                let interpPos = startPos + (target - startPos) * t
                bands[drag.bandIndex].positions[idx] = interpPos
                bands[drag.bandIndex].previousPositions[idx] = interpPos
                verletStep(collide: true, dt: dt)
            }
            dragStartPos = target
        } else {
            for _ in 0..<n {
                verletStep(collide: true, dt: dt)
            }
        }

        // Post-drag lower animation
        updateLowerAnimation(deltaTime: clampedDt)

        // logCrossingState disabled — costs 8% CPU (O(n²) per band pair)
    }

    private func updateLowerAnimation(deltaTime: Float) {
        guard !lowerAnimations.isEmpty else { return }

        for key in Array(lowerAnimations.keys) {
            guard var anim = lowerAnimations[key] else { continue }
            anim.timer += deltaTime

            let bi = anim.bandIndex
            let idx = anim.endIndex == 0 ? 0 : bands[bi].positions.count - 1

            if let returnTarget = anim.returnPos {
                let t = min(anim.timer / anim.returnDuration, 1.0)
                let eased = 1.0 - (1.0 - t) * (1.0 - t)
                let pos = anim.startPos + (returnTarget - anim.startPos) * eased
                bands[bi].positions[idx] = pos
                bands[bi].previousPositions[idx] = pos

                if t >= 1.0 {
                    anim.startPos = returnTarget
                    anim.returnPos = nil
                    anim.timer = 0
                }

                lowerAnimations[key] = anim
                continue
            }

            let holePos = holePosition3D(anim.targetHole)
            let t = min(anim.timer / LowerAnimation.duration, 1.0)
            let eased = 1.0 - (1.0 - t) * (1.0 - t)
            let pos = anim.startPos + (holePos - anim.startPos) * eased
            bands[bi].positions[idx] = pos
            bands[bi].previousPositions[idx] = pos

            if t >= 1.0 {
                if anim.endIndex == 0 {
                    bands[bi].pinStart = anim.targetHole
                } else {
                    bands[bi].pinEnd = anim.targetHole
                }
                bands[bi].positions[idx] = holePos
                bands[bi].previousPositions[idx] = holePos
                lowerAnimations.removeValue(forKey: key)
                continue
            }

            lowerAnimations[key] = anim
        }
    }

    /// Log min distances between band pairs and crossing Z info
    private func logCrossingState() {
        let activeBands = bands.indices.filter { bands[$0].active }
        for i in 0..<activeBands.count {
            let bi = activeBands[i]
            let bandI = bands[bi]
            // Log segment lengths vs rest length
            var maxStretch: Float = 0
            var avgStretch: Float = 0
            let n = bandI.positions.count
            for k in 0..<(n - 1) {
                let d = simd_length(bandI.positions[k + 1] - bandI.positions[k])
                let ratio = d / max(bandI.segmentLength, 1e-6)
                maxStretch = max(maxStretch, ratio)
                avgStretch += ratio
            }
            avgStretch /= Float(max(1, n - 1))
            Self.logger.info("""
                [BAND \(bi)] segs=\(n - 1) segLen=\(bandI.segmentLength, format: .fixed(precision: 4)) \
                radius=\(bandI.radius, format: .fixed(precision: 4)) \
                stretch avg=\(avgStretch, format: .fixed(precision: 2)) max=\(maxStretch, format: .fixed(precision: 2)) \
                zRange=[\(bandI.positions.map(\.z).min() ?? 0, format: .fixed(precision: 3))...\(bandI.positions.map(\.z).max() ?? 0, format: .fixed(precision: 3))]
                """)

            for j in (i + 1)..<activeBands.count {
                let bj = activeBands[j]
                let bandJ = bands[bj]
                // Find min distance between any two segments
                var minDist: Float = .greatestFiniteMagnitude
                var minI = 0, minJ = 0
                var crossings = 0
                let segsI = bandI.positions.count - 1
                let segsJ = bandJ.positions.count - 1
                let threshold = bandI.radius + bandJ.radius

                for si in 0..<segsI {
                    let a0 = SIMD2<Float>(bandI.positions[si].x, bandI.positions[si].y)
                    let a1 = SIMD2<Float>(bandI.positions[si + 1].x, bandI.positions[si + 1].y)
                    for sj in 0..<segsJ {
                        let b0 = SIMD2<Float>(bandJ.positions[sj].x, bandJ.positions[sj].y)
                        let b1 = SIMD2<Float>(bandJ.positions[sj + 1].x, bandJ.positions[sj + 1].y)
                        // 2D segment intersection check
                        let d1 = a1 - a0
                        let d2 = b1 - b0
                        let cross = d1.x * d2.y - d1.y * d2.x
                        if abs(cross) > 1e-9 {
                            let d = b0 - a0
                            let tA = (d.x * d2.y - d.y * d2.x) / cross
                            let tB = (d.x * d1.y - d.y * d1.x) / cross
                            if tA > 0.01 && tA < 0.99 && tB > 0.01 && tB < 0.99 {
                                let zA = bandI.positions[si].z * (1 - tA) + bandI.positions[si + 1].z * tA
                                let zB = bandJ.positions[sj].z * (1 - tB) + bandJ.positions[sj + 1].z * tB
                                crossings += 1
                                Self.logger.info("""
                                    [CROSS] band\(bi)seg\(si) x band\(bj)seg\(sj): \
                                    zA=\(zA, format: .fixed(precision: 4)) zB=\(zB, format: .fixed(precision: 4)) \
                                    diff=\(zA - zB, format: .fixed(precision: 4)) \
                                    (\(zA > zB ? "A over" : "B over"))
                                    """)
                            }
                        }
                        // 3D distance
                        let diff3 = bandI.positions[si] - bandJ.positions[sj]
                        let d3 = simd_length(diff3)
                        if d3 < minDist { minDist = d3; minI = si; minJ = sj }
                    }
                }
                Self.logger.info("""
                    [PAIR \(bi)-\(bj)] minDist=\(minDist, format: .fixed(precision: 4)) \
                    threshold=\(threshold, format: .fixed(precision: 4)) \
                    at seg(\(minI),\(minJ)) crossings=\(crossings)
                    """)
            }
        }
    }

    private let initDt: Float = 1.0 / 60.0

    func doSteps(_ n: Int, collide: Bool) {
        for _ in 0..<n {
            verletStep(collide: collide, dt: initDt)
        }
    }

    let profiler = PhysicsProfiler.shared

    private func verletStep(collide: Bool, dt: Float) {
        let dt2 = dt * dt
        profiler.begin()

        // 1. Verlet position update + velocity limiting
        let gravVec = SIMD3<Float>(0, 0, gravity * dt2)
        for bi in bands.indices {
            guard bands[bi].active && bands[bi].fadeOut == 0 else { continue }
            let n = bands[bi].positions.count
            let maxMove = bands[bi].radius * 2.0
            for i in 1..<(n - 1) {
                let pos = bands[bi].positions[i]
                let old = bands[bi].previousPositions[i]
                var vel = (pos - old) * damping
                let velLen = simd_length(vel)
                if velLen > maxMove { vel = vel * (maxMove / velLen) }
                bands[bi].previousPositions[i] = pos
                bands[bi].positions[i] = pos + vel + gravVec
            }

            if bands[bi].crossSection.isRectangular {
                let maxTwistVel: Float = 0.08
                for i in 1..<(n - 1) {
                    let twist = bands[bi].twistAngles[i]
                    let oldTwist = bands[bi].previousTwistAngles[i]
                    var twistVel = (twist - oldTwist) * twistDamping
                    twistVel = max(-maxTwistVel, min(maxTwistVel, twistVel))
                    bands[bi].previousTwistAngles[i] = twist
                    bands[bi].twistAngles[i] = twist + twistVel
                }
            }
        }
        profiler.end(.verletIntegration)

        // 2. Constraint + collision iterations (interleaved for robust PBD)
        let active = collide ? bands.indices.filter({ bands[$0].active && bands[$0].fadeOut == 0 }) : []

        // Scale iterations inversely with tension — stronger tension needs more solver work
        let effectiveIters = max(constraintIterations, Int(Float(constraintIterations) / max(currentTension, 0.3)))

        // Build collision pair list (broadphase). Rebuilt periodically during solve.
        var collisionPairs: [CollisionPair] = collide ? profiler.measure(.broadphase) { buildCollisionPairs(active) } : []

        // Recompute material frames for rectangular bands (needed for collision + twist)
        profiler.measure(.frames) { recomputeFrames() }

        // Gravity torque: rotate flat bands so wide side (d1) is horizontal.
        // d1.z measures how much the wide axis is tilted out of horizontal.
        // Restoring torque proportional to d1.z drives it toward zero.
        for bi in bands.indices {
            guard bands[bi].active && bands[bi].fadeOut == 0 && bands[bi].crossSection.isRectangular else { continue }
            let n = bands[bi].positions.count
            guard cachedFrames[bi].count == n else { continue }
            for i in 1..<(n - 1) {
                let d1z = cachedFrames[bi][i].d1.z
                let torque = -d1z * gravityTorqueStrength * dt2
                let maxTorque: Float = 0.01
                bands[bi].twistAngles[i] += max(-maxTorque, min(maxTorque, torque))
            }
        }

        profiler.begin()
        for iter in 0..<effectiveIters {
            for bi in bands.indices {
                guard bands[bi].active && bands[bi].fadeOut == 0 else { continue }
                bandConstraints(bi, dt: dt)
            }
            if collide {
                if iter > 0 && iter % 3 == 0 {
                    collisionPairs = profiler.measure(.broadphase) { buildCollisionPairs(active) }
                }
                resolveCollisionPairs(collisionPairs)
            }
        }
        profiler.end(.constraints)

        profiler.begin()
        // Post-solve: collision-only passes until converged
        if collide {
            for _ in 0..<3 {
                let hadCollision = resolveCollisionPairs(collisionPairs, injectVelocity: true)
                for bi in active {
                    let n = bands[bi].positions.count
                    if let startHole = bands[bi].pinStart {
                        let hp = holePosition3D(startHole)
                        bands[bi].positions[0] = hp
                        bands[bi].previousPositions[0] = hp
                    }
                    if let endHole = bands[bi].pinEnd {
                        let hp = holePosition3D(endHole)
                        bands[bi].positions[n - 1] = hp
                        bands[bi].previousPositions[n - 1] = hp
                    }
                    if bands[bi].crossSection.isRectangular && cachedFrames.indices.contains(bi) && cachedFrames[bi].count == n {
                        let cs = bands[bi].crossSection
                        for i in 1..<(n - 1) {
                            let frame = cachedFrames[bi][i]
                            let zExtent = cs.effectiveRadius(
                                normal: SIMD3<Float>(0, 0, 1),
                                d1: frame.d1,
                                d2: frame.d2
                            )
                            let floorZ = boardSurfaceZ(x: bands[bi].positions[i].x, y: bands[bi].positions[i].y) + zExtent
                            if bands[bi].positions[i].z < floorZ {
                                bands[bi].positions[i].z = floorZ
                            }
                        }
                    } else {
                        for i in 1..<(n - 1) {
                            let floorZ = boardSurfaceZ(x: bands[bi].positions[i].x, y: bands[bi].positions[i].y) + bands[bi].radius
                            if bands[bi].positions[i].z < floorZ {
                                bands[bi].positions[i].z = floorZ
                            }
                        }
                    }
                }
                if !hadCollision { break }
            }
        }

        // Board friction: applied once per substep after all solving is done.
        // Uses true Verlet velocity (pos - previousPos) which is clean at this point.
        let boardMu = frictionCoefficient * 0.5
        if boardMu > 0 {
            for bi in active {
                let n = bands[bi].positions.count
                let R = bands[bi].radius
                for i in 1..<(n - 1) {
                    let floorZ = boardSurfaceZ(x: bands[bi].positions[i].x, y: bands[bi].positions[i].y) + R
                    if bands[bi].positions[i].z <= floorZ + 1e-4 {
                        let vx = bands[bi].positions[i].x - bands[bi].previousPositions[i].x
                        let vy = bands[bi].positions[i].y - bands[bi].previousPositions[i].y
                        let velLen = sqrtf(vx * vx + vy * vy)
                        if velLen > 1e-8 {
                            let scale = max(0.0, 1.0 - boardMu)
                            bands[bi].previousPositions[i].x = bands[bi].positions[i].x - vx * scale
                            bands[bi].previousPositions[i].y = bands[bi].positions[i].y - vy * scale
                        }
                    }
                }
            }
        }

        profiler.end(.postCollision)

        profiler.setCounter("pairs", collisionPairs.count)
        profiler.setCounter("particles", bands.first?.positions.count ?? 0)
        profiler.setCounter("bands", active.count)
        profiler.setCounter("effIter", effectiveIters)
        profiler.logIfNeeded()
    }



    /// Redistribute particles by curvature: dense at bends, sparse on straight segments.
    /// Uses curvature-weighted arc-length so particles concentrate where the rope curves.
    private func resampleBand(_ bi: Int) {
        let n = bands[bi].positions.count
        guard n >= 4 else { return }

        let pos = bands[bi].positions
        let prev = bands[bi].previousPositions
        let twist = bands[bi].twistAngles
        let prevTwist = bands[bi].previousTwistAngles

        var curvature = [Float](repeating: 0, count: n)
        for i in 1..<(n - 1) {
            let d0 = pos[i] - pos[i - 1]
            let d1 = pos[i + 1] - pos[i]
            let len0 = simd_length(d0)
            let len1 = simd_length(d1)
            if len0 > 1e-9 && len1 > 1e-9 {
                let cosA = simd_dot(d0, d1) / (len0 * len1)
                curvature[i] = max(1.0 - cosA, 0)
            }
        }

        let curvatureScale: Float = 8.0
        var wArcLen = [Float](repeating: 0, count: n)
        for i in 1..<n {
            let segLen = simd_length(pos[i] - pos[i - 1])
            let avgCurv = (curvature[i - 1] + curvature[i]) * 0.5
            let weight = 1.0 + curvatureScale * avgCurv
            wArcLen[i] = wArcLen[i - 1] + segLen * weight
        }
        let totalW = wArcLen[n - 1]
        guard totalW > 1e-6 else { return }

        var arcLen = [Float](repeating: 0, count: n)
        for i in 1..<n {
            arcLen[i] = arcLen[i - 1] + simd_length(pos[i] - pos[i - 1])
        }

        var maxSeg: Float = 0, minSeg: Float = Float.greatestFiniteMagnitude
        for i in 0..<(n - 1) {
            let s = arcLen[i + 1] - arcLen[i]
            maxSeg = max(maxSeg, s)
            if s > 1e-9 { minSeg = min(minSeg, s) }
        }
        guard minSeg < 1e-9 || maxSeg / max(minSeg, 1e-9) > 1.5 else { return }

        let idealW = totalW / Float(n - 1)
        var seg = 0
        for i in 1..<(n - 1) {
            let targetW = idealW * Float(i)
            while seg < n - 2 && wArcLen[seg + 1] < targetW {
                seg += 1
            }
            let wStart = wArcLen[seg]
            let wLen = wArcLen[seg + 1] - wStart
            let t = wLen > 1e-9 ? (targetW - wStart) / wLen : 0

            bands[bi].positions[i] = pos[seg] + (pos[seg + 1] - pos[seg]) * t
            bands[bi].previousPositions[i] = prev[seg] + (prev[seg + 1] - prev[seg]) * t
            bands[bi].twistAngles[i] = twist[seg] + (twist[seg + 1] - twist[seg]) * t
            bands[bi].previousTwistAngles[i] = prevTwist[seg] + (prevTwist[seg + 1] - prevTwist[seg]) * t
        }
    }

    private func bandConstraints(_ bi: Int, dt: Float) {
        let n = bands[bi].positions.count
        let segLen = bands[bi].segmentLength * currentTension
        let alpha = max(bendCompliance, 0) / max(dt * dt, 1e-8)
        let bendCoupling = max(0, min(bendVelocityCoupling, 1))
        let pinS = bands[bi].pinStart
        let pinE = bands[bi].pinEnd
        let holeS = pinS.map { holePosition3D($0) }
        let holeE = pinE.map { holePosition3D($0) }
        let cs = bands[bi].crossSection
        let isRect = cs.isRectangular
        let R = bands[bi].radius
        let frames = (isRect && cachedFrames.indices.contains(bi) && cachedFrames[bi].count == n) ? cachedFrames[bi] : []

        var prev = bands[bi].previousPositions
        bands[bi].positions.withUnsafeMutableBufferPointer { pos in
            for offset in 0...1 {
                var idx = offset
                while idx < n - 1 {
                    let diff = pos[idx + 1] - pos[idx]
                    let dist2 = simd_dot(diff, diff)
                    if dist2 > 1e-12 {
                        let dist = sqrtf(dist2)
                        let corr = diff * ((dist - segLen) / dist * 0.5)
                        if idx > 0 { pos[idx] += corr }
                        if idx + 1 < n - 1 { pos[idx + 1] -= corr }
                    }
                    idx += 2
                }
            }

            if n >= 3 {
                for i in 1..<(n - 1) {
                    let p0 = pos[i - 1]
                    let p1 = pos[i]
                    let p2 = pos[i + 1]
                    let curvature = p0 - p1 * 2 + p2
                    let c2 = simd_dot(curvature, curvature)
                    if c2 < 1e-14 { continue }

                    let denom = 6.0 + alpha
                    let lambda = curvature / denom

                    var d0 = -lambda
                    var d1 = lambda * 2
                    var d2 = -lambda

                    if i - 1 == 0 {
                        d1 += d0 * 0.5
                        d2 += d0 * 0.5
                        d0 = .zero
                    }
                    if i + 1 == n - 1 {
                        d0 += d2 * 0.5
                        d1 += d2 * 0.5
                        d2 = .zero
                    }

                    pos[i - 1] += d0
                    pos[i] += d1
                    pos[i + 1] += d2

                    prev[i - 1] += d0 * bendCoupling
                    prev[i] += d1 * bendCoupling
                    prev[i + 1] += d2 * bendCoupling
                }
            }

            if let hp = holeS { pos[0] = hp }
            if let hp = holeE { pos[n - 1] = hp }

            if !frames.isEmpty {
                let upN = SIMD3<Float>(0, 0, 1)
                for i in 1..<(n - 1) {
                    let frame = frames[i]
                    let zExtent = cs.effectiveRadius(normal: upN, d1: frame.d1, d2: frame.d2)
                    let floorZ = boardSurfaceZ(x: pos[i].x, y: pos[i].y) + zExtent
                    if pos[i].z < floorZ { pos[i].z = floorZ }
                }
            } else {
                for i in 1..<(n - 1) {
                    let floorZ = boardSurfaceZ(x: pos[i].x, y: pos[i].y) + R
                    if pos[i].z < floorZ { pos[i].z = floorZ }
                }
            }
        }
        bands[bi].previousPositions = prev

        if isRect {
            let stiffness = twistStiffness
            for i in 0..<(n - 1) {
                let diff = bands[bi].twistAngles[i + 1] - bands[bi].twistAngles[i]
                let corr = diff * stiffness * 0.5
                if i > 0 { bands[bi].twistAngles[i] += corr }
                if i + 1 < n - 1 { bands[bi].twistAngles[i + 1] -= corr }
            }
            let zeroRestoring: Float = 0.15
            for i in 1..<(n - 1) {
                bands[bi].twistAngles[i] *= (1.0 - zeroRestoring)
            }
        }
    }

    // MARK: - Collision

    struct CollisionPair {
        let bandA: UInt16
        let segA: UInt16
        let bandB: UInt16
        let segB: UInt16
    }

    /// Broadphase: AABB sweep between band pairs and within the same band.
    func buildCollisionPairs(_ activeBands: [Int]) -> [CollisionPair] {
        guard !activeBands.isEmpty else { return [] }
        var pairs: [CollisionPair] = []
        pairs.reserveCapacity(512)

        for ai in 0..<activeBands.count {
            let bi = activeBands[ai]
            let posI = bands[bi].positions
            let segsI = posI.count - 1
            let ri = bands[bi].crossSection.collisionRadius

            if segsI > 4 {
                let minDist = ri + ri
                let segLen = bands[bi].segmentLength
                let selfSkip = segLen > 1e-6 ? max(4, Int(ceil(minDist * 2.5 / segLen))) : 4
                for si in 0..<segsI {
                    let a0 = posI[si]
                    let a1 = posI[si + 1]
                    let aMinX = min(a0.x, a1.x) - minDist
                    let aMaxX = max(a0.x, a1.x) + minDist
                    let aMinY = min(a0.y, a1.y) - minDist
                    let aMaxY = max(a0.y, a1.y) + minDist
                    let sjStart = si + selfSkip
                    guard sjStart < segsI else { continue }
                    for sj in sjStart..<segsI {
                        let b0 = posI[sj]
                        let b1 = posI[sj + 1]
                        if max(b0.x, b1.x) < aMinX || min(b0.x, b1.x) > aMaxX { continue }
                        if max(b0.y, b1.y) < aMinY || min(b0.y, b1.y) > aMaxY { continue }
                        pairs.append(CollisionPair(bandA: UInt16(bi), segA: UInt16(si), bandB: UInt16(bi), segB: UInt16(sj)))
                    }
                }
            }

            for aj in (ai + 1)..<activeBands.count {
                let bj = activeBands[aj]
                let posJ = bands[bj].positions
                let segsJ = posJ.count - 1
                let minDist = ri + bands[bj].crossSection.collisionRadius

                for si in 0..<segsI {
                    let a0 = posI[si]
                    let a1 = posI[si + 1]
                    let aMinX = min(a0.x, a1.x) - minDist
                    let aMaxX = max(a0.x, a1.x) + minDist
                    let aMinY = min(a0.y, a1.y) - minDist
                    let aMaxY = max(a0.y, a1.y) + minDist

                    for sj in 0..<segsJ {
                        let b0 = posJ[sj]
                        let b1 = posJ[sj + 1]
                        if max(b0.x, b1.x) < aMinX || min(b0.x, b1.x) > aMaxX { continue }
                        if max(b0.y, b1.y) < aMinY || min(b0.y, b1.y) > aMaxY { continue }
                        pairs.append(CollisionPair(bandA: UInt16(bi), segA: UInt16(si), bandB: UInt16(bj), segB: UInt16(sj)))
                    }
                }
            }
        }
        return pairs
    }

    @discardableResult
    private func resolveCollisionPairs(_ pairs: [CollisionPair], injectVelocity: Bool = false) -> Bool {
        var found = false
        for p in pairs {
            if collideSegments(Int(p.bandA), Int(p.segA), Int(p.bandB), Int(p.segB), injectVelocity: injectVelocity) {
                found = true
            }
        }
        return found
    }

    /// Collide two specific segments from different bands.
    /// For rectangular cross-sections, uses oriented effective radius via Minkowski support function.
    /// Applies Coulomb friction in the tangential plane to simulate rubber grip.
    /// Accounts for latex thinning: stretched segments have reduced collision radius matching visual mesh.
    @inline(__always)
    private func collideSegments(_ bi: Int, _ si: Int, _ bj: Int, _ sj: Int, injectVelocity: Bool) -> Bool {
        let maxDist = bands[bi].crossSection.collisionRadius + bands[bj].crossSection.collisionRadius
        let maxDist2 = maxDist * maxDist

        let a0 = bands[bi].positions[si]
        let a1 = bands[bi].positions[si + 1]
        let b0 = bands[bj].positions[sj]
        let b1 = bands[bj].positions[sj + 1]

        let d1 = a1 - a0
        let d2 = b1 - b0
        let r = a0 - b0
        let a = simd_dot(d1, d1)
        let e = simd_dot(d2, d2)
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

        guard dist2 < maxDist2 else { return false }

        let dist: Float
        let normal: SIMD3<Float>
        if dist2 > 1e-12 {
            dist = sqrtf(dist2)
            normal = diff / dist
        } else {
            let axis = simd_cross(d1, d2)
            let axisLen2 = simd_dot(axis, axis)
            if axisLen2 > 1e-12 {
                normal = simd_normalize(axis)
            } else {
                let tangent = simd_length_squared(d1) > 1e-12 ? simd_normalize(d1) : SIMD3<Float>(1, 0, 0)
                let up = abs(tangent.z) < 0.9 ? SIMD3<Float>(0, 0, 1) : SIMD3<Float>(0, 1, 0)
                normal = simd_normalize(simd_cross(tangent, up))
            }
            dist = 1e-6
        }

        let effRadiusA = effectiveCollisionRadius(bandIndex: bi, segIndex: si, param: s, normal: normal)
        let effRadiusB = effectiveCollisionRadius(bandIndex: bj, segIndex: sj, param: t, normal: normal)
        let minDist = effRadiusA + effRadiusB

        guard dist < minDist else { return false }

        let overlap = minDist - dist
        let corr = normal * (overlap * 0.35)

        bands[bi].positions[si] += corr * (1 - s)
        bands[bi].positions[si + 1] += corr * s
        bands[bj].positions[sj] -= corr * (1 - t)
        bands[bj].positions[sj + 1] -= corr * t

        let mu = frictionCoefficient
        if mu > 0 {
            let velA = (bands[bi].positions[si] - bands[bi].previousPositions[si]) * (1 - s)
                     + (bands[bi].positions[si + 1] - bands[bi].previousPositions[si + 1]) * s
            let velB = (bands[bj].positions[sj] - bands[bj].previousPositions[sj]) * (1 - t)
                     + (bands[bj].positions[sj + 1] - bands[bj].previousPositions[sj + 1]) * t
            let relVel = velA - velB
            let tangent = relVel - normal * simd_dot(relVel, normal)
            let tangentLen = simd_length(tangent)
            let minSlide: Float = 0.0002
            if tangentLen > minSlide {
                let maxFriction = mu * overlap * 0.25
                let frictionMag = min(tangentLen * 0.3, maxFriction)
                let frictionDir = tangent / tangentLen
                let velCorr = frictionDir * frictionMag

                bands[bi].previousPositions[si] += velCorr * (1 - s)
                bands[bi].previousPositions[si + 1] += velCorr * s
                bands[bj].previousPositions[sj] -= velCorr * (1 - t)
                bands[bj].previousPositions[sj + 1] -= velCorr * t

                let contactPos = (closestA + closestB) * 0.5
                frictionAccumulator += overlap
                frictionSpeedAccumulator += tangentLen
                frictionPositionAccumulator += contactPos
                frictionSampleCount += 1
            }
        }

        if injectVelocity {
            let velCorr = corr * 0.15
            bands[bi].previousPositions[si] -= velCorr * (1 - s)
            bands[bi].previousPositions[si + 1] -= velCorr * s
            bands[bj].previousPositions[sj] += velCorr * (1 - t)
            bands[bj].previousPositions[sj + 1] += velCorr * t
        }
        return true
    }

    @inline(__always)
    private func effectiveCollisionRadius(bandIndex bi: Int, segIndex si: Int, param s: Float, normal: SIMD3<Float>) -> Float {
        let cs = bands[bi].crossSection
        guard cs.isRectangular,
              cachedFrames.indices.contains(bi),
              cachedFrames[bi].count > si + 1 else {
            return bands[bi].radius
        }
        let f0 = cachedFrames[bi][si]
        let f1 = cachedFrames[bi][si + 1]
        let d1 = simd_normalize(f0.d1 * (1 - s) + f1.d1 * s)
        let d2 = simd_normalize(f0.d2 * (1 - s) + f1.d2 * s)
        return cs.effectiveRadius(normal: normal, d1: d1, d2: d2)
    }

    @inline(__always)
    private func latexThinningFactor(bandIndex bi: Int, segIndex si: Int, param s: Float) -> Float {
        let band = bands[bi]
        let n = band.positions.count
        guard n >= 2 else { return 1.0 }

        let segLen = band.segmentLength
        guard segLen > 1e-6 else { return 1.0 }

        let actualLen = simd_length(band.positions[si + 1] - band.positions[si])
        let localStretch = max(0.0, actualLen / segLen - 1.0)

        let particleT = (Float(si) + s) / Float(n - 1)
        let center = sin(particleT * Float.pi)
        let centerMask = center * center * center * center

        let tension = localStretch * centerMask
        return 1.0 / sqrt(max(1.0, 1.0 + tension * stretchThinning * 3.0))
    }

    var cachedFrames: [[MaterialFrame]] = []

    

}
