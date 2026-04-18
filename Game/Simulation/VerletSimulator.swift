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

    // MARK: - Weight (tension mode)

    struct Weight {
        var position: SIMD2<Float>
        var previousPosition: SIMD2<Float>
        var mass: Float
        var radius: Float
        var targetPosition: SIMD2<Float>?
        var targetRadius: Float
        var settled: Bool = false
        var attachedBandEnds: [(bandIndex: Int, endIndex: Int)] = []
    }

    var weights: [Weight] = []
    var isTensionMode: Bool = false
    var isRailMode: Bool = false
    var isRescueMode: Bool = false

    // MARK: - Platform (rescue mode)

    struct Platform {
        var corners: [SIMD3<Float>]          // 4 corner positions (TL, TR, BR, BL)
        var oldCorners: [SIMD3<Float>]       // previous positions for Verlet
        var restEdges: [(Int, Int, Float)]   // 6 distance constraints (4 edges + 2 diags)
        var mass: Float                       // total platform mass
        var width: Float
        var height: Float
        /// Which rope (band) is attached to which corner. Key=corner index, Value=band index
        var attachedBands: [Int: Int]
        /// Corner indices of empty slots (where free ropes should be inserted)
        var emptySlots: [Int]
        /// Band indices of free ropes (one per empty slot)
        var freeRopeIndices: [Int]
        /// How many free ropes have been connected
        var connectedCount: Int = 0
        /// Seconds the platform has been stable (for win condition)
        var stableTime: Float = 0
        /// Wind: random impulse timer
        var windTimer: Float = 0
        /// Current wind force
        var windForce: SIMD3<Float> = .zero

        // Convenience accessors
        var emptySlot: Int { emptySlots.first ?? 0 }
        var freeRopeIndex: Int { freeRopeIndices.first ?? 0 }
        var freeRopeConnected: Bool { connectedCount >= freeRopeIndices.count }
    }

    var platform: Platform?

    // Weight physics parameters
    var weightStaticFriction: Float = 0.02
    var weightKineticFriction: Float = 0.01
    var weightDamping: Float = 0.95
    var weightSettleThreshold: Float = 0.005

    // MARK: - Rail mode

    struct Rail {
        let points: [SIMD2<Float>]
        let segLengths: [Float]
        let totalLength: Float

        init(points: [SIMD2<Float>]) {
            self.points = points
            var lens: [Float] = []
            var total: Float = 0
            for i in 1..<points.count {
                let l = simd_length(points[i] - points[i-1])
                lens.append(l)
                total += l
            }
            self.segLengths = lens
            self.totalLength = total
        }

        func position(at t: Float) -> SIMD2<Float> {
            guard points.count >= 2, totalLength > 0 else { return points.first ?? .zero }
            let clamped = max(0, min(1, t))
            let targetLen = clamped * totalLength
            var accum: Float = 0
            for i in 0..<segLengths.count {
                if accum + segLengths[i] >= targetLen {
                    let local = (targetLen - accum) / max(segLengths[i], 1e-6)
                    return points[i] * (1 - local) + points[i+1] * local
                }
                accum += segLengths[i]
            }
            return points.last!
        }

        func tangent(at t: Float) -> SIMD2<Float> {
            guard points.count >= 2, totalLength > 0 else { return SIMD2(1, 0) }
            let clamped = max(0, min(1, t))
            let targetLen = clamped * totalLength
            var accum: Float = 0
            for i in 0..<segLengths.count {
                if accum + segLengths[i] >= targetLen {
                    let dir = points[i+1] - points[i]
                    let len = simd_length(dir)
                    return len > 1e-6 ? dir / len : SIMD2(1, 0)
                }
                accum += segLengths[i]
            }
            let dir = points[points.count-1] - points[points.count-2]
            let len = simd_length(dir)
            return len > 1e-6 ? dir / len : SIMD2(1, 0)
        }

        func nearestT(to pos: SIMD2<Float>) -> Float {
            guard points.count >= 2 else { return 0 }
            var bestDist: Float = .greatestFiniteMagnitude
            var bestArc: Float = 0
            var accum: Float = 0
            for i in 0..<segLengths.count {
                let a = points[i]
                let b = points[i+1]
                let dir = b - a
                let segLen = segLengths[i]
                let proj = segLen > 1e-6 ? max(0, min(segLen, simd_dot(pos - a, dir / segLen))) : 0
                let closest = a + (dir / max(segLen, 1e-6)) * proj
                let dist = simd_length(pos - closest)
                if dist < bestDist {
                    bestDist = dist
                    bestArc = accum + proj
                }
                accum += segLen
            }
            return totalLength > 0 ? bestArc / totalLength : 0
        }
    }

    struct Cart {
        var t: Float              // position on rail [0,1]
        var velocity: Float = 0   // speed along rail (arc-length/s)
        var railIndex: Int
        var radius: Float
        var mass: Float
        var settled: Bool = false
    }

    struct Station {
        let railIndex: Int
        let t: Float
        let radius: Float
        let cartIndex: Int
    }

    var rails: [Rail] = []
    var carts: [Cart] = []
    var railStations: [Station] = []

    // Rail physics
    var cartDamping: Float = 0.92
    var cartMaxSpeed: Float = 3.0
    var cartSettleThreshold: Float = 0.003

    var bands: [Band] = []

    /// 2D crossing pairs from previous step with Z ordering and side info for CCD
    struct CrossingRecord {
        let bandA: UInt16
        let segA: UInt16
        let bandB: UInt16
        let segB: UInt16
        let aAboveB: Bool   // sign of (zA - zB) at crossing point
        let sideOfB: Float  // sign of 2D cross product — which side of A is B's crossing point on
        let paramT: Float   // parameter on A at crossing
        let paramU: Float   // parameter on B at crossing
    }
    private var _prevCrossingRecords: [CrossingRecord] = []
    let holePositions: [SIMD2<Float>]
    let holeElevations: [Float]
    let holeRadius: Float
    let holeDepth: Float
    let boards: [BoardDef]
    var padMode: Bool = false
    var padHeight: Float = 0.18
    var padNeckHeight: Float = 0.08   // world-space vertical stub height at pinned pad
    var padNeckStiffness: Float = 0.5 // soft-pin strength per iteration (0 = free, 1 = hard)
    var holeRadiusScale: Float = 1.0

    // Physics parameters (tuneable)
    var gravity: Float = -14.298969268798828
    /// Normalized direction for gravity. (0,0,1) = into table (phone flat), (0,-1,0) = down screen (phone vertical).
    var gravityDirection: SIMD3<Float> = SIMD3<Float>(0, 0, 1)
    var damping: Float = 0.92867755889892578
    var constraintIterations: Int = 2 {
        didSet { constraintIterations = max(constraintIterations, 2) }
    }
    var broadphaseRebuildInterval: Int = 3
    /// Minimum physics substeps per frame during drag (prevents tunneling under high tension).
    var dragMinSubsteps: Int = 3
    var dragPickupDuration: Float = 0.12
    var settleSteps: Int = 5
    var liftHeight: Float = 0.30000001192092896
    var ropeTension: Float = 0.98000001907348633
    var currentTension: Float = 1.0
    private let tensionSpeed: Float = 0.5
    var frictionCoefficient: Float = 0.8
    var frictionDampingRatio: Float = 0.3
    var maxFrictionCap: Float = 0.25
    var boardFrictionRatio: Float = 0.5
    var collisionResponse: Float = 0.35
    var zSeparationStrength: Float = 1.0
    var particleCount: Int = 6
    var bendCompliance: Float = 0
    var bendVelocityCoupling: Float = 0.44999998807907104
    var twistStiffness: Float = 0.15 { didSet { wakeUp() } }
    var twistDamping: Float = 0.4 { didSet { wakeUp() } }
    var gravityTorqueStrength: Float = 0.8 { didSet { wakeUp() } }
    var stretchThinning: Float = 0.5
    var squareCrossSection: Bool = false

    private let dt: Float = 1.0 / 120.0  // fixed dt, supports ProMotion 120fps
    private var accumulator: Float = 0
    var dragTargetPos: SIMD3<Float>?
    var dragStartPos: SIMD3<Float>?
    var dragPickupElapsed: Float = .greatestFiniteMagnitude
    var logTimer: Float = 0
    var logEvery: Float = 1.0  // log every N seconds

    var dragInfo: DragInfo?

    // MARK: - Pre-allocated scratch buffers (avoid per-frame heap allocations)
    private var activeIndices: [Int] = []
    private var collisionPairsBuf: [CollisionPair] = []
    private var lowerKeysToRemove: [LowerAnimationKey] = []

    // MARK: - Idle sleep
    var fadeOutSpeed: Float = 45.0
    var lowerAnimDuration: Float = 0.55
    var idleTimeout: Float = 3.0
    private var idleTimer: Float = 0
    private(set) var isSleeping: Bool = false

    func wakeUp() {
        idleTimer = 0
        isSleeping = false
    }

    // MARK: - Friction sound feedback

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

        if dragInfo != nil || hasLowerAnimations || isRescueMode {
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

        // Advance fade-out animations
        for i in bands.indices where bands[i].fadeOut > 0 && bands[i].active {
            let n = bands[i].positions.count

            if padMode {
                // Pad mode: contract toward center + lift off + fade opacity
                let duration: Float = 0.45
                bands[i].suckConsumed += clampedDt
                let t = min(bands[i].suckConsumed / duration, 1.0)
                // Ease-in-out for smooth contraction
                let eased = t * t * (3.0 - 2.0 * t)

                let origPositions = bands[i].suckOrigPositions
                guard origPositions.count == n else { continue }

                // Compute center of rope
                var center = SIMD3<Float>.zero
                for k in 0..<n { center += origPositions[k] }
                center /= Float(n)

                for k in 0..<n {
                    let orig = origPositions[k]
                    // Contract toward center
                    let contracted = orig + (center - orig) * eased * 0.85
                    // Lift up
                    let lift = eased * 0.15
                    bands[i].positions[k] = SIMD3<Float>(contracted.x, contracted.y, contracted.z + lift)
                    bands[i].previousPositions[k] = bands[i].positions[k]
                }

                bands[i].fadeOut = min(eased, 0.999)
                if t >= 1.0 {
                    bands[i].fadeOut = 1
                    bands[i].active = false
                    bands[i].pinStart = nil
                    bands[i].pinEnd = nil
                    bands[i].suckHole = nil
                }
            } else {
                // Standard mode: suck into hole
                guard let hole = bands[i].suckHole else { continue }
                let holeXY = holePositions[hole]
                let holeElev = holeSurfaceZ(hole)
                let holeBelow = SIMD3<Float>(holeXY.x, holeXY.y, holeElev - holeDepth)

                let totalArcEst = bands[i].suckSegLengths.reduce(0, +)
                let progress = totalArcEst > 1e-6 ? min(bands[i].suckConsumed / totalArcEst, 1.0) : 0
                let accel = 1.0 + 2.0 * progress * progress
                let pullSpeed = fadeOutSpeed * bands[i].segmentLength * accel
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
            if isRescueMode, let plat = platform {
                let c0 = plat.corners[0]
                let c1 = plat.corners[1]
                let c2 = plat.corners[2]
                let c3 = plat.corners[3]
                Self.logger.info("[PLATFORM] corners z=[\(String(format:"%.3f",c0.z)),\(String(format:"%.3f",c1.z)),\(String(format:"%.3f",c2.z)),\(String(format:"%.3f",c3.z))] attached=\(plat.attachedBands.count) free=\(plat.freeRopeConnected) stable=\(String(format:"%.1f",plat.stableTime))")
            }
        }

        if let drag = dragInfo, let target = dragTargetPos {
            let dragFrameStart = CACurrentMediaTime()
            let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
            let startPos = dragStartPos ?? bands[drag.bandIndex].positions[idx]
            let resolvedTarget: SIMD3<Float>
            if dragPickupElapsed < dragPickupDuration {
                let pickupT = dragPickupDuration > 0 ? min(dragPickupElapsed / dragPickupDuration, 1.0) : 1.0
                let maxSpeed = 8.0 + (26.0 - 8.0) * pickupT
                let maxDistance = maxSpeed * dt * Float(max(n, dragMinSubsteps))
                let delta = target - startPos
                let distance = simd_length(delta)
                if distance > maxDistance && distance > 1e-6 {
                    resolvedTarget = startPos + delta * (maxDistance / distance)
                } else {
                    resolvedTarget = target
                }
                dragPickupElapsed = min(dragPickupElapsed + dt * Float(n), dragPickupDuration)
            } else {
                resolvedTarget = target
            }

            // Run at least dragMinSubsteps during drag to prevent tunneling.
            // The endpoint is interpolated across substeps so each substep moves it less,
            // giving collision resolution enough iterations to maintain separation.
            let substeps = max(n, dragMinSubsteps)
            let subDt = dt * Float(n) / Float(substeps)
            profiler.setCounter("dragSub", substeps)
            for s in 1...substeps {
                let t = Float(s) / Float(substeps)
                let interpPos = startPos + (resolvedTarget - startPos) * t
                bands[drag.bandIndex].positions[idx] = interpPos
                bands[drag.bandIndex].previousPositions[idx] = interpPos
                verletStep(collide: true, dt: subDt, updateCrossingState: s == substeps)
            }
            profiler.record(.dragFrame, microseconds: (CACurrentMediaTime() - dragFrameStart) * 1e6)
            dragStartPos = resolvedTarget
        } else {
            for _ in 0..<n {
                verletStep(collide: true, dt: dt)
            }
        }

        // Post-drag lower animation
        updateLowerAnimation(deltaTime: clampedDt)

        // Weight physics (tension mode) — runs per-frame, not per-substep
        if isTensionMode && !weights.isEmpty {
            updateWeights(dt: clampedDt)
        }

        // Platform stability check (rescue mode) — win condition timer, runs per-frame
        if isRescueMode, var plat = platform {
            if plat.freeRopeConnected {
                var maxVel: Float = 0
                for i in 0..<4 {
                    let vel = simd_length(plat.corners[i] - plat.oldCorners[i])
                    maxVel = max(maxVel, vel)
                }
                if maxVel < 0.001 {
                    plat.stableTime += clampedDt
                } else {
                    plat.stableTime = max(0, plat.stableTime - clampedDt * 0.5)
                }
                platform = plat
            }
        }

        // logCrossingState disabled — costs 8% CPU (O(n²) per band pair)
    }

    private func updateLowerAnimation(deltaTime: Float) {
        guard !lowerAnimations.isEmpty else { return }

        lowerKeysToRemove.removeAll(keepingCapacity: true)
        for (key, var anim) in lowerAnimations {
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
            let t = min(anim.timer / lowerAnimDuration, 1.0)
            let eased: Float
            if padMode {
                // Magnetic snap via cubic Hermite: C1-smooth, no derivative kink
                // p0=0, p1=1, m0=tangent at start, m1=tangent at end
                // m0=0.8 (gentle start), m1=2.5 (magnetic acceleration into pad)
                let t2 = t * t
                let t3 = t2 * t
                let m0: Float = 0.8
                let m1: Float = 2.5
                // Hermite basis: h00*p0 + h10*m0 + h01*p1 + h11*m1 (p0=0, p1=1)
                let h10 = t3 - 2 * t2 + t           // *m0
                let h01 = -2 * t3 + 3 * t2          // *p1 = 1
                let h11 = t3 - t2                    // *m1
                eased = h10 * m0 + h01 + h11 * m1
            } else {
                eased = 1.0 - (1.0 - t) * (1.0 - t)
            }
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
                lowerKeysToRemove.append(key)
                continue
            }

            lowerAnimations[key] = anim
        }
        for key in lowerKeysToRemove { lowerAnimations.removeValue(forKey: key) }
    }

    // MARK: - Weight physics

    /// Returns 3D position for a pin index. Positive = hole, negative = weight (-(weightIndex+1)).
    func pinPosition3D(_ pinIndex: Int) -> SIMD3<Float> {
        if pinIndex >= 0 {
            return holePosition3D(pinIndex)
        } else {
            let wi = -(pinIndex + 1)
            guard weights.indices.contains(wi) else { return .zero }
            return SIMD3<Float>(weights[wi].position.x, weights[wi].position.y, boardSurfaceZ(x: weights[wi].position.x, y: weights[wi].position.y))
        }
    }

    /// Weight index from negative pin index
    static func weightIndex(from pinIndex: Int) -> Int {
        return -(pinIndex + 1)
    }

    /// Pin index from weight index
    static func weightPinIndex(_ weightIndex: Int) -> Int {
        return -(weightIndex + 1)
    }

    private func updateWeights(dt: Float) {
        let weightStiffness: Float = 50.0

        for wi in weights.indices {
            // Compute total tension force from all attached band ends
            var totalForce = SIMD2<Float>.zero

            for (bi, endIdx) in weights[wi].attachedBandEnds {
                guard bands.indices.contains(bi), bands[bi].active else { continue }
                let n = bands[bi].positions.count

                // Use adjacent particle (not far anchor) to get actual local tension direction.
                // When a rope wraps around a weight, the real pull comes from the nearest segment,
                // not from the far hole. This prevents erratic forces during entanglement.
                let neighborIdx = endIdx == 0 ? 1 : (n - 2)
                guard neighborIdx >= 0 && neighborIdx < n else { continue }
                let neighborPos = bands[bi].positions[neighborIdx]

                let segRestLen = bands[bi].segmentLength * currentTension

                let wPos = SIMD2<Float>(weights[wi].position.x, weights[wi].position.y)
                let neighborXY = SIMD2<Float>(neighborPos.x, neighborPos.y)
                let diff = neighborXY - wPos
                let dist = simd_length(diff)

                // Force proportional to single-segment stretch
                let stretch = max(0, dist - segRestLen)
                if dist > 1e-6 && stretch > 0 {
                    let forceDir = diff / dist
                    totalForce += forceDir * stretch * weightStiffness
                }
            }

            // Static friction threshold
            let staticThreshold = weightStaticFriction * weights[wi].mass * abs(gravity)
            let forceLen = simd_length(totalForce)
            if forceLen < staticThreshold {
                // Not enough force — damp velocity to zero
                weights[wi].previousPosition = weights[wi].position
                continue
            }

            // Subtract kinetic friction
            let kineticFriction = weightKineticFriction * weights[wi].mass * abs(gravity)
            if forceLen > kineticFriction {
                totalForce -= simd_normalize(totalForce) * kineticFriction
            }

            // Simple Euler integration (more predictable than Verlet for this)
            let accel = totalForce / weights[wi].mass
            var vel = (weights[wi].position - weights[wi].previousPosition) / max(dt, 1e-6)
            vel = vel * weightDamping + accel * dt
            // Clamp velocity to prevent crazy movement
            let maxSpeed: Float = 2.0
            let speed = simd_length(vel)
            if speed > maxSpeed { vel = vel * (maxSpeed / speed) }

            weights[wi].previousPosition = weights[wi].position
            weights[wi].position = weights[wi].position + vel * dt

            // Update all band ends pinned to this weight
            let wPos3D = SIMD3<Float>(weights[wi].position.x, weights[wi].position.y,
                                      boardSurfaceZ(x: weights[wi].position.x, y: weights[wi].position.y))
            let pinIdx = Self.weightPinIndex(wi)
            for bi in bands.indices {
                if bands[bi].pinStart == pinIdx {
                    bands[bi].positions[0] = wPos3D
                    bands[bi].previousPositions[0] = wPos3D
                }
                let lastIdx = bands[bi].positions.count - 1
                if bands[bi].pinEnd == pinIdx {
                    bands[bi].positions[lastIdx] = wPos3D
                    bands[bi].previousPositions[lastIdx] = wPos3D
                }
            }

            // Check if settled in target
            if let target = weights[wi].targetPosition {
                let distToTarget = simd_length(weights[wi].position - target)
                weights[wi].settled = distToTarget < weights[wi].targetRadius && speed < weightSettleThreshold
            }
        }
    }

    var allWeightsSettled: Bool {
        guard isTensionMode && !weights.isEmpty else { return false }
        return weights.allSatisfy { $0.settled }
    }

    // MARK: - Platform (rescue mode)

    func initializePlatform(_ def: LevelDefinition.PlatformDef) {
        let hw = def.width / 2
        let hh = def.height / 2
        let z: Float = 0.3  // platform hangs at this Z

        // Corners: TL(0), TR(1), BR(2), BL(3)
        let corners: [SIMD3<Float>] = [
            SIMD3(-hw,  hh, z),  // TL
            SIMD3( hw,  hh, z),  // TR
            SIMD3( hw, -hh, z),  // BR
            SIMD3(-hw, -hh, z),  // BL
        ]

        // 6 constraints: 4 edges + 2 diagonals
        var edges: [(Int, Int, Float)] = []
        let pairs: [(Int, Int)] = [(0,1), (1,2), (2,3), (3,0), (0,2), (1,3)]
        for (a, b) in pairs {
            edges.append((a, b, simd_length(corners[a] - corners[b])))
        }

        var attachedBands: [Int: Int] = [:]
        for att in def.attachments {
            attachedBands[att.cornerIndex] = att.ropeIndex
        }

        platform = Platform(
            corners: corners,
            oldCorners: corners,
            restEdges: edges,
            mass: def.mass,
            width: def.width,
            height: def.height,
            attachedBands: attachedBands,
            emptySlots: def.emptySlots,
            freeRopeIndices: def.freeRopeIndices
        )

        // At start: ALL ropes connected (including "free" ones).
        // Free ropes will be detached after a brief animation delay.
        // This means at init, attach free ropes too.
        var allAttached = attachedBands
        for (i, freeIdx) in def.freeRopeIndices.enumerated() {
            if i < def.emptySlots.count {
                allAttached[def.emptySlots[i]] = freeIdx
            }
        }
        // Store all-connected state temporarily
        platform?.attachedBands = allAttached

        // Initialize ALL ropes: spread particles from ceiling anchor to platform corner
        for bandIdx in bands.indices {
            guard bands[bandIdx].pinStart != nil else { continue }
            let anchorHole = bands[bandIdx].pinStart!
            let anchorPos = holePosition3D(anchorHole)
            let n = bands[bandIdx].positions.count

            // Every rope goes to its assigned corner
            let bottomPos: SIMD3<Float>
            if let cornerIdx = allAttached.first(where: { $0.value == bandIdx })?.key {
                bottomPos = corners[cornerIdx]
            } else {
                continue
            }

            // Spread particles along straight line from anchor to bottom
            for i in 0..<n {
                let t = Float(i) / Float(max(1, n - 1))
                bands[bandIdx].positions[i] = anchorPos + (bottomPos - anchorPos) * t
            }
            bands[bandIdx].positions[0] = anchorPos
            bands[bandIdx].positions[n - 1] = bottomPos
            bands[bandIdx].previousPositions = bands[bandIdx].positions

            let dist = simd_length(bottomPos - anchorPos)
            bands[bandIdx].segmentLength = dist / Float(max(1, n - 1))
            bands[bandIdx].active = true

            Self.logger.info("[RESCUE-INIT] band=\(bandIdx) n=\(n) segLen=\(String(format: "%.4f", self.bands[bandIdx].segmentLength)) anchor=\(anchorHole)")
        }

        // All ropes have no pinEnd — they connect to platform via bilateral constraints
        for bandIdx in bands.indices {
            bands[bandIdx].pinEnd = nil
        }

        // No settle — platform starts mid-air and physics runs at runtime.
        // Free ropes will be detached after rescueBreakDelay seconds.
    }

    /// Connect a free rope to a specific empty slot
    func connectFreeRopeToPlatform(ropeIndex: Int, slotIndex: Int) {
        guard var plat = platform else { return }
        guard plat.emptySlots.contains(slotIndex) else { return }
        guard plat.freeRopeIndices.contains(ropeIndex) else { return }
        guard plat.attachedBands[slotIndex] == nil else { return }  // slot already filled
        plat.attachedBands[slotIndex] = ropeIndex
        plat.connectedCount += 1
        platform = plat
        Self.logger.info("[RESCUE] Rope \(ropeIndex) connected to slot \(slotIndex), \(plat.connectedCount)/\(plat.freeRopeIndices.count) done")
    }

    /// Legacy single-rope connect
    func connectFreeRopeToPlatform() {
        guard let plat = platform else { return }
        connectFreeRopeToPlatform(ropeIndex: plat.freeRopeIndex, slotIndex: plat.emptySlot)
    }

    var isPlatformStable: Bool {
        guard let plat = platform else { return false }
        return plat.freeRopeConnected && plat.stableTime >= 2.0
    }

    // updatePlatform removed — platform physics now runs inside verletStep() at 120Hz

    // MARK: - Rail mode initialization

    func initializeRails(railDefs: [LevelDefinition.RailDef],
                         cartDefs: [LevelDefinition.CartDef],
                         stationDefs: [LevelDefinition.StationDef]) {
        rails = railDefs.map { Rail(points: $0.points.map { $0.simd }) }
        carts = cartDefs.map { def in
            Cart(t: def.startT, railIndex: def.railIndex,
                 radius: def.radius ?? 0.12, mass: def.mass ?? 0.5)
        }
        railStations = stationDefs.map { def in
            Station(railIndex: def.railIndex, t: def.t,
                    radius: def.radius ?? 0.15, cartIndex: def.cartIndex)
        }
    }

    // MARK: - Cart physics

    var allCartsSettled: Bool {
        guard isRailMode && !carts.isEmpty else { return false }
        return carts.allSatisfy { $0.settled }
    }

    private let initDt: Float = 1.0 / 60.0

    func doSteps(_ n: Int, collide: Bool) {
        for _ in 0..<n {
            verletStep(collide: collide, dt: initDt)
        }
    }

    let profiler = PhysicsProfiler.shared

    private func verletStep(collide: Bool, dt: Float, updateCrossingState: Bool = true) {
        let dt2 = dt * dt
        profiler.begin()

        // 1. Verlet position update + velocity limiting
        let gravVec = gravityDirection * (gravity * dt2)
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
        // Platform Verlet integration (rescue mode) — runs per substep
        if isRescueMode, var plat = platform {
            // Wind: random impulses that change direction periodically
            plat.windTimer += dt
            if plat.windTimer > 0.4 + Float.random(in: 0...0.6) {
                plat.windTimer = 0
                let windStrength: Float = 0.3 * plat.mass
                plat.windForce = SIMD3<Float>(
                    Float.random(in: -windStrength...windStrength),
                    Float.random(in: -windStrength...windStrength),
                    Float.random(in: -windStrength * 0.3...windStrength * 0.1)
                ) * dt2
            }
            let windPerCorner = plat.windForce / 4.0

            let cornerGrav = gravityDirection * (gravity * dt2 * plat.mass / 4.0)
            for i in 0..<4 {
                let pos = plat.corners[i]
                let old = plat.oldCorners[i]
                var vel = (pos - old) * 0.98  // light damping — platform is heavy
                let maxVel: Float = 5.0 * dt   // allow real falling speed
                let vLen = simd_length(vel)
                if vLen > maxVel { vel *= maxVel / vLen }
                plat.oldCorners[i] = pos
                plat.corners[i] = pos + vel + cornerGrav + windPerCorner
            }
            platform = plat
        }

        profiler.end(.verletIntegration)

        // 2. Constraint + collision iterations (interleaved for robust PBD)
        activeIndices.removeAll(keepingCapacity: true)
        if collide {
            for i in bands.indices where bands[i].active && bands[i].fadeOut == 0 {
                activeIndices.append(i)
            }
        }
        let active = activeIndices

        // Scale iterations inversely with tension — stronger tension needs more solver work
        let effectiveIters = max(constraintIterations, Int(Float(constraintIterations) / max(currentTension, 0.3)))

        // Build collision pair list (broadphase). Rebuilt periodically during solve.
        if collide { profiler.measure(.broadphase) { rebuildCollisionPairs(active) } }

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

            // Platform shape + rope-corner constraints (rescue mode)
            if isRescueMode, var plat = platform {
                // Shape constraints: keep rectangle rigid
                for (a, b, rest) in plat.restEdges {
                    let delta = plat.corners[b] - plat.corners[a]
                    let dist = simd_length(delta)
                    guard dist > 1e-6 else { continue }
                    let correction = (dist - rest) / dist * 0.5
                    let offset = delta * correction
                    plat.corners[a] += offset
                    plat.corners[b] -= offset
                }

                // Rope-corner coupling via shared distance constraint.
                // Platform corner is heavy (mass/4), rope particle is light (1.0).
                // We solve a distance constraint (rest=0) between them, splitting
                // correction by inverse mass so the heavy corner moves less.
                let cornerInvMass = 4.0 / plat.mass  // 1/cornerMass
                let particleInvMass: Float = 1.0
                let totalInvMass = cornerInvMass + particleInvMass
                let cornerFrac = cornerInvMass / totalInvMass
                let particleFrac = particleInvMass / totalInvMass

                for (cornerIdx, bandIdx) in plat.attachedBands {
                    guard bands.indices.contains(bandIdx), bands[bandIdx].active else { continue }
                    let lastIdx = bands[bandIdx].positions.count - 1
                    let ropeEnd = bands[bandIdx].positions[lastIdx]
                    let corner = plat.corners[cornerIdx]
                    let delta = ropeEnd - corner
                    // Move both toward each other, weighted by inverse mass
                    plat.corners[cornerIdx] += delta * cornerFrac
                    bands[bandIdx].positions[lastIdx] -= delta * particleFrac
                    bands[bandIdx].previousPositions[lastIdx] = bands[bandIdx].positions[lastIdx]
                }

                // Floor constraint — far below, platform should hang in air
                let floorLimit: Float = -3.0
                for i in 0..<4 {
                    if plat.corners[i].z < floorLimit {
                        plat.corners[i].z = floorLimit
                    }
                }
                platform = plat
            }

            if collide {
                if broadphaseRebuildInterval > 0 && iter > 0 && iter % broadphaseRebuildInterval == 0 {
                    profiler.measure(.broadphase) { rebuildCollisionPairs(active) }
                }
                resolveCollisionPairs(collisionPairsBuf)
                profiler.measure(.crossingSolve) { resolve2DCrossingCollisions(active) }
            }
        }
        profiler.end(.constraints)

        profiler.begin()
        // Post-solve: collision-only passes until converged
        if collide {
            for _ in 0..<3 {
                let hadCollision = resolveCollisionPairs(collisionPairsBuf, injectVelocity: true)
                for bi in active {
                    let n = bands[bi].positions.count
                    if let startPin = bands[bi].pinStart {
                        let hp = pinPosition3D(startPin)
                        bands[bi].positions[0] = hp
                        bands[bi].previousPositions[0] = hp
                    }
                    if let endPin = bands[bi].pinEnd {
                        let hp = pinPosition3D(endPin)
                        bands[bi].positions[n - 1] = hp
                        bands[bi].previousPositions[n - 1] = hp
                    }
                    // Rescue mode: no floor — ropes hang freely in air
                    if !isRescueMode {
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
                }
                if !hadCollision { break }
            }
        }

        // Cart collision: push rope particles out of carts, accumulate force on cart
        if isRailMode && !carts.isEmpty {
            for ci in carts.indices {
                guard rails.indices.contains(carts[ci].railIndex) else { continue }
                let rail = rails[carts[ci].railIndex]
                let cartPos = rail.position(at: carts[ci].t)
                let cartTangent = rail.tangent(at: carts[ci].t)
                let cartR = carts[ci].radius
                let cartZ: Float = boardSurfaceZ(x: cartPos.x, y: cartPos.y)
                let cartHeight: Float = cartR * 2.5  // collision cylinder height

                var tangentImpulse: Float = 0

                for bi in active {
                    let n = bands[bi].positions.count
                    let ropeR = bands[bi].radius
                    let minDist = cartR + ropeR

                    for i in 0..<n {
                        let p = bands[bi].positions[i]
                        // Only collide if particle is at cart height
                        if p.z > cartZ + cartHeight + ropeR || p.z < cartZ - ropeR { continue }

                        let dx = p.x - cartPos.x
                        let dy = p.y - cartPos.y
                        let dist2D = sqrtf(dx * dx + dy * dy)

                        if dist2D < minDist && dist2D > 1e-6 {
                            let overlap = minDist - dist2D
                            let nx = dx / dist2D
                            let ny = dy / dist2D

                            // Push particle out
                            let isPinned = (i == 0 && bands[bi].pinStart != nil) || (i == n - 1 && bands[bi].pinEnd != nil)
                            if !isPinned {
                                bands[bi].positions[i].x += nx * overlap
                                bands[bi].positions[i].y += ny * overlap
                            }

                            // Accumulate force on cart along rail tangent
                            // Force direction is opposite to push (cart gets pushed by rope)
                            let forceOnCart = SIMD2<Float>(-nx, -ny) * overlap
                            tangentImpulse += simd_dot(forceOnCart, cartTangent) * 40.0
                        }
                    }
                }

                // Apply accumulated impulse to cart velocity
                let accel = tangentImpulse / carts[ci].mass
                carts[ci].velocity += accel * dt
                let speed = abs(carts[ci].velocity)
                if speed > cartMaxSpeed { carts[ci].velocity *= cartMaxSpeed / speed }

                // Move cart
                carts[ci].velocity *= cartDamping
                let tVel = carts[ci].velocity / max(rail.totalLength, 1e-6)
                carts[ci].t = max(0, min(1, carts[ci].t + tVel * dt))

                // Check settled
                for station in railStations where station.cartIndex == ci {
                    let stationPos = rails[station.railIndex].position(at: station.t)
                    let currentPos = rail.position(at: carts[ci].t)
                    let dist = simd_length(currentPos - stationPos)
                    carts[ci].settled = dist < station.radius && abs(carts[ci].velocity) < cartSettleThreshold
                }
            }
        }

        // Board friction: applied once per substep after all solving is done.
        // Uses true Verlet velocity (pos - previousPos) which is clean at this point.
        let boardMu = frictionCoefficient * boardFrictionRatio
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

        // CCD: fix tunneling by restoring Z order for pairs that were crossing last step
        if collide && updateCrossingState {
            profiler.measure(.crossingTunnel) { resolveTunneledCrossings() }
        }

        if collide && updateCrossingState {
            profiler.measure(.crossingRecords) { updateCrossingRecords(active) }
        }

        profiler.end(.postCollision)

        profiler.setCounter("pairs", collisionPairsBuf.count)
        profiler.setCounter("particles", bands.first?.positions.count ?? 0)
        profiler.setCounter("bands", active.count)
        profiler.setCounter("effIter", effectiveIters)
        profiler.logIfNeeded()
    }



    private func bandConstraints(_ bi: Int, dt: Float) {
        let n = bands[bi].positions.count
        let naturalSegLen = bands[bi].segmentLength
        let segLen = naturalSegLen * currentTension
        let alpha = max(bendCompliance, 0) / max(dt * dt, 1e-8)
        let bendCoupling = max(0, min(bendVelocityCoupling, 1))
        let pinS = bands[bi].pinStart
        let pinE = bands[bi].pinEnd
        let holeS = pinS.map { pinPosition3D($0) }
        let holeE = pinE.map { pinPosition3D($0) }
        let cs = bands[bi].crossSection
        let isRect = cs.isRectangular
        let R = bands[bi].radius
        let frames = (isRect && cachedFrames.indices.contains(bi) && cachedFrames[bi].count == n) ? cachedFrames[bi] : []

        var prev = ContiguousArray<SIMD3<Float>>()
        swap(&prev, &bands[bi].previousPositions)
        bands[bi].positions.withUnsafeMutableBufferPointer { pos in
            for offset in 0...1 {
                var idx = offset
                while idx < n - 1 {
                    let diff = pos[idx + 1] - pos[idx]
                    let dist2 = simd_dot(diff, diff)
                    if dist2 > 1e-12 {
                        let dist = sqrtf(dist2)
                        let corr = diff * ((dist - segLen) / dist * 0.5)
                        let startFree = pinS == nil
                        let endFree = pinE == nil
                        if idx > 0 || startFree { pos[idx] += corr }
                        if idx + 1 < n - 1 || endFree { pos[idx + 1] -= corr }
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

            // Pad mode: soft-pin a vertical "neck" of particles above each pad.
            // Stiffness tapers from max near the pin to 0 at neck top — rope blends
            // smoothly into free physics. Neck stays vertical under normal tension
            // but can yield to strong external forces.
            if padMode {
                let neckK = min(n / 3, max(1, Int(ceil(padNeckHeight / max(naturalSegLen, 1e-6)))))
                let maxStiffness = padNeckStiffness
                if holeS != nil {
                    let base = pos[0]
                    for k in 1...neckK where k < n - 1 {
                        let t = Float(k) / Float(neckK + 1)
                        let falloff = 1.0 - t * t * (3.0 - 2.0 * t)
                        let stiff = maxStiffness * falloff
                        let target = base + SIMD3<Float>(0, 0, Float(k) * naturalSegLen)
                        pos[k] += (target - pos[k]) * stiff
                    }
                }
                if holeE != nil {
                    let base = pos[n - 1]
                    for k in 1...neckK where n - 1 - k > 0 {
                        let t = Float(k) / Float(neckK + 1)
                        let falloff = 1.0 - t * t * (3.0 - 2.0 * t)
                        let stiff = maxStiffness * falloff
                        let target = base + SIMD3<Float>(0, 0, Float(k) * naturalSegLen)
                        pos[n - 1 - k] += (target - pos[n - 1 - k]) * stiff
                    }
                }
            }

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
        swap(&prev, &bands[bi].previousPositions)

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

    /// For pairs that cross in 2D, check Z separation at the exact 2D crossing point.
    /// This catches cases where the normal 3D closest-point collision misses due to long segments.
    private func resolve2DCrossingCollisions(_ activeBands: [Int]) {
        for ii in 0..<activeBands.count {
            let bi = activeBands[ii]
            let nA = bands[bi].positions.count
            for jj in (ii+1)..<activeBands.count {
                let bj = activeBands[jj]
                let nB = bands[bj].positions.count
                let minDist = bands[bi].crossSection.collisionRadius + bands[bj].crossSection.collisionRadius

                for si in 0..<(nA-1) {
                    let a0 = bands[bi].positions[si]
                    let a1 = bands[bi].positions[si+1]
                    let a02 = SIMD2<Float>(a0.x, a0.y)
                    let a12 = SIMD2<Float>(a1.x, a1.y)

                    for sj in 0..<(nB-1) {
                        let b0 = bands[bj].positions[sj]
                        let b1 = bands[bj].positions[sj+1]
                        let b02 = SIMD2<Float>(b0.x, b0.y)
                        let b12 = SIMD2<Float>(b1.x, b1.y)

                        // Check 2D crossing
                        let d1 = a12 - a02, d2 = b12 - b02
                        let cross2 = d1.x * d2.y - d1.y * d2.x
                        guard abs(cross2) > 1e-9 else { continue }
                        let dv = b02 - a02
                        let t2 = (dv.x * d2.y - dv.y * d2.x) / cross2
                        let u2 = (dv.x * d1.y - dv.y * d1.x) / cross2
                        guard t2 > 1e-6 && t2 < (1-1e-6) && u2 > 1e-6 && u2 < (1-1e-6) else { continue }

                        // 2D crossing confirmed — check Z separation at crossing point
                        let zA = a0.z + (a1.z - a0.z) * t2
                        let zB = b0.z + (b1.z - b0.z) * u2
                        let zDiff = zA - zB
                        let skin: Float = minDist * 0.5  // proactive margin
                        guard abs(zDiff) < minDist + skin else { continue }

                        // Enforce Z separation, maintaining current ordering
                        guard zSeparationStrength > 0 else { continue }
                        let sign: Float = zDiff >= 0 ? 1 : -1
                        let correction = ((minDist - abs(zDiff)) * 0.5 + 0.001) * zSeparationStrength

                        let aPinned0 = bands[bi].pinStart != nil && si == 0
                        let aPinned1 = bands[bi].pinEnd != nil && si == nA-2
                        let bPinned0 = bands[bj].pinStart != nil && sj == 0
                        let bPinned1 = bands[bj].pinEnd != nil && sj == nB-2

                        if !aPinned0 { bands[bi].positions[si].z += sign * correction * (1 - t2) }
                        if !aPinned1 { bands[bi].positions[si+1].z += sign * correction * t2 }
                        if !bPinned0 { bands[bj].positions[sj].z -= sign * correction * (1 - u2) }
                        if !bPinned1 { bands[bj].positions[sj+1].z -= sign * correction * u2 }
                    }
                }
            }
        }
    }

    /// Update the list of 2D crossing pairs with their Z ordering for the current step.
    private func updateCrossingRecords(_ activeBands: [Int]) {
        _prevCrossingRecords.removeAll(keepingCapacity: true)
        for ii in 0..<activeBands.count {
            let bi = activeBands[ii]
            let nA = bands[bi].positions.count
            for jj in (ii+1)..<activeBands.count {
                let bj = activeBands[jj]
                let nB = bands[bj].positions.count
                for si in 0..<(nA-1) {
                    let a0 = bands[bi].positions[si]
                    let a1 = bands[bi].positions[si+1]
                    let a02 = SIMD2<Float>(a0.x, a0.y)
                    let a12 = SIMD2<Float>(a1.x, a1.y)
                    for sj in 0..<(nB-1) {
                        let b0 = bands[bj].positions[sj]
                        let b1 = bands[bj].positions[sj+1]
                        let b02 = SIMD2<Float>(b0.x, b0.y)
                        let b12 = SIMD2<Float>(b1.x, b1.y)
                        let d1 = a12 - a02, d2 = b12 - b02
                        let cross2 = d1.x * d2.y - d1.y * d2.x
                        guard abs(cross2) > 1e-9 else { continue }
                        let d = b02 - a02
                        let t2 = (d.x * d2.y - d.y * d2.x) / cross2
                        let u2 = (d.x * d1.y - d.y * d1.x) / cross2
                        guard t2 > 1e-6 && t2 < (1-1e-6) && u2 > 1e-6 && u2 < (1-1e-6) else { continue }
                        // 2D crossing found — record Z ordering and which side B's midpoint is on relative to A
                        let zA = a0.z + (a1.z - a0.z) * t2
                        let zB = b0.z + (b1.z - b0.z) * u2
                        // B's crossing point on A's line (to determine side)
                        let crossPtB = b02 + (b12 - b02) * u2
                        let d1v = a12 - a02
                        let dB = crossPtB - a02
                        let side = d1v.x * dB.y - d1v.y * dB.x
                        _prevCrossingRecords.append(CrossingRecord(
                            bandA: UInt16(bi), segA: UInt16(si),
                            bandB: UInt16(bj), segB: UInt16(sj),
                            aAboveB: zA >= zB,
                            sideOfB: side,
                            paramT: t2,
                            paramU: u2
                        ))
                    }
                }
            }
        }
    }

    /// For crossing pairs from the previous step, check if Z ordering changed (tunneling).
    /// If so, restore Z separation to maintain the original ordering.
    private func resolveTunneledCrossings() {
        for rec in _prevCrossingRecords {
            let bi = Int(rec.bandA), si = Int(rec.segA)
            let bj = Int(rec.bandB), sj = Int(rec.segB)
            guard bands[bi].active && bands[bj].active else { continue }
            let nA = bands[bi].positions.count
            let nB = bands[bj].positions.count
            guard si < nA-1 && sj < nB-1 else { continue }

            let a0 = bands[bi].positions[si], a1 = bands[bi].positions[si+1]
            let b0 = bands[bj].positions[sj], b1 = bands[bj].positions[sj+1]
            let a02 = SIMD2<Float>(a0.x, a0.y), a12 = SIMD2<Float>(a1.x, a1.y)
            let b02 = SIMD2<Float>(b0.x, b0.y), b12 = SIMD2<Float>(b1.x, b1.y)
            let d1 = a12 - a02, d2 = b12 - b02
            let cross2 = d1.x * d2.y - d1.y * d2.x
            guard abs(cross2) > 1e-9 else { continue }
            let d = b02 - a02
            let t2 = (d.x * d2.y - d.y * d2.x) / cross2
            let u2 = (d.x * d1.y - d.y * d1.x) / cross2
            guard t2 > 1e-6 && t2 < (1-1e-6) && u2 > 1e-6 && u2 < (1-1e-6) else { continue }

            let minDist = bands[bi].crossSection.collisionRadius + bands[bj].crossSection.collisionRadius
            let aPinned0 = bands[bi].pinStart != nil && si == 0
            let aPinned1 = bands[bi].pinEnd != nil && si == nA-2
            let bPinned0 = bands[bj].pinStart != nil && sj == 0
            let bPinned1 = bands[bj].pinEnd != nil && sj == nB-2

            guard zSeparationStrength > 0 else { continue }
            if t2 > 1e-6 && t2 < (1-1e-6) && u2 > 1e-6 && u2 < (1-1e-6) {
                // Still crossing in 2D — check if Z ordering flipped
                let zA = a0.z + (a1.z - a0.z) * t2
                let zB = b0.z + (b1.z - b0.z) * u2
                let aAboveNow = zA >= zB
                guard aAboveNow != rec.aAboveB else { continue }
                let zDiff = zA - zB
                let needed = rec.aAboveB ? minDist : -minDist
                let correction = (needed - zDiff) * 0.5 * zSeparationStrength
                if !aPinned0 { bands[bi].positions[si].z += correction * (1 - t2) }
                if !aPinned1 { bands[bi].positions[si+1].z += correction * t2 }
                if !bPinned0 { bands[bj].positions[sj].z -= correction * (1 - u2) }
                if !bPinned1 { bands[bj].positions[sj+1].z -= correction * u2 }
            } else {
                // No longer crossing in 2D — check if B's midpoint crossed to the other side of A
                // Sample B at the parameter where crossing WAS, check which side of A it's on now
                let curPtB = SIMD2<Float>(b0.x + (b1.x-b0.x)*rec.paramU, b0.y + (b1.y-b0.y)*rec.paramU)
                let curPtA = SIMD2<Float>(a0.x + (a1.x-a0.x)*rec.paramT, a0.y + (a1.y-a0.y)*rec.paramT)
                let d1v = SIMD2<Float>(a1.x-a0.x, a1.y-a0.y)
                let dB = curPtB - curPtA
                let curSide = d1v.x * dB.y - d1v.y * dB.x
                guard rec.sideOfB * curSide < 0 else { continue }  // same side = no tunneling

                // B crossed to other side of A — tunneling detected. Apply Z correction at midpoints.
                let pT = rec.paramT, pU = rec.paramU
                let zA = a0.z + (a1.z - a0.z) * pT
                let zB = b0.z + (b1.z - b0.z) * pU
                let zDiff = zA - zB
                let needed = rec.aAboveB ? minDist : -minDist
                let correction = (needed - zDiff) * 0.5 * zSeparationStrength
                if !aPinned0 { bands[bi].positions[si].z += correction * (1 - pT) }
                if !aPinned1 { bands[bi].positions[si+1].z += correction * pT }
                if !bPinned0 { bands[bj].positions[sj].z -= correction * (1 - pU) }
                if !bPinned1 { bands[bj].positions[sj+1].z -= correction * pU }
            }
        }
    }

    struct CollisionPair {
        let bandA: UInt16
        let segA: UInt16
        let bandB: UInt16
        let segB: UInt16
    }

    /// Broadphase: AABB sweep between band pairs and within the same band.
    private func rebuildCollisionPairs(_ activeBands: [Int]) {
        collisionPairsBuf.removeAll(keepingCapacity: true)
        guard !activeBands.isEmpty else { return }

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
                        collisionPairsBuf.append(CollisionPair(bandA: UInt16(bi), segA: UInt16(si), bandB: UInt16(bi), segB: UInt16(sj)))
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
                        collisionPairsBuf.append(CollisionPair(bandA: UInt16(bi), segA: UInt16(si), bandB: UInt16(bj), segB: UInt16(sj)))
                    }
                }
            }
        }
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
        let corr = normal * (overlap * collisionResponse)

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
                let maxFriction = mu * overlap * maxFrictionCap
                let frictionMag = min(tangentLen * frictionDampingRatio, maxFriction)
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

    var cachedFrames: [[MaterialFrame]] = []

    

}
