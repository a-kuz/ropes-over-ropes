import simd
import os.log
import QuartzCore

final class VerletSimulator {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "VerletSim")

    // MARK: - Band (rope) state

    struct Band {
        var positions: ContiguousArray<SIMD3<Float>>
        var previousPositions: ContiguousArray<SIMD3<Float>>
        var segmentLength: Float
        var radius: Float
        var pinStart: Int?
        var pinEnd: Int?
        var active: Bool
        var fadeOut: Float = 0          // 0 = normal, >0 = fading out (0→1)
        static let fadeOutDuration: Float = 0.4
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
    var constraintIterations: Int = 8 {
        didSet { constraintIterations = max(constraintIterations, 8) }
    }
    var settleSteps: Int = 5
    var liftHeight: Float = 0.30
    /// Rope tension: multiplier on rest length. < 1 = taut (shorter rope), 1 = natural length.
    /// 0.95 = rope is 5% shorter than span → pulled tight. Don't go below ~0.85.
    var ropeTension: Float = 0.98
    private var currentTension: Float = 1.0
    private let tensionSpeed: Float = 0.5  // per second — slow tightening after drag
    var particleCount: Int = 6

    private let dt: Float = 1.0 / 120.0  // fixed dt, supports ProMotion 120fps
    private var accumulator: Float = 0
    private var resampleCounter: Int = 0
    private let resampleInterval: Int = 30  // resample every N substeps to avoid fighting solver
    private var dragTargetPos: SIMD3<Float>?
    private var dragStartPos: SIMD3<Float>?
    private var logTimer: Float = 0
    private var logEvery: Float = 1.0  // log every N seconds
    private var tensionLogTimer: Float = -1  // -1 = inactive, >= 0 = countdown
    private let tensionLogDuration: Float = 3.0
    private let tensionLogInterval: Float = 0.25  // 4 fps max — compact logs
    private var tensionLogCooldown: Float = 0

    /// Linking number snapshot from previous tension log — tracks (bandI, bandJ, linkingNumber)
    private var prevLinking: [(Int, Int, Int)] = []
    private let ropePhysics = RopePhysics()

    var dragInfo: DragInfo?

    // MARK: - Post-drag lower animation

    struct LowerAnimation {
        let bandIndex: Int
        let endIndex: Int
        let targetHole: Int
        let startPos: SIMD3<Float>
        var timer: Float = 0
        static let duration: Float = 0.3
    }

    var lowerAnimation: LowerAnimation?

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
            positions: ContiguousArray(repeating: .zero, count: n),
            previousPositions: ContiguousArray(repeating: .zero, count: n),
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
        let clampedDt = min(deltaTime, 1.0 / 15.0)  // spiral-of-death protection

        // Advance fade-out animations
        for i in bands.indices where bands[i].fadeOut > 0 && bands[i].active {
            bands[i].fadeOut += clampedDt / Band.fadeOutDuration
            if bands[i].fadeOut >= 1 {
                bands[i].fadeOut = 1
                bands[i].active = false
                bands[i].pinStart = nil
                bands[i].pinEnd = nil
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
        guard var anim = lowerAnimation else { return }
        anim.timer += deltaTime

        let bi = anim.bandIndex
        let idx = anim.endIndex == 0 ? 0 : bands[bi].positions.count - 1
        let holePos = holePosition3D(anim.targetHole)

        let t = min(anim.timer / LowerAnimation.duration, 1.0)
        let eased = 1.0 - (1.0 - t) * (1.0 - t)  // ease-out
        let pos = anim.startPos + (holePos - anim.startPos) * eased
        bands[bi].positions[idx] = pos
        bands[bi].previousPositions[idx] = pos

        if t >= 1.0 {
            // Pin it
            if anim.endIndex == 0 {
                bands[bi].pinStart = anim.targetHole
            } else {
                bands[bi].pinEnd = anim.targetHole
            }
            bands[bi].positions[idx] = holePos
            bands[bi].previousPositions[idx] = holePos
            lowerAnimation = nil
            return
        }

        lowerAnimation = anim
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

    private var perfAccum: (verlet: Double, constr: Double, postCol: Double, count: Int, logTime: Double, pairs: Int) = (0,0,0,0,0,0)

    private func verletStep(collide: Bool, dt: Float) {
        let dt2 = dt * dt
        let t0 = CACurrentMediaTime()

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
        }
        let t1 = CACurrentMediaTime()

        // 2. Constraint + collision iterations (interleaved for robust PBD)
        let active = collide ? bands.indices.filter({ bands[$0].active && bands[$0].fadeOut == 0 }) : []

        // Tension diagnostic logging: active for 3s after endDrag, max 10/sec
        var shouldLogStep = false
        if tensionLogTimer >= 0 {
            tensionLogTimer += dt
            tensionLogCooldown -= dt
            if tensionLogCooldown <= 0 {
                shouldLogStep = true
                tensionLogCooldown = tensionLogInterval
            }
            if tensionLogTimer > tensionLogDuration {
                tensionLogTimer = -1
                Self.logger.warning("[TENSION-END] logging stopped")
            }
        }

        // Scale iterations inversely with tension — stronger tension needs more solver work
        let effectiveIters = max(constraintIterations, Int(Float(constraintIterations) / max(currentTension, 0.3)))

        // Build collision pair list once per substep (broadphase)
        let collisionPairs = collide ? buildCollisionPairs(active) : []

        for _ in 0..<effectiveIters {
            for bi in bands.indices {
                guard bands[bi].active else { continue }
                bandConstraints(bi)
            }
            if collide {
                resolveCollisionPairs(collisionPairs)
            }
        }

        let t2Start = CACurrentMediaTime()
        // Post-solve: collision-only passes until converged
        if collide {
            for _ in 0..<3 {
                let hadCollision = resolveCollisionPairs(collisionPairs, injectVelocity: true)
                // Re-apply pin + board constraints
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
                    for i in 1..<(n - 1) {
                        if bands[bi].positions[i].z < bands[bi].radius {
                            bands[bi].positions[i].z = bands[bi].radius
                        }
                    }
                }
                if !hadCollision { break }
            }
        }

        let t3 = CACurrentMediaTime()
        perfAccum.verlet += t1 - t0
        perfAccum.constr += t2Start - t1
        perfAccum.postCol += t3 - t2Start
        perfAccum.count += 1
        perfAccum.pairs = collisionPairs.count
        let now = CACurrentMediaTime()
        if now - perfAccum.logTime >= 2.0 {
            let c = Double(max(perfAccum.count, 1))
            let vUs = perfAccum.verlet / c * 1e6
            let cUs = perfAccum.constr / c * 1e6
            let pUs = perfAccum.postCol / c * 1e6
            let tUs = vUs + cUs + pUs
            let pc = bands.first?.positions.count ?? 0
            let cp = perfAccum.pairs
            Self.logger.warning("[PERF] steps=\(Int(c)) verlet=\(vUs, format: .fixed(precision: 0))µs constr=\(cUs, format: .fixed(precision: 0))µs postCol=\(pUs, format: .fixed(precision: 0))µs total=\(tUs, format: .fixed(precision: 0))µs effIter=\(effectiveIters) particles=\(pc) pairs=\(cp)")
            perfAccum = (0, 0, 0, 0, now, 0)
        }

        if shouldLogStep && active.count >= 2 {
            var currentLinking: [(Int, Int, Int)] = []
            var summary: [String] = []

            for i in 0..<active.count {
                let bi = active[i]
                for j in (i + 1)..<active.count {
                    let bj = active[j]
                    let lk = ropePhysics.linkingNumber(bands[bi].positions, bands[bj].positions)
                    currentLinking.append((bi, bj, lk))
                    let prev = prevLinking.first(where: { $0.0 == bi && $0.1 == bj })?.2
                    if let prev, lk != prev {
                        summary.append("\(bi)-\(bj):\(prev)→\(lk)PASSTHROUGH")
                    } else {
                        summary.append("\(bi)-\(bj):\(lk)")
                    }
                }
            }

            Self.logger.warning("[T] ten=\(self.currentTension, format: .fixed(precision: 3)) \(summary.joined(separator: " "))")
            prevLinking = currentLinking
        }

    }



    /// Redistribute particles by curvature: dense at bends, sparse on straight segments.
    /// Uses curvature-weighted arc-length so particles concentrate where the rope curves.
    private func resampleBand(_ bi: Int) {
        let n = bands[bi].positions.count
        guard n >= 4 else { return }

        let pos = bands[bi].positions
        let prev = bands[bi].previousPositions

        // 1. Compute per-vertex curvature (angle between adjacent segments)
        //    Endpoints get 0 curvature.
        var curvature = [Float](repeating: 0, count: n)
        for i in 1..<(n - 1) {
            let d0 = pos[i] - pos[i - 1]
            let d1 = pos[i + 1] - pos[i]
            let len0 = simd_length(d0)
            let len1 = simd_length(d1)
            if len0 > 1e-9 && len1 > 1e-9 {
                let cosA = simd_dot(d0, d1) / (len0 * len1)
                // curvature ~ angle; acos is expensive, use 1-cos as proxy (0=straight, 2=hairpin)
                curvature[i] = max(1.0 - cosA, 0)
            }
        }

        // 2. Compute weighted cumulative arc length.
        //    Weight = 1 + curvatureScale * avgCurvature(segment)
        //    Higher weight = more particles allocated to that segment.
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

        // 3. Also compute plain arc length for interpolation
        var arcLen = [Float](repeating: 0, count: n)
        for i in 1..<n {
            arcLen[i] = arcLen[i - 1] + simd_length(pos[i] - pos[i - 1])
        }

        // 4. Check if resampling needed
        var maxSeg: Float = 0, minSeg: Float = Float.greatestFiniteMagnitude
        for i in 0..<(n - 1) {
            let s = arcLen[i + 1] - arcLen[i]
            maxSeg = max(maxSeg, s)
            if s > 1e-9 { minSeg = min(minSeg, s) }
        }
        guard minSeg < 1e-9 || maxSeg / max(minSeg, 1e-9) > 1.5 else { return }

        // 5. Place particles at uniform weighted-arc-length intervals
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
        }
    }

    private func bandConstraints(_ bi: Int) {
        let n = bands[bi].positions.count
        let segLen = bands[bi].segmentLength * currentTension
        let R = bands[bi].radius
        let pinS = bands[bi].pinStart
        let pinE = bands[bi].pinEnd
        let holeS = pinS.map { holePosition3D($0) }
        let holeE = pinE.map { holePosition3D($0) }

        bands[bi].positions.withUnsafeMutableBufferPointer { pos in
            // Even-odd (red-black) distance constraints
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

            // Pin constraints
            if let hp = holeS { pos[0] = hp }
            if let hp = holeE { pos[n - 1] = hp }

            // Board: z >= R
            for i in 1..<(n - 1) {
                if pos[i].z < R { pos[i].z = R }
            }
        }
    }

    // MARK: - Collision

    private struct CollisionPair {
        let bandA: UInt16
        let segA: UInt16
        let bandB: UInt16
        let segB: UInt16
    }

    /// Broadphase: AABB sweep between band pairs, returns collision pair list.
    private func buildCollisionPairs(_ activeBands: [Int]) -> [CollisionPair] {
        guard activeBands.count >= 2 else { return [] }
        var pairs: [CollisionPair] = []
        pairs.reserveCapacity(512)

        for ai in 0..<activeBands.count {
            let bi = activeBands[ai]
            let posI = bands[bi].positions
            let segsI = posI.count - 1
            let ri = bands[bi].radius

            for aj in (ai + 1)..<activeBands.count {
                let bj = activeBands[aj]
                let posJ = bands[bj].positions
                let segsJ = posJ.count - 1
                let minDist = ri + bands[bj].radius

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

    /// Collide two specific segments from different bands
    @inline(__always)
    private func collideSegments(_ bi: Int, _ si: Int, _ bj: Int, _ sj: Int, injectVelocity: Bool) -> Bool {
        let minDist = bands[bi].radius + bands[bj].radius
        let minDist2 = minDist * minDist

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

        guard dist2 < minDist2 && dist2 > 1e-12 else { return false }

        let dist = sqrtf(dist2)
        let overlap = minDist - dist
        let normal = diff / dist
        let corr = normal * (overlap * 0.5)

        bands[bi].positions[si] += corr * (1 - s)
        bands[bi].positions[si + 1] += corr * s
        bands[bj].positions[sj] -= corr * (1 - t)
        bands[bj].positions[sj + 1] -= corr * t

        if injectVelocity {
            let velCorr = corr * 0.5
            bands[bi].previousPositions[si] -= velCorr * (1 - s)
            bands[bi].previousPositions[si + 1] -= velCorr * s
            bands[bj].previousPositions[sj] += velCorr * (1 - t)
            bands[bj].previousPositions[sj + 1] += velCorr * t
        }
        return true
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

        // Cancel any running lower animation
        lowerAnimation = nil

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

        let idx = drag.endIndex == 0 ? 0 : bands[drag.bandIndex].positions.count - 1
        let currentPos = bands[drag.bandIndex].positions[idx]

        // Start lower animation: slowly bring endpoint into hole
        lowerAnimation = LowerAnimation(
            bandIndex: drag.bandIndex,
            endIndex: drag.endIndex,
            targetHole: targetHoleIndex,
            startPos: currentPos
        )

        dragInfo = nil
        dragStartPos = nil
        dragTargetPos = nil

        // Start tension diagnostic logging for 3 seconds
        tensionLogTimer = 0
        tensionLogCooldown = 0
        // Snapshot linking numbers as baseline
        prevLinking.removeAll()
        let activeBands = bands.indices.filter { bands[$0].active }
        for i in 0..<activeBands.count {
            for j in (i + 1)..<activeBands.count {
                let lk = ropePhysics.linkingNumber(bands[activeBands[i]].positions, bands[activeBands[j]].positions)
                prevLinking.append((activeBands[i], activeBands[j], lk))
            }
        }
        let s = prevLinking.map { "\($0.0)-\($0.1):\($0.2)" }.joined(separator: " ")
        Self.logger.warning("[T-START] \(s)")
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
        currentTension = ropeTension

        for config in ropeConfigs {
            addBand(radius: config.radius, particleCount: particleCount)
        }

        if actions.isEmpty {
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

        let dragSteps = 4

        // Lift
        for s in 1...3 {
            let t = Float(s) / 3.0
            bands[bandIndex].positions[idx] = fromPos + (liftFrom - fromPos) * t
            bands[bandIndex].previousPositions[idx] = bands[bandIndex].positions[idx]
            doSteps(dragSteps, collide: true)
        }

        // Traverse
        let traverseSteps = 12
        for s in 1...traverseSteps {
            let t = Float(s) / Float(traverseSteps)
            bands[bandIndex].positions[idx] = liftFrom + (liftTo - liftFrom) * t
            bands[bandIndex].previousPositions[idx] = bands[bandIndex].positions[idx]
            doSteps(dragSteps, collide: true)
        }

        // Lower
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

    // MARK: - Utility

    func holePosition3D(_ holeIndex: Int) -> SIMD3<Float> {
        guard holePositions.indices.contains(holeIndex) else { return .zero }
        let p = holePositions[holeIndex]
        return SIMD3<Float>(p.x, p.y, -holeDepth)
    }

}
