import simd
import os.log
import QuartzCore

enum TopologySampler {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "TopologySampler")
    
    nonisolated(unsafe) static var hookStepMultiplier: Float = 0.0900
    nonisolated(unsafe) static var hookRadiusMultiplier: Float = 0.920
    nonisolated(unsafe) static var hookStepLimitMultiplier: Float = 2.5000
    nonisolated(unsafe) static var debugSegmentColors: Bool = true
    nonisolated(unsafe) static var smoothSubdivisions: Int = 4
    nonisolated(unsafe) static var smoothIterations: Int = 0
    nonisolated(unsafe) static var smoothStrength: Float = 0.00
    nonisolated(unsafe) static var smoothZone: Float = 0.001
    
    static func sampleRope(
        engine: TopologyEngine,
        ropeIndex: Int,
        count: Int,
        lift: Float,
        dragLift: Float,
        ropeRadius: Float,
        ropeRadiusForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> [SIMD3<Float>] {
        let result = buildPoly(
            engine: engine,
            ropeIndex: ropeIndex,
            lift: lift,
            dragLift: dragLift,
            ropeRadius: ropeRadius,
            ropeRadiusForIndex: ropeRadiusForIndex,
            holeRadius: holeRadius
        )
        return resample(poly: result.points, count: count)
    }
    
    struct RopeRenderResult {
        let points: [SIMD3<Float>]
        let segmentStarts: [Int]
    }
    
    static func sampleRopeRender(
        engine: TopologyEngine,
        ropeIndex: Int,
        lift: Float,
        dragLift: Float,
        ropeRadius: Float,
        ropeRadiusForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> RopeRenderResult {
        return buildPoly(
            engine: engine,
            ropeIndex: ropeIndex,
            lift: lift,
            dragLift: dragLift,
            ropeRadius: ropeRadius,
            ropeRadiusForIndex: ropeRadiusForIndex,
            holeRadius: holeRadius
        )
    }
    
    static func ropePathLength(
        engine: TopologyEngine,
        ropeIndex: Int,
        ropeRadius: Float,
        ropeRadiusForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> Float {
        let result = buildPoly(
            engine: engine,
            ropeIndex: ropeIndex,
            lift: ropeRadius,
            dragLift: 0,
            ropeRadius: ropeRadius,
            ropeRadiusForIndex: ropeRadiusForIndex,
            holeRadius: holeRadius
        )
        let lengths = cumulativeLengths(poly: result.points)
        return lengths.last ?? 0
    }
    
    static func hookCenters(engine: TopologyEngine, ropeRadiusForIndex: (Int) -> Float) -> [SIMD2<Float>] {
        var centers: [SIMD2<Float>] = []
        for (_, hook) in engine.hooks {
            let A1 = engine.ropeStart(hook.ropeA)
            let A2 = engine.ropeEnd(hook.ropeA)
            let B1 = engine.ropeStart(hook.ropeB)
            let B2 = engine.ropeEnd(hook.ropeB)
            let R = max(ropeRadiusForIndex(hook.ropeA), ropeRadiusForIndex(hook.ropeB))
            
            if let geom = HookGeometryCalculator.calculateHookSequenceGeometry(
                A1: A1, A2: A2, B1: B1, B2: B2, R: R, crossingCount: hook.N,
                stepMultiplier: hookStepMultiplier, radiusMultiplier: hookRadiusMultiplier,
                stepLimitMultiplier: hookStepLimitMultiplier
            ) {
                centers.append(contentsOf: geom.centers)
            }
        }
        return centers
    }
    
    private static func buildPoly(
        engine: TopologyEngine,
        ropeIndex: Int,
        lift: Float,
        dragLift: Float,
        ropeRadius: Float,
        ropeRadiusForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> RopeRenderResult {
        guard engine.ropes.indices.contains(ropeIndex) else { return RopeRenderResult(points: [], segmentStarts: []) }
        let rope = engine.ropes[ropeIndex]
        guard rope.active else { return RopeRenderResult(points: [], segmentStarts: []) }
        
        let holeInnerRadius = holeRadius * 0.76
        let holeDepth = holeRadius * 1.25
        let baseZ = ropeRadius
        
        let rawStart = engine.ropeStart(ropeIndex)
        let rawEnd = engine.ropeEnd(ropeIndex)
        
        let startIsFloating = rope.floatingEnd == 0
        let endIsFloating = rope.floatingEnd == 1
        
        let hookId = rope.hooks.first
        var hookPath: [SIMD2<Float>]? = nil
        var hook: HookSequence? = nil
        
        if let hId = hookId, let h = engine.hooks[hId] {
            hook = h
            let isRopeA = (ropeIndex == h.ropeA)
            let otherRadius = ropeRadiusForIndex(isRopeA ? h.ropeB : h.ropeA)
            let R = max(ropeRadius, otherRadius)
            
            let A1 = engine.ropeStart(h.ropeA)
            let A2 = engine.ropeEnd(h.ropeA)
            let B1 = engine.ropeStart(h.ropeB)
            let B2 = engine.ropeEnd(h.ropeB)
            
            if let geom = HookGeometryCalculator.calculateHookSequenceGeometry(
                A1: A1, A2: A2, B1: B1, B2: B2, R: R, crossingCount: h.N,
                stepMultiplier: hookStepMultiplier, radiusMultiplier: hookRadiusMultiplier,
                stepLimitMultiplier: hookStepLimitMultiplier
            ) {
                hookPath = isRopeA ? geom.pathA : geom.pathB
            }
        }
        
        let startAnchor: SIMD2<Float>
        let startZ: Float
        if startIsFloating {
            startAnchor = rawStart
            startZ = dragLift
        } else {
            let targetPoint: SIMD2<Float>
            if let path = hookPath, !path.isEmpty {
                targetPoint = path[0]
            } else {
                targetPoint = rawEnd
            }
            let dir = targetPoint - rawStart
            let dirLen = simd_length(dir)
            if dirLen > 1e-6 {
                startAnchor = rawStart + (dir / dirLen) * holeInnerRadius
            } else {
                startAnchor = rawStart
            }
            startZ = -holeDepth
        }
        
        let endAnchor: SIMD2<Float>
        let endZ: Float
        if endIsFloating {
            endAnchor = rawEnd
            endZ = dragLift
        } else {
            let targetPoint: SIMD2<Float>
            if let path = hookPath, !path.isEmpty {
                targetPoint = path[path.count - 1]
            } else {
                targetPoint = rawStart
            }
            let dir = targetPoint - rawEnd
            let dirLen = simd_length(dir)
            if dirLen > 1e-6 {
                endAnchor = rawEnd + (dir / dirLen) * holeInnerRadius
            } else {
                endAnchor = rawEnd
            }
            endZ = -holeDepth
        }
        
        if let hook = hook {
            return buildHookPoly(
                engine: engine,
                ropeIndex: ropeIndex,
                hook: hook,
                startAnchor: startAnchor,
                endAnchor: endAnchor,
                startZ: startZ,
                endZ: endZ,
                baseZ: baseZ,
                ropeRadius: ropeRadius,
                ropeRadiusForIndex: ropeRadiusForIndex
            )
        }
        
        let simplePoly = buildSimplePoly(
            startAnchor: startAnchor,
            endAnchor: endAnchor,
            startZ: startZ,
            endZ: endZ,
            baseZ: baseZ
        )
        return RopeRenderResult(points: simplePoly, segmentStarts: [0])
    }
    
    private static func buildSimplePoly(
        startAnchor: SIMD2<Float>,
        endAnchor: SIMD2<Float>,
        startZ: Float,
        endZ: Float,
        baseZ: Float
    ) -> [SIMD3<Float>] {
        var poly: [SIMD3<Float>] = []
        let segmentCount = 64
        let dir = endAnchor - startAnchor
        
        for i in 0...segmentCount {
            let t = Float(i) / Float(segmentCount)
            let xy = startAnchor + dir * t
            let z: Float
            if startZ < 0 && endZ < 0 {
                let midT: Float = 0.5
                if t < midT {
                    let localT = t / midT
                    z = startZ + (baseZ - startZ) * smoothstep(localT)
                } else {
                    let localT = (t - midT) / (1 - midT)
                    z = baseZ + (endZ - baseZ) * smoothstep(localT)
                }
            } else if startZ < 0 {
                z = startZ + (baseZ - startZ) * smoothstep(min(1, t * 4))
            } else if endZ < 0 {
                z = baseZ + (endZ - baseZ) * smoothstep(max(0, (t - 0.75) * 4))
            } else {
                z = baseZ
            }
            poly.append(SIMD3<Float>(xy.x, xy.y, z))
        }
        return poly
    }
    
    private static func buildHookPoly(
        engine: TopologyEngine,
        ropeIndex: Int,
        hook: HookSequence,
        startAnchor: SIMD2<Float>,
        endAnchor: SIMD2<Float>,
        startZ: Float,
        endZ: Float,
        baseZ: Float,
        ropeRadius: Float,
        ropeRadiusForIndex: (Int) -> Float
    ) -> RopeRenderResult {
        let isRopeA = (ropeIndex == hook.ropeA)
        let otherRopeIndex = isRopeA ? hook.ropeB : hook.ropeA
        let otherRadius = ropeRadiusForIndex(otherRopeIndex)
        let currentRadius = ropeRadius
        
        let A1 = engine.ropeStart(hook.ropeA)
        let A2 = engine.ropeEnd(hook.ropeA)
        let B1 = engine.ropeStart(hook.ropeB)
        let B2 = engine.ropeEnd(hook.ropeB)
        let R = max(ropeRadius, otherRadius)
        
        guard let geom = HookGeometryCalculator.calculateHookSequenceGeometry(
            A1: A1, A2: A2, B1: B1, B2: B2, R: R, crossingCount: hook.N,
            stepMultiplier: hookStepMultiplier, radiusMultiplier: hookRadiusMultiplier,
            stepLimitMultiplier: hookStepLimitMultiplier
        ) else {
            let simplePoly = buildSimplePoly(
                startAnchor: startAnchor,
                endAnchor: endAnchor,
                startZ: startZ,
                endZ: endZ,
                baseZ: baseZ
            )
            return RopeRenderResult(points: simplePoly, segmentStarts: [0])
        }
        
        var rawPath = isRopeA ? geom.pathA : geom.pathB
        
        if rawPath.count >= 2 {
            let distToFirst = simd_length(startAnchor - rawPath[0])
            let distToLast = simd_length(startAnchor - rawPath[rawPath.count - 1])
            if distToLast < distToFirst {
                rawPath.reverse()
            }
        }
        
        let pathA = geom.pathA
        let pathB = geom.pathB
        
        let logKey = ZLogKey(ropeIndex: ropeIndex, hookId: hook.id)
        let now = CACurrentMediaTime()
        let timeSinceLastLog = now - lastZLogTime
        let shouldLog: Bool
        if let last = lastZLogParams[logKey] {
            let eps: Float = 0.001
            let pathChanged = pathA.count != last.pathA.count ||
                             pathB.count != last.pathB.count ||
                             zip(pathA, last.pathA).contains(where: { simd_length_squared($0 - $1) > eps * eps }) ||
                             zip(pathB, last.pathB).contains(where: { simd_length_squared($0 - $1) > eps * eps })
            let dataChanged = hook.N != last.N || pathChanged
            shouldLog = dataChanged && timeSinceLastLog >= 0.5
            if shouldLog {
                lastZLogParams[logKey] = ZLogValue(N: hook.N, pathA: pathA, pathB: pathB)
                lastZLogTime = now
            }
        } else {
            shouldLog = timeSinceLastLog >= 0.5
            if shouldLog {
                lastZLogParams[logKey] = ZLogValue(N: hook.N, pathA: pathA, pathB: pathB)
                lastZLogTime = now
            }
        }
        
        var fullPath2D: [SIMD2<Float>] = [startAnchor]
        fullPath2D.append(contentsOf: rawPath)
        fullPath2D.append(endAnchor)
        
        if shouldLog {
            let ropeStart = engine.ropeStart(ropeIndex)
            let ropeEnd = engine.ropeEnd(ropeIndex)
            logger.info("  fullPath2D (\(fullPath2D.count) pts):")
            for (i, p) in fullPath2D.enumerated() {
                logger.info("    [\(i)] = (\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
            }
            logger.info("  ropeStart = (\(String(format: "%.3f", ropeStart.x)), \(String(format: "%.3f", ropeStart.y)))")
            logger.info("  ropeEnd = (\(String(format: "%.3f", ropeEnd.x)), \(String(format: "%.3f", ropeEnd.y)))")
            logger.info("  startAnchor = (\(String(format: "%.3f", startAnchor.x)), \(String(format: "%.3f", startAnchor.y)))")
            logger.info("  endAnchor = (\(String(format: "%.3f", endAnchor.x)), \(String(format: "%.3f", endAnchor.y)))")
            logger.info("  rawPath (\(rawPath.count) pts):")
            for (i, p) in rawPath.enumerated() {
                logger.info("    [\(i)] = (\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
            }
        }
        
        let smoothResult = smoothPath2D(fullPath2D, ropeRadius: currentRadius)
        let smoothedPoints = smoothResult.points
        let segmentStarts = smoothResult.segmentStarts
        
        let firstIsOver: Bool
        if isRopeA {
            firstIsOver = hook.ropeAStartIsOver
        } else {
            firstIsOver = !hook.ropeAStartIsOver
        }
        
        let underZ: Float = 0
        let overZ = otherRadius * 2 + currentRadius
        
        if shouldLog {
            logger.info("  isRopeA=\(isRopeA) firstIsOver=\(firstIsOver)")
            logger.info("  currentRadius=\(String(format: "%.4f", currentRadius)) otherRadius=\(String(format: "%.4f", otherRadius))")
            logger.info("  underZ=\(String(format: "%.4f", underZ)) overZ=\(String(format: "%.4f", overZ)) baseZ=\(String(format: "%.4f", baseZ))")
            logger.info("  pathA (\(pathA.count) pts):")
            for (i, p) in pathA.enumerated() {
                logger.info("    [\(i)] = (\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
            }
            logger.info("  pathB (\(pathB.count) pts):")
            for (i, p) in pathB.enumerated() {
                logger.info("    [\(i)] = (\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
            }
        }
        
        var crossingPoints: [SIMD2<Float>] = []
        for i in 0..<(pathA.count - 1) {
            for j in 0..<(pathB.count - 1) {
                if let p = segIntersection(pathA[i], pathA[i + 1], pathB[j], pathB[j + 1]) {
                    crossingPoints.append(p)
                    if shouldLog {
                        logger.info("  CROSSING FOUND: (\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y))) segA[\(i)-\(i+1)] x segB[\(j)-\(j+1)]")
                    }
                }
            }
        }
        
        if shouldLog {
            logger.info("  Total crossings found: \(crossingPoints.count) (expected N=\(hook.N))")
        }
        
        func isOverAtCrossing(_ idx: Int) -> Bool {
            (idx % 2 == 0) ? firstIsOver : !firstIsOver
        }
        
        var smoothedDistances: [Float] = [0]
        for i in 1..<smoothedPoints.count {
            let d = simd_length(smoothedPoints[i] - smoothedPoints[i - 1])
            smoothedDistances.append(smoothedDistances[i - 1] + d)
        }
        let totalSmoothedDist = smoothedDistances.last ?? 1.0
        
        var crossingDists: [(dist: Float, isOver: Bool)] = []
        
        for (crossingIdx, crossingPt) in crossingPoints.enumerated() {
            var closestDist: Float = 0
            var minDist = Float.greatestFiniteMagnitude
            
            for i in 0..<smoothedPoints.count {
                let d = simd_length(smoothedPoints[i] - crossingPt)
                if d < minDist {
                    minDist = d
                    closestDist = smoothedDistances[i]
                }
            }
            let isOver = isOverAtCrossing(crossingIdx)
            crossingDists.append((dist: closestDist, isOver: isOver))
            
            if shouldLog {
                logger.info("  Crossing[\(crossingIdx)]: dist=\(String(format: "%.3f", closestDist)) isOver=\(isOver)")
            }
        }
        
        crossingDists.sort { $0.dist < $1.dist }
        
        if shouldLog {
            logger.info("  underZ=\(String(format: "%.4f", underZ)) overZ=\(String(format: "%.4f", overZ))")
            logger.info("  totalSmoothedDist=\(String(format: "%.3f", totalSmoothedDist))")
        }
        
        func zForCrossing(_ isOver: Bool) -> Float {
            isOver ? overZ : underZ
        }
        
        var poly: [SIMD3<Float>] = []
        
        for (i, xy) in smoothedPoints.enumerated() {
            let dist = smoothedDistances[i]
            var z: Float = baseZ
            
            if crossingDists.isEmpty {
                z = baseZ
            } else {
                let firstCrossingDist = crossingDists[0].dist
                let lastCrossingDist = crossingDists[crossingDists.count - 1].dist
                
                if dist <= firstCrossingDist {
                    let firstZ = zForCrossing(crossingDists[0].isOver)
                    let t = dist / max(0.001, firstCrossingDist)
                    z = baseZ + (firstZ - baseZ) * smoothstep(t)
                } else if dist >= lastCrossingDist {
                    let lastZ = zForCrossing(crossingDists[crossingDists.count - 1].isOver)
                    let t = (dist - lastCrossingDist) / max(0.001, totalSmoothedDist - lastCrossingDist)
                    z = lastZ + (baseZ - lastZ) * smoothstep(t)
                } else {
                    for j in 0..<(crossingDists.count - 1) {
                        let c0 = crossingDists[j]
                        let c1 = crossingDists[j + 1]
                        if dist >= c0.dist && dist <= c1.dist {
                            let z0 = zForCrossing(c0.isOver)
                            let z1 = zForCrossing(c1.isOver)
                            let t = (dist - c0.dist) / max(0.001, c1.dist - c0.dist)
                            z = z0 + (z1 - z0) * smoothstep(t)
                            break
                        }
                    }
                }
            }
            
            let startBlendDist = totalSmoothedDist * 0.15
            let endBlendDist = totalSmoothedDist * 0.15
            
            if dist < startBlendDist {
                let t = dist / startBlendDist
                z = startZ + (z - startZ) * smoothstep(t)
            }
            
            let distFromEnd = totalSmoothedDist - dist
            if distFromEnd < endBlendDist {
                let t = distFromEnd / endBlendDist
                z = endZ + (z - endZ) * smoothstep(t)
            }
            
            z = max(0, z)
            poly.append(SIMD3<Float>(xy.x, xy.y, z))
        }
        
        return RopeRenderResult(points: poly, segmentStarts: segmentStarts)
    }
    
    private static func smoothstep(_ t: Float) -> Float {
        let x = max(0, min(1, t))
        return x * x * (3 - 2 * x)
    }
    
    private static func segIntersection(_ a: SIMD2<Float>, _ b: SIMD2<Float>, _ c: SIMD2<Float>, _ d: SIMD2<Float>) -> SIMD2<Float>? {
        let x1 = a.x, y1 = a.y
        let x2 = b.x, y2 = b.y
        let x3 = c.x, y3 = c.y
        let x4 = d.x, y4 = d.y
        
        let den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-9 { return nil }
        
        let px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
        let py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
        
        let eps: Float = 1e-6
        
        func onSeg(_ p: Float, _ s1: Float, _ s2: Float) -> Bool {
            min(s1, s2) - eps <= p && p <= max(s1, s2) + eps
        }
        
        if onSeg(px, x1, x2) && onSeg(py, y1, y2) && onSeg(px, x3, x4) && onSeg(py, y3, y4) {
            return SIMD2<Float>(px, py)
        }
        return nil
    }
    
    private static func subdividePath(_ path: [SIMD2<Float>], n: Int) -> [SIMD2<Float>] {
        if path.count < 2 || n < 1 { return path }
        var out: [SIMD2<Float>] = []
        out.reserveCapacity((path.count - 1) * n + 1)
        
        for i in 0..<(path.count - 1) {
            let a = path[i]
            let b = path[i + 1]
            out.append(a)
            for j in 1..<n {
                let t = Float(j) / Float(n)
                out.append(a * (1 - t) + b * t)
            }
        }
        out.append(path[path.count - 1])
        return out
    }
    
    private static func relaxPath(_ pts: inout [SIMD2<Float>], fixed: Set<Int>, iterations: Int, k: Float) {
        for _ in 0..<iterations {
            for i in 1..<(pts.count - 1) {
                if fixed.contains(i) { continue }
                let mid = (pts[i - 1] + pts[i + 1]) * 0.5
                pts[i] = pts[i] + (mid - pts[i]) * k
            }
        }
    }
    
    private static func catmullToBezier(_ p0: SIMD2<Float>, _ p1: SIMD2<Float>, _ p2: SIMD2<Float>, _ p3: SIMD2<Float>, tension: Float = 0.5) -> (SIMD2<Float>, SIMD2<Float>, SIMD2<Float>, SIMD2<Float>) {
        let d1 = (p2 - p0) * (tension / 6.0)
        let d2 = (p3 - p1) * (tension / 6.0)
        return (p1, p1 + d1, p2 - d2, p2)
    }
    
    private static func evalBezier(_ b0: SIMD2<Float>, _ b1: SIMD2<Float>, _ b2: SIMD2<Float>, _ b3: SIMD2<Float>, t: Float) -> SIMD2<Float> {
        let mt = 1 - t
        return b0 * (mt * mt * mt) + b1 * (3 * mt * mt * t) + b2 * (3 * mt * t * t) + b3 * (t * t * t)
    }
    
    private struct ZLogKey: Hashable {
        let ropeIndex: Int
        let hookId: Int
    }
    private struct ZLogValue {
        let N: Int
        let pathA: [SIMD2<Float>]
        let pathB: [SIMD2<Float>]
    }
    nonisolated(unsafe) private static var lastZLogParams: [ZLogKey: ZLogValue] = [:]
    nonisolated(unsafe) private static var lastZLogTime: Double = 0
    
    struct SmoothResult {
        let points: [SIMD2<Float>]
        let segmentStarts: [Int]
    }
    
    private static func smoothPath2D(_ path: [SIMD2<Float>], ropeRadius: Float) -> SmoothResult {
        let n = max(1, smoothSubdivisions)
        let iterations = smoothIterations
        let k = smoothStrength
        
        if path.count < 2 {
            return SmoothResult(points: path, segmentStarts: [0])
        }
        
        var dense = subdividePath(path, n: n)
        
        var fixed = Set<Int>()
        for i in 0..<path.count {
            fixed.insert(i * n)
        }
        
        if iterations > 0 {
            relaxPath(&dense, fixed: fixed, iterations: iterations, k: k)
        }
        
        if dense.count < 4 {
            var segStarts: [Int] = []
            for i in 0..<path.count {
                segStarts.append(i * n)
            }
            return SmoothResult(points: dense, segmentStarts: segStarts)
        }
        
        let P = [dense[0]] + dense + [dense[dense.count - 1]]
        var curve: [SIMD2<Float>] = []
        var segmentStarts: [Int] = []
        let bezierSteps = 8
        
        for i in 0..<(dense.count - 1) {
            if i % n == 0 {
                segmentStarts.append(curve.count)
            }
            let (b0, b1, b2, b3) = catmullToBezier(P[i], P[i + 1], P[i + 2], P[i + 3])
            for j in 0..<bezierSteps {
                let t = Float(j) / Float(bezierSteps)
                curve.append(evalBezier(b0, b1, b2, b3, t: t))
            }
        }
        curve.append(dense[dense.count - 1])
        
        return SmoothResult(points: curve, segmentStarts: segmentStarts)
    }
    
    private static func cumulativeLengths(poly: [SIMD3<Float>]) -> [Float] {
        if poly.count < 2 { return [0] }
        var cum: [Float] = [0]
        cum.reserveCapacity(poly.count)
        for i in 1..<poly.count {
            cum.append(cum[i - 1] + simd_length(poly[i] - poly[i - 1]))
        }
        return cum
    }

    private static func sample(poly: [SIMD3<Float>], cum: [Float], dist: Float) -> SIMD3<Float> {
        if poly.count <= 1 { return poly.first ?? .zero }
        var lo = 0
        var hi = cum.count - 1
        while lo + 1 < hi {
            let mid = (lo + hi) / 2
            if cum[mid] <= dist {
                lo = mid
            } else {
                hi = mid
            }
        }
        let a = poly[lo]
        let b = poly[min(lo + 1, poly.count - 1)]
        let segStart = cum[lo]
        let segEnd = cum[min(lo + 1, cum.count - 1)]
        let span = max(1e-6, segEnd - segStart)
        let t = (dist - segStart) / span
        return a + (b - a) * t
    }

    private static func resample(poly: [SIMD3<Float>], count: Int) -> [SIMD3<Float>] {
        let n = max(0, count)
        if n == 0 { return [] }
        if poly.isEmpty { return Array(repeating: .zero, count: n) }
        if poly.count == 1 { return Array(repeating: poly[0], count: n) }

        let lengths = cumulativeLengths(poly: poly)
        let total = lengths.last ?? 0
        if total <= 1e-6 { return Array(repeating: poly[0], count: n) }

        var out: [SIMD3<Float>] = []
        out.reserveCapacity(n)
        for i in 0..<n {
            let t = Float(i) / Float(max(1, n - 1))
            out.append(sample(poly: poly, cum: lengths, dist: total * t))
        }
        return out
    }
}
