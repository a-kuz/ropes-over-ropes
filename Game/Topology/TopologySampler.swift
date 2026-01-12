import simd
import os.log

enum TopologySampler {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "TopologySampler")
    
    nonisolated(unsafe) static var hookStepMultiplier: Float = 1.0
    nonisolated(unsafe) static var hookRadiusMultiplier: Float = 1.0
    nonisolated(unsafe) static var hookStepLimitMultiplier: Float = 1.0
    nonisolated(unsafe) static var debugSegmentColors: Bool = false
    nonisolated(unsafe) static var smoothSubdivisions: Int = 6
    nonisolated(unsafe) static var smoothIterations: Int = 6
    nonisolated(unsafe) static var smoothStrength: Float = 0.4
    nonisolated(unsafe) static var smoothZone: Float = 1.0
    
    static func sampleRope(
        engine: TopologyEngine,
        ropeIndex: Int,
        count: Int,
        lift: Float,
        dragLift: Float,
        ropeWidth: Float,
        ropeWidthForIndex: (Int) -> Float,
        ropeHeightForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> [SIMD3<Float>] {
        let result = buildPoly(
            engine: engine,
            ropeIndex: ropeIndex,
            lift: lift,
            dragLift: dragLift,
            ropeWidth: ropeWidth,
            ropeWidthForIndex: ropeWidthForIndex,
            ropeHeightForIndex: ropeHeightForIndex,
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
        ropeWidth: Float,
        ropeWidthForIndex: (Int) -> Float,
        ropeHeightForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> RopeRenderResult {
        return buildPoly(
            engine: engine,
            ropeIndex: ropeIndex,
            lift: lift,
            dragLift: dragLift,
            ropeWidth: ropeWidth,
            ropeWidthForIndex: ropeWidthForIndex,
            ropeHeightForIndex: ropeHeightForIndex,
            holeRadius: holeRadius
        )
    }
    
    static func hookCenters(engine: TopologyEngine, ropeWidthForIndex: (Int) -> Float) -> [SIMD2<Float>] {
        var centers: [SIMD2<Float>] = []
        for (_, hook) in engine.hooks {
            let A1 = engine.ropeStart(hook.ropeA)
            let A2 = engine.ropeEnd(hook.ropeA)
            let B1 = engine.ropeStart(hook.ropeB)
            let B2 = engine.ropeEnd(hook.ropeB)
            let R = max(ropeWidthForIndex(hook.ropeA), ropeWidthForIndex(hook.ropeB)) * 0.5
            
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
        ropeWidth: Float,
        ropeWidthForIndex: (Int) -> Float,
        ropeHeightForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> RopeRenderResult {
        guard engine.ropes.indices.contains(ropeIndex) else { return RopeRenderResult(points: [], segmentStarts: []) }
        let rope = engine.ropes[ropeIndex]
        guard rope.active else { return RopeRenderResult(points: [], segmentStarts: []) }
        
        let holeInnerRadius = holeRadius * 0.76
        let holeDepth = holeRadius * 1.25
        let ropeHeight = ropeHeightForIndex(ropeIndex)
        let baseZ = ropeHeight * 0.5
        
        let rawStart = engine.ropeStart(ropeIndex)
        let rawEnd = engine.ropeEnd(ropeIndex)
        
        let startIsFloating = rope.floatingEnd == 0
        let endIsFloating = rope.floatingEnd == 1
        
        let startAnchor: SIMD2<Float>
        let startZ: Float
        if startIsFloating {
            startAnchor = rawStart
            startZ = dragLift
        } else {
            let dir = rawEnd - rawStart
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
            let dir = rawStart - rawEnd
            let dirLen = simd_length(dir)
            if dirLen > 1e-6 {
                endAnchor = rawEnd + (dir / dirLen) * holeInnerRadius
            } else {
                endAnchor = rawEnd
            }
            endZ = -holeDepth
        }
        
        let hookId = rope.hooks.first
        
        if let hookId = hookId, let hook = engine.hooks[hookId] {
            return buildHookPoly(
                engine: engine,
                ropeIndex: ropeIndex,
                hook: hook,
                startAnchor: startAnchor,
                endAnchor: endAnchor,
                startZ: startZ,
                endZ: endZ,
                baseZ: baseZ,
                ropeWidth: ropeWidth,
                ropeWidthForIndex: ropeWidthForIndex,
                ropeHeightForIndex: ropeHeightForIndex
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
        ropeWidth: Float,
        ropeWidthForIndex: (Int) -> Float,
        ropeHeightForIndex: (Int) -> Float
    ) -> RopeRenderResult {
        let isRopeA = (ropeIndex == hook.ropeA)
        let otherRopeIndex = isRopeA ? hook.ropeB : hook.ropeA
        let otherWidth = ropeWidthForIndex(otherRopeIndex)
        let otherHeight = ropeHeightForIndex(otherRopeIndex)
        let currentHeight = ropeHeightForIndex(ropeIndex)
        
        let A1 = engine.ropeStart(hook.ropeA)
        let A2 = engine.ropeEnd(hook.ropeA)
        let B1 = engine.ropeStart(hook.ropeB)
        let B2 = engine.ropeEnd(hook.ropeB)
        let R = max(ropeWidth, otherWidth) * 0.5
        
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
        
        let rawPath = isRopeA ? geom.pathA : geom.pathB
        
        var fullPath2D: [SIMD2<Float>] = [startAnchor]
        fullPath2D.append(contentsOf: rawPath)
        fullPath2D.append(endAnchor)
        
        let smoothedPath2D = smoothPath2D(fullPath2D)
        
        let firstIsOver: Bool
        if isRopeA {
            firstIsOver = hook.ropeAStartIsOver
        } else {
            firstIsOver = !hook.ropeAStartIsOver
        }
        
        let underZ = baseZ
        let overZ = otherHeight + currentHeight * 0.5
        
        func zForCrossing(_ idx: Int) -> Float {
            let isOver = (idx % 2 == 0) ? firstIsOver : !firstIsOver
            return isOver ? overZ : underZ
        }
        
        let originalPathCount = fullPath2D.count
        let subdivisions = max(1, smoothSubdivisions)
        
        var poly: [SIMD3<Float>] = []
        var segmentStarts: [Int] = []
        
        let N = hook.N
        
        for (smoothIdx, xy) in smoothedPath2D.enumerated() {
            let originalIdx = smoothIdx / subdivisions
            let localT = Float(smoothIdx % subdivisions) / Float(subdivisions)
            
            if smoothIdx % subdivisions == 0 {
                segmentStarts.append(poly.count)
            }
            
            let z: Float
            if originalIdx == 0 {
                let nextOriginalZ = baseZ
                z = startZ + (nextOriginalZ - startZ) * localT
            } else if originalIdx >= originalPathCount - 1 {
                z = baseZ + (endZ - baseZ) * localT
            } else {
                let crossingIdx = originalIdx - 1
                let nextCrossingIdx = min(crossingIdx + 1, N - 1)
                
                let fromZ: Float
                let toZ: Float
                
                if crossingIdx < 0 {
                    fromZ = baseZ
                    toZ = zForCrossing(0)
                } else if crossingIdx >= N - 1 {
                    fromZ = zForCrossing(N - 1)
                    toZ = baseZ
                } else {
                    fromZ = zForCrossing(crossingIdx)
                    toZ = zForCrossing(nextCrossingIdx)
                }
                
                z = fromZ + (toZ - fromZ) * smoothstep(localT)
            }
            
            poly.append(SIMD3<Float>(xy.x, xy.y, z))
        }
        
        return RopeRenderResult(points: poly, segmentStarts: segmentStarts)
    }
    
    private static func smoothstep(_ t: Float) -> Float {
        let x = max(0, min(1, t))
        return x * x * (3 - 2 * x)
    }
    
    private static func subdividePath2D(_ path: [SIMD2<Float>], subdivisions: Int) -> [SIMD2<Float>] {
        if path.count < 2 || subdivisions < 1 { return path }
        var result: [SIMD2<Float>] = []
        result.reserveCapacity((path.count - 1) * subdivisions + 1)
        
        for i in 0..<(path.count - 1) {
            let a = path[i]
            let b = path[i + 1]
            result.append(a)
            for j in 1..<subdivisions {
                let t = Float(j) / Float(subdivisions)
                result.append(a + (b - a) * t)
            }
        }
        result.append(path[path.count - 1])
        return result
    }
    
    private static func relaxPath2D(_ pts: [SIMD2<Float>], fixed: Set<Int>, iterations: Int, strength: Float, zoneRadius: Int) -> [SIMD2<Float>] {
        if pts.count < 3 { return pts }
        var P = pts
        
        let sortedFixed = fixed.sorted()
        
        func isInZone(_ idx: Int) -> Bool {
            for f in sortedFixed {
                if abs(idx - f) <= zoneRadius {
                    return true
                }
            }
            return false
        }
        
        for _ in 0..<iterations {
            for i in 1..<(P.count - 1) {
                if fixed.contains(i) { continue }
                if !isInZone(i) { continue }
                let mid = (P[i - 1] + P[i + 1]) * 0.5
                P[i] = P[i] + (mid - P[i]) * strength
            }
            
            for i in stride(from: P.count - 2, through: 1, by: -1) {
                if fixed.contains(i) { continue }
                if !isInZone(i) { continue }
                let mid = (P[i - 1] + P[i + 1]) * 0.5
                P[i] = P[i] + (mid - P[i]) * strength
            }
        }
        return P
    }
    
    private static func smoothPath2D(_ path: [SIMD2<Float>]) -> [SIMD2<Float>] {
        let subdivisions = smoothSubdivisions
        let iterations = smoothIterations
        let strength = smoothStrength
        let zone = smoothZone
        
        if subdivisions < 1 || iterations < 1 || path.count < 3 { return path }
        
        let subdivided = subdividePath2D(path, subdivisions: subdivisions)
        
        var fixed = Set<Int>()
        for i in 0..<path.count {
            fixed.insert(i * subdivisions)
        }
        
        let zoneRadius = max(1, Int(Float(subdivisions) * zone))
        
        let relaxed = relaxPath2D(subdivided, fixed: fixed, iterations: iterations, strength: strength, zoneRadius: zoneRadius)
        return relaxed
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
