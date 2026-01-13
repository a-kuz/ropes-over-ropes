import simd
import os.log
import QuartzCore

enum HookGeometryCalculator {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "HookGeometry")
    nonisolated(unsafe) private static var lastLogParams: (A1: SIMD2<Float>, A2: SIMD2<Float>, B1: SIMD2<Float>, B2: SIMD2<Float>, R: Float, N: Int)?
    nonisolated(unsafe) private static var lastLogTime: Double = 0
    
    static func calculateHookSequenceGeometry(
        A1: SIMD2<Float>,
        A2: SIMD2<Float>,
        B1: SIMD2<Float>,
        B2: SIMD2<Float>,
        R: Float,
        crossingCount: Int,
        stepMultiplier: Float = 1.0,
        radiusMultiplier: Float = 1.0,
        stepLimitMultiplier: Float = 1.0
    ) -> HookSequenceGeometry? {
        if let result = calculateHookSequenceGeometryInternal(
            A1: A1, A2: A2, B1: B1, B2: B2, R: R,
            crossingCount: crossingCount,
            stepMultiplier: stepMultiplier,
            radiusMultiplier: radiusMultiplier,
            stepLimitMultiplier: stepLimitMultiplier,
            forceSwap: false
        ) {
            let actualCrossings = countCrossings(pathA: result.pathA, pathB: result.pathB)
            if actualCrossings == crossingCount {
                return result
            }
            if let swappedResult = calculateHookSequenceGeometryInternal(
                A1: A1, A2: A2, B1: B1, B2: B2, R: R,
                crossingCount: crossingCount,
                stepMultiplier: stepMultiplier,
                radiusMultiplier: radiusMultiplier,
                stepLimitMultiplier: stepLimitMultiplier,
                forceSwap: true
            ) {
                let swappedCrossings = countCrossings(pathA: swappedResult.pathA, pathB: swappedResult.pathB)
                if swappedCrossings == crossingCount {
                    return swappedResult
                }
                if swappedCrossings > actualCrossings {
                    return swappedResult
                }
            }
            return result
        }
        return calculateHookSequenceGeometryInternal(
            A1: A1, A2: A2, B1: B1, B2: B2, R: R,
            crossingCount: crossingCount,
            stepMultiplier: stepMultiplier,
            radiusMultiplier: radiusMultiplier,
            stepLimitMultiplier: stepLimitMultiplier,
            forceSwap: true
        )
    }
    
    private static func countCrossings(pathA: [SIMD2<Float>], pathB: [SIMD2<Float>]) -> Int {
        var count = 0
        for i in 0..<(pathA.count - 1) {
            for j in 0..<(pathB.count - 1) {
                if segIntersection(pathA[i], pathA[i + 1], pathB[j], pathB[j + 1]) != nil {
                    count += 1
                }
            }
        }
        return count
    }
    
    private static func calculateHookSequenceGeometryInternal(
        A1: SIMD2<Float>,
        A2: SIMD2<Float>,
        B1: SIMD2<Float>,
        B2: SIMD2<Float>,
        R: Float,
        crossingCount: Int,
        stepMultiplier: Float = 1.0,
        radiusMultiplier: Float = 1.0,
        stepLimitMultiplier: Float = 1.0,
        forceSwap: Bool
    ) -> HookSequenceGeometry? {
        let R = R * radiusMultiplier
        let currentParams = (A1, A2, B1, B2, R, crossingCount)
        let now = CACurrentMediaTime()
        let timeSinceLastLog = now - lastLogTime
        let shouldLog: Bool
        if let last = lastLogParams {
            let eps: Float = 0.001
            let dataChanged = simd_length_squared(A1 - last.A1) > eps * eps ||
                       simd_length_squared(A2 - last.A2) > eps * eps ||
                       simd_length_squared(B1 - last.B1) > eps * eps ||
                       simd_length_squared(B2 - last.B2) > eps * eps ||
                       abs(R - last.R) > eps ||
                       crossingCount != last.N
            shouldLog = dataChanged && timeSinceLastLog >= 0.5
            if shouldLog {
                lastLogParams = currentParams
                lastLogTime = now
            }
        } else {
            shouldLog = timeSinceLastLog >= 0.5
            if shouldLog {
                lastLogParams = currentParams
                lastLogTime = now
            }
        }
        
        if shouldLog {
            logger.info("═══════════════════════════════════════════")
            logger.info("📐 HOOK GEOMETRY N=\(crossingCount)")
            logger.info("  INPUT:")
            logger.info("    A1=(\(String(format: "%.3f", A1.x)), \(String(format: "%.3f", A1.y)))")
            logger.info("    A2=(\(String(format: "%.3f", A2.x)), \(String(format: "%.3f", A2.y)))")
            logger.info("    B1=(\(String(format: "%.3f", B1.x)), \(String(format: "%.3f", B1.y)))")
            logger.info("    B2=(\(String(format: "%.3f", B2.x)), \(String(format: "%.3f", B2.y)))")
            logger.info("    R=\(String(format: "%.4f", R))")
        }
        
        if crossingCount < 1 {
            if shouldLog { logger.info("  ❌ N < 1, returning nil") }
            return nil
        }
        
        var B1eff = B1
        var B2eff = B2
        
        let shouldSwap: Bool
        if forceSwap {
            shouldSwap = true
        } else {
            let testS1_orig = (A1 + B1) * 0.5
            let testS2_orig = (A2 + B2) * 0.5
            let testSigmaLen_orig = simd_length(testS2_orig - testS1_orig)
            
            let testS1_swap = (A1 + B2) * 0.5
            let testS2_swap = (A2 + B1) * 0.5
            let testSigmaLen_swap = simd_length(testS2_swap - testS1_swap)
            
            if testSigmaLen_orig < 0.05 && lineIntersection(A1, A2, B1, B2) == nil {
                if segIntersection(A1, B1, A2, B2) != nil {
                    shouldSwap = true
                } else {
                    shouldSwap = false
                }
            } else if crossingCount > 1 && testSigmaLen_swap > testSigmaLen_orig * 2.0 && testSigmaLen_orig < 0.5 {
                shouldSwap = true
            } else {
                shouldSwap = false
            }
        }
        
        if shouldSwap {
            if shouldLog { logger.info("  ⚠️ Swapping B1/B2 for better sigma axis") }
            B1eff = B2
            B2eff = B1
        }
        
        let S1 = (A1 + B1eff) * 0.5
        let S2 = (A2 + B2eff) * 0.5
        
        var sigmaVec = S2 - S1
        var sigmaLen = simd_length(sigmaVec)
        
        if shouldLog {
            logger.info("  MIDPOINTS:")
            logger.info("    S1=(\(String(format: "%.3f", S1.x)), \(String(format: "%.3f", S1.y))) = mid(A1,B1eff)")
            logger.info("    S2=(\(String(format: "%.3f", S2.x)), \(String(format: "%.3f", S2.y))) = mid(A2,B2eff)")
            logger.info("    sigmaLen=\(String(format: "%.3f", sigmaLen))")
        }
        
        let mid: SIMD2<Float>
        let sigma: SIMD2<Float>
        let sigmaPerp: SIMD2<Float>
        
        if sigmaLen < 0.05 {
            if shouldLog { logger.info("  ⚠️ sigmaLen too small (\(String(format: "%.4f", sigmaLen))), using fallback") }
            
            if let intersection = lineIntersection(A1, A2, B1eff, B2eff) {
                mid = intersection
                if shouldLog {
                    logger.info("    lines intersect at (\(String(format: "%.3f", mid.x)), \(String(format: "%.3f", mid.y)))")
                }
                let dirA = A2 - A1
                let dirALen = simd_length(dirA)
                if dirALen > 1e-6 {
                    let perpA = SIMD2<Float>(-dirA.y, dirA.x) / dirALen
                    sigma = perpA
                    sigmaPerp = SIMD2<Float>(-sigma.y, sigma.x)
                    sigmaLen = 1.0
                } else {
                    if shouldLog { logger.info("  ❌ Cannot determine sigma direction") }
                    return nil
                }
            } else {
                if shouldLog { logger.info("    lines are PARALLEL, using A1-B1eff direction") }
                
                let midA = (A1 + A2) * 0.5
                let midB = (B1eff + B2eff) * 0.5
                mid = (midA + midB) * 0.5
                
                let dirAB = midB - midA
                let dirABLen = simd_length(dirAB)
                if dirABLen > 1e-6 {
                    sigmaPerp = dirAB / dirABLen
                    sigma = SIMD2<Float>(-sigmaPerp.y, sigmaPerp.x)
                    sigmaLen = dirABLen
                    if shouldLog {
                        logger.info("    mid=(\(String(format: "%.3f", mid.x)), \(String(format: "%.3f", mid.y)))")
                        logger.info("    sigma=(\(String(format: "%.3f", sigma.x)), \(String(format: "%.3f", sigma.y)))")
                        logger.info("    sigmaPerp=(\(String(format: "%.3f", sigmaPerp.x)), \(String(format: "%.3f", sigmaPerp.y)))")
                    }
                } else {
                    if shouldLog { logger.info("  ❌ Ropes are coincident") }
                    return nil
                }
            }
        } else {
            sigma = sigmaVec / sigmaLen
            mid = (S1 + S2) * 0.5
            sigmaPerp = SIMD2<Float>(-sigma.y, sigma.x)
        }
        
        if shouldLog {
            logger.info("  AXIS:")
            logger.info("    sigma=(\(String(format: "%.3f", sigma.x)), \(String(format: "%.3f", sigma.y)))")
            logger.info("    mid=(\(String(format: "%.3f", mid.x)), \(String(format: "%.3f", mid.y)))")
            logger.info("    sigmaPerp=(\(String(format: "%.3f", sigmaPerp.x)), \(String(format: "%.3f", sigmaPerp.y)))")
        }
        
        let centerCount = crossingCount - 1
        
        if centerCount == 0 {
            if shouldLog {
                logger.info("  N=1: No centers needed, simple crossing")
            }
            return HookSequenceGeometry(
                centers: [],
                R: R,
                pathA: [A1, A2],
                pathB: [B1eff, B2eff]
            )
        }
        
        let L = simd_length(A2 - A1) + simd_length(B2eff - B1eff)
        let d = (abs(simd_dot(A1 - mid, sigmaPerp)) +
                 abs(simd_dot(A2 - mid, sigmaPerp)) +
                 abs(simd_dot(B1eff - mid, sigmaPerp)) +
                 abs(simd_dot(B2eff - mid, sigmaPerp))) / 4.0
        let repulse = 100.0 / (d + 0.1)
        let baseStep = R + R * (2.0 * sigmaLen) / L
        let stepLimit = (sigmaLen / Float(max(1, centerCount))) * stepLimitMultiplier
        let step = min(baseStep * repulse * stepMultiplier, stepLimit)
        
        if shouldLog {
            logger.info("  STEP CALCULATION:")
            logger.info("    L=\(String(format: "%.3f", L)) (ropeA_len + ropeB_len)")
            logger.info("    d=\(String(format: "%.3f", d)) (avg perp dist)")
            logger.info("    repulse=\(String(format: "%.3f", repulse)) = 100/(d+0.1)")
            logger.info("    baseStep=\(String(format: "%.4f", baseStep)) = R + R*2*sigmaLen/L")
            logger.info("    stepMultiplier=\(String(format: "%.2f", stepMultiplier))")
            logger.info("    radiusMultiplier=\(String(format: "%.2f", radiusMultiplier))")
            logger.info("    stepLimitMultiplier=\(String(format: "%.2f", stepLimitMultiplier))")
            logger.info("    stepLimit=\(String(format: "%.4f", stepLimit))")
            logger.info("    step=\(String(format: "%.4f", step))")
            logger.info("    centerCount=\(centerCount)")
        }
        
        let centers = centeredPoints(mid: mid, sigma: sigma, count: centerCount, step: step)
        
        if shouldLog {
            logger.info("  CENTERS:")
            for (i, c) in centers.enumerated() {
                logger.info("    [\(i)]=(\(String(format: "%.3f", c.x)), \(String(format: "%.3f", c.y)))")
            }
        }
        
        let baseAngle = atan2(sigma.y, sigma.x)
        
        var pathA: [SIMD2<Float>] = [A1]
        var pathB: [SIMD2<Float>] = [B1eff]
        
        func side(_ p: SIMD2<Float>) -> Int {
            simd_dot(p - mid, sigmaPerp) > 0 ? 1 : -1
        }
        
        var prev = side(A1)
        
        if shouldLog {
            logger.info("  PATH BUILDING:")
            logger.info("    baseAngle=\(String(format: "%.1f", baseAngle * 180 / Float.pi))°")
            logger.info("    initial side(A1)=\(prev)")
        }
        
        for (i, c) in centers.enumerated() {
            let s = -prev
            let ang = baseAngle + Float(s) * Float.pi / 2.0
            let pA = contact(center: c, angle: ang, radius: R)
            let pB = contact(center: c, angle: ang + Float.pi, radius: R)
            
            if shouldLog {
                logger.info("    center[\(i)]: s=\(s), ang=\(String(format: "%.1f", ang * 180 / Float.pi))°")
                logger.info("      pA=(\(String(format: "%.3f", pA.x)), \(String(format: "%.3f", pA.y)))")
                logger.info("      pB=(\(String(format: "%.3f", pB.x)), \(String(format: "%.3f", pB.y)))")
            }
            
            pathA.append(pA)
            pathB.append(pB)
            
            prev = side(pA)
            if shouldLog { logger.info("      new prev=\(prev)") }
        }
        
        pathA.append(A2)
        pathB.append(B2eff)
        
        if shouldLog {
            logger.info("  RESULT pathA (\(pathA.count) pts):")
            for (i, p) in pathA.enumerated() {
                logger.info("    [\(i)]=(\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
            }
            logger.info("  RESULT pathB (\(pathB.count) pts):")
            for (i, p) in pathB.enumerated() {
                logger.info("    [\(i)]=(\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
            }
            
            logger.info("  CROSSING VERIFICATION:")
            var crossCount = 0
            for i in 0..<(pathA.count - 1) {
                for j in 0..<(pathB.count - 1) {
                    if let p = segIntersection(pathA[i], pathA[i + 1], pathB[j], pathB[j + 1]) {
                        crossCount += 1
                        logger.info("    ✓ segA[\(i)-\(i+1)] x segB[\(j)-\(j+1)] = (\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
                    }
                }
            }
            logger.info("    Total crossings: \(crossCount) (expected N=\(crossingCount))")
            if crossCount != crossingCount {
                logger.info("    ⚠️ MISMATCH! Expected \(crossingCount), got \(crossCount)")
            }
            logger.info("═══════════════════════════════════════════")
        }
        
        return HookSequenceGeometry(
            centers: centers,
            R: R,
            pathA: pathA,
            pathB: pathB
        )
    }
    
    private static func centeredPoints(mid: SIMD2<Float>, sigma: SIMD2<Float>, count: Int, step: Float) -> [SIMD2<Float>] {
        if count <= 0 { return [] }
        if count == 1 { return [mid] }
        let h = Float(count - 1) / 2.0
        var points: [SIMD2<Float>] = []
        points.reserveCapacity(count)
        for i in 0..<count {
            let offset = (Float(i) - h) * step
            points.append(mid + sigma * offset)
        }
        return points
    }
    
    private static func contact(center: SIMD2<Float>, angle: Float, radius: Float) -> SIMD2<Float> {
        SIMD2<Float>(center.x + radius * cos(angle), center.y + radius * sin(angle))
    }
    
    private static func lineIntersection(_ a1: SIMD2<Float>, _ a2: SIMD2<Float>, _ b1: SIMD2<Float>, _ b2: SIMD2<Float>) -> SIMD2<Float>? {
        let x1 = a1.x, y1 = a1.y
        let x2 = a2.x, y2 = a2.y
        let x3 = b1.x, y3 = b1.y
        let x4 = b2.x, y4 = b2.y
        
        let den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-9 { return nil }
        
        let px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / den
        let py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / den
        
        return SIMD2<Float>(px, py)
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
}
