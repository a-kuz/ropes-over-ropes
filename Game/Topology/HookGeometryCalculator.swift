import simd
import os.log

enum HookGeometryCalculator {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "HookGeometry")
    nonisolated(unsafe) private static var logCount = 0
    
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
        let R = R * radiusMultiplier
        logCount += 1
        let shouldLog = logCount <= 20
        
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
        
        let S1 = (A1 + B1) * 0.5
        let S2 = (A2 + B2) * 0.5
        
        let sigmaVec = S2 - S1
        let sigmaLen = simd_length(sigmaVec)
        
        if shouldLog {
            logger.info("  MIDPOINTS:")
            logger.info("    S1=(\(String(format: "%.3f", S1.x)), \(String(format: "%.3f", S1.y))) = mid(A1,B1)")
            logger.info("    S2=(\(String(format: "%.3f", S2.x)), \(String(format: "%.3f", S2.y))) = mid(A2,B2)")
            logger.info("    sigmaLen=\(String(format: "%.3f", sigmaLen))")
        }
        
        if sigmaLen < 1e-6 {
            if shouldLog { logger.info("  ❌ sigmaLen too small, returning nil") }
            return nil
        }
        
        let sigma = sigmaVec / sigmaLen
        let mid = (S1 + S2) * 0.5
        let sigmaPerp = SIMD2<Float>(-sigma.y, sigma.x)
        
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
                pathB: [B1, B2]
            )
        }
        
        let L = simd_length(A2 - A1) + simd_length(B2 - B1)
        let d = (abs(simd_dot(A1 - mid, sigmaPerp)) +
                 abs(simd_dot(A2 - mid, sigmaPerp)) +
                 abs(simd_dot(B1 - mid, sigmaPerp)) +
                 abs(simd_dot(B2 - mid, sigmaPerp))) / 4.0
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
        var pathB: [SIMD2<Float>] = [B1]
        
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
        pathB.append(B2)
        
        if shouldLog {
            logger.info("  RESULT pathA (\(pathA.count) pts):")
            for (i, p) in pathA.enumerated() {
                logger.info("    [\(i)]=(\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
            }
            logger.info("  RESULT pathB (\(pathB.count) pts):")
            for (i, p) in pathB.enumerated() {
                logger.info("    [\(i)]=(\(String(format: "%.3f", p.x)), \(String(format: "%.3f", p.y)))")
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
}
