import simd
import os.log

final class RopePhysics {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "RopePhysics")
    
    private struct RopeState: Equatable {
        let index: Int
        let count: Int
        let start: SIMD3<Float>
        let mid: SIMD3<Float>
        let end: SIMD3<Float>
    }
    
    private struct HookState: Equatable {
        let id: Int
        let ropeA: Int
        let ropeB: Int
        let N: Int
        let ropeAStartIsOver: Bool
    }
    
    private var lastRopeStates: [RopeState] = []
    private var lastHookStates: [HookState] = []
    
    func logStateIfNeeded(
        time: Double,
        ropes: [(index: Int, points: [SIMD3<Float>])],
        hooks: [Int: HookSequence]
    ) {
        var currentRopeStates: [RopeState] = []
        for rope in ropes {
            let pts = rope.points
            let count = pts.count
            if count == 0 {
                currentRopeStates.append(RopeState(index: rope.index, count: 0, start: SIMD3<Float>(0, 0, 0), mid: SIMD3<Float>(0, 0, 0), end: SIMD3<Float>(0, 0, 0)))
                continue
            }
            let startPt = pts.first!
            let midPt = pts[count / 2]
            let endPt = pts.last!
            currentRopeStates.append(RopeState(index: rope.index, count: count, start: startPt, mid: midPt, end: endPt))
        }
        
        var currentHookStates: [HookState] = []
        for (id, hook) in hooks {
            currentHookStates.append(HookState(id: id, ropeA: hook.ropeA, ropeB: hook.ropeB, N: hook.N, ropeAStartIsOver: hook.ropeAStartIsOver))
        }
        currentHookStates.sort { $0.id < $1.id }
        
        let ropesChanged = currentRopeStates != lastRopeStates
        let hooksChanged = currentHookStates != lastHookStates
        
        if ropesChanged || hooksChanged {
            lastRopeStates = currentRopeStates
            lastHookStates = currentHookStates
            
            Self.logger.info("═══════════════════════════════════════════")
            Self.logger.info("ROPE PHYSICS STATE @ t=\(String(format: "%.2f", time))s")
            
            for rope in ropes {
                let pts = rope.points
                let count = pts.count
                if count == 0 {
                    Self.logger.info("  Rope[\(rope.index)]: empty")
                    continue
                }
                let startPt = pts.first!
                let midPt = pts[count / 2]
                let endPt = pts.last!
                Self.logger.info("  Rope[\(rope.index)]: \(count) pts")
                Self.logger.info("    start: (\(String(format: "%.3f", startPt.x)), \(String(format: "%.3f", startPt.y)), \(String(format: "%.3f", startPt.z)))")
                Self.logger.info("    mid:   (\(String(format: "%.3f", midPt.x)), \(String(format: "%.3f", midPt.y)), \(String(format: "%.3f", midPt.z)))")
                Self.logger.info("    end:   (\(String(format: "%.3f", endPt.x)), \(String(format: "%.3f", endPt.y)), \(String(format: "%.3f", endPt.z)))")
            }
            
            Self.logger.info("  Hooks: \(hooks.count)")
            for (id, hook) in hooks {
                Self.logger.info("    [\(id)]: ropeA=\(hook.ropeA) ropeB=\(hook.ropeB) N=\(hook.N) ropeAStartIsOver=\(hook.ropeAStartIsOver)")
            }
            Self.logger.info("═══════════════════════════════════════════")
        }
    }
}
