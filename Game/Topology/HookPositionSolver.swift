import simd
import os.log
import QuartzCore

final class HookPhysicsState {
    var positions: [Int: SIMD2<Float>] = [:]
    var velocities: [Int: SIMD2<Float>] = [:]
    var initialized: Bool = false
}

enum HookPositionSolver {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "HookPositionSolver")
    
    static let stiffness: Float = 800.0
    static let damping: Float = 15.0
    static let mass: Float = 1.0
    
    static func stepPhysics(
        state: HookPhysicsState,
        hooks: [Int: HookSequence],
        ropeStartPos: (Int) -> SIMD2<Float>,
        ropeEndPos: (Int) -> SIMD2<Float>,
        ropeHookIds: (Int) -> [Int],
        deltaTime: Float
    ) {
        if hooks.isEmpty { return }
        
        if !state.initialized {
            initializeState(
                state: state,
                hooks: hooks,
                ropeStartPos: ropeStartPos,
                ropeEndPos: ropeEndPos,
                ropeHookIds: ropeHookIds
            )
            state.initialized = true
        }
        
        for hookId in hooks.keys {
            if state.positions[hookId] == nil {
                if let hook = hooks[hookId] {
                    let A1 = ropeStartPos(hook.ropeA)
                    let A2 = ropeEndPos(hook.ropeA)
                    let B1 = ropeStartPos(hook.ropeB)
                    let B2 = ropeEndPos(hook.ropeB)
                    state.positions[hookId] = (A1 + A2 + B1 + B2) * 0.25
                    state.velocities[hookId] = .zero
                }
            }
        }
        
        let dt = min(deltaTime, 0.033)
        
        var forces: [Int: SIMD2<Float>] = [:]
        
        for (hookId, hook) in hooks {
            guard let pos = state.positions[hookId] else { continue }
            
            var force: SIMD2<Float> = .zero
            
            let neighborsA = findNeighbors(
                hookId: hookId,
                ropeIndex: hook.ropeA,
                positions: state.positions,
                ropeStartPos: ropeStartPos,
                ropeEndPos: ropeEndPos,
                ropeHookIds: ropeHookIds
            )
            
            let neighborsB = findNeighbors(
                hookId: hookId,
                ropeIndex: hook.ropeB,
                positions: state.positions,
                ropeStartPos: ropeStartPos,
                ropeEndPos: ropeEndPos,
                ropeHookIds: ropeHookIds
            )
            
            force += springForce(from: pos, to: neighborsA.prev)
            force += springForce(from: pos, to: neighborsA.next)
            force += springForce(from: pos, to: neighborsB.prev)
            force += springForce(from: pos, to: neighborsB.next)
            
            let vel = state.velocities[hookId] ?? .zero
            force -= vel * damping
            
            forces[hookId] = force
        }
        
        for (hookId, force) in forces {
            let acc = force / mass
            var vel = state.velocities[hookId] ?? .zero
            vel += acc * dt
            state.velocities[hookId] = vel
            
            var pos = state.positions[hookId] ?? .zero
            pos += vel * dt
            state.positions[hookId] = pos
        }
    }
    
    private static func springForce(from pos: SIMD2<Float>, to target: SIMD2<Float>) -> SIMD2<Float> {
        let delta = target - pos
        return delta * stiffness
    }
    
    private static func initializeState(
        state: HookPhysicsState,
        hooks: [Int: HookSequence],
        ropeStartPos: (Int) -> SIMD2<Float>,
        ropeEndPos: (Int) -> SIMD2<Float>,
        ropeHookIds: (Int) -> [Int]
    ) {
        var allRopeIndices: Set<Int> = []
        for hook in hooks.values {
            allRopeIndices.insert(hook.ropeA)
            allRopeIndices.insert(hook.ropeB)
        }
        
        for ropeIndex in allRopeIndices {
            let hookIds = ropeHookIds(ropeIndex)
            if hookIds.isEmpty { continue }
            
            let start = ropeStartPos(ropeIndex)
            let end = ropeEndPos(ropeIndex)
            let segmentCount = hookIds.count + 1
            
            for (i, hookId) in hookIds.enumerated() {
                let t = Float(i + 1) / Float(segmentCount)
                let posOnRope = start + (end - start) * t
                
                if let existing = state.positions[hookId] {
                    state.positions[hookId] = (existing + posOnRope) * 0.5
                } else {
                    state.positions[hookId] = posOnRope
                }
                state.velocities[hookId] = .zero
            }
        }
    }
    
    private static func findNeighbors(
        hookId: Int,
        ropeIndex: Int,
        positions: [Int: SIMD2<Float>],
        ropeStartPos: (Int) -> SIMD2<Float>,
        ropeEndPos: (Int) -> SIMD2<Float>,
        ropeHookIds: (Int) -> [Int]
    ) -> (prev: SIMD2<Float>, next: SIMD2<Float>) {
        let hookIds = ropeHookIds(ropeIndex)
        let orderedHooks = orderHooksAlongRope(
            hookIds: hookIds,
            centers: positions,
            ropeStart: ropeStartPos(ropeIndex),
            ropeEnd: ropeEndPos(ropeIndex)
        )
        
        guard let idx = orderedHooks.firstIndex(of: hookId) else {
            return (ropeStartPos(ropeIndex), ropeEndPos(ropeIndex))
        }
        
        let prevPos: SIMD2<Float>
        if idx == 0 {
            prevPos = ropeStartPos(ropeIndex)
        } else {
            let prevHookId = orderedHooks[idx - 1]
            prevPos = positions[prevHookId] ?? ropeStartPos(ropeIndex)
        }
        
        let nextPos: SIMD2<Float>
        if idx == orderedHooks.count - 1 {
            nextPos = ropeEndPos(ropeIndex)
        } else {
            let nextHookId = orderedHooks[idx + 1]
            nextPos = positions[nextHookId] ?? ropeEndPos(ropeIndex)
        }
        
        return (prevPos, nextPos)
    }
    
    static func orderHooksAlongRope(
        hookIds: [Int],
        centers: [Int: SIMD2<Float>],
        ropeStart: SIMD2<Float>,
        ropeEnd: SIMD2<Float>
    ) -> [Int] {
        if hookIds.count <= 1 { return hookIds }
        
        let dir = ropeEnd - ropeStart
        let dirLen = simd_length(dir)
        if dirLen < 1e-6 { return hookIds }
        
        let dirNorm = dir / dirLen
        
        var hooksWithT: [(id: Int, t: Float)] = []
        for hookId in hookIds {
            guard let center = centers[hookId] else { continue }
            let toCenter = center - ropeStart
            let t = simd_dot(toCenter, dirNorm)
            hooksWithT.append((hookId, t))
        }
        
        hooksWithT.sort { $0.t < $1.t }
        return hooksWithT.map { $0.id }
    }
}
