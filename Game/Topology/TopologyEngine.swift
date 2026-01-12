import simd
import os.log

struct TopologySnapshot {
    var ropes: [TopologyRope]
    var hooks: [Int: HookSequence]
    var nextHookId: Int
}

final class TopologyEngine {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "TopologyEngine")
    private(set) var ropes: [TopologyRope]
    private(set) var hooks: [Int: HookSequence] = [:]
    private var nextHookId: Int = 1

    var holePositions: [SIMD2<Float>]

    init(holePositions: [SIMD2<Float>], ropeConfigs: [(startHole: Int, endHole: Int, color: SIMD3<Float>)]) {
        self.holePositions = holePositions
        self.ropes = ropeConfigs.map { config in
            TopologyRope(
                startHole: config.startHole,
                endHole: config.endHole,
                hooks: [],
                color: config.color,
                active: true,
                floatingEnd: nil,
                floatingPosition: nil
            )
        }
        buildInitialHooks()
    }

    func ropeStart(_ ropeIndex: Int) -> SIMD2<Float> {
        guard ropes.indices.contains(ropeIndex) else { return .zero }
        let rope = ropes[ropeIndex]
        if rope.floatingEnd == 0, let pos = rope.floatingPosition {
            return pos
        }
        return holePositions[safe: rope.startHole] ?? .zero
    }

    func ropeEnd(_ ropeIndex: Int) -> SIMD2<Float> {
        guard ropes.indices.contains(ropeIndex) else { return .zero }
        let rope = ropes[ropeIndex]
        if rope.floatingEnd == 1, let pos = rope.floatingPosition {
            return pos
        }
        return holePositions[safe: rope.endHole] ?? .zero
    }

    func snapshot() -> TopologySnapshot {
        TopologySnapshot(
            ropes: ropes,
            hooks: hooks,
            nextHookId: nextHookId
        )
    }

    func restore(_ snapshot: TopologySnapshot) {
        ropes = snapshot.ropes
        hooks = snapshot.hooks
        nextHookId = snapshot.nextHookId
    }

    func beginDrag(ropeIndex: Int, endIndex: Int, position: SIMD2<Float>) {
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        
        Self.logger.info("🎯 BEGIN DRAG: rope=\(ropeIndex) end=\(endIndex) pos=(\(position.x), \(position.y))")
        logState(label: "Before beginDrag")
        
        ropes[ropeIndex].floatingEnd = endIndex
        ropes[ropeIndex].floatingPosition = position
        
        logState(label: "After beginDrag")
    }

    func endDrag(ropeIndex: Int, endIndex: Int, holeIndex: Int) {
        guard ropes.indices.contains(ropeIndex) else { return }
        guard ropes[ropeIndex].active else { return }
        
        Self.logger.info("🏁 END DRAG: rope=\(ropeIndex) end=\(endIndex) hole=\(holeIndex)")
        logState(label: "Before endDrag")
        
        if endIndex == 0 {
            ropes[ropeIndex].startHole = holeIndex
        } else {
            ropes[ropeIndex].endHole = holeIndex
        }
        ropes[ropeIndex].floatingEnd = nil
        ropes[ropeIndex].floatingPosition = nil
        
        logState(label: "After endDrag")
    }
    
    func setFloating(ropeIndex: Int, position: SIMD2<Float>) {
        guard ropes.indices.contains(ropeIndex) else { return }
        ropes[ropeIndex].floatingPosition = position
    }

    func processCanonicalMove(ropeIndex: Int, endIndex: Int, from: SIMD2<Float>, to: SIMD2<Float>) {
        let dir = to - from
        if simd_length_squared(dir) < 1e-8 { return }

        Self.logger.info("🔄 CANONICAL MOVE: rope=\(ropeIndex) end=\(endIndex)")
        
        for otherRopeIndex in ropes.indices where otherRopeIndex != ropeIndex {
            if !ropes[otherRopeIndex].active { continue }
            
            let otherStart = ropeStart(otherRopeIndex)
            let otherEnd = ropeEnd(otherRopeIndex)
            
            guard let intersection = segmentIntersection(a0: from, a1: to, b0: otherStart, b1: otherEnd) else {
                continue
            }
            
            Self.logger.info("  ✖️ Crossed rope[\(otherRopeIndex)] at (\(intersection.x), \(intersection.y))")
            
            let low = min(ropeIndex, otherRopeIndex)
            let high = max(ropeIndex, otherRopeIndex)
            
            if let existingHookId = findHook(ropeA: low, ropeB: high) {
                var hook = hooks[existingHookId]!
                let isTop = isEndTop(ropeIndex: ropeIndex, endIndex: endIndex, hook: hook)
                
                if isTop {
                    hook.N -= 1
                    Self.logger.info("  ⬇️ TOP end crossed: N=\(hook.N + 1) → \(hook.N)")
                    
                    if hook.N <= 0 {
                        removeHook(hookId: existingHookId)
                        Self.logger.info("  ❌ Hook removed (N=0)")
                    } else {
                        hook.ropeAStartIsOver.toggle()
                        hooks[existingHookId] = hook
                    }
                } else {
                    hook.N += 1
                    hook.ropeAStartIsOver.toggle()
                    hooks[existingHookId] = hook
                    Self.logger.info("  ⬆️ BOTTOM end crossed: N=\(hook.N - 1) → \(hook.N)")
                }
            } else {
                let hookId = nextHookId
                nextHookId += 1
                
                let movingRopeIsA = (ropeIndex == low)
                let ropeAStartIsOver = !movingRopeIsA
                
                let newHook = HookSequence(
                    id: hookId,
                    ropeA: low,
                    ropeB: high,
                    N: 1,
                    ropeAStartIsOver: ropeAStartIsOver
                )
                hooks[hookId] = newHook
                ropes[low].hooks.append(hookId)
                ropes[high].hooks.append(hookId)
                
                Self.logger.info("  ✅ Created hook[\(hookId)] between ropes \(low) and \(high), N=1")
            }
        }
        
        ropes[ropeIndex].floatingPosition = to
    }

    func deactivateRope(ropeIndex: Int) {
        guard ropes.indices.contains(ropeIndex) else { return }
        ropes[ropeIndex].active = false
        
        let hooksToRemove = ropes[ropeIndex].hooks
        for hookId in hooksToRemove {
            removeHook(hookId: hookId)
        }
    }

    func findHook(ropeA: Int, ropeB: Int) -> Int? {
        for (id, hook) in hooks {
            if (hook.ropeA == ropeA && hook.ropeB == ropeB) ||
               (hook.ropeA == ropeB && hook.ropeB == ropeA) {
                return id
            }
        }
        return nil
    }

    func isEndTop(ropeIndex: Int, endIndex: Int, hook: HookSequence) -> Bool {
        let isRopeA = (ropeIndex == hook.ropeA)
        let ropeStartIsOver: Bool
        
        if isRopeA {
            ropeStartIsOver = hook.ropeAStartIsOver
        } else {
            ropeStartIsOver = !hook.ropeAStartIsOver
        }
        
        if endIndex == 0 {
            return ropeStartIsOver
        } else {
            if hook.N % 2 == 1 {
                return ropeStartIsOver
            } else {
                return !ropeStartIsOver
            }
        }
    }

    private func removeHook(hookId: Int) {
        guard let hook = hooks[hookId] else { return }
        
        ropes[hook.ropeA].hooks.removeAll { $0 == hookId }
        ropes[hook.ropeB].hooks.removeAll { $0 == hookId }
        hooks[hookId] = nil
    }

    private func buildInitialHooks() {
        for aIndex in ropes.indices {
            if !ropes[aIndex].active { continue }
            let a0 = ropeStart(aIndex)
            let a1 = ropeEnd(aIndex)

            for bIndex in (aIndex + 1)..<ropes.count {
                if !ropes[bIndex].active { continue }
                let b0 = ropeStart(bIndex)
                let b1 = ropeEnd(bIndex)

                if let _ = segmentIntersection(a0: a0, a1: a1, b0: b0, b1: b1) {
                    let hookId = nextHookId
                    nextHookId += 1

                    let newHook = HookSequence(
                        id: hookId,
                        ropeA: aIndex,
                        ropeB: bIndex,
                        N: 1,
                        ropeAStartIsOver: false
                    )
                    hooks[hookId] = newHook
                    ropes[aIndex].hooks.append(hookId)
                    ropes[bIndex].hooks.append(hookId)
                }
            }
        }
    }

    private func segmentIntersection(a0: SIMD2<Float>, a1: SIMD2<Float>, b0: SIMD2<Float>, b1: SIMD2<Float>) -> SIMD2<Float>? {
        let d1 = a1 - a0
        let d2 = b1 - b0
        let cross = d1.x * d2.y - d1.y * d2.x
        
        if abs(cross) < 1e-9 { return nil }
        
        let d = b0 - a0
        let t = (d.x * d2.y - d.y * d2.x) / cross
        let u = (d.x * d1.y - d.y * d1.x) / cross
        
        if t >= 0 && t <= 1 && u >= 0 && u <= 1 {
            return a0 + d1 * t
        }
        return nil
    }

    private func logState(label: String) {
        Self.logger.info("📊 \(label):")
        Self.logger.info("  Hooks: \(self.hooks.count)")
        for (id, hook) in hooks {
            Self.logger.info("    [\(id)]: ropeA=\(hook.ropeA) ropeB=\(hook.ropeB) N=\(hook.N) ropeAStartIsOver=\(hook.ropeAStartIsOver)")
        }
        for ropeIndex in ropes.indices where ropes[ropeIndex].active {
            let rope = ropes[ropeIndex]
            let hookList = rope.hooks.map { "H\($0)" }.joined(separator: ",")
            let floating = rope.floatingEnd != nil ? " [floating end \(rope.floatingEnd!)]" : ""
            Self.logger.info("  Rope[\(ropeIndex)]: hole\(rope.startHole) → hole\(rope.endHole) hooks=[\(hookList)]\(floating)")
        }
    }
}
