import Foundation
import simd

extension LevelGenerator {

    // MARK: - Tension mode levels

    static func generateTensionLevel(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let localId = levelId - 1000  // 1=first, 2=second, etc.
        switch localId {
        case 1: return tensionLevel1(particleCount: particleCount)
        case 2: return tensionLevel2(particleCount: particleCount)
        case 3: return tensionLevel3(particleCount: particleCount)
        case 4: return tensionLevel4(particleCount: particleCount)
        case 5: return tensionLevel5(particleCount: particleCount)
        default: return tensionLevel1(particleCount: particleCount)
        }
    }

    // MARK: — Tension Level 1: "First Pull"
    // 1 weight, 2 ropes, pull right. Tutorial.
    private static func tensionLevel1(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.6, yPosition: -0.3),  // 0
            .init(xPosition: -0.6, yPosition:  0.3),  // 1
            .init(xPosition:  0.6, yPosition: -0.3),  // 2
            .init(xPosition:  0.6, yPosition:  0.3),  // 3
        ]
        let wi = holes.count
        return LevelDefinition(
            mode: "tension", id: 1001, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: wi, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: wi, color: colors[1], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, wi), (1, 1, wi)], ),
            boards: nil,
            weights: [.init(x: -0.3, y: 0, mass: 0.3, radius: 0.1)],
            targets: [.init(x: 0.4, y: 0, radius: 0.15, weightIndex: 0)],
            rails: nil, carts: nil, stations: nil
        )
    }

    // MARK: — Tension Level 2: "Crossroads"
    // 1 weight center, 4 ropes from corners. Target is top-right. Must redirect pull.
    private static func tensionLevel2(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.6, yPosition: -0.5),  // 0: bottom-left
            .init(xPosition:  0.6, yPosition: -0.5),  // 1: bottom-right
            .init(xPosition: -0.6, yPosition:  0.5),  // 2: top-left
            .init(xPosition:  0.6, yPosition:  0.5),  // 3: top-right
            .init(xPosition:  0.0, yPosition: -0.7),  // 4: far bottom
            .init(xPosition:  0.0, yPosition:  0.7),  // 5: far top
        ]
        let wi = holes.count
        return LevelDefinition(
            mode: "tension", id: 1002, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: wi, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: wi, color: colors[1], radius: 0.035),
                .init(startHole: 2, endHole: wi, color: colors[2], radius: 0.035),
                .init(startHole: 4, endHole: wi, color: colors[3], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, wi), (1, 1, wi), (2, 2, wi), (3, 4, wi)], ),
            boards: nil,
            weights: [.init(x: 0, y: 0, mass: 0.4, radius: 0.1)],
            targets: [.init(x: 0.45, y: 0.4, radius: 0.15, weightIndex: 0)],
            rails: nil, carts: nil, stations: nil
        )
    }

    // MARK: — Tension Level 3: "Two Weights"
    // 2 weights, each with 2 ropes. Must deliver both to targets.
    private static func tensionLevel3(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.7, yPosition: -0.5),  // 0
            .init(xPosition: -0.7, yPosition:  0.0),  // 1
            .init(xPosition: -0.7, yPosition:  0.5),  // 2
            .init(xPosition:  0.7, yPosition: -0.5),  // 3
            .init(xPosition:  0.7, yPosition:  0.0),  // 4
            .init(xPosition:  0.7, yPosition:  0.5),  // 5
            .init(xPosition:  0.0, yPosition: -0.7),  // 6
            .init(xPosition:  0.0, yPosition:  0.7),  // 7
        ]
        let w0 = holes.count      // weight 0 index = 8
        let w1 = holes.count + 1  // weight 1 index = 9
        return LevelDefinition(
            mode: "tension", id: 1003, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: w0, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: w0, color: colors[1], radius: 0.035),
                .init(startHole: 2, endHole: w1, color: colors[2], radius: 0.035),
                .init(startHole: 7, endHole: w1, color: colors[3], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, w0), (1, 1, w0), (2, 2, w1), (3, 7, w1)], ),
            boards: nil,
            weights: [
                .init(x: -0.3, y: -0.25, mass: 0.3, radius: 0.1),
                .init(x: -0.3, y:  0.25, mass: 0.3, radius: 0.1),
            ],
            targets: [
                .init(x: 0.5, y: -0.3, radius: 0.15, weightIndex: 0),
                .init(x: 0.5, y:  0.3, radius: 0.15, weightIndex: 1),
            ],
            rails: nil, carts: nil, stations: nil
        )
    }

    // MARK: — Tension Level 4: "Heavy Lifter"
    // 1 heavy weight, 5 ropes needed. Many holes to choose from.
    private static func tensionLevel4(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.8, yPosition: -0.4),  // 0
            .init(xPosition: -0.8, yPosition:  0.4),  // 1
            .init(xPosition: -0.4, yPosition: -0.6),  // 2
            .init(xPosition: -0.4, yPosition:  0.6),  // 3
            .init(xPosition:  0.4, yPosition: -0.6),  // 4
            .init(xPosition:  0.4, yPosition:  0.6),  // 5
            .init(xPosition:  0.8, yPosition: -0.4),  // 6
            .init(xPosition:  0.8, yPosition:  0.4),  // 7
            .init(xPosition:  0.0, yPosition:  0.0),  // 8: center
        ]
        let wi = holes.count
        return LevelDefinition(
            mode: "tension", id: 1004, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: wi, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: wi, color: colors[1], radius: 0.035),
                .init(startHole: 2, endHole: wi, color: colors[2], radius: 0.035),
                .init(startHole: 3, endHole: wi, color: colors[3], radius: 0.035),
                .init(startHole: 8, endHole: wi, color: colors[4 % colors.count], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, wi), (1, 1, wi), (2, 2, wi), (3, 3, wi), (4, 8, wi)], ),
            boards: nil,
            weights: [.init(x: -0.5, y: 0, mass: 0.8, radius: 0.12)],
            targets: [.init(x: 0.6, y: 0, radius: 0.15, weightIndex: 0)],
            rails: nil, carts: nil, stations: nil
        )
    }

    // MARK: — Tension Level 5: "Triangle"
    // 3 weights in a triangle, each with 2 ropes. Targets form inverted triangle.
    private static func tensionLevel5(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.7, yPosition: -0.5),  // 0
            .init(xPosition:  0.7, yPosition: -0.5),  // 1
            .init(xPosition: -0.7, yPosition:  0.5),  // 2
            .init(xPosition:  0.7, yPosition:  0.5),  // 3
            .init(xPosition:  0.0, yPosition: -0.7),  // 4
            .init(xPosition:  0.0, yPosition:  0.7),  // 5
            .init(xPosition: -0.4, yPosition:  0.0),  // 6
            .init(xPosition:  0.4, yPosition:  0.0),  // 7
        ]
        let w0 = holes.count
        let w1 = w0 + 1
        let w2 = w0 + 2
        return LevelDefinition(
            mode: "tension", id: 1005, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: w0, color: colors[0], radius: 0.035),
                .init(startHole: 6, endHole: w0, color: colors[1], radius: 0.035),
                .init(startHole: 2, endHole: w1, color: colors[2], radius: 0.035),
                .init(startHole: 4, endHole: w1, color: colors[3], radius: 0.035),
                .init(startHole: 5, endHole: w2, color: colors[4 % colors.count], radius: 0.035),
                .init(startHole: 6, endHole: w2, color: colors[5 % colors.count], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [
                (0, 0, w0), (1, 6, w0),
                (2, 2, w1), (3, 4, w1),
                (4, 5, w2), (5, 6, w2),
            ], ),
            boards: nil,
            weights: [
                .init(x: -0.2, y: -0.3, mass: 0.3, radius: 0.09),
                .init(x: -0.2, y:  0.3, mass: 0.3, radius: 0.09),
                .init(x: -0.3, y:  0.0, mass: 0.3, radius: 0.09),
            ],
            targets: [
                .init(x: 0.5, y:  0.3, radius: 0.14, weightIndex: 0),
                .init(x: 0.5, y: -0.3, radius: 0.14, weightIndex: 1),
                .init(x: 0.4, y:  0.0, radius: 0.14, weightIndex: 2),
            ],
            rails: nil, carts: nil, stations: nil
        )
    }

    /// Helper: generate pin actions for tension levels.
    /// Each tuple: (ropeIndex, holeIndex for start, weightOrHoleIndex for end)
    static func pinActions(ropes: [(Int, Int, Int)]) -> [LevelDefinition.Action] {
        var actions: [LevelDefinition.Action] = []
        for (ri, startHole, endHole) in ropes {
            actions.append(.init(type: "pin", ropeIndex: ri, endIndex: 0, holeIndex: startHole))
            actions.append(.init(type: "pin", ropeIndex: ri, endIndex: 1, holeIndex: endHole))
        }
        return actions
    }
}
