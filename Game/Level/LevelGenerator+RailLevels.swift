import Foundation
import simd

extension LevelGenerator {

    // MARK: - Rail mode levels

    static func generateRailLevel(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let localId = levelId - 2000
        switch localId {
        case 1: return railLevel1(particleCount: particleCount)
        case 2: return railLevel2(particleCount: particleCount)
        case 3: return railLevel3(particleCount: particleCount)
        default: return railLevel1(particleCount: particleCount)
        }
    }

    // Rail Level 1: "First Push" — straight rail, 1 cart, 2 ropes + 4 holes
    // Player wraps rope around cart and pulls to push it to the station
    private static func railLevel1(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.5, yPosition:  0.4),  // 0: top-left
            .init(xPosition:  0.5, yPosition:  0.4),  // 1: top-right
            .init(xPosition: -0.5, yPosition: -0.4),  // 2: bottom-left
            .init(xPosition:  0.5, yPosition: -0.4),  // 3: bottom-right
        ]
        let rails: [LevelDefinition.RailDef] = [
            .init(points: [
                .init(xPosition: -0.6, yPosition: 0),
                .init(xPosition:  0.6, yPosition: 0),
            ])
        ]
        let carts: [LevelDefinition.CartDef] = [
            .init(railIndex: 0, startT: 0.15, radius: 0.12, mass: 0.5)
        ]
        let stations: [LevelDefinition.StationDef] = [
            .init(railIndex: 0, t: 0.85, radius: 0.15, cartIndex: 0)
        ]
        let ropes: [LevelDefinition.Rope] = [
            .init(startHole: 0, endHole: 2, color: colors[0], radius: 0.038),
            .init(startHole: 1, endHole: 3, color: colors[1], radius: 0.038),
        ]
        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }
        return LevelDefinition(
            mode: "rail", id: 2001, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: rails, carts: carts, stations: stations
        )
    }

    // Rail Level 2: "L-Turn" — L-shaped rail with turn
    private static func railLevel2(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.6, yPosition:  0.6),  // 0
            .init(xPosition:  0.6, yPosition:  0.6),  // 1
            .init(xPosition: -0.6, yPosition: -0.6),  // 2
            .init(xPosition:  0.6, yPosition: -0.6),  // 3
            .init(xPosition:  0.0, yPosition:  0.6),  // 4
            .init(xPosition:  0.0, yPosition: -0.6),  // 5
        ]
        let rails: [LevelDefinition.RailDef] = [
            .init(points: [
                .init(xPosition: -0.5, yPosition: -0.5),
                .init(xPosition: -0.5, yPosition:  0.2),
                .init(xPosition:  0.5, yPosition:  0.2),
            ])
        ]
        let carts: [LevelDefinition.CartDef] = [
            .init(railIndex: 0, startT: 0.05, radius: 0.12, mass: 0.5)
        ]
        let stations: [LevelDefinition.StationDef] = [
            .init(railIndex: 0, t: 0.95, radius: 0.15, cartIndex: 0)
        ]
        let ropes: [LevelDefinition.Rope] = [
            .init(startHole: 0, endHole: 2, color: colors[0], radius: 0.038),
            .init(startHole: 1, endHole: 3, color: colors[1], radius: 0.038),
            .init(startHole: 4, endHole: 5, color: colors[2], radius: 0.038),
        ]
        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }
        return LevelDefinition(
            mode: "rail", id: 2002, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: rails, carts: carts, stations: stations
        )
    }

    // Rail Level 3: "S-Curve" — S-shaped rail
    private static func railLevel3(particleCount: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.7, yPosition:  0.5),  // 0
            .init(xPosition:  0.7, yPosition:  0.5),  // 1
            .init(xPosition: -0.7, yPosition: -0.5),  // 2
            .init(xPosition:  0.7, yPosition: -0.5),  // 3
            .init(xPosition: -0.3, yPosition:  0.5),  // 4
            .init(xPosition:  0.3, yPosition:  0.5),  // 5
            .init(xPosition: -0.3, yPosition: -0.5),  // 6
            .init(xPosition:  0.3, yPosition: -0.5),  // 7
        ]
        let rails: [LevelDefinition.RailDef] = [
            .init(points: [
                .init(xPosition: -0.6, yPosition: -0.4),
                .init(xPosition: -0.2, yPosition: -0.4),
                .init(xPosition:  0.2, yPosition:  0.4),
                .init(xPosition:  0.6, yPosition:  0.4),
            ])
        ]
        let carts: [LevelDefinition.CartDef] = [
            .init(railIndex: 0, startT: 0.05, radius: 0.10, mass: 0.4)
        ]
        let stations: [LevelDefinition.StationDef] = [
            .init(railIndex: 0, t: 0.95, radius: 0.12, cartIndex: 0)
        ]
        let ropes: [LevelDefinition.Rope] = [
            .init(startHole: 0, endHole: 2, color: colors[0], radius: 0.038),
            .init(startHole: 1, endHole: 3, color: colors[1], radius: 0.038),
            .init(startHole: 4, endHole: 6, color: colors[2], radius: 0.038),
            .init(startHole: 5, endHole: 7, color: colors[3], radius: 0.038),
        ]
        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }
        return LevelDefinition(
            mode: "rail", id: 2003, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: rails, carts: carts, stations: stations
        )
    }
}
