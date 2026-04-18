import Foundation
import simd

extension LevelGenerator {

    // MARK: - Rescue mode levels

    static func generateRescueLevel(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let localId = levelId - 4000
        switch localId {
        case 1: return rescueLevel1(particleCount: particleCount)
        default: return rescueLevel1(particleCount: particleCount)
        }
    }

    // MARK: - Rescue Level 1: "Falling Platform"
    //
    // Platform hangs on 2 ropes. 2 ropes detached — free ends dangle.
    // Platform tilts dangerously. Wind pushes it.
    // Player must grab 2 free ends and insert them into slots
    // before platform crashes or ropes break.
    //
    // Corners: TL(0), TR(1), BR(2), BL(3)
    // Connected: TL, BL (left side holds)
    // Free: TR, BR (right side dangling)
    //
    private static func rescueLevel1(particleCount: Int) -> LevelDefinition {
        // Ceiling anchors — spread wide, high up
        let ceilZ: Float = 1.4
        let spreadX: Float = 0.6
        let spreadY: Float = 0.4
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -spreadX, yPosition:  spreadY, zPosition: ceilZ),  // 0: TL anchor
            .init(xPosition:  spreadX, yPosition:  spreadY, zPosition: ceilZ),  // 1: TR anchor
            .init(xPosition:  spreadX, yPosition: -spreadY, zPosition: ceilZ),  // 2: BR anchor
            .init(xPosition: -spreadX, yPosition: -spreadY, zPosition: ceilZ),  // 3: BL anchor
        ]

        // 4 ropes, thinner for drama
        let ropes: [LevelDefinition.Rope] = [
            .init(startHole: 0, endHole: 0, color: colors[0], radius: 0.030),  // TL — connected
            .init(startHole: 1, endHole: 1, color: colors[1], radius: 0.030),  // TR — FREE
            .init(startHole: 2, endHole: 2, color: colors[2], radius: 0.030),  // BR — FREE
            .init(startHole: 3, endHole: 3, color: colors[3], radius: 0.030),  // BL — connected
        ]

        // Pin top ends only
        var actions: [LevelDefinition.Action] = []
        for i in 0..<4 {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: i))
        }

        // Platform: bigger, heavier. Only left side connected → tilts right
        let platform = LevelDefinition.PlatformDef(
            width: 0.8,
            height: 0.5,
            mass: 20.0,
            attachments: [
                .init(ropeIndex: 0, cornerIndex: 0),  // rope 0 → TL
                .init(ropeIndex: 3, cornerIndex: 3),  // rope 3 → BL
            ],
            emptySlots: [1, 2],       // TR and BR corners empty
            freeRopeIndices: [1, 2]   // ropes 1 and 2 are free
        )

        return LevelDefinition(
            mode: "rescue",
            id: 4001,
            holeRadius: 0.08,
            particlesPerRope: particleCount,
            holes: holes,
            ropes: ropes,
            hooks: nil,
            actions: actions,
            boards: nil,
            weights: nil,
            targets: nil,
            rails: nil,
            carts: nil,
            stations: nil,
            platform: platform
        )
    }
}
