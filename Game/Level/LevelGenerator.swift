import Foundation
import simd

enum LevelGenerator {

    // MARK: - Public

    static func generate(levelId: Int, boardElevation: Float = 0.12, particleCount: Int = 55) -> LevelDefinition {
        var rng = SeededRNG(seed: UInt64(levelId) &* 2654435761)

        let diff = difficulty(for: levelId)
        let use3D = levelId >= 10 && levelId % 3 == 0
        let n = minHoles(for: levelId, ropeCount: diff.ropeCount)

        let holes: [LevelDefinition.Vec2]
        let boards: [LevelDefinition.Board]?

        if use3D {
            let bl = pickBoardLayout(for: levelId)
            let result = generateBoardLayout(bl, n: n, elevation: boardElevation)
            holes = result.holes
            boards = result.boards
        } else {
            let layout = pickLayout(for: levelId)
            holes = layout.generate(n: n)
            boards = nil
        }

        let maxRopes = max(1, (holes.count - 3) / 2)
        let ropeCount = min(diff.ropeCount, maxRopes)
        let shortCount = min(diff.shortRopeCount, max(0, ropeCount - 1))
        let ropePairs = pickStructuredPairs(holes: holes, count: ropeCount, shortCount: shortCount, rng: &rng)
        let ropes = ropePairs.enumerated().map { i, pair in
            LevelDefinition.Rope(startHole: pair.0, endHole: pair.1,
                                 color: colors[i % colors.count], radius: 0.038)
        }
        let actions = buildActions(ropes: ropes, holes: holes,
                                   totalDrags: diff.totalDrags, shortCount: shortCount,
                                   swapPercent: diff.swapPercent)

        return LevelDefinition(
            mode: nil,
            id: levelId,
            holeRadius: 0.08,
            particlesPerRope: particleCount,
            holes: holes,
            ropes: ropes,
            hooks: nil,
            actions: actions,
            boards: boards,
            weights: nil,
            targets: nil,
            rails: nil,
            carts: nil,
            stations: nil
        )
    }

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
    private static func pinActions(ropes: [(Int, Int, Int)]) -> [LevelDefinition.Action] {
        var actions: [LevelDefinition.Action] = []
        for (ri, startHole, endHole) in ropes {
            actions.append(.init(type: "pin", ropeIndex: ri, endIndex: 0, holeIndex: startHole))
            actions.append(.init(type: "pin", ropeIndex: ri, endIndex: 1, holeIndex: endHole))
        }
        return actions
    }

    // MARK: - Particle-based structured levels

    static func generateParticleBraidLevel(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let localId = levelId - 3000
        let strandCount: Int
        let crossings: Int
        switch localId {
        case 1:  strandCount = 2; crossings = 4
        case 2:  strandCount = 3; crossings = 4
        case 3:  strandCount = 3; crossings = 6
        case 4:  strandCount = 2; crossings = 8
        case 5:  strandCount = 4; crossings = 4
        case 6:  strandCount = 3; crossings = 8
        case 7:  strandCount = 5; crossings = 4
        case 8:  strandCount = 4; crossings = 6
        case 9:  strandCount = 3; crossings = 10
        case 10: strandCount = 6; crossings = 4
        default:
            strandCount = min(6, 2 + (localId - 1) / 3)
            crossings = min(12, 4 + localId / 2)
        }
        let width: Float = min(0.56, Float(strandCount) * 0.16)
        return buildStructuredLevel(levelId: levelId, particleCount: particleCount, structures: [
            .braid(BraidConfig(center: .zero, axis: SIMD2(0, 1), length: 1.1,
                               strandCount: strandCount, crossings: crossings,
                               width: width, zRadius: 0.06))
        ])
    }

    // MARK: - Structure primitives

    /// A single structural element that generates ropes with particle positions.
    enum StructurePrimitive {
        /// Helical braid along arbitrary axis
        case helix(HelixConfig)
        /// Classic braid: N strands swap positions along axis (like hair braid)
        case braid(BraidConfig)
        /// Central knot: N ropes loop from outer ring through center, creating tension
        case centralKnot(CentralKnotConfig)
        /// Radial vortex: N ropes converge to center and twist around each other
        case vortex(VortexConfig)
        /// Woven lattice: horizontal + vertical ropes weaving over/under
        case weave(WeaveConfig)
    }

    struct HelixConfig {
        let center: SIMD2<Float>    // center of braid in XY
        let axis: SIMD2<Float>      // direction (will be normalized)
        let length: Float           // total length along axis
        let strandCount: Int
        let turns: Float
        let helixRadius: Float
        let zRadius: Float
    }

    struct BraidConfig {
        let center: SIMD2<Float>    // center of braid in XY
        let axis: SIMD2<Float>      // direction (will be normalized)
        let length: Float           // total length along axis
        let strandCount: Int        // number of strands (2..6)
        let crossings: Int          // number of crossing events
        let width: Float            // total width across braid
        let zRadius: Float          // Z over/under amplitude
    }

    struct CentralKnotConfig {
        let center: SIMD2<Float>
        let outerRadius: Float      // outer ring radius (~0.62)
        let innerRadius: Float      // inner ring radius (~0.35)
        let ropeCount: Int          // number of ropes (3 = classic)
        let loopDepth: Float        // how far control point goes to opposite side
        let zRadius: Float          // Z over/under amplitude
    }

    struct VortexConfig {
        let center: SIMD2<Float>    // center of vortex
        let ropeCount: Int          // number of ropes through center
        let armLength: Float        // distance from center to holes
        let knotRadius: Float       // radius of twist zone
        let orbitRadius: Float      // orbital displacement in knot
        let turns: Float            // number of twists
        let zRadius: Float          // Z over/under amplitude
    }

    struct WeaveConfig {
        let center: SIMD2<Float>
        let width: Float            // horizontal extent
        let height: Float           // vertical extent
        let hCount: Int             // horizontal ropes
        let vCount: Int             // vertical ropes
        let zRadius: Float          // Z over/under amplitude
    }

    /// Build a level from one or more structure primitives.
    /// - `extraHoleRings`: concentric rings of decorative holes `(radius, count)`.
    ///   Pass empty array to skip decorative holes.
    static func buildStructuredLevel(
        levelId: Int,
        particleCount: Int = 55,
        ropeRadius: Float = 0.038,
        holeRadius: Float = 0.08,
        structures: [StructurePrimitive],
        extraHoleRings: [(radius: Float, count: Int)]? = nil
    ) -> LevelDefinition {
        let P = max(10, particleCount)
        let zBase: Float = 0.06
        let convergeFrac: Float = 0.12

        var holes: [LevelDefinition.Vec2] = []
        var ropes: [LevelDefinition.Rope] = []
        var allParticles: [[LevelDefinition.Vec2]] = []
        var colorIdx = 0

        for structure in structures {
            switch structure {
            case .helix(let cfg):
                buildHelixParticles(cfg: cfg, P: P, zBase: zBase, convergeFrac: convergeFrac,
                                    ropeRadius: ropeRadius, colorIdx: &colorIdx,
                                    holes: &holes, ropes: &ropes, allParticles: &allParticles)

            case .braid(let cfg):
                buildClassicBraidParticles(cfg: cfg, P: P, zBase: zBase,
                                           ropeRadius: ropeRadius, colorIdx: &colorIdx,
                                           holes: &holes, ropes: &ropes, allParticles: &allParticles)

            case .centralKnot(let cfg):
                buildCentralKnotParticles(cfg: cfg, P: P, zBase: zBase,
                                          ropeRadius: ropeRadius, colorIdx: &colorIdx,
                                          holes: &holes, ropes: &ropes, allParticles: &allParticles)

            case .vortex(let cfg):
                buildVortexParticles(cfg: cfg, P: P, zBase: zBase,
                                     ropeRadius: ropeRadius, colorIdx: &colorIdx,
                                     holes: &holes, ropes: &ropes, allParticles: &allParticles)

            case .weave(let cfg):
                buildWeaveParticles(cfg: cfg, P: P, zBase: zBase,
                                    ropeRadius: ropeRadius, colorIdx: &colorIdx,
                                    holes: &holes, ropes: &ropes, allParticles: &allParticles)
            }
        }

        // Add decorative holes in concentric rings (default: 2 rings)
        let rings = extraHoleRings ?? [(0.30, 8), (0.55, 14)]
        addDecorativeHoles(existing: &holes, rings: rings, minSpacing: holeRadius * 2.5)

        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }

        return LevelDefinition(
            mode: nil, id: levelId, holeRadius: holeRadius, particlesPerRope: P,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: nil, carts: nil, stations: nil,
            ropeParticles: allParticles
        )
    }

    /// Add decorative holes in concentric rings, skipping positions too close to existing holes.
    private static func addDecorativeHoles(
        existing: inout [LevelDefinition.Vec2],
        center: SIMD2<Float> = .zero,
        rings: [(radius: Float, count: Int)],
        minSpacing: Float = 0.20
    ) {
        // Center hole
        if !existing.contains(where: { simd_distance($0.simd, center) < minSpacing }) {
            existing.append(.init(xPosition: center.x, yPosition: center.y))
        }

        for ring in rings {
            for i in 0..<ring.count {
                let angle = 2 * Float.pi * Float(i) / Float(ring.count)
                let pos = center + ring.radius * SIMD2<Float>(cos(angle), sin(angle))
                if !existing.contains(where: { simd_distance($0.simd, pos) < minSpacing }) {
                    existing.append(.init(xPosition: pos.x, yPosition: pos.y))
                }
            }
        }
    }

    // MARK: - Helix builder (generalized direction)

    private static func buildHelixParticles(
        cfg: HelixConfig, P: Int, zBase: Float, convergeFrac: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.strandCount
        let dir = simd_normalize(cfg.axis)
        let perp = SIMD2<Float>(-dir.y, dir.x) // perpendicular in XY
        let halfLen = cfg.length / 2

        // Ensure Z separation ≥ ropeRadius at all crossings.
        // At crossing, gap = 2 × zRadius × sin(π/N). Need gap ≥ ropeRadius.
        let minZRadius = n > 1 ? (ropeRadius * 0.6) / sin(Float.pi / Float(n)) : cfg.zRadius
        let zR = max(cfg.zRadius, minZRadius)

        let startPt = cfg.center - dir * halfLen
        let endPt = cfg.center + dir * halfLen

        for k in 0..<n {
            let phase = 2 * Float.pi * Float(k) / Float(n)

            // Hole positions: helix offset at endpoints
            let startOffset = perp * cfg.helixRadius * cos(phase)
            let startHolePos = startPt + startOffset
            let endAngle = 2 * Float.pi * cfg.turns + phase
            let endOffset = perp * cfg.helixRadius * cos(endAngle)
            let endHolePos = endPt + endOffset

            // Small offset along axis per strand to prevent hole overlap
            let axisOff = dir * Float(k) * 0.015

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startHolePos.x + axisOff.x,
                               yPosition: startHolePos.y + axisOff.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endHolePos.x - axisOff.x,
                               yPosition: endHolePos.y - axisOff.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let baseXY = startPt + axisOff + (endPt - axisOff - startPt - axisOff) * t
                let angle = 2 * Float.pi * cfg.turns * t + phase

                let env = smoothEnvelope(t: t, convergeFrac: convergeFrac)

                let helixOffset = perp * cfg.helixRadius * cos(angle)
                let x = baseXY.x + helixOffset.x
                let y = baseXY.y + helixOffset.y
                let z = max(0, zBase + zR * sin(angle) * env)

                particles.append(.init(xPosition: x, yPosition: y, zPosition: z))
            }

            // Snap endpoints
            particles[0] = .init(xPosition: startHolePos.x + axisOff.x,
                                 yPosition: startHolePos.y + axisOff.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endHolePos.x - axisOff.x,
                                     yPosition: endHolePos.y - axisOff.y, zPosition: 0)

            allParticles.append(particles)
        }
    }

    // MARK: - Classic braid builder (strand-swapping, like hair braid)

    private static func buildClassicBraidParticles(
        cfg: BraidConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.strandCount
        let dir = simd_normalize(cfg.axis)
        let perp = SIMD2<Float>(-dir.y, dir.x)
        let halfLen = cfg.length / 2
        let halfW = cfg.width / 2

        // Slot X-offsets across the braid width
        let slotX: [Float] = (0..<n).map { i in
            n == 1 ? 0 : -halfW + cfg.width * Float(i) / Float(n - 1)
        }

        // Build crossing sequence: alternating even/odd pair swaps (classic braid)
        // Each crossing swaps two adjacent strands.
        struct SwapEvent {
            let t: Float       // position along axis [0,1]
            let slotA: Int     // left slot
            let slotB: Int     // right slot
            let strandA: Int   // strand currently in slotA
            let strandB: Int   // strand currently in slotB
            let aGoesOver: Bool
        }

        var strandInSlot = Array(0..<n)
        var swaps: [SwapEvent] = []
        var overToggle = true

        for c in 0..<cfg.crossings {
            let t = (Float(c) + 0.5) / Float(cfg.crossings)
            let startSlot = c % 2 == 0 ? 0 : 1
            for s in stride(from: startSlot, to: n - 1, by: 2) {
                let sA = strandInSlot[s]
                let sB = strandInSlot[s + 1]
                swaps.append(SwapEvent(t: t, slotA: s, slotB: s + 1,
                                       strandA: sA, strandB: sB,
                                       aGoesOver: overToggle))
                strandInSlot[s] = sB
                strandInSlot[s + 1] = sA
                overToggle.toggle()
            }
        }

        // For each strand: compute keyframes (t → slot), then interpolate path
        for k in 0..<n {
            // Collect keyframes: (t, slotIndex) for this strand
            var keyframes: [(t: Float, slot: Int)] = [(0, k)]
            var curSlot = k
            for sw in swaps {
                if sw.strandA == k {
                    curSlot = sw.slotB
                    keyframes.append((sw.t, curSlot))
                } else if sw.strandB == k {
                    curSlot = sw.slotA
                    keyframes.append((sw.t, curSlot))
                }
            }
            keyframes.append((1.0, curSlot))

            let startSlotIdx = keyframes.first!.slot
            let endSlotIdx = keyframes.last!.slot
            let startPos = cfg.center - dir * halfLen + perp * slotX[startSlotIdx]
            let endPos = cfg.center + dir * halfLen + perp * slotX[endSlotIdx]

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            // Build Z keyframes: at each crossing this strand is involved in, assign over/under Z.
            // Between crossings, z = zBase. This prevents overlapping bump interference.
            let zOver = zBase + ropeRadius * 2.0
            let zUnder = max(0, zBase - ropeRadius * 0.5)

            struct ZKey { let t: Float; let z: Float }
            var zKeys: [ZKey] = [ZKey(t: 0, z: 0)] // start in hole
            let halfCross: Float = 0.3 / Float(max(1, cfg.crossings)) // transition half-width

            for sw in swaps {
                let isA = sw.strandA == k
                let isB = sw.strandB == k
                guard isA || isB else { continue }
                let over = isA ? sw.aGoesOver : !sw.aGoesOver
                let zAtCross = over ? zOver : zUnder
                // Z must be fully separated BEFORE XY transition starts,
                // and return to zBase AFTER XY transition ends.
                let preT = max(0.005, sw.t - halfCross * 2)
                let postT = min(0.995, sw.t + halfCross * 2)
                zKeys.append(ZKey(t: preT, z: zAtCross))
                zKeys.append(ZKey(t: sw.t, z: zAtCross))
                zKeys.append(ZKey(t: postT, z: zBase))
            }
            zKeys.append(ZKey(t: 1, z: 0)) // end in hole
            zKeys.sort { $0.t < $1.t }

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)

                // Interpolate slot position via keyframes with smoothstep
                var segIdx = 0
                for j in 1..<keyframes.count - 1 {
                    if keyframes[j].t <= t { segIdx = j }
                }
                let kf0 = keyframes[segIdx]
                let kf1 = keyframes[min(segIdx + 1, keyframes.count - 1)]
                let localT: Float
                if kf1.t > kf0.t {
                    localT = min(1, max(0, (t - kf0.t) / (kf1.t - kf0.t)))
                } else {
                    localT = 0
                }
                let smooth = localT * localT * (3 - 2 * localT)
                let slotOffset = slotX[kf0.slot] * (1 - smooth) + slotX[kf1.slot] * smooth

                let baseXY = cfg.center - dir * halfLen + dir * cfg.length * t
                let pos = baseXY + perp * slotOffset

                // Interpolate Z from explicit keyframes
                var zSeg = 0
                for j in 1..<zKeys.count {
                    if zKeys[j].t > t { break }
                    zSeg = j
                }
                let z0 = zKeys[zSeg]
                let z1 = zKeys[min(zSeg + 1, zKeys.count - 1)]
                let zt: Float = z1.t > z0.t ? min(1, max(0, (t - z0.t) / (z1.t - z0.t))) : 0
                let zSmooth = zt * zt * (3 - 2 * zt)
                let z = z0.z + (z1.z - z0.z) * zSmooth

                particles.append(.init(xPosition: pos.x, yPosition: pos.y, zPosition: max(0, z)))
            }

            // Snap endpoints to hole positions
            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)

            allParticles.append(particles)
        }
    }

    // MARK: - Central knot builder (loops through center, like user's manual example)

    private static func buildCentralKnotParticles(
        cfg: CentralKnotConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.ropeCount
        let outerCount = n * 4          // 12 outer holes for 3 ropes
        let innerCount = max(4, n * 2)  // 6 inner holes for 3 ropes

        // Place concentric ring holes: center + inner + outer
        holes.append(.init(xPosition: cfg.center.x, yPosition: cfg.center.y))

        for i in 0..<innerCount {
            let angle = 2 * Float.pi * Float(i) / Float(innerCount)
            holes.append(.init(xPosition: cfg.center.x + cfg.innerRadius * cos(angle),
                               yPosition: cfg.center.y + cfg.innerRadius * sin(angle)))
        }

        var outerHoleIndices: [Int] = []
        for i in 0..<outerCount {
            let angle = 2 * Float.pi * Float(i) / Float(outerCount)
            outerHoleIndices.append(holes.count)
            holes.append(.init(xPosition: cfg.center.x + cfg.outerRadius * cos(angle),
                               yPosition: cfg.center.y + cfg.outerRadius * sin(angle)))
        }

        // Each rope goes from one side of the ring to the OPPOSITE side, passing through center.
        // This guarantees ropes cross each other.
        // Rope k: start at angle (k * 360/N), end at angle (k * 360/N + 180 + small offset)
        for k in 0..<n {
            let startOuter = k * (outerCount / n)
            let endOuter = (startOuter + outerCount / 2 + 1) % outerCount // opposite + 1 slot offset

            let startIdx = outerHoleIndices[startOuter]
            let endIdx = outerHoleIndices[endOuter]

            let startPos = SIMD2<Float>(holes[startIdx].xPosition, holes[startIdx].yPosition)
            let endPos = SIMD2<Float>(holes[endIdx].xPosition, holes[endIdx].yPosition)

            // Control point: offset perpendicular to the start-end line, pulling through center
            let mid = (startPos + endPos) / 2
            let dir = simd_normalize(endPos - startPos)
            let perp = SIMD2<Float>(-dir.y, dir.x)
            let controlPt = mid + perp * cfg.loopDepth * (k % 2 == 0 ? 1 : -1)

            ropes.append(.init(startHole: startIdx, endHole: endIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let u = 1 - t

                // Quadratic Bézier for XY
                let xy = u * u * startPos + 2 * u * t * controlPt + t * t * endPos

                // Z: each rope at a distinct layer across the FULL path (not just center).
                let zStep = ropeRadius * 1.5
                var z = zBase + zStep * Float(k) + cfg.zRadius * 0.3

                // Gentle fade only very close to endpoints (for hole entry)
                let env = smoothEnvelope(t: t, convergeFrac: 0.03)
                z = z * env

                particles.append(.init(xPosition: xy.x, yPosition: xy.y, zPosition: max(0, z)))
            }

            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)

            allParticles.append(particles)
        }
    }

    // MARK: - Vortex builder (radial convergence knot)

    private static func buildVortexParticles(
        cfg: VortexConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let n = cfg.ropeCount

        for k in 0..<n {
            // Evenly spaced directions radiating from center
            let baseAngle = 2 * Float.pi * Float(k) / Float(n)
            let dir = SIMD2<Float>(cos(baseAngle), sin(baseAngle))

            let startPos = cfg.center - dir * cfg.armLength
            let endPos = cfg.center + dir * cfg.armLength

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            let phase = 2 * Float.pi * Float(k) / Float(n)

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let baseXY = startPos + (endPos - startPos) * t

                // Smoothstep envelope: wide zone around center
                let halfZone = cfg.knotRadius
                let distFromCenter = abs(t - 0.5) * 2 // normalized: 0 at center, 1 at endpoints
                let normalizedDist = distFromCenter * cfg.armLength
                let env: Float
                if normalizedDist >= halfZone {
                    env = 0
                } else {
                    let s = normalizedDist / halfZone
                    env = 1 - s * s * (3 - 2 * s) // inverse smoothstep
                }

                // Helical orbit around the straight-line path
                let angle = phase + 2 * Float.pi * cfg.turns * t
                // Perpendicular to rope's own direction for orbit
                let perp = SIMD2<Float>(-dir.y, dir.x)
                let orbitXY = perp * cfg.orbitRadius * cos(angle) * env
                let z = max(0, zBase + cfg.zRadius * sin(angle) * env)

                particles.append(.init(xPosition: baseXY.x + orbitXY.x,
                                       yPosition: baseXY.y + orbitXY.y,
                                       zPosition: z))
            }

            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)

            allParticles.append(particles)
        }
    }

    // MARK: - Weave builder (lattice over/under pattern)

    private static func buildWeaveParticles(
        cfg: WeaveConfig, P: Int, zBase: Float,
        ropeRadius: Float, colorIdx: inout Int,
        holes: inout [LevelDefinition.Vec2],
        ropes: inout [LevelDefinition.Rope],
        allParticles: inout [[LevelDefinition.Vec2]]
    ) {
        let halfW = cfg.width / 2
        let halfH = cfg.height / 2
        let bumpWidth: Float = 0.06 // width of Z bump at each crossing

        // Precompute crossing positions
        let colXs = (0..<cfg.vCount).map { c in
            cfg.center.x - halfW + cfg.width * Float(c) / Float(max(1, cfg.vCount - 1))
        }
        let rowYs = (0..<cfg.hCount).map { r in
            cfg.center.y - halfH + cfg.height * Float(r) / Float(max(1, cfg.hCount - 1))
        }

        // Horizontal ropes
        for row in 0..<cfg.hCount {
            let y = rowYs[row]
            let startPos = SIMD2<Float>(cfg.center.x - halfW, y)
            let endPos = SIMD2<Float>(cfg.center.x + halfW, y)

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let x = startPos.x + (endPos.x - startPos.x) * t
                let env = smoothEnvelope(t: t, convergeFrac: 0.08)

                // Sum Z bumps at each vertical crossing
                var zBump: Float = 0
                for (col, colX) in colXs.enumerated() {
                    let dx = x - colX
                    let bump = exp(-(dx * dx) / (2 * bumpWidth * bumpWidth))
                    let sign: Float = (row + col) % 2 == 0 ? 1 : -1 // checkerboard over/under
                    zBump += sign * cfg.zRadius * bump
                }

                let z = max(0, zBase + zBump * env)
                particles.append(.init(xPosition: x, yPosition: y, zPosition: z))
            }
            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)
            allParticles.append(particles)
        }

        // Vertical ropes
        for col in 0..<cfg.vCount {
            let x = colXs[col]
            let startPos = SIMD2<Float>(x, cfg.center.y + halfH)
            let endPos = SIMD2<Float>(x, cfg.center.y - halfH)

            let startHoleIdx = holes.count
            holes.append(.init(xPosition: startPos.x, yPosition: startPos.y))
            let endHoleIdx = holes.count
            holes.append(.init(xPosition: endPos.x, yPosition: endPos.y))

            ropes.append(.init(startHole: startHoleIdx, endHole: endHoleIdx,
                               color: colors[colorIdx % colors.count], radius: ropeRadius))
            colorIdx += 1

            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let y = startPos.y + (endPos.y - startPos.y) * t
                let env = smoothEnvelope(t: t, convergeFrac: 0.08)

                // Sum Z bumps at each horizontal crossing (opposite parity)
                var zBump: Float = 0
                for (row, rowY) in rowYs.enumerated() {
                    let dy = y - rowY
                    let bump = exp(-(dy * dy) / (2 * bumpWidth * bumpWidth))
                    let sign: Float = (row + col) % 2 == 0 ? -1 : 1 // opposite to horizontal
                    zBump += sign * cfg.zRadius * bump
                }

                let z = max(0, zBase + zBump * env)
                particles.append(.init(xPosition: x, yPosition: y, zPosition: z))
            }
            particles[0] = .init(xPosition: startPos.x, yPosition: startPos.y, zPosition: 0)
            particles[P - 1] = .init(xPosition: endPos.x, yPosition: endPos.y, zPosition: 0)
            allParticles.append(particles)
        }
    }

    // MARK: - Helpers

    private static func smoothEnvelope(t: Float, convergeFrac: Float) -> Float {
        if t < convergeFrac {
            let s = t / convergeFrac
            return s * s * (3 - 2 * s)
        } else if t > 1 - convergeFrac {
            let s = (1 - t) / convergeFrac
            return s * s * (3 - 2 * s)
        }
        return 1.0
    }

    // MARK: - Structure showcase levels (3100+)

    /// Router for structured showcase levels.
    static func generateStructureShowcase(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let P = particleCount
        switch levelId {
        case 3100: return showcase3100(P)  // одна большая 3-прядная косичка
        case 3101: return showcase3101(P)  // DNA двойная спираль
        case 3102: return showcase3102(P)  // X-крест: две косички под углом
        case 3103: return showcase3103(P)  // звезда: 3 косички по 120°
        case 3104: return showcase3104(P)  // толстая 5-прядная косичка
        case 3105: return showcase3105(P)  // решётка: 2h + 2v braids
        case 3106: return showcase3106(P)  // веер: 5 косичек дугой
        case 3107: return showcase3107(P)  // каскад: 3 коротких косички вертикально
        case 3108: return showcase3108(P)  // диагональный крест из braids
        case 3109: return showcase3109(P)  // вихрь: 4 helix вокруг центра
        case 3110: return showcase3110(P)  // 3 параллельных одинаковых braid
        case 3111: return showcase3111(P)  // центральный узел (как ручной пример на 19 дырках)
        case 3112: return showcase3112(P)  // центральный узел + braid
        case 3113: return showcase3113(P)  // 4-лепестковый узел
        case 3114: return showcase3114(P)  // braid + helix рядом
        case 3115: return showcase3115(P)  // ручной пример: 3 верёвки через центр на 19 дырках
        case 3116: return showcase3116(P)  // идеальная длинная косичка (ручной эталон)
        case 3117: return showcase3117(P)  // X из двух косичек
        default:   return showcase3100(P)
        }
    }

    // ── 3100: Одна большая 3-прядная классическая косичка ──
    private static func showcase3100(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3100, particleCount: P, structures: [
            .braid(BraidConfig(center: .zero, axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 8, width: 0.56, zRadius: 0.07)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    // ── 3101: DNA — тугая двойная спираль ──
    private static func showcase3101(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3101, particleCount: P, structures: [
            .helix(HelixConfig(center: .zero, axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 2, turns: 5, helixRadius: 0.12, zRadius: 0.06)),
        ], extraHoleRings: [(0.30, 8), (0.60, 14)])
    }

    // ── 3102: X-крест — две 3-прядных косички, разнесены чтобы не пересекаться ──
    private static func showcase3102(_ P: Int) -> LevelDefinition {
        let a: Float = Float.pi / 6
        return buildStructuredLevel(levelId: 3102, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.45, 0), axis: SIMD2(sin(a), cos(a)), length: 1.0,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.07)),
            .braid(BraidConfig(center: SIMD2(0.45, 0), axis: SIMD2(-sin(a), cos(a)), length: 1.0,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.07)),
        ], extraHoleRings: [(0.35, 8), (0.70, 14)])
    }

    // ── 3103: Звезда — 3 двойных helix, разнесены от центра ──
    private static func showcase3103(_ P: Int) -> LevelDefinition {
        let a120 = 2 * Float.pi / 3
        let r: Float = 0.55
        return buildStructuredLevel(levelId: 3103, particleCount: P, structures: [
            .helix(HelixConfig(center: SIMD2(0, r), axis: SIMD2(0, 1), length: 0.50,
                               strandCount: 2, turns: 3, helixRadius: 0.06, zRadius: 0.06)),
            .helix(HelixConfig(center: SIMD2(cos(Float.pi/2 + a120) * r, sin(Float.pi/2 + a120) * r),
                               axis: SIMD2(cos(Float.pi/2 + a120), sin(Float.pi/2 + a120)), length: 0.50,
                               strandCount: 2, turns: 3, helixRadius: 0.06, zRadius: 0.06)),
            .helix(HelixConfig(center: SIMD2(cos(Float.pi/2 + 2*a120) * r, sin(Float.pi/2 + 2*a120) * r),
                               axis: SIMD2(cos(Float.pi/2 + 2*a120), sin(Float.pi/2 + 2*a120)), length: 0.50,
                               strandCount: 2, turns: 3, helixRadius: 0.06, zRadius: 0.06)),
        ], extraHoleRings: [(0.30, 8), (0.60, 14)])
    }

    // ── 3104: 3-прядная косичка (5-strand нужен multi-layer Z, пока 3) ──
    private static func showcase3104(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3104, particleCount: P, structures: [
            .braid(BraidConfig(center: .zero, axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 10, width: 0.56, zRadius: 0.07)),
        ], extraHoleRings: [(0.20, 6), (0.45, 10), (0.70, 14)])
    }

    // ── 3105: Решётка — 2h + 2v braids, разнесены шире ──
    private static func showcase3105(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3105, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(0, -0.55), axis: SIMD2(1, 0), length: 0.6,
                               strandCount: 2, crossings: 4, width: 0.15, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0, 0.55), axis: SIMD2(1, 0), length: 0.6,
                               strandCount: 2, crossings: 4, width: 0.15, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(-0.55, 0), axis: SIMD2(0, 1), length: 0.6,
                               strandCount: 2, crossings: 4, width: 0.15, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0.55, 0), axis: SIMD2(0, 1), length: 0.6,
                               strandCount: 2, crossings: 4, width: 0.15, zRadius: 0.06)),
        ], extraHoleRings: [(0.40, 8), (0.75, 14)])
    }

    // ── 3106: Веер — 5 двойных helix дугой от 30° до 150° ──
    private static func showcase3106(_ P: Int) -> LevelDefinition {
        var s: [StructurePrimitive] = []
        for i in 0..<5 {
            let angle = Float.pi * (0.17 + 0.66 * Float(i) / 4)
            s.append(.helix(HelixConfig(
                center: .zero, axis: SIMD2(cos(angle), sin(angle)), length: 0.90,
                strandCount: 2, turns: Float(2 + i % 3), helixRadius: 0.07, zRadius: 0.05)))
        }
        return buildStructuredLevel(levelId: 3106, particleCount: P,
                                    structures: s, extraHoleRings: [(0.25, 6), (0.50, 12)])
    }

    // ── 3107: Каскад — 3 коротких 3-прядных косички, расположенных сверху вниз ──
    private static func showcase3107(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3107, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.35, 0.32), axis: SIMD2(0, 1), length: 0.45,
                               strandCount: 3, crossings: 4, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0.10, 0), axis: SIMD2(0, 1), length: 0.45,
                               strandCount: 3, crossings: 4, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(-0.35, -0.32), axis: SIMD2(0, 1), length: 0.45,
                               strandCount: 3, crossings: 4, width: 0.30, zRadius: 0.06)),
        ], extraHoleRings: [(0.30, 8), (0.60, 14)])
    }

    // ── 3108: Диагональный крест — две braid, разнесены ──
    private static func showcase3108(_ P: Int) -> LevelDefinition {
        let d = SIMD2<Float>(1, 1) / sqrt(2.0)
        let d2 = SIMD2<Float>(1, -1) / sqrt(2.0)
        return buildStructuredLevel(levelId: 3108, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.40, 0), axis: d, length: 0.8,
                               strandCount: 3, crossings: 5, width: 0.22, zRadius: 0.07)),
            .braid(BraidConfig(center: SIMD2(0.40, 0), axis: d2, length: 0.8,
                               strandCount: 3, crossings: 5, width: 0.22, zRadius: 0.07)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    // ── 3109: Вихрь — 4 тройных helix радиально из центра ──
    private static func showcase3109(_ P: Int) -> LevelDefinition {
        var s: [StructurePrimitive] = []
        for i in 0..<4 {
            let angle = Float.pi / 4 + Float.pi / 2 * Float(i) // 45°, 135°, 225°, 315°
            s.append(.helix(HelixConfig(
                center: .zero, axis: SIMD2(cos(angle), sin(angle)), length: 0.85,
                strandCount: 3, turns: 2, helixRadius: 0.08, zRadius: 0.06)))
        }
        return buildStructuredLevel(levelId: 3109, particleCount: P,
                                    structures: s, extraHoleRings: [(0.25, 6), (0.50, 12)])
    }

    // ── 3110: Три параллельных одинаковых 3-прядных braid ──
    private static func showcase3110(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3110, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.40, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0.40, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    // ── 3111: Центральный узел — 3 петли через центр на 19 дырках (как ручной пример) ──
    private static func showcase3111(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3111, particleCount: P, structures: [
            .centralKnot(CentralKnotConfig(center: .zero,
                                           outerRadius: 0.62, innerRadius: 0.35,
                                           ropeCount: 3, loopDepth: 0.55, zRadius: 0.08)),
        ], extraHoleRings: [])  // centralKnot creates its own holes
    }

    // ── 3112: Центральный узел + вертикальная braid сбоку ──
    private static func showcase3112(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3112, particleCount: P, structures: [
            .centralKnot(CentralKnotConfig(center: SIMD2(-0.25, 0),
                                           outerRadius: 0.45, innerRadius: 0.25,
                                           ropeCount: 3, loopDepth: 0.40, zRadius: 0.07)),
            .braid(BraidConfig(center: SIMD2(0.45, 0), axis: SIMD2(0, 1), length: 1.0,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
        ], extraHoleRings: [])
    }

    // ── 3113: 4-лепестковый узел — 4 петли через центр ──
    private static func showcase3113(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3113, particleCount: P, structures: [
            .centralKnot(CentralKnotConfig(center: .zero,
                                           outerRadius: 0.62, innerRadius: 0.35,
                                           ropeCount: 4, loopDepth: 0.55, zRadius: 0.08)),
        ], extraHoleRings: [])
    }

    // ── 3114: Braid + helix рядом для сравнения ──
    private static func showcase3114(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3114, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.35, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.35, zRadius: 0.06)),
            .helix(HelixConfig(center: SIMD2(0.35, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, turns: 3, helixRadius: 0.12, zRadius: 0.06)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    // ── 3115: Точная копия ручного примера — 3 верёвки через центр на 19 дырках ──
    private static func showcase3115(_ P: Int) -> LevelDefinition {
        // Exact hole layout from user's manual example
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: 0.0000, yPosition: 0.0000),       // 0: center
            .init(xPosition: 0.3461, yPosition: 0.0523),       // 1: inner ring
            .init(xPosition: 0.1277, yPosition: 0.3259),       // 2
            .init(xPosition: -0.2183, yPosition: 0.2736),      // 3
            .init(xPosition: -0.3461, yPosition: -0.0523),     // 4
            .init(xPosition: -0.1277, yPosition: -0.3259),     // 5
            .init(xPosition: 0.2183, yPosition: -0.2736),      // 6
            .init(xPosition: 0.6050, yPosition: 0.1872),       // 7: outer ring
            .init(xPosition: 0.4304, yPosition: 0.4646),       // 8
            .init(xPosition: 0.1404, yPosition: 0.6176),       // 9
            .init(xPosition: -0.1872, yPosition: 0.6050),      // 10
            .init(xPosition: -0.4646, yPosition: 0.4304),      // 11
            .init(xPosition: -0.6176, yPosition: 0.1404),      // 12
            .init(xPosition: -0.6050, yPosition: -0.1872),     // 13
            .init(xPosition: -0.4304, yPosition: -0.4646),     // 14
            .init(xPosition: -0.1404, yPosition: -0.6176),     // 15
            .init(xPosition: 0.1872, yPosition: -0.6050),      // 16
            .init(xPosition: 0.4646, yPosition: -0.4304),      // 17
            .init(xPosition: 0.6176, yPosition: -0.1404),      // 18
        ]

        // Waypoints sampled from user's geometry dump (every ~10th particle)
        // Band 0: pin 12→13
        let wp0: [(Float, Float, Float)] = [
            (-0.6176, 0.1404, 0), (-0.5285, 0.1221, 0.038), (-0.4295, 0.1018, 0.038),
            (-0.3306, 0.0815, 0.038), (-0.2318, 0.0611, 0.038), (-0.1813, 0.0314, 0.049),
            (-0.1649, 0.0036, 0.073), (-0.1729, -0.0241, 0.103), (-0.2007, -0.0526, 0.117),
            (-0.2591, -0.0767, 0.110), (-0.3190, -0.0958, 0.097), (-0.3987, -0.1213, 0.081),
            (-0.4785, -0.1468, 0.064), (-0.5582, -0.1722, 0.047), (-0.6050, -0.1872, 0),
        ]
        // Band 1: pin 17→8
        let wp1: [(Float, Float, Float)] = [
            (0.4646, -0.4304, 0), (0.3997, -0.2871, 0.040), (0.3526, -0.1824, 0.044),
            (0.3021, -0.1308, 0.096), (0.2278, -0.1216, 0.111), (0.1267, -0.1075, 0.092),
            (0.0123, -0.0915, 0.071), (-0.1021, -0.0754, 0.050), (-0.1783, -0.0643, 0.038),
            (-0.2376, -0.0205, 0.056), (-0.2279, 0.0306, 0.111), (-0.1595, 0.0659, 0.120),
            (-0.0456, 0.0950, 0.103), (0.0682, 0.1244, 0.087), (0.1820, 0.1537, 0.071),
            (0.2713, 0.1756, 0.072), (0.3162, 0.2108, 0.104), (0.3484, 0.2831, 0.105),
            (0.3956, 0.3874, 0.065), (0.4304, 0.4646, 0),
        ]
        // Band 2: pin 7→18
        let wp2: [(Float, Float, Float)] = [
            (0.6050, 0.1872, 0), (0.5054, 0.2101, 0.039), (0.4056, 0.2331, 0.038),
            (0.3307, 0.2510, 0.038), (0.2834, 0.2601, 0.054), (0.2499, 0.2499, 0.092),
            (0.2364, 0.2151, 0.128), (0.2358, 0.1908, 0.138), (0.2387, 0.1388, 0.135),
            (0.2405, 0.0898, 0.115), (0.2432, 0.0162, 0.086), (0.2459, -0.0574, 0.057),
            (0.2476, -0.1067, 0.038), (0.2523, -0.1588, 0.043), (0.2724, -0.1979, 0.072),
            (0.3117, -0.2121, 0.105), (0.3630, -0.2052, 0.117), (0.4136, -0.1924, 0.104),
            (0.4884, -0.1734, 0.079), (0.5632, -0.1543, 0.055), (0.6176, -0.1404, 0),
        ]

        let ropes: [LevelDefinition.Rope] = [
            .init(startHole: 12, endHole: 13, color: colors[0], radius: 0.038),
            .init(startHole: 17, endHole: 8, color: colors[1], radius: 0.038),
            .init(startHole: 7, endHole: 18, color: colors[2], radius: 0.038),
        ]

        let allWaypoints = [wp0, wp1, wp2]
        var allParticles: [[LevelDefinition.Vec2]] = []

        for wp in allWaypoints {
            // Compute cumulative arc length along waypoints
            var cumLen: [Float] = [0]
            for i in 1..<wp.count {
                let dx = wp[i].0 - wp[i-1].0
                let dy = wp[i].1 - wp[i-1].1
                let dz = wp[i].2 - wp[i-1].2
                cumLen.append(cumLen[i-1] + sqrt(dx*dx + dy*dy + dz*dz))
            }
            let totalLen = cumLen.last!

            // Resample to P evenly-spaced points along the arc
            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let targetDist = totalLen * Float(i) / Float(P - 1)
                // Find segment
                var seg = 0
                for j in 1..<cumLen.count {
                    if cumLen[j] >= targetDist { seg = j - 1; break }
                    seg = j - 1
                }
                let segStart = cumLen[seg]
                let segEnd = cumLen[min(seg + 1, cumLen.count - 1)]
                let localT = segEnd > segStart ? (targetDist - segStart) / (segEnd - segStart) : 0
                let a = wp[seg]
                let b = wp[min(seg + 1, wp.count - 1)]
                let x = a.0 + (b.0 - a.0) * localT
                let y = a.1 + (b.1 - a.1) * localT
                let z = a.2 + (b.2 - a.2) * localT
                particles.append(.init(xPosition: x, yPosition: y, zPosition: z))
            }
            // Snap endpoints
            particles[0] = .init(xPosition: wp[0].0, yPosition: wp[0].1, zPosition: 0)
            particles[P - 1] = .init(xPosition: wp[wp.count-1].0, yPosition: wp[wp.count-1].1, zPosition: 0)
            allParticles.append(particles)
        }

        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }

        return LevelDefinition(
            mode: nil, id: 3115, holeRadius: 0.08, particlesPerRope: P,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil, rails: nil, carts: nil, stations: nil,
            ropeParticles: allParticles
        )
    }

    // ── 3116: Идеальная длинная 3-прядная косичка (ручной эталон, board 0.56×4.4) ──
    private static func showcase3116(_ P: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.2800, yPosition: 2.2000),   // 0
            .init(xPosition: 0.0000, yPosition: 2.2000),    // 1
            .init(xPosition: 0.2800, yPosition: 2.2000),    // 2
            .init(xPosition: -0.2800, yPosition: -2.2000),  // 3
            .init(xPosition: 0.0000, yPosition: -2.2000),   // 4
            .init(xPosition: 0.2800, yPosition: -2.2000),   // 5
        ]
        // Band 0: pin 0→5 (top-left → bottom-right), every ~6th particle
        let wp0: [(Float,Float,Float)] = [
            (-0.280, 2.200, 0), (-0.216, 2.001, 0.051), (-0.110, 1.672, 0.095),
            (-0.031, 1.434, 0.124), (0.025, 1.276, 0.077), (0.083, 1.112, 0.079),
            (0.065, 0.945, 0.129), (0.024, 0.778, 0.150), (-0.002, 0.623, 0.075),
            (-0.051, 0.459, 0.046), (-0.067, 0.292, 0.100), (-0.016, 0.128, 0.125),
            (0.063, -0.022, 0.080), (0.086, -0.113, 0.099), (0.060, -0.281, 0.151),
            (0.004, -0.441, 0.122), (0.000, -0.503, 0.069), (-0.064, -0.752, 0.098),
            (-0.010, -0.912, 0.122), (0.065, -1.054, 0.052), (0.074, -1.219, 0.116),
            (0.016, -1.379, 0.118), (0.000, -1.438, 0.061), (-0.062, -1.596, 0.062),
            (-0.050, -1.762, 0.127), (0.030, -1.897, 0.042), (0.145, -2.031, 0.038),
            (0.280, -2.200, 0),
        ]
        // Band 1: pin 1→4 (top-center → bottom-center)
        let wp1: [(Float,Float,Float)] = [
            (0.000, 2.200, 0), (0.029, 2.005, 0.039), (0.064, 1.759, 0.038),
            (0.090, 1.592, 0.051), (0.080, 1.431, 0.106), (0.057, 1.269, 0.152),
            (0.009, 1.113, 0.110), (0.003, 1.034, 0.061), (-0.053, 0.875, 0.038),
            (-0.072, 0.712, 0.091), (-0.074, 0.637, 0.116), (-0.008, 0.481, 0.115),
            (0.068, 0.336, 0.067), (0.087, 0.247, 0.090), (0.063, 0.088, 0.157),
            (0.006, -0.070, 0.132), (0.009, -0.132, 0.081), (-0.045, -0.286, 0.038),
            (-0.075, -0.446, 0.101), (-0.058, -0.535, 0.127), (0.029, -0.672, 0.069),
            (0.086, -0.831, 0.090), (0.078, -0.904, 0.121), (0.019, -1.065, 0.126),
            (-0.004, -1.136, 0.063), (-0.067, -1.294, 0.064), (-0.054, -1.459, 0.126),
            (-0.011, -1.528, 0.117), (0.070, -1.668, 0.048), (0.081, -1.762, 0.075),
            (0.067, -1.933, 0.120), (0.026, -2.093, 0.049), (0.000, -2.200, 0),
        ]
        // Band 2: pin 2→3 (top-right → bottom-left)
        let wp2: [(Float,Float,Float)] = [
            (0.280, 2.200, 0), (0.198, 2.005, 0.058), (0.100, 1.775, 0.108),
            (0.029, 1.616, 0.098), (0.005, 1.526, 0.069), (-0.041, 1.360, 0.039),
            (-0.061, 1.190, 0.078), (-0.059, 1.019, 0.116), (0.008, 0.860, 0.088),
            (0.073, 0.699, 0.077), (0.076, 0.623, 0.099), (0.066, 0.454, 0.150),
            (0.012, 0.293, 0.121), (0.007, 0.211, 0.068), (-0.049, 0.048, 0.050),
            (-0.070, -0.119, 0.102), (-0.050, -0.194, 0.121), (0.031, -0.342, 0.072),
            (0.077, -0.505, 0.101), (0.070, -0.597, 0.133), (0.015, -0.758, 0.126),
            (0.003, -0.816, 0.068), (-0.062, -0.973, 0.060), (-0.066, -1.061, 0.100),
            (-0.012, -1.220, 0.122), (0.064, -1.357, 0.047), (0.083, -1.432, 0.071),
            (0.078, -1.519, 0.111), (0.019, -1.675, 0.120), (-0.004, -1.727, 0.057),
            (-0.054, -1.800, 0.038), (-0.140, -1.953, 0.039), (-0.280, -2.200, 0),
        ]

        let ropes: [LevelDefinition.Rope] = [
            .init(startHole: 0, endHole: 5, color: colors[0], radius: 0.038),
            .init(startHole: 1, endHole: 4, color: colors[1], radius: 0.038),
            .init(startHole: 2, endHole: 3, color: colors[2], radius: 0.038),
        ]

        let allWaypoints = [wp0, wp1, wp2]
        var allParticles: [[LevelDefinition.Vec2]] = []
        for wp in allWaypoints {
            var cumLen: [Float] = [0]
            for i in 1..<wp.count {
                let dx = wp[i].0 - wp[i-1].0, dy = wp[i].1 - wp[i-1].1, dz = wp[i].2 - wp[i-1].2
                cumLen.append(cumLen[i-1] + sqrt(dx*dx + dy*dy + dz*dz))
            }
            let totalLen = cumLen.last!
            var particles: [LevelDefinition.Vec2] = []
            for i in 0..<P {
                let targetDist = totalLen * Float(i) / Float(P - 1)
                var seg = 0
                for j in 1..<cumLen.count { if cumLen[j] >= targetDist { seg = j - 1; break }; seg = j - 1 }
                let s0 = cumLen[seg], s1 = cumLen[min(seg+1, cumLen.count-1)]
                let lt = s1 > s0 ? (targetDist - s0) / (s1 - s0) : 0
                let a = wp[seg], b = wp[min(seg+1, wp.count-1)]
                particles.append(.init(xPosition: a.0+(b.0-a.0)*lt, yPosition: a.1+(b.1-a.1)*lt, zPosition: a.2+(b.2-a.2)*lt))
            }
            particles[0] = .init(xPosition: wp[0].0, yPosition: wp[0].1, zPosition: 0)
            particles[P-1] = .init(xPosition: wp[wp.count-1].0, yPosition: wp[wp.count-1].1, zPosition: 0)
            allParticles.append(particles)
        }

        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }
        return LevelDefinition(
            mode: nil, id: 3116, holeRadius: 0.08, particlesPerRope: P,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil, rails: nil, carts: nil, stations: nil,
            ropeParticles: allParticles
        )
    }

    // ── 3117: Две косички петлями через центр на 10 дырках ──
    private static func showcase3117(_ P: Int) -> LevelDefinition {
        // 10 holes on a circle (decagon), radius 0.75
        let holeCount = 10
        let holeR: Float = 0.75
        var holes: [LevelDefinition.Vec2] = []
        for i in 0..<holeCount {
            let angle = Float.pi / 2 + 2 * Float.pi * Float(i) / Float(holeCount)
            holes.append(.init(xPosition: holeR * cos(angle), yPosition: holeR * sin(angle)))
        }

        // Centerline waypoints from user's dump (2 ropes looping through center)
        // Rope A: hole 4→5, rope B: hole 0→9
        let clA: [(Float,Float,Float)] = [
            (0.441, 0.607, 0), (0.358, 0.470, 0.047), (0.268, 0.322, 0.067),
            (0.163, 0.150, 0.089), (0.073, 0.002, 0.108), (0.011, -0.107, 0.127),
            (-0.043, -0.158, 0.113), (-0.084, -0.162, 0.057), (-0.111, -0.092, 0.038),
            (-0.088, 0.049, 0.038), (-0.071, 0.182, 0.038), (-0.055, 0.315, 0.039),
            (-0.038, 0.448, 0.039), (-0.021, 0.580, 0.040), (0.000, 0.750, 0),
        ]
        let clB: [(Float,Float,Float)] = [
            (0.000, -0.750, 0), (-0.002, -0.600, 0.038), (-0.005, -0.451, 0.038),
            (-0.007, -0.331, 0.038), (-0.004, -0.150, 0.038), (-0.024, -0.095, 0.048),
            (-0.097, -0.092, 0.118), (-0.125, -0.120, 0.117), (-0.177, -0.203, 0.102),
            (-0.266, -0.339, 0.088), (-0.336, -0.446, 0.075), (-0.425, -0.582, 0.047),
            (-0.441, -0.607, 0),
        ]

        // Generate 3-strand braid along each centerline
        let strandCount = 3
        let helixR: Float = 0.04     // lateral helix radius
        let zR: Float = 0.04         // Z over/under amplitude
        let turns: Float = 4         // twists along the path
        let ropeRadius: Float = 0.038

        var ropes: [LevelDefinition.Rope] = []
        var allParticles: [[LevelDefinition.Vec2]] = []
        var colorIdx = 0

        for (cl, startHole, endHole) in [(clA, 4, 5), (clB, 0, 9)] {
            // Compute cumulative arc length of centerline
            var cumLen: [Float] = [0]
            for i in 1..<cl.count {
                let dx = cl[i].0-cl[i-1].0, dy = cl[i].1-cl[i-1].1, dz = cl[i].2-cl[i-1].2
                cumLen.append(cumLen[i-1] + sqrt(dx*dx+dy*dy+dz*dz))
            }
            let totalLen = cumLen.last!

            // For each strand
            for k in 0..<strandCount {
                let phase = 2 * Float.pi * Float(k) / Float(strandCount)

                // Holes for this strand: offset from the braid's start/end holes
                let sAngle = 2 * Float.pi * Float(startHole) / Float(holeCount) + Float.pi / 2
                let eAngle = 2 * Float.pi * Float(endHole) / Float(holeCount) + Float.pi / 2
                let sOff = SIMD2<Float>(cos(sAngle + Float.pi/2), sin(sAngle + Float.pi/2)) * helixR * cos(phase)
                let eOff = SIMD2<Float>(cos(eAngle + Float.pi/2), sin(eAngle + Float.pi/2)) * helixR * cos(2*Float.pi*turns + phase)

                let sHoleIdx = holes.count
                holes.append(.init(xPosition: holes[startHole].xPosition + sOff.x,
                                   yPosition: holes[startHole].yPosition + sOff.y))
                let eHoleIdx = holes.count
                holes.append(.init(xPosition: holes[endHole].xPosition + eOff.x,
                                   yPosition: holes[endHole].yPosition + eOff.y))

                ropes.append(.init(startHole: sHoleIdx, endHole: eHoleIdx,
                                   color: colors[colorIdx % colors.count], radius: ropeRadius))
                colorIdx += 1

                // Interpolate P particles along centerline, adding helix offset
                var particles: [LevelDefinition.Vec2] = []
                for i in 0..<P {
                    let t = Float(i) / Float(P - 1)
                    let dist = totalLen * t

                    // Find segment on centerline
                    var seg = 0
                    for j in 1..<cumLen.count {
                        if cumLen[j] >= dist { seg = j - 1; break }
                        seg = j - 1
                    }
                    let s0 = cumLen[seg], s1 = cumLen[min(seg+1, cumLen.count-1)]
                    let lt = s1 > s0 ? (dist - s0) / (s1 - s0) : 0
                    let a = cl[seg], b = cl[min(seg+1, cl.count-1)]
                    let cx = a.0 + (b.0 - a.0) * lt
                    let cy = a.1 + (b.1 - a.1) * lt
                    let cz = a.2 + (b.2 - a.2) * lt

                    // Tangent direction
                    let next = min(seg+1, cl.count-1), prev = max(seg-1, 0)
                    let tx = cl[next].0 - cl[prev].0
                    let ty = cl[next].1 - cl[prev].1
                    let tLen = sqrt(tx*tx + ty*ty)
                    let nx: Float, ny: Float // perpendicular in XY
                    if tLen > 0.001 { nx = -ty/tLen; ny = tx/tLen }
                    else { nx = 1; ny = 0 }

                    let angle = 2 * Float.pi * turns * t + phase
                    let env = smoothEnvelope(t: t, convergeFrac: 0.10)

                    let offX = nx * helixR * cos(angle) * env
                    let offY = ny * helixR * cos(angle) * env
                    let offZ = zR * sin(angle) * env

                    particles.append(.init(xPosition: cx + offX, yPosition: cy + offY,
                                           zPosition: max(0, cz + offZ)))
                }
                // Snap endpoints
                particles[0] = .init(xPosition: holes[sHoleIdx].xPosition,
                                     yPosition: holes[sHoleIdx].yPosition, zPosition: 0)
                particles[P-1] = .init(xPosition: holes[eHoleIdx].xPosition,
                                       yPosition: holes[eHoleIdx].yPosition, zPosition: 0)
                allParticles.append(particles)
            }
        }

        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }
        return LevelDefinition(
            mode: nil, id: 3117, holeRadius: 0.08, particlesPerRope: P,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil, rails: nil, carts: nil, stations: nil,
            ropeParticles: allParticles
        )
    }

    // MARK: - Braid mode levels

    static func generateBraidLevel(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let localId = levelId - 3000
        let strandCount: Int
        let swapCount: Int
        switch localId {
        case 1:  strandCount = 3; swapCount = 100
        case 2:  strandCount = 3; swapCount = 4
        case 3:  strandCount = 3; swapCount = 6
        case 4:  strandCount = 4; swapCount = 4
        case 5:  strandCount = 4; swapCount = 6
        case 6:  strandCount = 4; swapCount = 8
        case 7:  strandCount = 5; swapCount = 6
        case 8:  strandCount = 5; swapCount = 8
        case 9:  strandCount = 5; swapCount = 10
        case 10: strandCount = 6; swapCount = 10
        default:
            let sc = min(8, 3 + (localId - 1) / 3)
            let sw = min(16, 2 + localId)
            strandCount = sc; swapCount = sw
        }
        let halfHeight: Float = localId == 1 ? 2.2 : 0.55
        return buildBraidModeLevel(levelId: levelId, strandCount: strandCount, swapCount: swapCount, halfHeight: halfHeight, particleCount: particleCount)
    }

    /// Build a braid puzzle level:
    /// - N holes at top (anchors), N holes at bottom (moveable ends)
    /// - N strands connecting top[i] to bottom[i]
    /// - Target: a permutation achieved by `swapCount` adjacent swaps (classic braid)
    private static func buildBraidModeLevel(levelId: Int, strandCount: Int, swapCount: Int, halfHeight: Float = 0.55, particleCount: Int) -> LevelDefinition {
        let n = strandCount
        let spacing: Float = 0.28
        let totalWidth = Float(n - 1) * spacing
        let startX = -totalWidth / 2

        // Top holes: indices 0..<n, Bottom holes: indices n..<2n
        var holes: [LevelDefinition.Vec2] = []
        for i in 0..<n {
            let x = startX + Float(i) * spacing
            holes.append(.init(xPosition: x, yPosition: halfHeight))  // top
        }
        for i in 0..<n {
            let x = startX + Float(i) * spacing
            holes.append(.init(xPosition: x, yPosition: -halfHeight)) // bottom
        }

        // Ropes: strand i goes from top[i] to bottom[i]
        let ropes = (0..<n).map { i in
            LevelDefinition.Rope(startHole: i, endHole: n + i,
                                 color: colors[i % colors.count], radius: 0.038)
        }

        // Pin actions
        var actions: [LevelDefinition.Action] = []
        for i in 0..<n {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: i))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: n + i))
        }

        // Simulate braid swaps to compute target permutation
        // Classic braid: alternate swap(0,1), swap(n-2,n-1), swap(0,1), ...
        // For 3 strands: swap(0,1), swap(1,2), swap(0,1), swap(1,2), ...
        var slots = Array(0..<n) // slots[position] = strandIndex
        for step in 0..<swapCount {
            let swapPos: Int
            if n == 3 {
                swapPos = (step % 2 == 0) ? 0 : 1
            } else {
                // For N strands: cycle through swap positions
                swapPos = step % (n - 1)
            }
            slots.swapAt(swapPos, swapPos + 1)
        }

        // braidTargets[strandIndex] = target bottom hole index
        // slots[position] = strandIndex, so we need the inverse:
        // strandIndex -> which position it ended up in -> bottom hole = n + position
        var braidTargets = Array(repeating: 0, count: n)
        for pos in 0..<n {
            braidTargets[slots[pos]] = n + pos
        }

        // let particleCount = Int(Float(24) * (halfHeight / 0.55))
        return LevelDefinition(
            mode: "braid", id: levelId, holeRadius: 0.08, particlesPerRope: particleCount,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: nil, carts: nil, stations: nil,
            braidTargets: braidTargets, braidMinCrossings: max(1, swapCount / 2)
        )
    }

    // MARK: - 3D Board layouts

    enum BoardLayout: CaseIterable {
        case twoSides
        case bridge
        case staircase
        case platform
        case valley
    }

    struct BoardLayoutResult {
        let holes: [LevelDefinition.Vec2]
        let boards: [LevelDefinition.Board]
    }

    private static func pickBoardLayout(for levelId: Int) -> BoardLayout {
        let layouts = BoardLayout.allCases
        return layouts[(levelId / 3) % layouts.count]
    }

    // MARK: - Difficulty

    private struct Difficulty {
        let ropeCount: Int
        let totalDrags: Int
        let shortRopeCount: Int
        let swapPercent: Int  // 0-100: percentage of moves that are swaps vs drags
    }

    /// Cyclic swap percent: repeats every 15 levels
    /// 0,0,0,15,25,35,50,60,75,85,100,100,50,25,0
    private static let swapCycle: [Int] = [0, 0, 0, 15, 25, 35, 50, 60, 75, 85, 100, 100, 50, 25, 0]

    private static func difficulty(for levelId: Int) -> Difficulty {
        let swap = swapCycle[levelId % swapCycle.count]
        switch levelId {
        case 1...2:   return Difficulty(ropeCount: 3, totalDrags: 3, shortRopeCount: 0, swapPercent: swap)
        case 3...5:   return Difficulty(ropeCount: 4, totalDrags: 4, shortRopeCount: levelId >= 4 ? 2 : 0, swapPercent: swap)
        case 6...9:   return Difficulty(ropeCount: 5, totalDrags: 6, shortRopeCount: 3, swapPercent: swap)
        case 10...15: return Difficulty(ropeCount: 6, totalDrags: 8, shortRopeCount: 4, swapPercent: swap)
        case 16...25: return Difficulty(ropeCount: 7, totalDrags: 10, shortRopeCount: 5, swapPercent: swap)
        case 26...50: return Difficulty(ropeCount: 8, totalDrags: 12 + (levelId - 25) / 3, shortRopeCount: 6, swapPercent: swap)
        case 51...100: return Difficulty(ropeCount: min(12, 9 + (levelId - 50) / 15),
                                         totalDrags: min(35, 20 + (levelId - 50) / 5),
                                         shortRopeCount: min(8, 6 + (levelId - 50) / 20),
                                         swapPercent: swap)
        case 101...200: return Difficulty(ropeCount: min(14, 12 + (levelId - 100) / 30),
                                          totalDrags: min(50, 35 + (levelId - 100) / 5),
                                          shortRopeCount: min(10, 8 + (levelId - 100) / 25),
                                          swapPercent: swap)
        default:       return Difficulty(ropeCount: min(16, 14 + (levelId - 200) / 50),
                                         totalDrags: min(70, 50 + (levelId - 200) / 5),
                                         shortRopeCount: min(12, 10 + (levelId - 200) / 40),
                                         swapPercent: swap)
        }
    }

    private static func minHoles(for levelId: Int, ropeCount: Int) -> Int {
        let base = ropeCount * 3 + 6
        switch levelId {
        case 1...5:    return max(base, 12)
        case 6...15:   return max(base, 16)
        case 16...30:  return max(base, 22)
        case 31...60:  return max(base, 28)
        case 61...100: return max(base, 36)
        case 101...200: return max(base, 48)
        case 201...500: return max(base, 64)
        default:       return max(base, 80)
        }
    }

    // MARK: - Layout selection

    private static func pickLayout(for levelId: Int) -> HoleLayout {
        // For higher levels with many holes, prefer layouts that scale well
        // (circles, rings, spirals, honeycomb, clusters, stars)
        if levelId > 60 {
            let scalableLayouts: [HoleLayout] = [
                .circle, .rings, .spiral, .honeycomb, .star,
                .clusters, .clover, .ringWithSpokes, .hexagon,
                .centerPlusRing, .triangleInCircle, .grid,
            ]
            return scalableLayouts[levelId % scalableLayouts.count]
        }
        let layouts = HoleLayout.allCases
        return layouts[levelId % layouts.count]
    }

    // MARK: - Colors

    private static let colors: [LevelDefinition.Color] = [
        .init(redChannel: 0.95, greenChannel: 0.30, blueChannel: 0.05),
        .init(redChannel: 0.10, greenChannel: 0.35, blueChannel: 0.92),
        .init(redChannel: 0.90, greenChannel: 0.12, blueChannel: 0.25),
        .init(redChannel: 0.15, greenChannel: 0.75, blueChannel: 0.30),
        .init(redChannel: 0.92, greenChannel: 0.78, blueChannel: 0.05),
        .init(redChannel: 0.60, greenChannel: 0.10, blueChannel: 0.72),
        .init(redChannel: 0.05, greenChannel: 0.65, blueChannel: 0.72),
        .init(redChannel: 0.85, greenChannel: 0.15, blueChannel: 0.55),
        .init(redChannel: 0.20, greenChannel: 0.55, blueChannel: 0.90),
        .init(redChannel: 0.80, greenChannel: 0.50, blueChannel: 0.05),
    ]

    // MARK: - Rope pair selection

    private static func pickStructuredPairs(
        holes: [LevelDefinition.Vec2], count: Int, shortCount: Int, rng: inout SeededRNG
    ) -> [(Int, Int)] {
        guard holes.count >= 4 else { return [] }

        let cx = holes.map { $0.simd.x }.reduce(0, +) / Float(holes.count)
        let cy = holes.map { $0.simd.y }.reduce(0, +) / Float(holes.count)

        let sorted = holes.enumerated()
            .map { (idx: $0.offset, angle: atan2($0.element.simd.y - cy, $0.element.simd.x - cx)) }
            .sorted { $0.angle < $1.angle }

        let offset = rng.next(bound: sorted.count)
        let n = sorted.count
        let half = n / 2
        let quarter = max(1, n / 4)

        var pairs: [(Int, Int)] = []
        var usedHoles = Set<Int>()

        if shortCount > 0 {
            let sectorStart = offset % n
            let sectorLen = min(quarter + shortCount, n / 2)

            var sectorHoles: [Int] = []
            for k in 0..<sectorLen {
                sectorHoles.append(sorted[(sectorStart + k) % n].idx)
            }

            let minShortDist: Float = 0.3
            for i in 0..<sectorHoles.count {
                guard pairs.count < shortCount else { break }
                let a = sectorHoles[i]
                if usedHoles.contains(a) { continue }
                let shift = max(2, min(quarter / 2, sectorHoles.count / 3))
                for delta in stride(from: shift, to: sectorHoles.count, by: 1) {
                    let b = sectorHoles[(i + delta) % sectorHoles.count]
                    if b == a || usedHoles.contains(b) { continue }
                    if simd_length(holes[a].simd - holes[b].simd) < minShortDist { continue }
                    pairs.append((a, b))
                    usedHoles.insert(a)
                    usedHoles.insert(b)
                    break
                }
            }
        }

        for i in 0..<count {
            guard pairs.count < count else { break }
            let aIdx = (i * 2 + offset) % n
            let a = sorted[aIdx].idx
            if usedHoles.contains(a) { continue }

            let shift = (i % 2 == 0) ? 1 : -1
            let bIdx = (aIdx + half + shift + n) % n
            let b = sorted[bIdx].idx

            if b != a && !usedHoles.contains(b) {
                pairs.append((a, b))
                usedHoles.insert(a)
                usedHoles.insert(b)
            }
        }

        if pairs.count < count {
            for i in 0..<n {
                guard pairs.count < count else { break }
                let a = sorted[(i + offset) % n].idx
                if usedHoles.contains(a) { continue }
                for j in (i+1)..<n {
                    let b = sorted[(j + offset) % n].idx
                    if usedHoles.contains(b) { continue }
                    if simd_length(holes[a].simd - holes[b].simd) > 0.5 {
                        pairs.append((a, b))
                        usedHoles.insert(a)
                        usedHoles.insert(b)
                        break
                    }
                }
            }
        }

        return pairs
    }

    // MARK: - Tangle generation

    private static func buildActions(
        ropes: [LevelDefinition.Rope],
        holes: [LevelDefinition.Vec2],
        totalDrags: Int,
        shortCount: Int,
        swapPercent: Int = 0
    ) -> [LevelDefinition.Action] {
        var actions: [LevelDefinition.Action] = []
        let holeSimd = holes.map { $0.simd }

        var shortCenter: SIMD2<Float> = .zero
        if shortCount > 0 {
            var sum = SIMD2<Float>.zero
            var cnt: Float = 0
            for i in 0..<shortCount {
                sum += holeSimd[ropes[i].startHole]
                sum += holeSimd[ropes[i].endHole]
                cnt += 2
            }
            shortCenter = sum / cnt
        }
        let shortRadius: Float = {
            guard shortCount > 0 else { return 0 }
            var maxD: Float = 0
            for i in 0..<shortCount {
                maxD = max(maxD, simd_length(holeSimd[ropes[i].startHole] - shortCenter))
                maxD = max(maxD, simd_length(holeSimd[ropes[i].endHole] - shortCenter))
            }
            return maxD + 0.35
        }()

        for i in 0..<ropes.count {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: ropes[i].startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: ropes[i].endHole))
        }

        var endpoints = ropes.map { ($0.startHole, $0.endHole) }
        var usedHoles = Set(endpoints.flatMap { [$0.0, $0.1] })

        @discardableResult
        func tryDrag(ropeIdx: Int, endIdx: Int, targetRopeIdx: Int, unconstrained: Bool = false) -> Bool {
            let currentHole = endIdx == 0 ? endpoints[ropeIdx].0 : endpoints[ropeIdx].1
            let anchorHole = endIdx == 0 ? endpoints[ropeIdx].1 : endpoints[ropeIdx].0
            let anchorPos = holeSimd[anchorHole]
            let tS = holeSimd[endpoints[targetRopeIdx].0]
            let tE = holeSimd[endpoints[targetRopeIdx].1]
            let isShort = !unconstrained && ropeIdx < shortCount

            let otherUsed = Set(
                (0..<ropes.count).filter { $0 != ropeIdx }
                    .flatMap { [endpoints[$0].0, endpoints[$0].1] }
            )

            // Compute centroid of all current endpoints to spread ropes out
            var cx: Float = 0, cy: Float = 0
            for ep in endpoints { cx += holeSimd[ep.0].x + holeSimd[ep.1].x; cy += holeSimd[ep.0].y + holeSimd[ep.1].y }
            let epCount = Float(endpoints.count * 2)
            let centroid = SIMD2<Float>(cx / epCount, cy / epCount)

            var bestHole: Int?
            var bestScore: Float = -.greatestFiniteMagnitude

            for candidate in 0..<holes.count {
                if candidate == currentHole || candidate == anchorHole { continue }
                if otherUsed.contains(candidate) { continue }
                let cPos = holeSimd[candidate]
                if isShort && simd_length(cPos - shortCenter) > shortRadius { continue }
                if segmentsCross(anchorPos, cPos, tS, tE) {
                    // Prefer holes far from the centroid (spreads ropes across the board)
                    let distFromCentroid = simd_length(cPos - centroid)
                    // But also ensure crossing is meaningful (not too far from target rope)
                    let distFromTarget = simd_length(cPos - (tS + tE) * 0.5)
                    let score = distFromCentroid - distFromTarget * 0.3
                    if score > bestScore {
                        bestScore = score
                        bestHole = candidate
                    }
                }
            }

            guard let targetHole = bestHole else { return false }

            // Check that this drag does not reduce total crossing count
            let crossingsBefore = totalCrossingPairs(endpoints: endpoints, holes: holeSimd)
            var testEndpoints = endpoints
            if endIdx == 0 { testEndpoints[ropeIdx].0 = targetHole } else { testEndpoints[ropeIdx].1 = targetHole }
            let crossingsAfter = totalCrossingPairs(endpoints: testEndpoints, holes: holeSimd)
            if crossingsAfter < crossingsBefore { return false }

            usedHoles.remove(currentHole)
            usedHoles.insert(targetHole)
            if endIdx == 0 {
                endpoints[ropeIdx].0 = targetHole
            } else {
                endpoints[ropeIdx].1 = targetHole
            }
            actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: endIdx, holeIndex: targetHole))
            return true
        }

        // Swap: exchange two rope ends (A goes to B's hole, B goes to A's hole)
        func trySwap(ropeA: Int, endA: Int, ropeB: Int, endB: Int) -> Bool {
            guard ropeA != ropeB else { return false }
            let holeA = endA == 0 ? endpoints[ropeA].0 : endpoints[ropeA].1
            let holeB = endB == 0 ? endpoints[ropeB].0 : endpoints[ropeB].1
            guard holeA != holeB else { return false }
            // Check that swap does not reduce crossings
            let crossingsBefore = totalCrossingPairs(endpoints: endpoints, holes: holeSimd)
            var testEndpoints = endpoints
            if endA == 0 { testEndpoints[ropeA].0 = holeB } else { testEndpoints[ropeA].1 = holeB }
            if endB == 0 { testEndpoints[ropeB].0 = holeA } else { testEndpoints[ropeB].1 = holeA }
            let crossingsAfter = totalCrossingPairs(endpoints: testEndpoints, holes: holeSimd)
            if crossingsAfter < crossingsBefore { return false }
            // Swap endpoints
            if endA == 0 { endpoints[ropeA].0 = holeB } else { endpoints[ropeA].1 = holeB }
            if endB == 0 { endpoints[ropeB].0 = holeA } else { endpoints[ropeB].1 = holeA }
            actions.append(.init(type: "swap", ropeIndex: ropeA, endIndex: endA, holeIndex: holeB,
                                 ropeIndex2: ropeB, endIndex2: endB))
            return true
        }

        if shortCount >= 2 {
            for d in 0..<shortCount {
                let ropeIdx = d
                for targetIdx in 0..<shortCount where targetIdx != ropeIdx {
                    let endIdx = d % 2
                    if tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: targetIdx) { break }
                    if tryDrag(ropeIdx: ropeIdx, endIdx: 1 - endIdx, targetRopeIdx: targetIdx) { break }
                }
            }
        }

        for d in 0..<totalDrags {
            let ropeIdx = d % ropes.count
            let targetRopeIdx = (ropeIdx + 1 + d / ropes.count) % ropes.count
            if targetRopeIdx == ropeIdx { continue }
            let endIdx = d / ropes.count % 2

            // Decide swap vs drag based on swapPercent (deterministic per d)
            let useSwap = swapPercent > 0 && ((d * 97 + 13) % 100) < swapPercent
            if useSwap {
                let targetEnd = (d / ropes.count + 1) % 2
                if trySwap(ropeA: ropeIdx, endA: endIdx, ropeB: targetRopeIdx, endB: targetEnd) {
                    continue
                }
                // Fallback to drag if swap failed
            }
            if !tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: targetRopeIdx) {
                tryDrag(ropeIdx: ropeIdx, endIdx: 1 - endIdx, targetRopeIdx: targetRopeIdx)
            }
        }

        for attempt in 0..<ropes.count * 8 {
            let crossings = ropeCrossings(endpoints: endpoints, holes: holeSimd)
            let isolated = (0..<ropes.count).filter { crossings[$0] == 0 }
            if isolated.isEmpty { break }

            let ropeIdx = isolated[attempt % isolated.count]
            var fixed = false
            for other in 0..<ropes.count where other != ropeIdx && !fixed {
                for endIdx in 0...1 where !fixed {
                    fixed = tryDrag(ropeIdx: ropeIdx, endIdx: endIdx, targetRopeIdx: other, unconstrained: true)
                }
            }
            if !fixed {
                for other in 0..<ropes.count where other != ropeIdx && !fixed {
                    for endIdx in 0...1 where !fixed {
                        fixed = tryDrag(ropeIdx: other, endIdx: endIdx, targetRopeIdx: ropeIdx, unconstrained: true)
                    }
                }
            }
            if !fixed {
                let allUsed = Set(endpoints.flatMap { [$0.0, $0.1] })
                let freeHoles = (0..<holes.count).filter { !allUsed.contains($0) }
                for other in 0..<ropes.count where other != ropeIdx && !fixed {
                    let oS = holeSimd[endpoints[other].0]
                    let oE = holeSimd[endpoints[other].1]
                    for h0 in freeHoles where !fixed {
                        for h1 in freeHoles where h1 != h0 && !fixed {
                            if segmentsCross(holeSimd[h0], holeSimd[h1], oS, oE) {
                                let old0 = endpoints[ropeIdx].0
                                let old1 = endpoints[ropeIdx].1
                                usedHoles.remove(old0)
                                usedHoles.remove(old1)
                                endpoints[ropeIdx].0 = h0
                                endpoints[ropeIdx].1 = h1
                                usedHoles.insert(h0)
                                usedHoles.insert(h1)
                                actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: 0, holeIndex: h0))
                                actions.append(.init(type: "drag", ropeIndex: ropeIdx, endIndex: 1, holeIndex: h1))
                                fixed = true
                            }
                        }
                    }
                }
            }
        }

        return actions
    }

    // MARK: - Geometry helpers

    private static func totalCrossingPairs(endpoints: [(Int, Int)], holes: [SIMD2<Float>]) -> Int {
        let n = endpoints.count
        var total = 0
        for i in 0..<n {
            let a0 = holes[endpoints[i].0]
            let a1 = holes[endpoints[i].1]
            for j in (i+1)..<n {
                let b0 = holes[endpoints[j].0]
                let b1 = holes[endpoints[j].1]
                if segmentsCross(a0, a1, b0, b1) { total += 1 }
            }
        }
        return total
    }

    private static func ropeCrossings(endpoints: [(Int, Int)], holes: [SIMD2<Float>]) -> [Int] {
        let n = endpoints.count
        var counts = Array(repeating: 0, count: n)
        for i in 0..<n {
            let a0 = holes[endpoints[i].0]
            let a1 = holes[endpoints[i].1]
            for j in (i+1)..<n {
                let b0 = holes[endpoints[j].0]
                let b1 = holes[endpoints[j].1]
                if segmentsCross(a0, a1, b0, b1) {
                    counts[i] += 1
                    counts[j] += 1
                }
            }
        }
        return counts
    }

    private static func segmentsCross(
        _ a0: SIMD2<Float>, _ a1: SIMD2<Float>,
        _ b0: SIMD2<Float>, _ b1: SIMD2<Float>
    ) -> Bool {
        let d1 = a1 - a0
        let d2 = b1 - b0
        let cross = d1.x * d2.y - d1.y * d2.x
        if abs(cross) < 1e-9 { return false }
        let d = b0 - a0
        let t = (d.x * d2.y - d.y * d2.x) / cross
        let u = (d.x * d1.y - d.y * d1.x) / cross
        return t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99
    }

    // MARK: - 3D Board layout implementations

    static func generateBoardLayout(_ layout: BoardLayout, n: Int, elevation: Float = 0.12) -> BoardLayoutResult {
        switch layout {
        case .twoSides:     return twoSidesLayout(n: n, elevation: elevation)
        case .bridge:       return bridgeLayout(n: n, elevation: elevation)
        case .staircase:    return staircaseLayout(n: n, elevation: elevation)
        case .platform:     return platformLayout(n: n, elevation: elevation)
        case .valley:       return valleyLayout(n: n, elevation: elevation)
        }
    }

    private static func twoSidesLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 0.55
        let boardH: Float = 1.4
        let gap: Float = 0.35

        let leftBoard = LevelDefinition.Board(centerX: -(gap + boardW * 0.5), centerY: 0, width: boardW, height: boardH, elevation: elevation)
        let rightBoard = LevelDefinition.Board(centerX: gap + boardW * 0.5, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let centerHoles = max(4, n / 3)
        let sideHoles = max(3, (n - centerHoles) / 2)
        let s: Float = 0.35

        var holes: [LevelDefinition.Vec2] = []

        let centerCols = max(2, Int(ceil(sqrt(Float(centerHoles) * 1.5))))
        let centerRows = max(2, (centerHoles + centerCols - 1) / centerCols)
        let cs = min(s, gap * 1.5 / Float(centerCols))
        for row in 0..<centerRows {
            for col in 0..<centerCols {
                let x = (Float(col) - Float(centerCols - 1) / 2) * cs
                let y = (Float(row) - Float(centerRows - 1) / 2) * cs
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        let sideRows = max(2, sideHoles)
        let ss = min(s, boardH * 0.8 / Float(sideRows - 1))
        for i in 0..<sideRows {
            let y = (Float(i) - Float(sideRows - 1) / 2) * ss
            holes.append(.init(xPosition: leftBoard.centerX, yPosition: y, zPosition: elevation))
        }
        for i in 0..<sideRows {
            let y = (Float(i) - Float(sideRows - 1) / 2) * ss
            holes.append(.init(xPosition: rightBoard.centerX, yPosition: y, zPosition: elevation))
        }

        return BoardLayoutResult(holes: holes, boards: [leftBoard, rightBoard])
    }

    private static func bridgeLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 1.8
        let boardH: Float = 0.45

        let bridge = LevelDefinition.Board(centerX: 0, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let topHoles = max(3, n / 3)
        let bottomHoles = max(3, n / 3)
        let bridgeHoles = max(3, n - topHoles - bottomHoles)
        let s: Float = 0.35

        var holes: [LevelDefinition.Vec2] = []

        let cols = max(2, Int(ceil(sqrt(Float(topHoles) * 2))))
        let rows = max(2, (topHoles + cols - 1) / cols)
        let ts = min(s, 1.4 / Float(cols))
        for row in 0..<rows {
            for col in 0..<cols {
                let x = (Float(col) - Float(cols - 1) / 2) * ts
                let y = boardH * 0.5 + 0.25 + Float(row) * ts
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        for row in 0..<rows {
            for col in 0..<cols {
                guard holes.count < topHoles + bottomHoles else { break }
                let x = (Float(col) - Float(cols - 1) / 2) * ts
                let y = -(boardH * 0.5 + 0.25 + Float(row) * ts)
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        let bCols = max(2, bridgeHoles)
        let bs = min(s, boardW * 0.7 / Float(bCols - 1))
        for i in 0..<bCols {
            let x = (Float(i) - Float(bCols - 1) / 2) * bs
            holes.append(.init(xPosition: x, yPosition: 0, zPosition: elevation))
        }

        return BoardLayoutResult(holes: holes, boards: [bridge])
    }

    private static func staircaseLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let steps = 3
        let stepW: Float = 0.55
        let stepH: Float = 1.2
        let stepGap: Float = 0.08

        var boards: [LevelDefinition.Board] = []
        var holes: [LevelDefinition.Vec2] = []
        let holesPerStep = max(3, n / (steps + 1))
        let s: Float = 0.32

        let floorHoles = max(3, n - holesPerStep * steps)
        let floorX: Float = -(Float(steps) * (stepW + stepGap)) * 0.5 - 0.3
        let floorRows = max(2, floorHoles)
        let fs = min(s, stepH * 0.8 / Float(floorRows - 1))
        for i in 0..<floorRows {
            let y = (Float(i) - Float(floorRows - 1) / 2) * fs
            holes.append(.init(xPosition: floorX, yPosition: y))
        }

        for step in 0..<steps {
            let elev: Float = Float(step + 1) * elevation
            let cx = Float(step) * (stepW + stepGap) - Float(steps - 1) * (stepW + stepGap) * 0.5 + stepW * 0.5
            let board = LevelDefinition.Board(centerX: cx, centerY: 0, width: stepW, height: stepH, elevation: elev)
            boards.append(board)

            let stepRows = max(2, holesPerStep)
            let ss = min(s, stepH * 0.7 / Float(stepRows - 1))
            for i in 0..<stepRows {
                let y = (Float(i) - Float(stepRows - 1) / 2) * ss
                holes.append(.init(xPosition: cx, yPosition: y, zPosition: elev))
            }
        }

        return BoardLayoutResult(holes: holes, boards: boards)
    }

    private static func platformLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 0.8
        let boardH: Float = 0.8

        let platform = LevelDefinition.Board(centerX: 0, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let platformHoles = max(4, n / 3)
        let groundHoles = n - platformHoles
        let s: Float = 0.35

        var holes: [LevelDefinition.Vec2] = []

        let pCols = max(2, Int(ceil(sqrt(Float(platformHoles)))))
        let pRows = max(2, (platformHoles + pCols - 1) / pCols)
        let ps = min(s, boardW * 0.7 / Float(max(pCols, pRows) - 1))
        for row in 0..<pRows {
            for col in 0..<pCols {
                let x = (Float(col) - Float(pCols - 1) / 2) * ps
                let y = (Float(row) - Float(pRows - 1) / 2) * ps
                holes.append(.init(xPosition: x, yPosition: y, zPosition: elevation))
            }
        }

        let ringR: Float = boardW * 0.5 + 0.35
        let ringCount = max(8, groundHoles)
        for i in 0..<ringCount {
            let a = Float(i) / Float(ringCount) * 2 * .pi - .pi / 2
            holes.append(.init(xPosition: ringR * cos(a), yPosition: ringR * sin(a)))
        }

        return BoardLayoutResult(holes: holes, boards: [platform])
    }

    private static func valleyLayout(n: Int, elevation: Float) -> BoardLayoutResult {
        let boardW: Float = 0.50
        let boardH: Float = 1.6

        let leftBoard = LevelDefinition.Board(centerX: -0.65, centerY: 0, width: boardW, height: boardH, elevation: elevation)
        let rightBoard = LevelDefinition.Board(centerX: 0.65, centerY: 0, width: boardW, height: boardH, elevation: elevation)

        let sideHoles = max(3, n / 3)
        let valleyHoles = max(4, n - sideHoles * 2)
        let s: Float = 0.32

        var holes: [LevelDefinition.Vec2] = []

        let sRows = max(2, sideHoles)
        let ss = min(s, boardH * 0.7 / Float(sRows - 1))
        for i in 0..<sRows {
            let y = (Float(i) - Float(sRows - 1) / 2) * ss
            holes.append(.init(xPosition: leftBoard.centerX, yPosition: y, zPosition: elevation))
        }
        for i in 0..<sRows {
            let y = (Float(i) - Float(sRows - 1) / 2) * ss
            holes.append(.init(xPosition: rightBoard.centerX, yPosition: y, zPosition: elevation))
        }

        let vCols = max(2, Int(ceil(sqrt(Float(valleyHoles) * 2))))
        let vRows = max(2, (valleyHoles + vCols - 1) / vCols)
        let vs = min(s, 0.5 / Float(max(vCols, vRows) - 1))
        for row in 0..<vRows {
            for col in 0..<vCols {
                let x = (Float(col) - Float(vCols - 1) / 2) * vs
                let y = (Float(row) - Float(vRows - 1) / 2) * vs
                holes.append(.init(xPosition: x, yPosition: y))
            }
        }

        return BoardLayoutResult(holes: holes, boards: [leftBoard, rightBoard])
    }
}
