import Foundation
import simd

enum LevelGenerator {

    // MARK: - Public

    static func generate(levelId: Int, boardElevation: Float = 0.12) -> LevelDefinition {
        var rng = SeededRNG(seed: UInt64(levelId) &* 2654435761)

        let diff = difficulty(for: levelId)
        let use3D = levelId >= 10 && levelId % 3 == 0
        let boardLayout: BoardLayout? = use3D ? pickBoardLayout(for: levelId) : nil

        let useWrapping = levelId >= 5 && (levelId - 5) % 3 == 0

        let holes: [LevelDefinition.Vec2]
        let boards: [LevelDefinition.Board]?

        if let bl = boardLayout {
            let result = generateBoardLayout(bl, n: minHoles(for: levelId, ropeCount: diff.ropeCount), elevation: boardElevation)
            holes = result.holes
            boards = result.boards
        } else if useWrapping {
            // Braid levels need holes at various distances from center (not just 2 radii).
            // Grid layout guarantees enough free holes near ideal drag targets.
            // At least 20 holes (5x4 grid) for good braid geometry.
            let braidHoles = max(diff.ropeCount * 3 + 4, max(20, minHoles(for: levelId, ropeCount: diff.ropeCount)))
            holes = HoleLayout.grid.generate(n: braidHoles)
            boards = nil
        } else {
            let layout = pickLayout(for: levelId)
            holes = layout.generate(n: minHoles(for: levelId, ropeCount: diff.ropeCount))
            boards = nil
        }

        let ropes: [LevelDefinition.Rope]
        let actions: [LevelDefinition.Action]

        if useWrapping {
            // Build star-braid: compute ideal geometry first, snap to nearest free holes.
            // This avoids searching for crossing holes and always produces a clean star shape.
            let braid = buildBraidLevel(holes: holes, ropeCount: diff.ropeCount, totalDrags: diff.totalDrags, rng: &rng)
            ropes = braid.ropes
            actions = braid.actions
        } else {
            let maxRopes = max(1, (holes.count - 3) / 2)
            let ropeCount = min(diff.ropeCount, maxRopes)
            let shortCount = min(diff.shortRopeCount, ropeCount - 1)
            let ropePairs = pickStructuredPairs(holes: holes, count: ropeCount, shortCount: shortCount, rng: &rng)
            ropes = ropePairs.enumerated().map { i, pair in
                LevelDefinition.Rope(startHole: pair.0, endHole: pair.1,
                                     color: colors[i % colors.count], radius: 0.038)
            }
            actions = buildActions(ropes: ropes, holes: holes,
                                   totalDrags: diff.totalDrags, shortCount: shortCount)
        }

        return LevelDefinition(
            mode: nil,
            id: levelId,
            holeRadius: 0.08,
            particlesPerRope: 32,
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

    static func generateRailLevel(levelId: Int) -> LevelDefinition {
        let localId = levelId - 2000
        switch localId {
        case 1: return railLevel1()
        case 2: return railLevel2()
        case 3: return railLevel3()
        default: return railLevel1()
        }
    }

    // Rail Level 1: "First Push" — straight rail, 1 cart, 2 ropes + 4 holes
    // Player wraps rope around cart and pulls to push it to the station
    private static func railLevel1() -> LevelDefinition {
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
            mode: "rail", id: 2001, holeRadius: 0.08, particlesPerRope: 20,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: rails, carts: carts, stations: stations
        )
    }

    // Rail Level 2: "L-Turn" — L-shaped rail with turn
    private static func railLevel2() -> LevelDefinition {
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
            mode: "rail", id: 2002, holeRadius: 0.08, particlesPerRope: 20,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: rails, carts: carts, stations: stations
        )
    }

    // Rail Level 3: "S-Curve" — S-shaped rail
    private static func railLevel3() -> LevelDefinition {
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
            mode: "rail", id: 2003, holeRadius: 0.08, particlesPerRope: 24,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: rails, carts: carts, stations: stations
        )
    }

    // MARK: - Tension mode levels

    static func generateTensionLevel(levelId: Int) -> LevelDefinition {
        let localId = levelId - 1000  // 1=first, 2=second, etc.
        switch localId {
        case 1: return tensionLevel1()
        case 2: return tensionLevel2()
        case 3: return tensionLevel3()
        case 4: return tensionLevel4()
        case 5: return tensionLevel5()
        default: return tensionLevel1()
        }
    }

    // MARK: — Tension Level 1: "First Pull"
    // 1 weight, 2 ropes, pull right. Tutorial.
    private static func tensionLevel1() -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.6, yPosition: -0.3),  // 0
            .init(xPosition: -0.6, yPosition:  0.3),  // 1
            .init(xPosition:  0.6, yPosition: -0.3),  // 2
            .init(xPosition:  0.6, yPosition:  0.3),  // 3
        ]
        let wi = holes.count
        return LevelDefinition(
            mode: "tension", id: 1001, holeRadius: 0.08, particlesPerRope: 8,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: wi, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: wi, color: colors[1], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, wi), (1, 1, wi)], holeCount: holes.count),
            boards: nil,
            weights: [.init(x: -0.3, y: 0, mass: 0.3, radius: 0.1)],
            targets: [.init(x: 0.4, y: 0, radius: 0.15, weightIndex: 0)],
            rails: nil, carts: nil, stations: nil
        )
    }

    // MARK: — Tension Level 2: "Crossroads"
    // 1 weight center, 4 ropes from corners. Target is top-right. Must redirect pull.
    private static func tensionLevel2() -> LevelDefinition {
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
            mode: "tension", id: 1002, holeRadius: 0.08, particlesPerRope: 8,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: wi, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: wi, color: colors[1], radius: 0.035),
                .init(startHole: 2, endHole: wi, color: colors[2], radius: 0.035),
                .init(startHole: 4, endHole: wi, color: colors[3], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, wi), (1, 1, wi), (2, 2, wi), (3, 4, wi)], holeCount: holes.count),
            boards: nil,
            weights: [.init(x: 0, y: 0, mass: 0.4, radius: 0.1)],
            targets: [.init(x: 0.45, y: 0.4, radius: 0.15, weightIndex: 0)],
            rails: nil, carts: nil, stations: nil
        )
    }

    // MARK: — Tension Level 3: "Two Weights"
    // 2 weights, each with 2 ropes. Must deliver both to targets.
    private static func tensionLevel3() -> LevelDefinition {
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
            mode: "tension", id: 1003, holeRadius: 0.08, particlesPerRope: 8,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: w0, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: w0, color: colors[1], radius: 0.035),
                .init(startHole: 2, endHole: w1, color: colors[2], radius: 0.035),
                .init(startHole: 7, endHole: w1, color: colors[3], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, w0), (1, 1, w0), (2, 2, w1), (3, 7, w1)], holeCount: holes.count),
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
    private static func tensionLevel4() -> LevelDefinition {
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
            mode: "tension", id: 1004, holeRadius: 0.08, particlesPerRope: 6,
            holes: holes,
            ropes: [
                .init(startHole: 0, endHole: wi, color: colors[0], radius: 0.035),
                .init(startHole: 1, endHole: wi, color: colors[1], radius: 0.035),
                .init(startHole: 2, endHole: wi, color: colors[2], radius: 0.035),
                .init(startHole: 3, endHole: wi, color: colors[3], radius: 0.035),
                .init(startHole: 8, endHole: wi, color: colors[4 % colors.count], radius: 0.035),
            ],
            hooks: nil,
            actions: pinActions(ropes: [(0, 0, wi), (1, 1, wi), (2, 2, wi), (3, 3, wi), (4, 8, wi)], holeCount: holes.count),
            boards: nil,
            weights: [.init(x: -0.5, y: 0, mass: 0.8, radius: 0.12)],
            targets: [.init(x: 0.6, y: 0, radius: 0.15, weightIndex: 0)],
            rails: nil, carts: nil, stations: nil
        )
    }

    // MARK: — Tension Level 5: "Triangle"
    // 3 weights in a triangle, each with 2 ropes. Targets form inverted triangle.
    private static func tensionLevel5() -> LevelDefinition {
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
            mode: "tension", id: 1005, holeRadius: 0.08, particlesPerRope: 8,
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
            ], holeCount: holes.count),
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
    private static func pinActions(ropes: [(Int, Int, Int)], holeCount: Int) -> [LevelDefinition.Action] {
        var actions: [LevelDefinition.Action] = []
        for (ri, startHole, endHole) in ropes {
            actions.append(.init(type: "pin", ropeIndex: ri, endIndex: 0, holeIndex: startHole))
            actions.append(.init(type: "pin", ropeIndex: ri, endIndex: 1, holeIndex: endHole))
        }
        return actions
    }

    // MARK: - Particle-based braid levels

    static func generateParticleBraidLevel(levelId: Int) -> LevelDefinition {
        let localId = levelId - 3000
        let strandCount: Int
        let crossingCount: Int
        switch localId {
        case 1:  strandCount = 3; crossingCount = 4
        case 2:  strandCount = 3; crossingCount = 6
        case 3:  strandCount = 3; crossingCount = 8
        case 4:  strandCount = 4; crossingCount = 6
        case 5:  strandCount = 4; crossingCount = 8
        case 6:  strandCount = 4; crossingCount = 10
        case 7:  strandCount = 5; crossingCount = 8
        case 8:  strandCount = 5; crossingCount = 10
        case 9:  strandCount = 5; crossingCount = 12
        case 10: strandCount = 6; crossingCount = 12
        default:
            strandCount = min(8, 3 + (localId - 1) / 3)
            crossingCount = min(16, 4 + localId)
        }
        return buildParticleBraidLevel(levelId: levelId, strandCount: strandCount, crossingCount: crossingCount)
    }

    private static func buildParticleBraidLevel(levelId: Int, strandCount: Int, crossingCount: Int) -> LevelDefinition {
        let n = strandCount
        let K = crossingCount
        let P = 48 // particles per strand
        let ropeRadius: Float = 0.038
        let spacing: Float = 0.14 // lane spacing — tight braid
        let totalWidth = Float(n - 1) * spacing
        let startX = -totalWidth / 2
        let yTop: Float = 0.55
        let yBot: Float = -0.55
        let yRange = yTop - yBot

        let zOver: Float = ropeRadius * 4.0
        let zUnder: Float = ropeRadius * 0.3
        let zNeutral: Float = ropeRadius * 1.5

        func laneX(_ lane: Int) -> Float {
            startX + Float(lane) * spacing
        }

        // Crossing schedule
        struct CrossingInfo {
            let lane: Int
            let y: Float
            let overStrand: Int
            let underStrand: Int
        }

        let crossingSpacing = yRange / Float(K + 1)
        let regionHalf = crossingSpacing * 0.4

        var strandAtLane = Array(0..<n)
        var crossingList: [CrossingInfo] = []
        // laneHistory[strand][k]: lane of strand after k crossings (index 0 = initial)
        var laneHistory = (0..<n).map { [$0] }

        for k in 0..<K {
            let lane: Int
            if n == 3 {
                // Classic 3-strand braid: alternate σ₀, σ₁
                lane = k % 2 == 0 ? 0 : 1
            } else {
                // N strands: cycle through σ₀, σ₁, ..., σ_{n-2}
                lane = k % (n - 1)
            }
            let y = yTop - crossingSpacing * Float(k + 1)
            let overStrand = strandAtLane[lane]
            let underStrand = strandAtLane[lane + 1]
            crossingList.append(CrossingInfo(lane: lane, y: y, overStrand: overStrand, underStrand: underStrand))
            strandAtLane.swapAt(lane, lane + 1)
            for s in 0..<n {
                laneHistory[s].append(strandAtLane.firstIndex(of: s)!)
            }
        }

        // Generate particles for each strand
        var allParticles: [[LevelDefinition.Vec2]] = []

        for s in 0..<n {
            var particles: [LevelDefinition.Vec2] = []

            for i in 0..<P {
                let t = Float(i) / Float(P - 1)
                let y = yTop + (yBot - yTop) * t

                // Find current segment (how many crossings we've passed)
                var segIdx = K
                for k in 0..<K {
                    if y > crossingList[k].y {
                        segIdx = k
                        break
                    }
                }

                var x: Float = laneX(laneHistory[s][segIdx])
                var z: Float = zNeutral

                // Check if in a crossing transition region
                for k in 0..<K {
                    let cy = crossingList[k].y
                    guard y <= cy + regionHalf && y >= cy - regionHalf else { continue }

                    let cr = crossingList[k]
                    let transT = (cy + regionHalf - y) / (2 * regionHalf) // 0 at top, 1 at bottom
                    let smoothT = (1 - cos(Float.pi * transT)) / 2

                    if cr.overStrand == s || cr.underStrand == s {
                        let fromLane = laneHistory[s][k]
                        let toLane = laneHistory[s][k + 1]
                        x = laneX(fromLane) + (laneX(toLane) - laneX(fromLane)) * smoothT

                        // Bell curve for Z peaking at midpoint
                        let bellT = 4 * transT * (1 - transT)
                        if cr.overStrand == s {
                            z = zNeutral + (zOver - zNeutral) * bellT
                        } else {
                            z = zNeutral + (zUnder - zNeutral) * bellT
                        }
                    }
                    break
                }

                // Endpoints at hole level
                if i == 0 || i == P - 1 {
                    z = 0
                }

                particles.append(LevelDefinition.Vec2(xPosition: x, yPosition: y, zPosition: z))
            }

            allParticles.append(particles)
        }

        // Holes: N at top (index 0..<n), N at bottom (index n..<2n)
        var holes: [LevelDefinition.Vec2] = []
        for i in 0..<n {
            holes.append(.init(xPosition: laneX(i), yPosition: yTop))
        }
        for i in 0..<n {
            holes.append(.init(xPosition: laneX(i), yPosition: yBot))
        }

        // Ropes: strand i connects top[i] to bottom[finalLane(i)]
        let ropes = (0..<n).map { i in
            let finalLane = laneHistory[i][K]
            return LevelDefinition.Rope(startHole: i, endHole: n + finalLane,
                                        color: colors[i % colors.count], radius: ropeRadius)
        }

        // Pin actions (needed for pin tracking, but positions come from ropeParticles)
        var actions: [LevelDefinition.Action] = []
        for (i, rope) in ropes.enumerated() {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: rope.startHole))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: rope.endHole))
        }

        return LevelDefinition(
            mode: nil, id: levelId, holeRadius: 0.08, particlesPerRope: P,
            holes: holes, ropes: ropes, hooks: nil, actions: actions, boards: nil,
            weights: nil, targets: nil,
            rails: nil, carts: nil, stations: nil,
            ropeParticles: allParticles
        )
    }

    // MARK: - Braid mode levels

    static func generateBraidLevel(levelId: Int) -> LevelDefinition {
        let localId = levelId - 3000
        let strandCount: Int
        let swapCount: Int
        switch localId {
        case 1:  strandCount = 3; swapCount = 2
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
        return buildBraidModeLevel(levelId: levelId, strandCount: strandCount, swapCount: swapCount)
    }

    /// Build a braid puzzle level:
    /// - N holes at top (anchors), N holes at bottom (moveable ends)
    /// - N strands connecting top[i] to bottom[i]
    /// - Target: a permutation achieved by `swapCount` adjacent swaps (classic braid)
    private static func buildBraidModeLevel(levelId: Int, strandCount: Int, swapCount: Int) -> LevelDefinition {
        let n = strandCount
        let spacing: Float = 0.28
        let totalWidth = Float(n - 1) * spacing
        let startX = -totalWidth / 2

        // Top holes: indices 0..<n, Bottom holes: indices n..<2n
        var holes: [LevelDefinition.Vec2] = []
        for i in 0..<n {
            let x = startX + Float(i) * spacing
            holes.append(.init(xPosition: x, yPosition: 0.55))  // top
        }
        for i in 0..<n {
            let x = startX + Float(i) * spacing
            holes.append(.init(xPosition: x, yPosition: -0.55)) // bottom
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

        return LevelDefinition(
            mode: "braid", id: levelId, holeRadius: 0.08, particlesPerRope: 24,
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
    }

    private static func difficulty(for levelId: Int) -> Difficulty {
        switch levelId {
        case 1...2:   return Difficulty(ropeCount: 3, totalDrags: 3, shortRopeCount: 0)
        case 3...5:   return Difficulty(ropeCount: 4, totalDrags: 4, shortRopeCount: levelId >= 4 ? 2 : 0)
        case 6...9:   return Difficulty(ropeCount: 5, totalDrags: 6, shortRopeCount: 3)
        case 10...15: return Difficulty(ropeCount: 6, totalDrags: 8, shortRopeCount: 4)
        case 16...25: return Difficulty(ropeCount: 7, totalDrags: 10, shortRopeCount: 5)
        case 26...50: return Difficulty(ropeCount: 8, totalDrags: 12 + (levelId - 25) / 5, shortRopeCount: 6)
        default:      return Difficulty(ropeCount: min(10, 8 + (levelId - 50) / 25),
                                        totalDrags: min(22, 16 + (levelId - 50) / 10),
                                        shortRopeCount: min(8, 6 + (levelId - 50) / 30))
        }
    }

    private static func minHoles(for levelId: Int, ropeCount: Int) -> Int {
        let base = ropeCount * 2 + 4
        switch levelId {
        case 1...5:    return max(base, 10)
        case 6...15:   return max(base, 14)
        case 16...30:  return max(base, 18)
        case 31...60:  return max(base, 20)
        case 61...100: return max(base, 22)
        default:       return max(base, 24)
        }
    }

    // MARK: - Layout selection

    private static func pickLayout(for levelId: Int) -> HoleLayout {
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
        shortCount: Int
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

            var bestHole: Int?
            var bestScore: Float = -.greatestFiniteMagnitude

            for candidate in 0..<holes.count {
                if candidate == currentHole || candidate == anchorHole { continue }
                if otherUsed.contains(candidate) { continue }
                let cPos = holeSimd[candidate]
                if isShort && simd_length(cPos - shortCenter) > shortRadius { continue }
                if segmentsCross(anchorPos, cPos, tS, tE) {
                    let score = -simd_length(cPos - (tS + tE) * 0.5)
                    if score > bestScore {
                        bestScore = score
                        bestHole = candidate
                    }
                }
            }

            guard let targetHole = bestHole else { return false }

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

    // MARK: - Braid tangle generation

    private struct BraidResult {
        let ropes: [LevelDefinition.Rope]
        let actions: [LevelDefinition.Action]
    }

    struct BraidDebugGeometry {
        let mid: SIMD2<Float>
        let sigma: SIMD2<Float>       // sigma direction (unit)
        let sigmaLen: Float
        let centers: [SIMD2<Float>]    // crossing centers along sigma
        let contactsA: [SIMD2<Float>]  // contact points for rope A
        let contactsB: [SIMD2<Float>]  // contact points for rope B
        let dragTargets: [(from: SIMD2<Float>, to: SIMD2<Float>, ropeIndex: Int)]
        let holes: [SIMD2<Float>]      // all hole positions for labeling
    }
    // Store last computed debug geometry for visualization
    nonisolated(unsafe) static var lastBraidDebug: BraidDebugGeometry?

    /// Build a braid level: 3 parallel ropes, all drags to center hole.
    ///
    /// Algorithm (from user's recording on level 3001):
    /// 1. Find 3 top holes + 3 bottom holes from existing layout.
    /// 2. Pin 3 ropes parallel: left→left, center→center, right→right.
    /// 3. Cycle drags: rope2→centerBot, rope0→centerBot, rope1→centerBot, repeat.
    /// 4. simulateDrag lifts each end over the others → classic 3-strand braid.
    private static func buildBraidLevel(
        holes: [LevelDefinition.Vec2],
        ropeCount: Int,
        totalDrags: Int,
        rng: inout SeededRNG
    ) -> BraidResult {
        let holeSimd = holes.map { $0.simd }
        let center = holeSimd.reduce(SIMD2<Float>.zero, +) / Float(holeSimd.count)
        guard holes.count >= 6 else { return BraidResult(ropes: [], actions: []) }

        // Pick a random direction as "up"
        let baseAngle = Float(rng.next(bound: 1000)) / 1000.0 * 2 * .pi
        let upDir = SIMD2<Float>(cos(baseAngle), sin(baseAngle))
        let perpDir = SIMD2<Float>(-upDir.y, upDir.x)

        // Sort holes by projection onto upDir (highest first)
        let sorted = holeSimd.indices.sorted {
            simd_dot(holeSimd[$0] - center, upDir) > simd_dot(holeSimd[$1] - center, upDir)
        }

        // Top 3 holes (highest projection), sorted left-to-right by perpDir
        let topCandidates = Array(sorted.prefix(max(3, holes.count / 3)))
        let topSorted = topCandidates.sorted { simd_dot(holeSimd[$0], perpDir) < simd_dot(holeSimd[$1], perpDir) }
        let top = [topSorted.first!, topSorted[topSorted.count / 2], topSorted.last!]

        // Bottom 3 holes (lowest projection), sorted left-to-right
        let botCandidates = Array(sorted.suffix(max(3, holes.count / 3)))
        let botSorted = botCandidates.sorted { simd_dot(holeSimd[$0], perpDir) < simd_dot(holeSimd[$1], perpDir) }
        let bot = [botSorted.first!, botSorted[botSorted.count / 2], botSorted.last!]

        let centerBot = bot[1] // ALL drags go here

        // 3 ropes: left, center, right
        let ropesDef = (0..<3).map { i in
            LevelDefinition.Rope(startHole: top[i], endHole: bot[i],
                                 color: colors[i % colors.count], radius: 0.055)
        }

        var actions = [LevelDefinition.Action]()
        for i in 0..<3 {
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 0, holeIndex: top[i]))
            actions.append(.init(type: "pin", ropeIndex: i, endIndex: 1, holeIndex: bot[i]))
        }

        // Drags: cycle 2, 0, 1 — all end1 to centerBot (from user recording)
        let dragCount = max(totalDrags, 6)
        let cycle = [2, 0, 1]
        for d in 0..<dragCount {
            actions.append(.init(type: "drag", ropeIndex: cycle[d % 3], endIndex: 1, holeIndex: centerBot))
        }

        lastBraidDebug = nil
        return BraidResult(ropes: ropesDef, actions: actions)
    }

    // MARK: - Geometry helpers

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
