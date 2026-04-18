import Foundation
import simd

// MARK: - Individual showcase level definitions (3100-3117)

extension LevelGenerator {

    static func showcase3100(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3100, particleCount: P, structures: [
            .braid(BraidConfig(center: .zero, axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 8, width: 0.56, zRadius: 0.07)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    static func showcase3101(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3101, particleCount: P, structures: [
            .helix(HelixConfig(center: .zero, axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 2, turns: 5, helixRadius: 0.12, zRadius: 0.06)),
        ], extraHoleRings: [(0.30, 8), (0.60, 14)])
    }

    static func showcase3102(_ P: Int) -> LevelDefinition {
        let a: Float = Float.pi / 6
        return buildStructuredLevel(levelId: 3102, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.45, 0), axis: SIMD2(sin(a), cos(a)), length: 1.0,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.07)),
            .braid(BraidConfig(center: SIMD2(0.45, 0), axis: SIMD2(-sin(a), cos(a)), length: 1.0,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.07)),
        ], extraHoleRings: [(0.35, 8), (0.70, 14)])
    }

    static func showcase3103(_ P: Int) -> LevelDefinition {
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

    static func showcase3104(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3104, particleCount: P, structures: [
            .braid(BraidConfig(center: .zero, axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 10, width: 0.56, zRadius: 0.07)),
        ], extraHoleRings: [(0.20, 6), (0.45, 10), (0.70, 14)])
    }

    static func showcase3105(_ P: Int) -> LevelDefinition {
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

    static func showcase3106(_ P: Int) -> LevelDefinition {
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

    static func showcase3107(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3107, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.35, 0.32), axis: SIMD2(0, 1), length: 0.45,
                               strandCount: 3, crossings: 4, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0.10, 0), axis: SIMD2(0, 1), length: 0.45,
                               strandCount: 3, crossings: 4, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(-0.35, -0.32), axis: SIMD2(0, 1), length: 0.45,
                               strandCount: 3, crossings: 4, width: 0.30, zRadius: 0.06)),
        ], extraHoleRings: [(0.30, 8), (0.60, 14)])
    }

    static func showcase3108(_ P: Int) -> LevelDefinition {
        let d = SIMD2<Float>(1, 1) / sqrt(2.0)
        let d2 = SIMD2<Float>(1, -1) / sqrt(2.0)
        return buildStructuredLevel(levelId: 3108, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.40, 0), axis: d, length: 0.8,
                               strandCount: 3, crossings: 5, width: 0.22, zRadius: 0.07)),
            .braid(BraidConfig(center: SIMD2(0.40, 0), axis: d2, length: 0.8,
                               strandCount: 3, crossings: 5, width: 0.22, zRadius: 0.07)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    static func showcase3109(_ P: Int) -> LevelDefinition {
        var s: [StructurePrimitive] = []
        for i in 0..<4 {
            let angle = Float.pi / 4 + Float.pi / 2 * Float(i)
            s.append(.helix(HelixConfig(
                center: .zero, axis: SIMD2(cos(angle), sin(angle)), length: 0.85,
                strandCount: 3, turns: 2, helixRadius: 0.08, zRadius: 0.06)))
        }
        return buildStructuredLevel(levelId: 3109, particleCount: P,
                                    structures: s, extraHoleRings: [(0.25, 6), (0.50, 12)])
    }

    static func showcase3110(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3110, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.40, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
            .braid(BraidConfig(center: SIMD2(0.40, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    static func showcase3111(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3111, particleCount: P, structures: [
            .centralKnot(CentralKnotConfig(center: .zero,
                                           outerRadius: 0.62, innerRadius: 0.35,
                                           ropeCount: 3, loopDepth: 0.55, zRadius: 0.08)),
        ], extraHoleRings: [])
    }

    static func showcase3112(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3112, particleCount: P, structures: [
            .centralKnot(CentralKnotConfig(center: SIMD2(-0.25, 0),
                                           outerRadius: 0.45, innerRadius: 0.25,
                                           ropeCount: 3, loopDepth: 0.40, zRadius: 0.07)),
            .braid(BraidConfig(center: SIMD2(0.45, 0), axis: SIMD2(0, 1), length: 1.0,
                               strandCount: 3, crossings: 6, width: 0.30, zRadius: 0.06)),
        ], extraHoleRings: [])
    }

    static func showcase3113(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3113, particleCount: P, structures: [
            .centralKnot(CentralKnotConfig(center: .zero,
                                           outerRadius: 0.62, innerRadius: 0.35,
                                           ropeCount: 4, loopDepth: 0.55, zRadius: 0.08)),
        ], extraHoleRings: [])
    }

    static func showcase3114(_ P: Int) -> LevelDefinition {
        buildStructuredLevel(levelId: 3114, particleCount: P, structures: [
            .braid(BraidConfig(center: SIMD2(-0.35, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, crossings: 6, width: 0.35, zRadius: 0.06)),
            .helix(HelixConfig(center: SIMD2(0.35, 0), axis: SIMD2(0, 1), length: 1.1,
                               strandCount: 3, turns: 3, helixRadius: 0.12, zRadius: 0.06)),
        ], extraHoleRings: [(0.35, 8), (0.65, 14)])
    }

    static func showcase3115(_ P: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: 0.0000, yPosition: 0.0000),
            .init(xPosition: 0.3461, yPosition: 0.0523),
            .init(xPosition: 0.1277, yPosition: 0.3259),
            .init(xPosition: -0.2183, yPosition: 0.2736),
            .init(xPosition: -0.3461, yPosition: -0.0523),
            .init(xPosition: -0.1277, yPosition: -0.3259),
            .init(xPosition: 0.2183, yPosition: -0.2736),
            .init(xPosition: 0.6050, yPosition: 0.1872),
            .init(xPosition: 0.4304, yPosition: 0.4646),
            .init(xPosition: 0.1404, yPosition: 0.6176),
            .init(xPosition: -0.1872, yPosition: 0.6050),
            .init(xPosition: -0.4646, yPosition: 0.4304),
            .init(xPosition: -0.6176, yPosition: 0.1404),
            .init(xPosition: -0.6050, yPosition: -0.1872),
            .init(xPosition: -0.4304, yPosition: -0.4646),
            .init(xPosition: -0.1404, yPosition: -0.6176),
            .init(xPosition: 0.1872, yPosition: -0.6050),
            .init(xPosition: 0.4646, yPosition: -0.4304),
            .init(xPosition: 0.6176, yPosition: -0.1404),
        ]

        let wp0: [(Float, Float, Float)] = [
            (-0.6176, 0.1404, 0), (-0.5285, 0.1221, 0.038), (-0.4295, 0.1018, 0.038),
            (-0.3306, 0.0815, 0.038), (-0.2318, 0.0611, 0.038), (-0.1813, 0.0314, 0.049),
            (-0.1649, 0.0036, 0.073), (-0.1729, -0.0241, 0.103), (-0.2007, -0.0526, 0.117),
            (-0.2591, -0.0767, 0.110), (-0.3190, -0.0958, 0.097), (-0.3987, -0.1213, 0.081),
            (-0.4785, -0.1468, 0.064), (-0.5582, -0.1722, 0.047), (-0.6050, -0.1872, 0),
        ]
        let wp1: [(Float, Float, Float)] = [
            (0.4646, -0.4304, 0), (0.3997, -0.2871, 0.040), (0.3526, -0.1824, 0.044),
            (0.3021, -0.1308, 0.096), (0.2278, -0.1216, 0.111), (0.1267, -0.1075, 0.092),
            (0.0123, -0.0915, 0.071), (-0.1021, -0.0754, 0.050), (-0.1783, -0.0643, 0.038),
            (-0.2376, -0.0205, 0.056), (-0.2279, 0.0306, 0.111), (-0.1595, 0.0659, 0.120),
            (-0.0456, 0.0950, 0.103), (0.0682, 0.1244, 0.087), (0.1820, 0.1537, 0.071),
            (0.2713, 0.1756, 0.072), (0.3162, 0.2108, 0.104), (0.3484, 0.2831, 0.105),
            (0.3956, 0.3874, 0.065), (0.4304, 0.4646, 0),
        ]
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

        let allParticles = resampleWaypoints([wp0, wp1, wp2], P: P)

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

    static func showcase3116(_ P: Int) -> LevelDefinition {
        let holes: [LevelDefinition.Vec2] = [
            .init(xPosition: -0.2800, yPosition: 2.2000),
            .init(xPosition: 0.0000, yPosition: 2.2000),
            .init(xPosition: 0.2800, yPosition: 2.2000),
            .init(xPosition: -0.2800, yPosition: -2.2000),
            .init(xPosition: 0.0000, yPosition: -2.2000),
            .init(xPosition: 0.2800, yPosition: -2.2000),
        ]
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

        let allParticles = resampleWaypoints([wp0, wp1, wp2], P: P)

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

    static func showcase3117(_ P: Int) -> LevelDefinition {
        let holeCount = 10
        let holeR: Float = 0.75
        var holes: [LevelDefinition.Vec2] = []
        for i in 0..<holeCount {
            let angle = Float.pi / 2 + 2 * Float.pi * Float(i) / Float(holeCount)
            holes.append(.init(xPosition: holeR * cos(angle), yPosition: holeR * sin(angle)))
        }

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

        let strandCount = 3
        let helixR: Float = 0.04
        let zR: Float = 0.04
        let turns: Float = 4
        let ropeRadius: Float = 0.038

        var ropes: [LevelDefinition.Rope] = []
        var allParticles: [[LevelDefinition.Vec2]] = []
        var colorIdx = 0

        for (cl, startHole, endHole) in [(clA, 4, 5), (clB, 0, 9)] {
            var cumLen: [Float] = [0]
            for i in 1..<cl.count {
                let dx = cl[i].0-cl[i-1].0, dy = cl[i].1-cl[i-1].1, dz = cl[i].2-cl[i-1].2
                cumLen.append(cumLen[i-1] + sqrt(dx*dx+dy*dy+dz*dz))
            }
            let totalLen = cumLen.last!

            for k in 0..<strandCount {
                let phase = 2 * Float.pi * Float(k) / Float(strandCount)

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

                var particles: [LevelDefinition.Vec2] = []
                for i in 0..<P {
                    let t = Float(i) / Float(P - 1)
                    let dist = totalLen * t

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

                    let next = min(seg+1, cl.count-1), prev = max(seg-1, 0)
                    let tx = cl[next].0 - cl[prev].0
                    let ty = cl[next].1 - cl[prev].1
                    let tLen = sqrt(tx*tx + ty*ty)
                    let nx: Float, ny: Float
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

    // MARK: - Waypoint resampling helper

    static func resampleWaypoints(_ allWaypoints: [[(Float, Float, Float)]], P: Int) -> [[LevelDefinition.Vec2]] {
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
        return allParticles
    }
}
