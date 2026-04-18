import Foundation
import simd

// MARK: - Particle-based structured levels & showcase (experimental)

extension LevelGenerator {

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

    enum StructurePrimitive {
        case helix(HelixConfig)
        case braid(BraidConfig)
        case centralKnot(CentralKnotConfig)
        case vortex(VortexConfig)
        case weave(WeaveConfig)
    }

    struct HelixConfig {
        let center: SIMD2<Float>
        let axis: SIMD2<Float>
        let length: Float
        let strandCount: Int
        let turns: Float
        let helixRadius: Float
        let zRadius: Float
    }

    struct BraidConfig {
        let center: SIMD2<Float>
        let axis: SIMD2<Float>
        let length: Float
        let strandCount: Int
        let crossings: Int
        let width: Float
        let zRadius: Float
    }

    struct CentralKnotConfig {
        let center: SIMD2<Float>
        let outerRadius: Float
        let innerRadius: Float
        let ropeCount: Int
        let loopDepth: Float
        let zRadius: Float
    }

    struct VortexConfig {
        let center: SIMD2<Float>
        let ropeCount: Int
        let armLength: Float
        let knotRadius: Float
        let orbitRadius: Float
        let turns: Float
        let zRadius: Float
    }

    struct WeaveConfig {
        let center: SIMD2<Float>
        let width: Float
        let height: Float
        let hCount: Int
        let vCount: Int
        let zRadius: Float
    }

    static func buildStructuredLevel(
        levelId: Int, particleCount: Int = 55, ropeRadius: Float = 0.038,
        holeRadius: Float = 0.08, structures: [StructurePrimitive],
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
            weights: nil, targets: nil, rails: nil, carts: nil, stations: nil,
            ropeParticles: allParticles
        )
    }

    static func addDecorativeHoles(
        existing: inout [LevelDefinition.Vec2], center: SIMD2<Float> = .zero,
        rings: [(radius: Float, count: Int)], minSpacing: Float = 0.20
    ) {
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

    static func smoothEnvelope(t: Float, convergeFrac: Float) -> Float {
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

    static func generateStructureShowcase(levelId: Int, particleCount: Int = 55) -> LevelDefinition {
        let P = particleCount
        switch levelId {
        case 3100: return showcase3100(P)
        case 3101: return showcase3101(P)
        case 3102: return showcase3102(P)
        case 3103: return showcase3103(P)
        case 3104: return showcase3104(P)
        case 3105: return showcase3105(P)
        case 3106: return showcase3106(P)
        case 3107: return showcase3107(P)
        case 3108: return showcase3108(P)
        case 3109: return showcase3109(P)
        case 3110: return showcase3110(P)
        case 3111: return showcase3111(P)
        case 3112: return showcase3112(P)
        case 3113: return showcase3113(P)
        case 3114: return showcase3114(P)
        case 3115: return showcase3115(P)
        case 3116: return showcase3116(P)
        case 3117: return showcase3117(P)
        default:   return showcase3100(P)
        }
    }
}
