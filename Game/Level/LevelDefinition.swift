import Foundation
import simd

struct LevelDefinition: Codable {
    struct Vec2: Codable {
        let xPosition: Float
        let yPosition: Float
        let zPosition: Float

        enum CodingKeys: String, CodingKey {
            case xPosition = "x"
            case yPosition = "y"
            case zPosition = "z"
        }

        init(xPosition: Float, yPosition: Float, zPosition: Float = 0) {
            self.xPosition = xPosition
            self.yPosition = yPosition
            self.zPosition = zPosition
        }

        init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            xPosition = try container.decode(Float.self, forKey: .xPosition)
            yPosition = try container.decode(Float.self, forKey: .yPosition)
            zPosition = try container.decodeIfPresent(Float.self, forKey: .zPosition) ?? 0
        }

        var simd: SIMD2<Float> { SIMD2<Float>(xPosition, yPosition) }
        var simd3: SIMD3<Float> { SIMD3<Float>(xPosition, yPosition, zPosition) }
    }

    struct Board: Codable {
        let centerX: Float
        let centerY: Float
        let width: Float
        let height: Float
        let elevation: Float

        enum CodingKeys: String, CodingKey {
            case centerX = "cx"
            case centerY = "cy"
            case width = "w"
            case height = "h"
            case elevation = "z"
        }

        init(centerX: Float, centerY: Float, width: Float, height: Float, elevation: Float) {
            self.centerX = centerX
            self.centerY = centerY
            self.width = width
            self.height = height
            self.elevation = elevation
        }

        init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            centerX = try container.decode(Float.self, forKey: .centerX)
            centerY = try container.decode(Float.self, forKey: .centerY)
            width = try container.decode(Float.self, forKey: .width)
            height = try container.decode(Float.self, forKey: .height)
            elevation = try container.decodeIfPresent(Float.self, forKey: .elevation) ?? 0
        }
    }

    struct Color: Codable {
        let redChannel: Float
        let greenChannel: Float
        let blueChannel: Float

        enum CodingKeys: String, CodingKey {
            case redChannel = "r"
            case greenChannel = "g"
            case blueChannel = "b"
        }

        var simd: SIMD3<Float> { SIMD3<Float>(redChannel, greenChannel, blueChannel) }
    }

    struct CrossSectionDef: Codable {
        let type: String
        let width: Float?
        let height: Float?

        func toCrossSection(fallbackRadius: Float) -> CrossSection {
            switch type {
            case "rectangular":
                let w = width ?? fallbackRadius * 2
                let h = height ?? fallbackRadius * 0.7
                return .rectangular(width: w, height: h)
            default:
                return .circular(radius: fallbackRadius)
            }
        }
    }

    struct Rope: Codable {
        let startHole: Int
        let endHole: Int
        let color: Color
        let radius: Float
        let crossSectionDef: CrossSectionDef?

        enum CodingKeys: String, CodingKey {
            case startHole
            case endHole
            case color
            case radius
            case crossSectionDef = "crossSection"
        }

        init(startHole: Int, endHole: Int, color: Color, radius: Float, crossSectionDef: CrossSectionDef? = nil) {
            self.startHole = startHole
            self.endHole = endHole
            self.color = color
            self.radius = radius
            self.crossSectionDef = crossSectionDef
        }

        init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            startHole = try container.decode(Int.self, forKey: .startHole)
            endHole = try container.decode(Int.self, forKey: .endHole)
            color = try container.decode(Color.self, forKey: .color)
            crossSectionDef = try container.decodeIfPresent(CrossSectionDef.self, forKey: .crossSectionDef)
            if let radius = try? container.decode(Float.self, forKey: .radius) {
                self.radius = radius
            } else {
                let allKeys = try decoder.container(keyedBy: AnyCodingKey.self)
                let width = try allKeys.decodeIfPresent(Float.self, forKey: AnyCodingKey(stringValue: "width")!) ?? 0.085
                let height = try allKeys.decodeIfPresent(Float.self, forKey: AnyCodingKey(stringValue: "height")!) ?? 0.030
                self.radius = max(width, height) * 0.5
            }
        }
        
        var crossSection: CrossSection {
            crossSectionDef?.toCrossSection(fallbackRadius: radius) ?? .circular(radius: radius)
        }

        private struct AnyCodingKey: CodingKey {
            var stringValue: String
            var intValue: Int?
            
            init?(stringValue: String) {
                self.stringValue = stringValue
            }
            
            init?(intValue: Int) {
                self.intValue = intValue
                self.stringValue = "\(intValue)"
            }
        }
    }

    struct HookRopeRef: Codable {
        let fromType: String   // "hole" or "hook"
        let index: Int         // rope index (for "hole") or hook index (for "hook")
        let hookIndex: Int?    // which side of the referenced hook (0=ropeA, 1=ropeB)
    }

    struct Hook: Codable {
        let ropeA: HookRopeRef
        let ropeB: HookRopeRef
        let N: Int
        let ropeAStartIsOver: Bool
    }

    struct Action: Codable {
        let type: String       // "pin" or "drag"
        let ropeIndex: Int
        let endIndex: Int      // 0 = start, 1 = end
        let holeIndex: Int
    }

    struct WeightDef: Codable {
        let x: Float
        let y: Float
        let mass: Float?
        let radius: Float?

        var position: SIMD2<Float> { SIMD2<Float>(x, y) }
    }

    struct TargetDef: Codable {
        let x: Float
        let y: Float
        let radius: Float?
        let weightIndex: Int

        var position: SIMD2<Float> { SIMD2<Float>(x, y) }
    }

    struct RailDef: Codable {
        let points: [Vec2]
    }

    struct CartDef: Codable {
        let railIndex: Int
        let startT: Float
        let radius: Float?
        let mass: Float?
    }

    struct StationDef: Codable {
        let railIndex: Int
        let t: Float
        let radius: Float?
        let cartIndex: Int
    }

    /// "untangle" (default), "tension", "rail", or "braid"
    let mode: String?

    let id: Int
    let holeRadius: Float
    let particlesPerRope: Int
    let holes: [Vec2]
    let ropes: [Rope]
    let hooks: [Hook]?
    let actions: [Action]?
    let boards: [Board]?
    let weights: [WeightDef]?
    let targets: [TargetDef]?
    let rails: [RailDef]?
    let carts: [CartDef]?
    let stations: [StationDef]?

    /// Braid mode: target bottom hole index for each rope (rope i should end at braidTargets[i])
    let braidTargets: [Int]?
    /// Braid mode: minimum total 2D crossing count for a valid braid
    let braidMinCrossings: Int?

    /// Explicit particle positions per rope (alternative to action-based initialization).
    /// ropeParticles[ropeIndex] = array of Vec2 (with z) for each particle along the rope.
    let ropeParticles: [[Vec2]]?

    var isTensionMode: Bool { mode == "tension" }
    var isRailMode: Bool { mode == "rail" }
    var isBraidMode: Bool { mode == "braid" }

    init(mode: String?, id: Int, holeRadius: Float, particlesPerRope: Int,
         holes: [Vec2], ropes: [Rope], hooks: [Hook]?, actions: [Action]?,
         boards: [Board]?, weights: [WeightDef]?, targets: [TargetDef]?,
         rails: [RailDef]?, carts: [CartDef]?, stations: [StationDef]?,
         braidTargets: [Int]? = nil, braidMinCrossings: Int? = nil,
         ropeParticles: [[Vec2]]? = nil) {
        self.mode = mode
        self.id = id
        self.holeRadius = holeRadius
        self.particlesPerRope = particlesPerRope
        self.holes = holes
        self.ropes = ropes
        self.hooks = hooks
        self.actions = actions
        self.boards = boards
        self.weights = weights
        self.targets = targets
        self.rails = rails
        self.carts = carts
        self.stations = stations
        self.braidTargets = braidTargets
        self.braidMinCrossings = braidMinCrossings
        self.ropeParticles = ropeParticles
    }
}

