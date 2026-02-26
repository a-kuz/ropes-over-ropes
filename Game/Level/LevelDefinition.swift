import Foundation
import simd

struct LevelDefinition: Codable {
    struct Vec2: Codable {
        let xPosition: Float
        let yPosition: Float

        enum CodingKeys: String, CodingKey {
            case xPosition = "x"
            case yPosition = "y"
        }

        var simd: SIMD2<Float> { SIMD2<Float>(xPosition, yPosition) }
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

    let id: Int
    let holeRadius: Float
    let particlesPerRope: Int
    let holes: [Vec2]
    let ropes: [Rope]
    let hooks: [Hook]?
    let actions: [Action]?
}

