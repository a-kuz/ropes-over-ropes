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

    struct Rope: Codable {
        let startHole: Int
        let endHole: Int
        let color: Color
        let radius: Float

        enum CodingKeys: String, CodingKey {
            case startHole
            case endHole
            case color
            case radius
        }

        init(startHole: Int, endHole: Int, color: Color, radius: Float) {
            self.startHole = startHole
            self.endHole = endHole
            self.color = color
            self.radius = radius
        }

        init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            startHole = try container.decode(Int.self, forKey: .startHole)
            endHole = try container.decode(Int.self, forKey: .endHole)
            color = try container.decode(Color.self, forKey: .color)
            if let radius = try? container.decode(Float.self, forKey: .radius) {
                self.radius = radius
            } else {
                let allKeys = try decoder.container(keyedBy: AnyCodingKey.self)
                let width = try allKeys.decodeIfPresent(Float.self, forKey: AnyCodingKey(stringValue: "width")!) ?? 0.085
                let height = try allKeys.decodeIfPresent(Float.self, forKey: AnyCodingKey(stringValue: "height")!) ?? 0.030
                self.radius = max(width, height) * 0.5
            }
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

    let id: Int
    let holeRadius: Float
    let particlesPerRope: Int
    let holes: [Vec2]
    let ropes: [Rope]
}

