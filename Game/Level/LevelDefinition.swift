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
        let width: Float
        let height: Float

        enum CodingKeys: String, CodingKey {
            case startHole
            case endHole
            case color
            case width
            case height
        }

        init(startHole: Int, endHole: Int, color: Color, width: Float, height: Float) {
            self.startHole = startHole
            self.endHole = endHole
            self.color = color
            self.width = width
            self.height = height
        }

        init(from decoder: Decoder) throws {
            let container = try decoder.container(keyedBy: CodingKeys.self)
            startHole = try container.decode(Int.self, forKey: .startHole)
            endHole = try container.decode(Int.self, forKey: .endHole)
            color = try container.decode(Color.self, forKey: .color)
            width = try container.decodeIfPresent(Float.self, forKey: .width) ?? 0.085
            height = try container.decodeIfPresent(Float.self, forKey: .height) ?? 0.030
        }
    }

    let id: Int
    let holeRadius: Float
    let particlesPerRope: Int
    let holes: [Vec2]
    let ropes: [Rope]
}

