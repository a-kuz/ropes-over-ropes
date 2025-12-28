import Foundation
import simd

struct LevelDefinition: Codable {
    struct Vec2: Codable {
        let posX: Float
        let posY: Float

        enum CodingKeys: String, CodingKey {
            case posX = "x"
            case posY = "y"
        }

        var simd: SIMD2<Float> { SIMD2<Float>(posX, posY) }
    }

    struct Color: Codable {
        let red: Float
        let green: Float
        let blue: Float

        enum CodingKeys: String, CodingKey {
            case red = "r"
            case green = "g"
            case blue = "b"
        }

        var simd: SIMD3<Float> { SIMD3<Float>(red, green, blue) }
    }

    struct Rope: Codable {
        let startHole: Int
        let endHole: Int
        let color: Color
    }

    let id: Int
    let holeRadius: Float
    let particlesPerRope: Int
    let holes: [Vec2]
    let ropes: [Rope]
}

