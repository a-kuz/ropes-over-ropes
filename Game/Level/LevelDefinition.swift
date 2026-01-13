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
    
    struct Hook: Codable {
        let ropeA: Int
        let ropeB: Int
        let N: Int
        let ropeAStartIsOver: Bool
    }

    let id: Int
    let holeRadius: Float
    let particlesPerRope: Int
    let holes: [Vec2]
    let ropes: [Rope]
    let hooks: [Hook]?
    
    func validateHooks() -> [String] {
        var errors: [String] = []
        guard let hooks = hooks else { return errors }
        
        let holePositions = holes.map { $0.simd }
        
        for (i, hook) in hooks.enumerated() {
            guard hook.ropeA >= 0 && hook.ropeA < ropes.count else {
                errors.append("Hook[\(i)]: ropeA=\(hook.ropeA) out of range")
                continue
            }
            guard hook.ropeB >= 0 && hook.ropeB < ropes.count else {
                errors.append("Hook[\(i)]: ropeB=\(hook.ropeB) out of range")
                continue
            }
            guard hook.ropeA != hook.ropeB else {
                errors.append("Hook[\(i)]: ropeA == ropeB")
                continue
            }
            guard hook.N >= 1 && hook.N <= 10 else {
                errors.append("Hook[\(i)]: N=\(hook.N) out of range [1..10]")
                continue
            }
            
            let ropeA = ropes[hook.ropeA]
            let ropeB = ropes[hook.ropeB]
            
            guard let A1 = holePositions[safe: ropeA.startHole],
                  let A2 = holePositions[safe: ropeA.endHole],
                  let B1 = holePositions[safe: ropeB.startHole],
                  let B2 = holePositions[safe: ropeB.endHole] else {
                errors.append("Hook[\(i)]: invalid hole indices")
                continue
            }
            
            let segmentsCross = segmentsIntersect(A1, A2, B1, B2)
            let shouldCross = (hook.N % 2 == 1)
            
            if segmentsCross != shouldCross {
                if shouldCross {
                    errors.append("Hook[\(i)]: N=\(hook.N) (odd) requires A1-A2 and B1-B2 to intersect, but they don't")
                } else {
                    errors.append("Hook[\(i)]: N=\(hook.N) (even) requires A1-A2 and B1-B2 to NOT intersect, but they do")
                }
            }
        }
        
        return errors
    }
    
    private func segmentsIntersect(_ a0: SIMD2<Float>, _ a1: SIMD2<Float>, _ b0: SIMD2<Float>, _ b1: SIMD2<Float>) -> Bool {
        let d1 = a1 - a0
        let d2 = b1 - b0
        let cross = d1.x * d2.y - d1.y * d2.x
        if abs(cross) < 1e-9 { return false }
        let d = b0 - a0
        let t = (d.x * d2.y - d.y * d2.x) / cross
        let u = (d.x * d1.y - d.y * d1.x) / cross
        return t >= 0 && t <= 1 && u >= 0 && u <= 1
    }
}

