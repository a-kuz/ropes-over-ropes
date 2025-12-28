import Metal
import simd

extension Renderer {
    struct HoleLayout {
        let positions: [SIMD2<Float>]
        let radius: Float
    }

    static func makeHoleLayout() -> HoleLayout {
        let gridColumns = 4
        let gridRows = 5
        let spacingX: Float = 0.55
        let spacingY: Float = 0.45
        let origin = SIMD2<Float>(-0.82, -0.86)
        let radius: Float = 0.105

        var positions: [SIMD2<Float>] = []
        positions.reserveCapacity(gridColumns * gridRows)
        for rowIndex in 0..<gridRows {
            for columnIndex in 0..<gridColumns {
                let offset = SIMD2<Float>(Float(columnIndex) * spacingX, Float(rowIndex) * spacingY)
                positions.append(origin + offset)
            }
        }

        return HoleLayout(positions: positions, radius: radius)
    }

    static func makeHoleInstances(device: MTLDevice, positions: [SIMD2<Float>], radius: Float) -> MTLBuffer {
        var holes: [HoleInstance] = []
        holes.reserveCapacity(positions.count)
        for position in positions {
            holes.append(HoleInstance(position_radius: SIMD4<Float>(position.x, position.y, 0, radius)))
        }

        guard let buffer = device.makeBuffer(
            bytes: holes,
            length: holes.count * MemoryLayout<HoleInstance>.stride,
            options: [.storageModeShared]
        ) else {
            fatalError("Failed to create hole instance buffer")
        }

        return buffer
    }
}

