import simd

extension VerletSimulator {
    func holePosition3D(_ holeIndex: Int) -> SIMD3<Float> {
        guard holePositions.indices.contains(holeIndex) else { return .zero }
        let p = holePositions[holeIndex]
        let elev = holeElevations.indices.contains(holeIndex) ? holeElevations[holeIndex] : 0
        return SIMD3<Float>(p.x, p.y, elev - holeDepth)
    }

    func holeSurfaceZ(_ holeIndex: Int) -> Float {
        guard holeElevations.indices.contains(holeIndex) else { return 0 }
        return holeElevations[holeIndex]
    }

    @inline(__always)
    func isInsideAnyHole(x: Float, y: Float) -> Bool {
        let r2 = holeRadius * holeRadius
        for hp in holePositions {
            let dx = x - hp.x
            let dy = y - hp.y
            if dx * dx + dy * dy < r2 { return true }
        }
        return false
    }

    @inline(__always)
    func boardSurfaceZ(x: Float, y: Float) -> Float {
        var maxZ: Float = 0
        for b in boards {
            let halfW = b.width * 0.5
            let halfH = b.height * 0.5
            if x >= b.centerX - halfW && x <= b.centerX + halfW &&
               y >= b.centerY - halfH && y <= b.centerY + halfH {
                maxZ = max(maxZ, b.elevation)
            }
        }
        return maxZ
    }
}
