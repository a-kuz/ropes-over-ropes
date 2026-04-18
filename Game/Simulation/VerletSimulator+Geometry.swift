import simd

extension VerletSimulator {
    func holePosition3D(_ holeIndex: Int) -> SIMD3<Float> {
        guard holePositions.indices.contains(holeIndex) else { return .zero }
        let p = holePositions[holeIndex]
        let elev = holeElevations.indices.contains(holeIndex) ? holeElevations[holeIndex] : 0
        if padMode {
            // Rope endpoint sits on top of the pad surface
            // padHeight must match rendered geometry: buildPad height=0.18 scaled by instance radius
            let padHeight = holeRadius * holeRadiusScale * self.padHeight
            return SIMD3<Float>(p.x, p.y, elev + padHeight)
        }
        return SIMD3<Float>(p.x, p.y, elev - holeDepth)
    }

    func holeSurfaceZ(_ holeIndex: Int) -> Float {
        guard holeElevations.indices.contains(holeIndex) else { return 0 }
        return holeElevations[holeIndex]
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
