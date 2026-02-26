import simd

struct Camera {
    var center: SIMD3<Float> = .zero
    var distance: Float = 2.8
    var orthoHalfHeight: Float = 2.05
    var tiltAngle: Float = 0.0

    mutating func fitToHoles(_ holes: [SIMD2<Float>], holeRadius: Float, aspect: Float) {
        guard !holes.isEmpty else { return }
        var minX: Float = .greatestFiniteMagnitude
        var maxX: Float = -.greatestFiniteMagnitude
        var minY: Float = .greatestFiniteMagnitude
        var maxY: Float = -.greatestFiniteMagnitude
        for h in holes {
            minX = min(minX, h.x)
            maxX = max(maxX, h.x)
            minY = min(minY, h.y)
            maxY = max(maxY, h.y)
        }
        let margin = holeRadius * 2.5
        let contentW = (maxX - minX) + margin * 2
        let contentH = (maxY - minY) + margin * 2
        let centerX = (minX + maxX) * 0.5
        let centerY = (minY + maxY) * 0.5

        let halfHFromHeight = contentH * 0.5
        let halfHFromWidth = (contentW * 0.5) / max(aspect, 0.01)
        orthoHalfHeight = max(halfHFromHeight, halfHFromWidth) * 1.2
        center = SIMD3<Float>(centerX, centerY, 0)
    }

    func viewProj(aspect: Float) -> simd_float4x4 {
        let yOffset = distance * sin(tiltAngle)
        let zOffset = distance * cos(tiltAngle)
        let eye = center + SIMD3<Float>(0, yOffset, zOffset)
        let view = simd_float4x4.lookAt(eye: eye, center: center, up: SIMD3<Float>(0, 1, 0))
        let halfH = orthoHalfHeight
        let halfW = orthoHalfHeight * aspect
        let proj = simd_float4x4.ortho(left: -halfW, right: halfW, bottom: -halfH, top: halfH, near: 0.01, far: 10.0)
        return proj * view
    }
}

extension simd_float4x4 {
    static func ortho(left: Float, right: Float, bottom: Float, top: Float, near: Float, far: Float) -> simd_float4x4 {
        let rl = right - left
        let tb = top - bottom
        let fn = far - near

        return simd_float4x4(
            SIMD4<Float>(2.0 / rl, 0, 0, 0),
            SIMD4<Float>(0, 2.0 / tb, 0, 0),
            SIMD4<Float>(0, 0, -1.0 / fn, 0),
            SIMD4<Float>(-(right + left) / rl, -(top + bottom) / tb, -near / fn, 1)
        )
    }

    static func lookAt(eye: SIMD3<Float>, center: SIMD3<Float>, up: SIMD3<Float>) -> simd_float4x4 {
        let f = simd_normalize(center - eye)
        let s = simd_normalize(simd_cross(f, up))
        let u = simd_cross(s, f)

        let m = simd_float4x4(
            SIMD4<Float>(s.x, u.x, -f.x, 0),
            SIMD4<Float>(s.y, u.y, -f.y, 0),
            SIMD4<Float>(s.z, u.z, -f.z, 0),
            SIMD4<Float>(-simd_dot(s, eye), -simd_dot(u, eye), simd_dot(f, eye), 1)
        )
        return m
    }
}
