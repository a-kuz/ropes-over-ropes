import simd

struct Camera {
    var center: SIMD3<Float> = .zero
    var distance: Float = 2.8
    var orthoHalfHeight: Float = 2.05
    var tiltAngle: Float = 0.25

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

