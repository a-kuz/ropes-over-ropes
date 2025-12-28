import simd

struct FrameUniforms {
    var viewProj: simd_float4x4
    var lightDir_intensity: SIMD4<Float>
    var ambientColor: SIMD4<Float>
    var cameraPos: SIMD4<Float>
}

struct HoleInstance {
    var position_radius: SIMD4<Float>
}

struct RopeVertex {
    var position: SIMD3<Float>
    var normal: SIMD3<Float>
    var color: SIMD3<Float>
    var uv: SIMD2<Float>
}

