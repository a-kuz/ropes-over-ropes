import simd

struct FrameUniforms {
    var viewProj: simd_float4x4
    var lightViewProj: simd_float4x4
    var lightDirIntensity: SIMD4<Float>
    var ambientColor: SIMD4<Float>
    var cameraPos: SIMD4<Float>
    var orthoHalfSizeShadowBias: SIMD4<Float>
    var shadowInvSizeUnused: SIMD4<Float>
    var timeDrag: SIMD4<Float>
    var levelSeed: SIMD4<Float>
}

struct HoleInstance {
    var positionRadius: SIMD4<Float>
}

struct RopeVertex {
    var position: SIMD3<Float>
    var normal: SIMD3<Float>
    var color: SIMD3<Float>
    var texCoord: SIMD2<Float>
    var params: SIMD4<Float>
}

struct HoleVertex {
    var position: SIMD3<Float>
    var normal: SIMD3<Float>
}

