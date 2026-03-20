import simd

struct FrameUniforms {
    var viewProj: simd_float4x4
    var invViewProj: simd_float4x4
    var lightViewProj: simd_float4x4
    var lightDirIntensity: SIMD4<Float>
    var ambientColor: SIMD4<Float>
    var cameraPos: SIMD4<Float>
    var orthoHalfSizeShadowBias: SIMD4<Float>
    var shadowInvSizeUnused: SIMD4<Float>
    var timeDrag: SIMD4<Float>
    var woodBoundsMin: SIMD4<Float>
    var woodBoundsMax: SIMD4<Float>
    var holeTint: SIMD4<Float>
    var visualParams: SIMD4<Float>
    var lightingParams: SIMD4<Float>
    var tableParams: SIMD4<Float>
    var tableParams2: SIMD4<Float>
    var ropeMatParams: SIMD4<Float>
    var ropeMatParams2: SIMD4<Float>
    var ropeMatParams3: SIMD4<Float>
    var cartoonParams: SIMD4<Float>
    var wormParams1: SIMD4<Float>
    var wormParams2: SIMD4<Float>
    var wormParams3: SIMD4<Float>
    var wormParams4: SIMD4<Float>
    var ropeMatParams4: SIMD4<Float>
    var ropeMatParams5: SIMD4<Float>
    var ropeMatParams6: SIMD4<Float>
    var ropeMatParams7: SIMD4<Float>
    var ropeMatParams8: SIMD4<Float>
}

struct PostParams {
    var exposure: Float
    var bloomStrength: Float
    var cartoonEdgeStrength: Float = 0.88
    var cartoonMode: Float = 0
    var cartoonEdgeSmooth: Float = 0.5
    var _pad0: Float = 0
    var _pad1: Float = 0
    var _pad2: Float = 0
}

struct BakeWoodParams {
    var worldMin: SIMD2<Float>
    var worldMax: SIMD2<Float>
    var seed: Float
    var brightness: Float
    var patternScale: Float
    var _pad0: Float = 0
    var _pad1: Float = 0
}

struct BakeBoardWoodVolumeParams {
    var worldMin: SIMD4<Float>
    var worldMax: SIMD4<Float>
    var seed: Float
    var brightness: Float
    var _pad0: SIMD2<Float> = .zero
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

struct BoardVertex {
    var position: SIMD3<Float>
    var normal: SIMD3<Float>
    var worldXY: SIMD2<Float>
}

struct Debug2DVertexData {
    var position: SIMD2<Float>
    var color: SIMD4<Float>
}

