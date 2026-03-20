import simd

/// All visual / shader parameters extracted from Renderer.
/// Pure value type — no side effects, no Metal references.
struct RendererShaderParams {
    // MARK: - Lighting
    var exposure: Float = 0.5
    var lightIntensity: Float = 0.40238875150680542
    var lightDir: SIMD3<Float> = SIMD3<Float>(-0.029445827007293701, -0.22127896547317505, 0.87485504150390625)
    var ambient: Float = 0.10394008457660675
    var shadowBias: Float = 0.0013682411517947912
    var shadowDarkness: Float = 0.15453849732875824
    var lightSize: Float = 0.073253527283668518
    var shadowsEnabled: Bool = true
    var shadowType: Int = 2
    var pcssPenumbraScale: Float = 80.0
    var shadowDebugMode: Int = 0

    // MARK: - Bloom
    var bloomStrength: Float = 0
    var bloomEnabled: Bool = true

    // MARK: - Cartoon
    var cartoonShaderEnabled: Bool = false
    var cartoonExposure: Float = 1.3291559219360352
    var cartoonBloom: Float = 0
    var cartoonEdgeStrength: Float = 1
    var cartoonLevels: Int = 2
    var cartoonShadowBright: Float = 0.38
    var cartoonWrap: Float = 0.15
    var cartoonEdgeSmooth: Float = 0.5

    // MARK: - Rope material
    var ropeMatte: Float = 0.33392396569252014
    var ropeGloss: Float = 0.69268685579299927
    var ropeDiffuseWrap: Float = 0.0080702323466539383
    var ropeSubsurface: Float = 0.33765289187431335
    var ropeEdgeLight: Float = 0.41452157497406006
    var ropeSaturation: Float = 1.35732102394104
    var ropeMicroBump: Float = 0.142847940325737
    var ropeBumpScale: Float = 3.0
    var ropeContactAO: Float = 1
    var ropeLiftGlow: Float = 0.97152864933013916
    var ropeStretchGloss: Float = 0.7
    var ropeStretchSpec: Float = 1.0
    var ropeEnvReflect: Float = 0.15
    var ropeEnvSpread: Float = 0.15
    var ropeEnvDebug: Bool = false
    var ropeOpacity: Float = 1.0
    var ropeFlatNormals: Bool = false
    var ropeCracksEnabled: Bool = false
    var ropeCrackAmount: Float = 0.45
    var ropeCrackWidth: Float = 0.16
    var ropeCrackDepth: Float = 0.5
    var ropeSeamEnabled: Bool = false
    var ropeSeamWidth: Float = 0.06
    var ropeSeamDepth: Float = 0.45
    var ropeSeamDarkness: Float = 1.4
    var ropeSeamHighlight: Float = 0.35
    var ropeSeamCrackAmount: Float = 0.45
    var ropeSeamCrackScale: Float = 18.0
    var ropeSeamRandomize: Bool = true
    var ropeSeamPosition: Float = 0.5

    // MARK: - Table / wood
    var tableStyle: Int = 0
    var tableColor1: SIMD3<Float> = SIMD3<Float>(0.079999998211860657, 0.090000003576278687, 0.12999999523162842)
    var tableColor2: SIMD3<Float> = SIMD3<Float>(0.11999999731779099, 0.12999999523162842, 0.20000000298023224)
    var woodSeed: Float = 0.045950364321470261
    var woodBrightness: Float = 1.0
    var woodPatternScale: Float = 1.7154961824417114

    // MARK: - Hole appearance
    var holeTint: SIMD4<Float> = SIMD4<Float>(1, 0.90870898962020874, 1, 1)

    // MARK: - Accelerometer light tilt
    var tiltStrength: Float = 0.45

    // MARK: - Rope caps
    var capRadiusScale: Float = 0.90614998340606689
    var capSegments: Int = 12
    var capRings: Int = 6
    var capDarken: Float = 0

    // MARK: - Chain mode
    var chainMode: Bool = false
    var chainLinkLength: Float = 2.8
    var chainLinkThickness: Float = 0.35
    var chainLinkWidth: Float = 0.85

    // MARK: - Worm mode
    var wormMode: Bool = false
    var wormSegFreq: Float = 28.0
    var wormSegBulge: Float = 0.12
    var wormThickness: Float = 1.35
    var wormTaperLen: Float = 0.12
    var wormGrooveDepth: Float = 0.35
    var wormBellyBright: Float = 1.15
    var wormBackDark: Float = 0.7
    var wormSkinNoise: Float = 0.08
    var wormSSS: Float = 0.25
    var wormRoughness: Float = 0.25
    var wormSpecular: Float = 0.8
    var wormRimStrength: Float = 0.08
    var wormEyeSize: Float = 0.015
    var wormPulseSpeed: Float = 2.5
    var wormPulseAmp: Float = 0.02
    var wormCrawlSpeed: Float = 3.5
    var wormCrawlAmp: Float = 0.012
    var wormSideAmp: Float = 0.008
}
