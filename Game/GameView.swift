#if os(iOS)
import UIKit
#elseif os(macOS)
import AppKit
#endif
import MetalKit
import SwiftUI

@MainActor
class GameController: ObservableObject {
    weak var renderer: Renderer?

    struct Preset: Identifiable, Equatable {
        let id = UUID()
        let name: String
        // Physics
        let gravity: Float?
        let damping: Float?
        let ropeTension: Float?
        let bendCompliance: Float?
        let stretchThinning: Float?
        let particleCount: Float?
        let constraintIterations: Float?
        let settleSteps: Float?
        let bendVelocityCoupling: Float?
        let dragHeight: Float?
        let liftHeight: Float?
        let frictionSoundEnabled: Bool?
        // Visuals
        let profileSegments: Float?
        let holeRadiusScale: Float?
        let holeSegments: Float?
        let ropeRadiusScale: Float?
        let exposure: Float?
        let lightIntensity: Float?
        let lightDirX: Float?
        let lightDirY: Float?
        let lightDirZ: Float?
        let shadowType: ShadowType?
        let ambient: Float?
        let shadowBias: Float?
        let shadowDarkness: Float?
        let lightSize: Float?
        let shadowsEnabled: Bool?
        let bloomStrength: Float?
        let cartoonShaderEnabled: Bool?
        let cartoonExposure: Float?
        let cartoonBloom: Float?
        let cartoonEdgeStrength: Float?
        let cartoonLevels: Float?
        let tableStyle: TableStyle?
        let tableColor1R: Float?
        let tableColor1G: Float?
        let tableColor1B: Float?
        let tableColor2R: Float?
        let tableColor2G: Float?
        let tableColor2B: Float?
        let woodSeed: Float?
        let woodBrightness: Float?
        let woodPatternScale: Float?
        let capRadiusScale: Float?
        let capSegments: Float?
        let capRings: Float?
        let capDarken: Float?
        let ropeMatte: Float?
        let ropeGloss: Float?
        let ropeDiffuseWrap: Float?
        let ropeSubsurface: Float?
        let ropeEdgeLight: Float?
        let ropeSaturation: Float?
        let ropeMicroBump: Float?
        let ropeBumpScale: Float?
        let ropeContactAO: Float?
        let ropeLiftGlow: Float?
        let ropeStretchGloss: Float?
        let ropeStretchSpec: Float?
        let ropeEnvReflect: Float?
        let ropeEnvSpread: Float?
        let ropeOpacity: Float?
        let wormMode: Bool?
        let chainMode: Bool?
        let squareCrossSection: Bool?
        let renderScale: Float?
        let zoomScale: Float?

        init(
            name: String,
            gravity: Float? = nil,
            damping: Float? = nil,
            ropeTension: Float? = nil,
            bendCompliance: Float? = nil,
            stretchThinning: Float? = nil,
            particleCount: Float? = nil,
            constraintIterations: Float? = nil,
            settleSteps: Float? = nil,
            bendVelocityCoupling: Float? = nil,
            dragHeight: Float? = nil,
            liftHeight: Float? = nil,
            frictionSoundEnabled: Bool? = nil,
            profileSegments: Float? = nil,
            holeRadiusScale: Float? = nil,
            holeSegments: Float? = nil,
            ropeRadiusScale: Float? = nil,
            exposure: Float? = nil,
            lightIntensity: Float? = nil,
            lightDirX: Float? = nil,
            lightDirY: Float? = nil,
            lightDirZ: Float? = nil,
            shadowType: ShadowType? = nil,
            ambient: Float? = nil,
            shadowBias: Float? = nil,
            shadowDarkness: Float? = nil,
            lightSize: Float? = nil,
            shadowsEnabled: Bool? = nil,
            bloomStrength: Float? = nil,
            cartoonShaderEnabled: Bool? = nil,
            cartoonExposure: Float? = nil,
            cartoonBloom: Float? = nil,
            cartoonEdgeStrength: Float? = nil,
            cartoonLevels: Float? = nil,
            tableStyle: TableStyle? = nil,
            tableColor1R: Float? = nil,
            tableColor1G: Float? = nil,
            tableColor1B: Float? = nil,
            tableColor2R: Float? = nil,
            tableColor2G: Float? = nil,
            tableColor2B: Float? = nil,
            woodSeed: Float? = nil,
            woodBrightness: Float? = nil,
            woodPatternScale: Float? = nil,
            capRadiusScale: Float? = nil,
            capSegments: Float? = nil,
            capRings: Float? = nil,
            capDarken: Float? = nil,
            ropeMatte: Float? = nil,
            ropeGloss: Float? = nil,
            ropeDiffuseWrap: Float? = nil,
            ropeSubsurface: Float? = nil,
            ropeEdgeLight: Float? = nil,
            ropeSaturation: Float? = nil,
            ropeMicroBump: Float? = nil,
            ropeBumpScale: Float? = nil,
            ropeContactAO: Float? = nil,
            ropeLiftGlow: Float? = nil,
            ropeStretchGloss: Float? = nil,
            ropeStretchSpec: Float? = nil,
            ropeEnvReflect: Float? = nil,
            ropeEnvSpread: Float? = nil,
            ropeOpacity: Float? = nil,
            wormMode: Bool? = nil,
            chainMode: Bool? = nil,
            squareCrossSection: Bool? = nil,
            renderScale: Float? = nil,
            zoomScale: Float? = nil
        ) {
            self.name = name
            self.gravity = gravity
            self.damping = damping
            self.ropeTension = ropeTension
            self.bendCompliance = bendCompliance
            self.stretchThinning = stretchThinning
            self.particleCount = particleCount
            self.constraintIterations = constraintIterations
            self.settleSteps = settleSteps
            self.bendVelocityCoupling = bendVelocityCoupling
            self.dragHeight = dragHeight
            self.liftHeight = liftHeight
            self.frictionSoundEnabled = frictionSoundEnabled
            self.profileSegments = profileSegments
            self.holeRadiusScale = holeRadiusScale
            self.holeSegments = holeSegments
            self.ropeRadiusScale = ropeRadiusScale
            self.exposure = exposure
            self.lightIntensity = lightIntensity
            self.lightDirX = lightDirX
            self.lightDirY = lightDirY
            self.lightDirZ = lightDirZ
            self.shadowType = shadowType
            self.ambient = ambient
            self.shadowBias = shadowBias
            self.shadowDarkness = shadowDarkness
            self.lightSize = lightSize
            self.shadowsEnabled = shadowsEnabled
            self.bloomStrength = bloomStrength
            self.cartoonShaderEnabled = cartoonShaderEnabled
            self.cartoonExposure = cartoonExposure
            self.cartoonBloom = cartoonBloom
            self.cartoonEdgeStrength = cartoonEdgeStrength
            self.cartoonLevels = cartoonLevels
            self.tableStyle = tableStyle
            self.tableColor1R = tableColor1R
            self.tableColor1G = tableColor1G
            self.tableColor1B = tableColor1B
            self.tableColor2R = tableColor2R
            self.tableColor2G = tableColor2G
            self.tableColor2B = tableColor2B
            self.woodSeed = woodSeed
            self.woodBrightness = woodBrightness
            self.woodPatternScale = woodPatternScale
            self.capRadiusScale = capRadiusScale
            self.capSegments = capSegments
            self.capRings = capRings
            self.capDarken = capDarken
            self.ropeMatte = ropeMatte
            self.ropeGloss = ropeGloss
            self.ropeDiffuseWrap = ropeDiffuseWrap
            self.ropeSubsurface = ropeSubsurface
            self.ropeEdgeLight = ropeEdgeLight
            self.ropeSaturation = ropeSaturation
            self.ropeMicroBump = ropeMicroBump
            self.ropeBumpScale = ropeBumpScale
            self.ropeContactAO = ropeContactAO
            self.ropeLiftGlow = ropeLiftGlow
            self.ropeStretchGloss = ropeStretchGloss
            self.ropeStretchSpec = ropeStretchSpec
            self.ropeEnvReflect = ropeEnvReflect
            self.ropeEnvSpread = ropeEnvSpread
            self.ropeOpacity = ropeOpacity
            self.wormMode = wormMode
            self.chainMode = chainMode
            self.squareCrossSection = squareCrossSection
            self.renderScale = renderScale
            self.zoomScale = zoomScale
        }
    }

    let presets: [Preset] = [
        Preset(
            name: "Default",
            gravity: Defaults.gravity,
            damping: Defaults.damping,
            ropeTension: Defaults.ropeTension,
            bendCompliance: Defaults.bendCompliance,
            stretchThinning: 0.039,
            cartoonShaderEnabled: false,
            ropeMatte: 0.69,
            ropeGloss: 1.17,
            ropeSubsurface: 0.303,
            ropeEdgeLight: 0.009,
            ropeSaturation: 0.979,
            ropeMicroBump: 1.5,
            ropeBumpScale: 7.37,
            ropeEnvReflect: 0.15,
            ropeOpacity: 1.0
        ),
        Preset(
            name: "Jelly",
            gravity: -0.1953949,
            damping: 0.9901605,
            ropeTension: 1.0,
            bendCompliance: 0.0074757283,
            stretchThinning: 0.1,
            particleCount: 69,
            constraintIterations: 9,
            settleSteps: 1,
            bendVelocityCoupling: 0,
            dragHeight: 0.85,
            liftHeight: 0.9,
            frictionSoundEnabled: false,
            profileSegments: 6,
            holeRadiusScale: 1.0,
            holeSegments: 19,
            ropeRadiusScale: 1.0,
            exposure: 0.6805594,
            lightIntensity: 1.0721008,
            lightDirX: 0.68410814,
            lightDirY: -0.79935646,
            lightDirZ: 0.6411401,
            shadowType: ShadowType.pcf,
            ambient: 0,
            shadowBias: 0.0009,
            shadowDarkness: 0.14629121,
            lightSize: 0.020752506,
            shadowsEnabled: true,
            bloomStrength: 0.35820743,
            cartoonShaderEnabled: false,
            cartoonExposure: 1.33,
            cartoonBloom: 0,
            cartoonEdgeStrength: 1,
            cartoonLevels: 2,
            tableStyle: TableStyle.wood,
            tableColor1R: 0,
            tableColor1G: 0,
            tableColor1B: 0,
            tableColor2R: 0.044013005,
            tableColor2G: 0.0649406,
            tableColor2B: 0.07229897,
            woodSeed: 0.44339293,
            woodBrightness: 0.8587378,
            woodPatternScale: 2.393613,
            capRadiusScale: 0.38643488,
            capSegments: 12,
            capRings: 2,
            capDarken: 0.116093226,
            ropeMatte: 0.05,
            ropeGloss: 2.5,
            ropeDiffuseWrap: 0.14709696,
            ropeSubsurface: 0.0,
            ropeEdgeLight: 0.17961414,
            ropeSaturation: 1.0922331,
            ropeMicroBump: 0.8786408,
            ropeBumpScale: 3.1103795,
            ropeContactAO: 0,
            ropeLiftGlow: 0,
            ropeStretchGloss: 0.014713832,
            ropeStretchSpec: 0.6748382,
            ropeEnvReflect: 0.8177177,
            ropeEnvSpread: 0.059403844,
            ropeOpacity: 0.7696207,
            wormMode: false,
            chainMode: false,
            squareCrossSection: false,
            renderScale: 0.6312977,
            zoomScale: 1.2382903
        ),
        Preset(
            name: "Steel Cable",
            gravity: -15.0,
            damping: 0.85,
            ropeTension: 1.0,
            bendCompliance: 0.0,
            stretchThinning: 0.0,
            ropeMatte: 0.4,
            ropeGloss: 1.5,
            ropeSubsurface: 0.0,
            ropeEdgeLight: 0.2,
            ropeSaturation: 0.0,
            ropeMicroBump: 0.5,
            ropeBumpScale: 2.0,
            ropeEnvReflect: 0.8,
            ropeOpacity: 1.0
        )
    ]

    func applyPreset(_ preset: Preset) {
        // Physics
        if let particleCount = preset.particleCount { self.particleCount = particleCount }
        if let gravity = preset.gravity { self.gravity = gravity }
        if let damping = preset.damping { self.damping = damping }
        if let constraintIterations = preset.constraintIterations { self.constraintIterations = constraintIterations }
        if let settleSteps = preset.settleSteps { self.settleSteps = settleSteps }
        if let bendCompliance = preset.bendCompliance { self.bendCompliance = bendCompliance }
        if let bendVelocityCoupling = preset.bendVelocityCoupling { self.bendVelocityCoupling = bendVelocityCoupling }
        if let dragHeight = preset.dragHeight { self.dragHeight = dragHeight }
        if let liftHeight = preset.liftHeight { self.liftHeight = liftHeight }
        if let ropeTension = preset.ropeTension { self.ropeTension = ropeTension }
        if let frictionSoundEnabled = preset.frictionSoundEnabled { self.frictionSoundEnabled = frictionSoundEnabled }
        if let profileSegments = preset.profileSegments { self.profileSegments = profileSegments }
        if let holeRadiusScale = preset.holeRadiusScale { self.holeRadiusScale = holeRadiusScale }
        if let holeSegments = preset.holeSegments { self.holeSegments = holeSegments }
        if let ropeRadiusScale = preset.ropeRadiusScale { self.ropeRadiusScale = ropeRadiusScale }
        if let stretchThinning = preset.stretchThinning { self.stretchThinning = stretchThinning }

        // Visuals
        if let exposure = preset.exposure { self.exposure = exposure }
        if let lightIntensity = preset.lightIntensity { self.lightIntensity = lightIntensity }
        if let lightDirX = preset.lightDirX { self.lightDirX = lightDirX }
        if let lightDirY = preset.lightDirY { self.lightDirY = lightDirY }
        if let lightDirZ = preset.lightDirZ { self.lightDirZ = lightDirZ }
        if let shadowType = preset.shadowType { self.shadowType = shadowType }
        if let ambient = preset.ambient { self.ambient = ambient }
        if let shadowBias = preset.shadowBias { self.shadowBias = shadowBias }
        if let shadowDarkness = preset.shadowDarkness { self.shadowDarkness = shadowDarkness }
        if let lightSize = preset.lightSize { self.lightSize = lightSize }
        if let shadowsEnabled = preset.shadowsEnabled { self.shadowsEnabled = shadowsEnabled }
        if let bloomStrength = preset.bloomStrength { self.bloomStrength = bloomStrength }
        if let cartoonShaderEnabled = preset.cartoonShaderEnabled { self.cartoonShaderEnabled = cartoonShaderEnabled }
        if let cartoonExposure = preset.cartoonExposure { self.cartoonExposure = cartoonExposure }
        if let cartoonBloom = preset.cartoonBloom { self.cartoonBloom = cartoonBloom }
        if let cartoonEdgeStrength = preset.cartoonEdgeStrength { self.cartoonEdgeStrength = cartoonEdgeStrength }
        if let cartoonLevels = preset.cartoonLevels { self.cartoonLevels = cartoonLevels }
        if let tableStyle = preset.tableStyle { self.tableStyle = tableStyle }
        if let tableColor1R = preset.tableColor1R { self.tableColor1R = tableColor1R }
        if let tableColor1G = preset.tableColor1G { self.tableColor1G = tableColor1G }
        if let tableColor1B = preset.tableColor1B { self.tableColor1B = tableColor1B }
        if let tableColor2R = preset.tableColor2R { self.tableColor2R = tableColor2R }
        if let tableColor2G = preset.tableColor2G { self.tableColor2G = tableColor2G }
        if let tableColor2B = preset.tableColor2B { self.tableColor2B = tableColor2B }
        if let woodSeed = preset.woodSeed { self.woodSeed = woodSeed }
        if let woodBrightness = preset.woodBrightness { self.woodBrightness = woodBrightness }
        if let woodPatternScale = preset.woodPatternScale { self.woodPatternScale = woodPatternScale }
        if let capRadiusScale = preset.capRadiusScale { self.capRadiusScale = capRadiusScale }
        if let capSegments = preset.capSegments { self.capSegments = capSegments }
        if let capRings = preset.capRings { self.capRings = capRings }
        if let capDarken = preset.capDarken { self.capDarken = capDarken }
        if let ropeMatte = preset.ropeMatte { self.ropeMatte = ropeMatte }
        if let ropeGloss = preset.ropeGloss { self.ropeGloss = ropeGloss }
        if let ropeDiffuseWrap = preset.ropeDiffuseWrap { self.ropeDiffuseWrap = ropeDiffuseWrap }
        if let ropeSubsurface = preset.ropeSubsurface { self.ropeSubsurface = ropeSubsurface }
        if let ropeEdgeLight = preset.ropeEdgeLight { self.ropeEdgeLight = ropeEdgeLight }
        if let ropeSaturation = preset.ropeSaturation { self.ropeSaturation = ropeSaturation }
        if let ropeMicroBump = preset.ropeMicroBump { self.ropeMicroBump = ropeMicroBump }
        if let ropeBumpScale = preset.ropeBumpScale { self.ropeBumpScale = ropeBumpScale }
        if let ropeContactAO = preset.ropeContactAO { self.ropeContactAO = ropeContactAO }
        if let ropeLiftGlow = preset.ropeLiftGlow { self.ropeLiftGlow = ropeLiftGlow }
        if let ropeStretchGloss = preset.ropeStretchGloss { self.ropeStretchGloss = ropeStretchGloss }
        if let ropeStretchSpec = preset.ropeStretchSpec { self.ropeStretchSpec = ropeStretchSpec }
        if let ropeEnvReflect = preset.ropeEnvReflect { self.ropeEnvReflect = ropeEnvReflect }
        if let ropeEnvSpread = preset.ropeEnvSpread { self.ropeEnvSpread = ropeEnvSpread }
        if let ropeOpacity = preset.ropeOpacity { self.ropeOpacity = ropeOpacity }
        if let wormMode = preset.wormMode { self.wormMode = wormMode }
        if let chainMode = preset.chainMode { self.chainMode = chainMode }
        if let squareCrossSection = preset.squareCrossSection { self.squareCrossSection = squareCrossSection }
        if let renderScale = preset.renderScale { self.renderScale = renderScale }
        if let zoomScale = preset.zoomScale { self.zoomScale = zoomScale }
    }

    struct Defaults {
        // Rope
        static let particleCount: Float = 77
        static let gravity: Float = -7.374
        static let damping: Float = 0.908
        // Solver
        static let constraintIterations: Float = 12
        static let settleSteps: Float = 1
        static let bendCompliance: Float = 0.0006
        static let bendVelocityCoupling: Float = 0
        // Drag
        static let dragHeight: Float = 0.35
        static let liftHeight: Float = 0.3
        static let ropeTension: Float = 0.918
        static let frictionCoefficient: Float = 0.8
        static let frictionDampingRatio: Float = 0.3
        static let maxFrictionCap: Float = 0.25
        static let boardFrictionRatio: Float = 0.5
        static let collisionResponse: Float = 0.35
        static let zSeparation: Float = 1.0
        static let twistStiffness: Float = 0.15
        static let twistDamping: Float = 0.4
        static let gravityTorque: Float = 0.8
        static let dragPickupDuration: Float = 0.12
        static let dragMinSubsteps: Float = 3
        static let fadeOutSpeed: Float = 45.0
        static let lowerAnimDuration: Float = 0.55
        static let idleTimeout: Float = 3.0
        static let boardElevation: Float = 0.12
    }

    enum RopePalette: Int, CaseIterable {
        case original = 0
        case warm
        case cool
        case earth
        case candy
        case mono
        case random

        var label: String {
            switch self {
            case .original: return "Original"
            case .warm: return "Warm"
            case .cool: return "Cool"
            case .earth: return "Earth"
            case .candy: return "Candy"
            case .mono: return "Mono"
            case .random: return "Random"
            }
        }

        static func hsb(_ h: Float, _ s: Float, _ b: Float) -> SIMD3<Float> {
            let c = b * s
            let x = c * (1 - abs(fmodf(h / 60.0, 2) - 1))
            let m = b - c
            let r1, g1, b1: Float
            switch Int(h / 60.0) % 6 {
            case 0: (r1, g1, b1) = (c, x, 0)
            case 1: (r1, g1, b1) = (x, c, 0)
            case 2: (r1, g1, b1) = (0, c, x)
            case 3: (r1, g1, b1) = (0, x, c)
            case 4: (r1, g1, b1) = (x, 0, c)
            default: (r1, g1, b1) = (c, 0, x)
            }
            return SIMD3<Float>(r1 + m, g1 + m, b1 + m)
        }

        var colors: [SIMD3<Float>] {
            switch self {
            case .original:
                return [
                    SIMD3<Float>(0.95, 0.30, 0.05),
                    SIMD3<Float>(0.10, 0.35, 0.92),
                    SIMD3<Float>(0.90, 0.12, 0.25),
                    SIMD3<Float>(0.15, 0.75, 0.30),
                    SIMD3<Float>(0.92, 0.78, 0.05),
                    SIMD3<Float>(0.60, 0.10, 0.72),
                    SIMD3<Float>(0.05, 0.65, 0.72),
                    SIMD3<Float>(0.85, 0.15, 0.55),
                    SIMD3<Float>(0.20, 0.55, 0.90),
                    SIMD3<Float>(0.80, 0.50, 0.05),
                ]
            case .warm:
                let hues: [(Float, Float, Float)] = [
                    (5, 0.72, 0.88), (30, 0.70, 0.85), (50, 0.68, 0.82),
                    (350, 0.65, 0.80), (20, 0.75, 0.78), (40, 0.60, 0.90),
                    (10, 0.68, 0.75), (55, 0.72, 0.80), (345, 0.58, 0.85),
                    (25, 0.65, 0.82),
                ]
                return hues.map { Self.hsb($0.0, $0.1, $0.2) }
            case .cool:
                let hues: [(Float, Float, Float)] = [
                    (195, 0.65, 0.82), (225, 0.60, 0.80), (260, 0.55, 0.78),
                    (175, 0.62, 0.76), (210, 0.68, 0.84), (290, 0.50, 0.75),
                    (185, 0.58, 0.80), (240, 0.55, 0.82), (165, 0.60, 0.78),
                    (270, 0.52, 0.80),
                ]
                return hues.map { Self.hsb($0.0, $0.1, $0.2) }
            case .earth:
                let hues: [(Float, Float, Float)] = [
                    (25, 0.55, 0.68), (80, 0.45, 0.62), (38, 0.58, 0.72),
                    (12, 0.50, 0.65), (95, 0.40, 0.58), (45, 0.52, 0.70),
                    (70, 0.42, 0.60), (30, 0.60, 0.66), (8, 0.48, 0.62),
                    (55, 0.50, 0.65),
                ]
                return hues.map { Self.hsb($0.0, $0.1, $0.2) }
            case .candy:
                let hues: [(Float, Float, Float)] = [
                    (330, 0.55, 0.90), (190, 0.50, 0.88), (35, 0.55, 0.90),
                    (155, 0.48, 0.85), (280, 0.45, 0.88), (60, 0.50, 0.85),
                    (170, 0.52, 0.82), (0, 0.50, 0.92), (220, 0.45, 0.88),
                    (90, 0.48, 0.85),
                ]
                return hues.map { Self.hsb($0.0, $0.1, $0.2) }
            case .mono:
                let vals: [Float] = [0.82, 0.52, 0.68, 0.38, 0.90, 0.28, 0.60, 0.45, 0.75, 0.33]
                return vals.map { SIMD3<Float>($0, $0, $0) }
            case .random:
                return Self.generateRandom()
            }
        }

        static func generateRandom() -> [SIMD3<Float>] {
            let baseHue = Float.random(in: 0..<360)
            let goldenAngle: Float = 137.508
            return (0..<10).map { i in
                let h = fmodf(baseHue + Float(i) * goldenAngle, 360)
                let s = Float.random(in: 0.55...0.75)
                let b = Float.random(in: 0.75...0.90)
                return hsb(h, s, b)
            }
        }
    }

    @Published var ropePalette: RopePalette = .original {
        didSet { applyCurrentPalette(); persist("v.rpal", Float(ropePalette.rawValue)) }
    }

    func applyCurrentPalette() {
        renderer?.applyRopePalette(ropePalette.colors)
    }

    // Rope
    @Published var particleCount: Float = Defaults.particleCount {
        didSet { renderer?.physicsParticleCount = Int(particleCount); persist("p.ptc", particleCount) }
    }
    @Published var gravity: Float = Defaults.gravity {
        didSet { renderer?.physicsGravity = gravity; persist("p.grv", gravity) }
    }
    @Published var damping: Float = Defaults.damping {
        didSet { renderer?.physicsDamping = damping; persist("p.dmp", damping) }
    }

    // Solver
    @Published var constraintIterations: Float = Defaults.constraintIterations {
        didSet { renderer?.physicsConstraintIterations = Int(constraintIterations); persist("p.cit", constraintIterations) }
    }
    @Published var settleSteps: Float = Defaults.settleSteps {
        didSet { renderer?.physicsSettleSteps = Int(settleSteps); persist("p.stl", settleSteps) }
    }
    @Published var bendCompliance: Float = Defaults.bendCompliance {
        didSet { renderer?.physicsBendCompliance = bendCompliance; persist("p.bcp", bendCompliance) }
    }
    @Published var bendVelocityCoupling: Float = Defaults.bendVelocityCoupling {
        didSet { renderer?.physicsBendVelocityCoupling = bendVelocityCoupling; persist("p.bvc", bendVelocityCoupling) }
    }
    @Published var broadphaseRebuildInterval: Float = 3 {
        didSet { renderer?.physicsBroadphaseInterval = Int(broadphaseRebuildInterval); persist("p.bph", broadphaseRebuildInterval) }
    }

    // Drag
    @Published var dragHeight: Float = Defaults.dragHeight {
        didSet { renderer?.dragHeight = dragHeight; persist("p.drg", dragHeight) }
    }
    @Published var liftHeight: Float = Defaults.liftHeight {
        didSet { renderer?.physicsLiftHeight = liftHeight; persist("p.lft", liftHeight) }
    }
    @Published var ropeTension: Float = Defaults.ropeTension {
        didSet { renderer?.physicsRopeTension = ropeTension; persist("p.rtn", ropeTension) }
    }

    @Published var frictionCoefficient: Float = Defaults.frictionCoefficient {
        didSet { renderer?.physicsFriction = frictionCoefficient; persist("p.frc", frictionCoefficient) }
    }
    @Published var collisionResponse: Float = Defaults.collisionResponse {
        didSet { renderer?.physicsCollisionResponse = collisionResponse; persist("p.crs", collisionResponse) }
    }
    @Published var zSeparation: Float = Defaults.zSeparation {
        didSet { renderer?.physicsZSeparation = zSeparation; persist("p.zsep", zSeparation) }
    }
    @Published var frictionDampingRatio: Float = Defaults.frictionDampingRatio {
        didSet { renderer?.physicsFrictionDampingRatio = frictionDampingRatio; persist("p.fdr", frictionDampingRatio) }
    }
    @Published var maxFrictionCap: Float = Defaults.maxFrictionCap {
        didSet { renderer?.physicsMaxFrictionCap = maxFrictionCap; persist("p.mfc", maxFrictionCap) }
    }
    @Published var boardFrictionRatio: Float = Defaults.boardFrictionRatio {
        didSet { renderer?.physicsBoardFrictionRatio = boardFrictionRatio; persist("p.bfr", boardFrictionRatio) }
    }
    @Published var twistStiffness: Float = Defaults.twistStiffness {
        didSet { renderer?.physicsTwistStiffness = twistStiffness; persist("p.tws", twistStiffness) }
    }
    @Published var twistDamping: Float = Defaults.twistDamping {
        didSet { renderer?.physicsTwistDamping = twistDamping; persist("p.twd", twistDamping) }
    }
    @Published var gravityTorque: Float = Defaults.gravityTorque {
        didSet { renderer?.physicsGravityTorque = gravityTorque; persist("p.gtq", gravityTorque) }
    }
    @Published var dragPickupDuration: Float = Defaults.dragPickupDuration {
        didSet { renderer?.physicsDragPickupDuration = dragPickupDuration; persist("p.dpd", dragPickupDuration) }
    }
    @Published var dragMinSubsteps: Float = Defaults.dragMinSubsteps {
        didSet { renderer?.physicsDragMinSubsteps = Int(dragMinSubsteps); persist("p.dms", dragMinSubsteps) }
    }
    @Published var fadeOutSpeed: Float = Defaults.fadeOutSpeed {
        didSet { renderer?.physicsFadeOutSpeed = fadeOutSpeed; persist("p.fos", fadeOutSpeed) }
    }
    @Published var lowerAnimDuration: Float = Defaults.lowerAnimDuration {
        didSet { renderer?.physicsLowerAnimDuration = lowerAnimDuration; persist("p.lad", lowerAnimDuration) }
    }
    @Published var idleTimeout: Float = Defaults.idleTimeout {
        didSet { renderer?.physicsIdleTimeout = idleTimeout; persist("p.ito", idleTimeout) }
    }
    @Published var boardElevation: Float = Defaults.boardElevation {
        didSet { renderer?.boardElevation = boardElevation; persist("p.bel", boardElevation) }
    }
    @Published var useParticleBraid: Bool = false {
        didSet { renderer?.useParticleBraid = useParticleBraid }
    }
    @Published var physicsPaused: Bool = false {
        didSet { renderer?.physicsPaused = physicsPaused }
    }

    @Published var profileSegments: Float = 6 {
        didSet { renderer?.profileSegments = Int(profileSegments); persist("v.prf", profileSegments) }
    }
    @Published var holeRadiusScale: Float = 0.734 {
        didSet { renderer?.holeRadiusScale = holeRadiusScale; persist("v.hrs", holeRadiusScale) }
    }
    @Published var holeTintR: Float = 1.0 { didSet { updateHoleTint(); persist("v.htr", holeTintR) } }
    @Published var holeTintG: Float = 0.89 { didSet { updateHoleTint(); persist("v.htg", holeTintG) } }
    @Published var holeTintB: Float = 1.0 { didSet { updateHoleTint(); persist("v.htb", holeTintB) } }
    @Published var holeTintAmount: Float = 1.0 { didSet { updateHoleTint(); persist("v.hta", holeTintAmount) } }
    @Published var holeSegments: Float = 19 {
        didSet { renderer?.holeSegments = Int(holeSegments); persist("v.hsg", holeSegments) }
    }
    @Published var ropeRadiusScale: Float = 0.761 {
        didSet { renderer?.ropeRadiusScale = ropeRadiusScale; persist("v.rrs", ropeRadiusScale) }
    }
    @Published var stretchThinning: Float = 0.039 {
        didSet { renderer?.stretchThinning = stretchThinning; persist("v.stn", stretchThinning) }
    }
    @Published var exposure: Float = 1.409 {
        didSet { renderer?.shaderParams.exposure = exposure; persist("v.exp", exposure) }
    }
    @Published var lightIntensity: Float = 0.766 {
        didSet { renderer?.shaderParams.lightIntensity = lightIntensity; persist("v.lit", lightIntensity) }
    }
    @Published var lightDirX: Float = -0.125 {
        didSet { updateLightDir(); persist("v.ldx", lightDirX) }
    }
    @Published var lightDirY: Float = -0.156 {
        didSet { updateLightDir(); persist("v.ldy", lightDirY) }
    }
    @Published var lightDirZ: Float = 0.532 {
        didSet { updateLightDir(); persist("v.ldz", lightDirZ) }
    }

    enum ShadowType: Int, CaseIterable {
        case shadowMap = 0
        case pcf = 1
        case pcss = 2

        var label: String {
            switch self {
            case .shadowMap: return "ShadowMap"
            case .pcf: return "PCF"
            case .pcss: return "PCSS"
            }
        }
    }

    @Published var shadowType: ShadowType = .pcf {
        didSet { renderer?.shaderParams.shadowType = shadowType.rawValue; persist("v.stp", Float(shadowType.rawValue)) }
    }
    @Published var ambient: Float = 0 {
        didSet { renderer?.shaderParams.ambient = ambient; persist("v.amb", ambient) }
    }
    @Published var shadowBias: Float = 0.0001 {
        didSet { renderer?.shaderParams.shadowBias = shadowBias; persist("v.sb", shadowBias) }
    }
    @Published var shadowDarkness: Float = 0 {
        didSet { renderer?.shaderParams.shadowDarkness = shadowDarkness; persist("v.sd", shadowDarkness) }
    }
    @Published var lightSize: Float = 0.002 {
        didSet { renderer?.shaderParams.lightSize = lightSize; persist("v.lsz", lightSize) }
    }
    @Published var shadowsEnabled: Bool = true {
        didSet { renderer?.shaderParams.shadowsEnabled = shadowsEnabled; persist("v.sen", shadowsEnabled ? 1 : 0) }
    }
    @Published var shadowMapSize: Int = 1024 {
        didSet { renderer?.shadowMapSize = shadowMapSize; persist("v.sms", Float(shadowMapSize)) }
    }
    @Published var bloomEnabled: Bool = true {
        didSet { renderer?.shaderParams.bloomEnabled = bloomEnabled; persist("v.blen", bloomEnabled ? 1 : 0) }
    }

    private func updateLightDir() {
        var d = SIMD3<Float>(lightDirX, lightDirY, lightDirZ)
        if simd_length_squared(d) < 1e-6 { d = SIMD3<Float>(-0.65, -0.35, 0.67) }
        renderer?.shaderParams.lightDir = simd_normalize(d)
    }
    @Published var bloomStrength: Float = 0 {
        didSet { renderer?.shaderParams.bloomStrength = bloomStrength; persist("v.blm", bloomStrength) }
    }
    @Published var cartoonShaderEnabled: Bool = false {
        didSet { renderer?.shaderParams.cartoonShaderEnabled = cartoonShaderEnabled; persist("v.crt", cartoonShaderEnabled ? 1 : 0) }
    }
    @Published var cartoonExposure: Float = 1.33 {
        didSet { renderer?.shaderParams.cartoonExposure = cartoonExposure; persist("v.cex", cartoonExposure) }
    }
    @Published var cartoonBloom: Float = 0 {
        didSet { renderer?.shaderParams.cartoonBloom = cartoonBloom; persist("v.cbl", cartoonBloom) }
    }
    @Published var cartoonEdgeStrength: Float = 1 {
        didSet { renderer?.shaderParams.cartoonEdgeStrength = cartoonEdgeStrength; persist("v.ced", cartoonEdgeStrength) }
    }
    @Published var cartoonLevels: Float = 2 {
        didSet { renderer?.shaderParams.cartoonLevels = Int(cartoonLevels); persist("v.clv", cartoonLevels) }
    }
    @Published var cartoonShadowBright: Float = 0.38 {
        didSet { renderer?.shaderParams.cartoonShadowBright = cartoonShadowBright; persist("v.csb", cartoonShadowBright) }
    }
    @Published var cartoonWrap: Float = 0.15 {
        didSet { renderer?.shaderParams.cartoonWrap = cartoonWrap; persist("v.cwp", cartoonWrap) }
    }
    @Published var cartoonEdgeSmooth: Float = 0.5 {
        didSet { renderer?.shaderParams.cartoonEdgeSmooth = cartoonEdgeSmooth; persist("v.ces", cartoonEdgeSmooth) }
    }
    @Published var ropeFlatNormals: Bool = false {
        didSet { renderer?.shaderParams.ropeFlatNormals = ropeFlatNormals; persist("v.rfn", ropeFlatNormals ? 1 : 0) }
    }
    @Published var pcssPenumbraScale: Float = 80.0 {
        didSet { renderer?.shaderParams.pcssPenumbraScale = pcssPenumbraScale; persist("v.pps", pcssPenumbraScale) }
    }
    enum TableStyle: Int, CaseIterable {
        case wood = 0
        case gradient = 1
        case solid = 2

        var label: String {
            switch self {
            case .wood: return "Wood"
            case .gradient: return "Gradient"
            case .solid: return "Solid"
            }
        }
    }

    @Published var tableStyle: TableStyle = .gradient {
        didSet { renderer?.shaderParams.tableStyle = tableStyle.rawValue; persist("v.tst", Float(tableStyle.rawValue)) }
    }
    @Published var tableColor1R: Float = 0 { didSet { updateTableColor(); persist("v.tc1r", tableColor1R) } }
    @Published var tableColor1G: Float = 0 { didSet { updateTableColor(); persist("v.tc1g", tableColor1G) } }
    @Published var tableColor1B: Float = 0 { didSet { updateTableColor(); persist("v.tc1b", tableColor1B) } }
    @Published var tableColor2R: Float = 0.044 { didSet { updateTableColor(); persist("v.tc2r", tableColor2R) } }
    @Published var tableColor2G: Float = 0.065 { didSet { updateTableColor(); persist("v.tc2g", tableColor2G) } }
    @Published var tableColor2B: Float = 0.072 { didSet { updateTableColor(); persist("v.tc2b", tableColor2B) } }
    @Published var woodSeed: Float = 0.052 {
        didSet { renderer?.woodSeed = woodSeed; persist("v.wsd", woodSeed) }
    }
    @Published var woodBrightness: Float = 0.551 {
        didSet { renderer?.woodBrightness = woodBrightness; persist("v.wbr", woodBrightness) }
    }
    @Published var woodPatternScale: Float = 3.95 {
        didSet { renderer?.woodPatternScale = woodPatternScale; persist("v.wps", woodPatternScale) }
    }
    @Published var capRadiusScale: Float = 0.91 {
        didSet { renderer?.shaderParams.capRadiusScale = capRadiusScale; persist("v.crs", capRadiusScale) }
    }
    @Published var capSegments: Float = 12 {
        didSet { renderer?.shaderParams.capSegments = Int(capSegments); persist("v.csg", capSegments) }
    }
    @Published var capRings: Float = 6 {
        didSet { renderer?.shaderParams.capRings = Int(capRings); persist("v.crg", capRings) }
    }
    @Published var capDarken: Float = 0 {
        didSet { renderer?.shaderParams.capDarken = capDarken; persist("v.cdk", capDarken) }
    }

    @Published var ropeMatte: Float = 1 {
        didSet { renderer?.shaderParams.ropeMatte = ropeMatte; persist("v.rmat", ropeMatte) }
    }
    @Published var ropeGloss: Float = 0 {
        didSet { renderer?.shaderParams.ropeGloss = ropeGloss; persist("v.rgls", ropeGloss) }
    }
    @Published var ropeDiffuseWrap: Float = 0 {
        didSet { renderer?.shaderParams.ropeDiffuseWrap = ropeDiffuseWrap; persist("v.rdwp", ropeDiffuseWrap) }
    }
    @Published var ropeSubsurface: Float = 0.303 {
        didSet { renderer?.shaderParams.ropeSubsurface = ropeSubsurface; persist("v.rsss", ropeSubsurface) }
    }
    @Published var ropeEdgeLight: Float = 0 {
        didSet { renderer?.shaderParams.ropeEdgeLight = ropeEdgeLight; persist("v.redg", ropeEdgeLight) }
    }
    @Published var ropeSaturation: Float = 0.501 {
        didSet { renderer?.shaderParams.ropeSaturation = ropeSaturation; persist("v.rsat", ropeSaturation) }
    }
    @Published var ropeMicroBump: Float = 1.265 {
        didSet { renderer?.shaderParams.ropeMicroBump = ropeMicroBump; persist("v.rmbp", ropeMicroBump) }
    }
    @Published var ropeBumpScale: Float = 0.5 {
        didSet { renderer?.shaderParams.ropeBumpScale = ropeBumpScale; persist("v.rbsc", ropeBumpScale) }
    }
    @Published var ropeContactAO: Float = 0 {
        didSet { renderer?.shaderParams.ropeContactAO = ropeContactAO; persist("v.rcao", ropeContactAO) }
    }
    @Published var ropeLiftGlow: Float = 0 {
        didSet { renderer?.shaderParams.ropeLiftGlow = ropeLiftGlow; persist("v.rlgw", ropeLiftGlow) }
    }
    @Published var ropeStretchGloss: Float = 0.313 {
        didSet { renderer?.shaderParams.ropeStretchGloss = ropeStretchGloss; persist("v.rsg", ropeStretchGloss) }
    }
    @Published var ropeStretchSpec: Float = 0.256 {
        didSet { renderer?.shaderParams.ropeStretchSpec = ropeStretchSpec; persist("v.rss", ropeStretchSpec) }
    }
    @Published var ropeEnvReflect: Float = 0 {
        didSet { renderer?.shaderParams.ropeEnvReflect = ropeEnvReflect; persist("v.renv", ropeEnvReflect) }
    }
    @Published var ropeEnvSpread: Float = 0.042 {
        didSet { renderer?.shaderParams.ropeEnvSpread = ropeEnvSpread; persist("v.rens", ropeEnvSpread) }
    }
    @Published var ropeOpacity: Float = 1.0 {
        didSet { renderer?.shaderParams.ropeOpacity = ropeOpacity; persist("v.ropa", ropeOpacity) }
    }
    @Published var ropeSeamEnabled: Bool = false {
        didSet { renderer?.shaderParams.ropeSeamEnabled = ropeSeamEnabled; persist("v.rsen", ropeSeamEnabled ? 1 : 0) }
    }
    @Published var ropeSeamWidth: Float = 0.06 {
        didSet { renderer?.shaderParams.ropeSeamWidth = ropeSeamWidth; persist("v.rsw", ropeSeamWidth) }
    }
    @Published var ropeSeamDepth: Float = 0.45 {
        didSet { renderer?.shaderParams.ropeSeamDepth = ropeSeamDepth; persist("v.rsd", ropeSeamDepth) }
    }
    @Published var ropeSeamDarkness: Float = 1.4 {
        didSet { renderer?.shaderParams.ropeSeamDarkness = ropeSeamDarkness; persist("v.rsdk", ropeSeamDarkness) }
    }
    @Published var ropeSeamHighlight: Float = 0.35 {
        didSet { renderer?.shaderParams.ropeSeamHighlight = ropeSeamHighlight; persist("v.rshl", ropeSeamHighlight) }
    }
    @Published var ropeSeamCrackAmount: Float = 0.45 {
        didSet { renderer?.shaderParams.ropeSeamCrackAmount = ropeSeamCrackAmount; persist("v.rsca", ropeSeamCrackAmount) }
    }
    @Published var ropeSeamCrackScale: Float = 18.0 {
        didSet { renderer?.shaderParams.ropeSeamCrackScale = ropeSeamCrackScale; persist("v.rscs", ropeSeamCrackScale) }
    }
    @Published var ropeSeamRandomize: Bool = true {
        didSet { renderer?.shaderParams.ropeSeamRandomize = ropeSeamRandomize; persist("v.rsrn", ropeSeamRandomize ? 1 : 0) }
    }
    @Published var ropeSeamPosition: Float = 0.5 {
        didSet { renderer?.shaderParams.ropeSeamPosition = ropeSeamPosition; persist("v.rspo", ropeSeamPosition) }
    }
    @Published var ropeCracksEnabled: Bool = false {
        didSet { renderer?.shaderParams.ropeCracksEnabled = ropeCracksEnabled; persist("v.rcren", ropeCracksEnabled ? 1 : 0) }
    }
    @Published var ropeCrackAmount: Float = 0.45 {
        didSet { renderer?.shaderParams.ropeCrackAmount = ropeCrackAmount; persist("v.rcra", ropeCrackAmount) }
    }
    @Published var ropeCrackWidth: Float = 0.16 {
        didSet { renderer?.shaderParams.ropeCrackWidth = ropeCrackWidth; persist("v.rcrw", ropeCrackWidth) }
    }
    @Published var ropeCrackDepth: Float = 0.5 {
        didSet { renderer?.shaderParams.ropeCrackDepth = ropeCrackDepth; persist("v.rcrd", ropeCrackDepth) }
    }
    @Published var ropeEnvDebug: Bool = false {
        didSet { renderer?.shaderParams.ropeEnvDebug = ropeEnvDebug }
    }
    @Published var shadowDebugMode: Int = 0 {
        didSet { renderer?.shaderParams.shadowDebugMode = shadowDebugMode }
    }
    @Published var chainMode: Bool = false {
        didSet { renderer?.shaderParams.chainMode = chainMode; persist("v.chn", chainMode ? 1 : 0) }
    }
    @Published var chainLinkLength: Float = 2.8 {
        didSet { renderer?.shaderParams.chainLinkLength = chainLinkLength; persist("c.ll", chainLinkLength) }
    }
    @Published var chainLinkThickness: Float = 0.35 {
        didSet { renderer?.shaderParams.chainLinkThickness = chainLinkThickness; persist("c.lt", chainLinkThickness) }
    }
    @Published var chainLinkWidth: Float = 0.85 {
        didSet { renderer?.shaderParams.chainLinkWidth = chainLinkWidth; persist("c.lw", chainLinkWidth) }
    }
    @Published var wormMode: Bool = false {
        didSet { renderer?.shaderParams.wormMode = wormMode; persist("v.wrm", wormMode ? 1 : 0) }
    }
    @Published var wormSegFreq: Float = 28.0 {
        didSet { renderer?.shaderParams.wormSegFreq = wormSegFreq; persist("w.sf", wormSegFreq) }
    }
    @Published var wormSegBulge: Float = 0.12 {
        didSet { renderer?.shaderParams.wormSegBulge = wormSegBulge; persist("w.sb", wormSegBulge) }
    }
    @Published var wormThickness: Float = 1.35 {
        didSet { renderer?.shaderParams.wormThickness = wormThickness; persist("w.th", wormThickness) }
    }
    @Published var wormTaperLen: Float = 0.12 {
        didSet { renderer?.shaderParams.wormTaperLen = wormTaperLen; persist("w.tl", wormTaperLen) }
    }
    @Published var wormGrooveDepth: Float = 0.35 {
        didSet { renderer?.shaderParams.wormGrooveDepth = wormGrooveDepth; persist("w.gd", wormGrooveDepth) }
    }
    @Published var wormBellyBright: Float = 1.15 {
        didSet { renderer?.shaderParams.wormBellyBright = wormBellyBright; persist("w.bb", wormBellyBright) }
    }
    @Published var wormBackDark: Float = 0.7 {
        didSet { renderer?.shaderParams.wormBackDark = wormBackDark; persist("w.bd", wormBackDark) }
    }
    @Published var wormSkinNoise: Float = 0.08 {
        didSet { renderer?.shaderParams.wormSkinNoise = wormSkinNoise; persist("w.sn", wormSkinNoise) }
    }
    @Published var wormSSS: Float = 0.25 {
        didSet { renderer?.shaderParams.wormSSS = wormSSS; persist("w.ss", wormSSS) }
    }
    @Published var wormRoughness: Float = 0.25 {
        didSet { renderer?.shaderParams.wormRoughness = wormRoughness; persist("w.rg", wormRoughness) }
    }
    @Published var wormSpecular: Float = 0.8 {
        didSet { renderer?.shaderParams.wormSpecular = wormSpecular; persist("w.sp", wormSpecular) }
    }
    @Published var wormRimStrength: Float = 0.08 {
        didSet { renderer?.shaderParams.wormRimStrength = wormRimStrength; persist("w.rm", wormRimStrength) }
    }
    @Published var wormEyeSize: Float = 0.015 {
        didSet { renderer?.shaderParams.wormEyeSize = wormEyeSize; persist("w.es", wormEyeSize) }
    }
    @Published var wormPulseSpeed: Float = 2.5 {
        didSet { renderer?.shaderParams.wormPulseSpeed = wormPulseSpeed; persist("w.ps", wormPulseSpeed) }
    }
    @Published var wormPulseAmp: Float = 0.02 {
        didSet { renderer?.shaderParams.wormPulseAmp = wormPulseAmp; persist("w.pa", wormPulseAmp) }
    }
    @Published var wormCrawlSpeed: Float = 3.5 {
        didSet { renderer?.shaderParams.wormCrawlSpeed = wormCrawlSpeed; persist("w.cs", wormCrawlSpeed) }
    }
    @Published var wormCrawlAmp: Float = 0.012 {
        didSet { renderer?.shaderParams.wormCrawlAmp = wormCrawlAmp; persist("w.ca", wormCrawlAmp) }
    }
    @Published var wormSideAmp: Float = 0.008 {
        didSet { renderer?.shaderParams.wormSideAmp = wormSideAmp; persist("w.sa", wormSideAmp) }
    }
    @Published var squareCrossSection: Bool = false {
        didSet { renderer?.squareCrossSection = squareCrossSection; persist("v.sqcs", squareCrossSection ? 1 : 0) }
    }
    @Published var renderScale: Float = 1.0 {
        didSet { renderer?.renderScale = renderScale; persist("v.rsc", renderScale) }
    }

    private func updateHoleTint() {
        renderer?.shaderParams.holeTint = SIMD4<Float>(holeTintR, holeTintG, holeTintB, holeTintAmount)
    }

    private func updateTableColor() {
        renderer?.shaderParams.tableColor1 = SIMD3<Float>(tableColor1R, tableColor1G, tableColor1B)
        renderer?.shaderParams.tableColor2 = SIMD3<Float>(tableColor2R, tableColor2G, tableColor2B)
    }

    @Published var zoomScale: Float = 0.955 {
        didSet { renderer?.cameraZoomScale = zoomScale; persist("p.zoom", zoomScale) }
    }

    @Published var fps: Float = 0
    @Published var currentLevel: Int = 1 {
        didSet { persist("p.lvl", Float(currentLevel)) }
    }
    @Published var moveCount: Int = 0
    @Published var showLevelComplete: Bool = false
    @Published var isNewRecord: Bool = false
    @Published var previousRecord: Int? = nil
    @Published var canUndo: Bool = false
    @Published var percentile: Int? = nil
    @Published var globalBest: Int? = nil
    @Published var starCount: Int = 3
    @Published var leaderboardUsername: String = ""
    private var levelStartTime: Date?
    @Published var frictionSoundEnabled: Bool = true {
        didSet { renderer?.frictionSound.enabled = frictionSoundEnabled; persist("p.snd", frictionSoundEnabled ? 1 : 0) }
    }
    @Published var profilerActive: Bool = false {
        didSet { PhysicsProfiler.shared.enabled = profilerActive }
    }
    @Published var profilerSummary: String = ""

    struct BraidTargetEntry {
        var strandColor: SIMD3<Float>
        var targetSlot: Int  // 0-based bottom slot index
    }
    @Published var braidTargetEntries: [BraidTargetEntry] = []

    private var fpsTimer: Timer?

    private func persist(_ key: String, _ v: Float) { UserDefaults.standard.set(v, forKey: key) }

    init() {
        loadSaved()
        leaderboardUsername = LeaderboardAPI.shared.username
        fpsTimer = Timer.scheduledTimer(withTimeInterval: 0.25, repeats: true) { _ in
            Task { @MainActor [weak self] in
                guard let self, let r = self.renderer else { return }
                self.fps = max(r.currentFPS, r.potentialFPS)
                if self.currentLevel != r.currentLevelId {
                    self.currentLevel = r.currentLevelId
                }
                if self.profilerActive {
                    self.profilerSummary = PhysicsProfiler.shared.summaryString()
                }
            }
        }
        Task {
            await LeaderboardAPI.shared.ensureRegistered()
            self.leaderboardUsername = LeaderboardAPI.shared.username
        }
    }

    private func loadSaved() {
        let ud = UserDefaults.standard
        func f(_ k: String) -> Float? { ud.object(forKey: k) != nil ? ud.float(forKey: k) : nil }

        if let v = f("p.ptc") { particleCount = v }
        if let v = f("p.grv") { gravity = v }
        if let v = f("p.dmp") { damping = v }
        if let v = f("p.cit") { constraintIterations = v }
        if let v = f("p.stl") { settleSteps = v }
        if let v = f("p.bcp") { bendCompliance = v }
        if let v = f("p.bvc") { bendVelocityCoupling = v }
        if let v = f("p.bph") { broadphaseRebuildInterval = v }
        if let v = f("p.drg") { dragHeight = v }
        if let v = f("p.lft") { liftHeight = v }
        if let v = f("p.rtn") { ropeTension = v }
        if let v = f("p.frc") { frictionCoefficient = v }
        if let v = f("p.crs") { collisionResponse = v }
        if let v = f("p.zsep") { zSeparation = v }
        if let v = f("p.fdr") { frictionDampingRatio = v }
        if let v = f("p.mfc") { maxFrictionCap = v }
        if let v = f("p.bfr") { boardFrictionRatio = v }
        if let v = f("p.tws") { twistStiffness = v }
        if let v = f("p.twd") { twistDamping = v }
        if let v = f("p.gtq") { gravityTorque = v }
        if let v = f("p.dpd") { dragPickupDuration = v }
        if let v = f("p.dms") { dragMinSubsteps = v }
        if let v = f("p.fos") { fadeOutSpeed = v }
        if let v = f("p.lad") { lowerAnimDuration = v }
        if let v = f("p.ito") { idleTimeout = v }
        if let v = f("p.bel") { boardElevation = v }
        if let v = f("p.snd") { frictionSoundEnabled = v > 0.5 }
        if let v = f("p.zoom"), v > 0 { zoomScale = v }
        if let v = f("p.lvl"), v >= 1 { currentLevel = Int(v) }
        if let v = f("v.prf") { profileSegments = v }
        if let v = f("v.hrs") { holeRadiusScale = v }
        if let v = f("v.htr") { holeTintR = v }
        if let v = f("v.htg") { holeTintG = v }
        if let v = f("v.htb") { holeTintB = v }
        if let v = f("v.hta") { holeTintAmount = v }
        if let v = f("v.hsg") { holeSegments = v }
        if let v = f("v.rrs") { ropeRadiusScale = v }
        if let v = f("v.stn") { stretchThinning = v }
        if let v = f("v.exp") { exposure = v }
        if let v = f("v.lit") { lightIntensity = v }
        if let v = f("v.ldx") { lightDirX = v }
        if let v = f("v.ldy") { lightDirY = v }
        if let v = f("v.ldz") { lightDirZ = v }
        updateLightDir()
        if let v = f("v.stp") { shadowType = ShadowType(rawValue: Int(v)) ?? .pcss }
        if let v = f("v.amb") { ambient = v }
        if let v = f("v.sb") { shadowBias = v }
        if let v = f("v.sd") { shadowDarkness = v }
        if let v = f("v.lsz") { lightSize = v }
        if let v = f("v.sen") { shadowsEnabled = v > 0.5 }
        if let v = f("v.sms"), v > 0 { shadowMapSize = Int(v) }
        if let v = f("v.blen") { bloomEnabled = v > 0.5 }
        if let v = f("v.blm") { bloomStrength = v }
        if let v = f("v.crt") { cartoonShaderEnabled = v > 0.5 }
        if let v = f("v.cex") { cartoonExposure = v }
        if let v = f("v.cbl") { cartoonBloom = v }
        if let v = f("v.ced") { cartoonEdgeStrength = v }
        if let v = f("v.clv") { cartoonLevels = v }
        if let v = f("v.csb") { cartoonShadowBright = v }
        if let v = f("v.cwp") { cartoonWrap = v }
        if let v = f("v.ces") { cartoonEdgeSmooth = v }
        if let v = f("v.rfn") { ropeFlatNormals = v > 0.5 }
        if let v = f("v.pps") { pcssPenumbraScale = v }
        if let v = f("v.tst") { tableStyle = TableStyle(rawValue: Int(v)) ?? .wood }
        if let v = f("v.tc1r") { tableColor1R = v }
        if let v = f("v.tc1g") { tableColor1G = v }
        if let v = f("v.tc1b") { tableColor1B = v }
        if let v = f("v.tc2r") { tableColor2R = v }
        if let v = f("v.tc2g") { tableColor2G = v }
        if let v = f("v.tc2b") { tableColor2B = v }
        updateTableColor()
        if let v = f("v.wsd") { woodSeed = v }
        if let v = f("v.wbr") { woodBrightness = v }
        if let v = f("v.wps") { woodPatternScale = v }
        if let v = f("v.crs") { capRadiusScale = v }
        if let v = f("v.csg") { capSegments = v }
        if let v = f("v.crg") { capRings = v }
        if let v = f("v.cdk") { capDarken = v }
        if let v = f("v.rmat") { ropeMatte = v }
        if let v = f("v.rgls") { ropeGloss = v }
        if let v = f("v.rdwp") { ropeDiffuseWrap = v }
        if let v = f("v.rsss") { ropeSubsurface = v }
        if let v = f("v.redg") { ropeEdgeLight = v }
        if let v = f("v.rsat") { ropeSaturation = v }
        if let v = f("v.rmbp") { ropeMicroBump = v }
        if let v = f("v.rbsc") { ropeBumpScale = v }
        if let v = f("v.rcao") { ropeContactAO = v }
        if let v = f("v.rlgw") { ropeLiftGlow = v }
        if let v = f("v.rsg") { ropeStretchGloss = v }
        if let v = f("v.rss") { ropeStretchSpec = v }
        if let v = f("v.renv") { ropeEnvReflect = v }
        if let v = f("v.ropa") { ropeOpacity = v }
        if let v = f("v.rens") { ropeEnvSpread = v }
        if let v = f("v.rsen") { ropeSeamEnabled = v > 0.5 }
        if let v = f("v.rsw") { ropeSeamWidth = v }
        if let v = f("v.rsd") { ropeSeamDepth = v }
        if let v = f("v.rsdk") { ropeSeamDarkness = v }
        if let v = f("v.rshl") { ropeSeamHighlight = v }
        if let v = f("v.rsca") { ropeSeamCrackAmount = v }
        if let v = f("v.rscs") { ropeSeamCrackScale = v }
        if let v = f("v.rsrn") { ropeSeamRandomize = v > 0.5 }
        if let v = f("v.rspo") { ropeSeamPosition = v }
        if let v = f("v.rcren") { ropeCracksEnabled = v > 0.5 }
        if let v = f("v.rcra") { ropeCrackAmount = v }
        if let v = f("v.rcrw") { ropeCrackWidth = v }
        if let v = f("v.rcrd") { ropeCrackDepth = v }
        if let v = f("v.sqcs") { squareCrossSection = v > 0.5 }
        if let v = f("v.rsc"), v > 0 { renderScale = v }
        if let v = f("v.chn") { chainMode = v > 0.5 }
        if let v = f("c.ll") { chainLinkLength = v }
        if let v = f("c.lt") { chainLinkThickness = v }
        if let v = f("c.lw") { chainLinkWidth = v }
        if let v = f("v.wrm") { wormMode = v > 0.5 }
        if let v = f("w.sf") { wormSegFreq = v }
        if let v = f("w.sb") { wormSegBulge = v }
        if let v = f("w.th") { wormThickness = v }
        if let v = f("w.tl") { wormTaperLen = v }
        if let v = f("w.gd") { wormGrooveDepth = v }
        if let v = f("w.bb") { wormBellyBright = v }
        if let v = f("w.bd") { wormBackDark = v }
        if let v = f("w.sn") { wormSkinNoise = v }
        if let v = f("w.ss") { wormSSS = v }
        if let v = f("w.rg") { wormRoughness = v }
        if let v = f("w.sp") { wormSpecular = v }
        if let v = f("w.rm") { wormRimStrength = v }
        if let v = f("w.es") { wormEyeSize = v }
        if let v = f("w.ps") { wormPulseSpeed = v }
        if let v = f("w.pa") { wormPulseAmp = v }
        if let v = f("w.cs") { wormCrawlSpeed = v }
        if let v = f("w.ca") { wormCrawlAmp = v }
        if let v = f("w.sa") { wormSideAmp = v }
        if let v = f("v.rpal") { ropePalette = RopePalette(rawValue: Int(v)) ?? .original }
        updateHoleTint()
    }

    private static let allKeys = [
        "p.ptc", "p.grv", "p.dmp", "p.cit", "p.stl", "p.bcp", "p.bvc", "p.bph", "p.drg", "p.lft", "p.rtn",
        "p.frc", "p.crs", "p.zsep", "p.fdr", "p.mfc", "p.bfr", "p.tws", "p.twd", "p.gtq", "p.dpd", "p.dms", "p.fos", "p.lad", "p.ito",
        "p.bel", "p.snd", "p.zoom", "p.lvl",
        "v.prf", "v.hrs", "v.htr", "v.htg", "v.htb", "v.hta", "v.hsg", "v.rrs", "v.stn", "v.exp", "v.lit", "v.ldx", "v.ldy", "v.ldz",
        "v.stp", "v.amb", "v.sb", "v.sd", "v.lsz", "v.sen", "v.sms", "v.blen", "v.blm", "v.crt", "v.cex", "v.cbl", "v.ced", "v.clv", "v.csb", "v.cwp", "v.ces",
        "v.tst", "v.tc1r", "v.tc1g", "v.tc1b", "v.tc2r", "v.tc2g", "v.tc2b",
        "v.wsd", "v.wbr", "v.wps", "v.crs", "v.csg", "v.crg", "v.cdk",
        "v.rmat", "v.rgls", "v.rdwp", "v.rsss", "v.redg", "v.rsat",
        "v.rmbp", "v.rbsc", "v.rcao", "v.rlgw", "v.rsg", "v.rss", "v.rsen", "v.rsw", "v.rsd", "v.rsdk", "v.rshl", "v.rsca", "v.rscs", "v.rsrn", "v.rspo", "v.rcren", "v.rcra", "v.rcrw", "v.rcrd", "v.sqcs", "v.rsc", "v.wrm",
        "w.sf", "w.sb", "w.th", "w.tl", "w.gd", "w.bb", "w.bd", "w.sn",
        "w.ss", "w.rg", "w.sp", "w.rm", "w.es", "w.ps", "w.pa", "w.cs", "w.ca", "w.sa",
        "v.rpal"
    ]

    func resetToDefaults() {
        particleCount = Defaults.particleCount
        gravity = Defaults.gravity
        damping = Defaults.damping
        constraintIterations = Defaults.constraintIterations
        settleSteps = Defaults.settleSteps
        bendCompliance = Defaults.bendCompliance
        bendVelocityCoupling = Defaults.bendVelocityCoupling
        dragHeight = Defaults.dragHeight
        liftHeight = Defaults.liftHeight
        ropeTension = Defaults.ropeTension
        frictionCoefficient = Defaults.frictionCoefficient
        frictionDampingRatio = Defaults.frictionDampingRatio
        maxFrictionCap = Defaults.maxFrictionCap
        boardFrictionRatio = Defaults.boardFrictionRatio
        collisionResponse = Defaults.collisionResponse
        zSeparation = Defaults.zSeparation
        twistStiffness = Defaults.twistStiffness
        twistDamping = Defaults.twistDamping
        gravityTorque = Defaults.gravityTorque
        dragPickupDuration = Defaults.dragPickupDuration
        dragMinSubsteps = Defaults.dragMinSubsteps
        fadeOutSpeed = Defaults.fadeOutSpeed
        lowerAnimDuration = Defaults.lowerAnimDuration
        idleTimeout = Defaults.idleTimeout
        zoomScale = 1.0
        frictionSoundEnabled = true
        profileSegments = 10
        holeRadiusScale = 0.734
        holeTintR = 1; holeTintG = 0.89; holeTintB = 1; holeTintAmount = 1
        holeSegments = 19
        ropeRadiusScale = 0.767
        stretchThinning = 0.027
        exposure = 0.735
        lightIntensity = 1.24
        lightDirX = -0.131
        lightDirY = -0.156
        lightDirZ = 0.125
        updateLightDir()
        shadowType = .pcss
        bloomStrength = 0
        cartoonShaderEnabled = false
        cartoonExposure = 1.33
        cartoonBloom = 0
        cartoonEdgeStrength = 1
        cartoonLevels = 2
        cartoonShadowBright = 0.38
        cartoonWrap = 0.15
        cartoonEdgeSmooth = 0.5
        tableStyle = .wood
        tableColor1R = 0.08
        tableColor1G = 0.09
        tableColor1B = 0.13
        tableColor2R = 0.12
        tableColor2G = 0.13
        tableColor2B = 0.2
        updateTableColor()
        woodSeed = 0.613
        woodBrightness = 1.18
        woodPatternScale = 3.95
        capRadiusScale = 0.91
        capSegments = 12
        capRings = 6
        capDarken = 0
        ropeMatte = 0.69
        ropeGloss = 1.17
        ropeDiffuseWrap = 0.045
        ropeSubsurface = 0.303
        ropeEdgeLight = 0.009
        ropeSaturation = 0.979
        ropeMicroBump = 1.5
        ropeBumpScale = 7.37
        ropeContactAO = 0.649
        ropeLiftGlow = 0
        ropeStretchGloss = 0.879
        ropeStretchSpec = 0.371
        ropeEnvReflect = 0.15
        ropeEnvSpread = 0.15
        ropeSeamEnabled = false
        ropeSeamWidth = 0.06
        ropeSeamDepth = 0.45
        ropeSeamDarkness = 1.4
        ropeSeamHighlight = 0.35
        ropeSeamCrackAmount = 0.45
        ropeSeamCrackScale = 18.0
        ropeSeamRandomize = true
        ropeSeamPosition = 0.5
        ropeCracksEnabled = false
        ropeCrackAmount = 0.45
        ropeCrackWidth = 0.16
        ropeCrackDepth = 0.5
        squareCrossSection = true
        renderScale = 1.0
        wormMode = false
        wormSegFreq = 28.0; wormSegBulge = 0.12; wormThickness = 1.35; wormTaperLen = 0.12
        wormGrooveDepth = 0.35; wormBellyBright = 1.15; wormBackDark = 0.7; wormSkinNoise = 0.08
        wormSSS = 0.25; wormRoughness = 0.25; wormSpecular = 0.8; wormRimStrength = 0.08
        wormEyeSize = 0.015; wormPulseSpeed = 2.5; wormPulseAmp = 0.02
        wormCrawlSpeed = 3.5; wormCrawlAmp = 0.012; wormSideAmp = 0.008
        ropePalette = .original
        Self.allKeys.forEach { UserDefaults.standard.removeObject(forKey: $0) }
    }

    func undo() {
        guard let renderer = renderer else { return }
        renderer.performUndo()
    }

    func bestRecord(for level: Int) -> Int? {
        let key = "record.lvl.\(level)"
        return UserDefaults.standard.object(forKey: key) != nil ? UserDefaults.standard.integer(forKey: key) : nil
    }

    func saveRecord(for level: Int, moves: Int) {
        let key = "record.lvl.\(level)"
        UserDefaults.standard.set(moves, forKey: key)
    }

    func onLevelComplete() {
        let moves = moveCount
        let level = currentLevel
        let prev = bestRecord(for: level)
        previousRecord = prev
        if prev == nil || moves < prev! {
            isNewRecord = true
            saveRecord(for: level, moves: moves)
        } else {
            isNewRecord = false
        }
        percentile = nil
        globalBest = nil
        starCount = 3
        showLevelComplete = true

        let timeMs = Int((Date().timeIntervalSince(levelStartTime ?? Date())) * 1000)
        Task {
            if isNewRecord {
                let _ = await LeaderboardAPI.shared.submitResult(
                    levelId: level, moves: moves, timeMs: timeMs
                )
            }
            let stats = await LeaderboardAPI.shared.fetchStats(levelId: level, moves: moves)
            self.percentile = stats?.percentile
            if let best = stats?.best_moves, best > 0 {
                self.globalBest = best
                if moves <= best || moves <= Int(ceil(Double(best) * 1.5)) {
                    self.starCount = 3
                } else if moves > best * 5 {
                    self.starCount = 1
                } else {
                    self.starCount = 2
                }
            }
        }
    }

    func updateLeaderboardUsername(_ newName: String) {
        LeaderboardAPI.shared.setUsername(newName)
        leaderboardUsername = newName
    }

    func loginAsPlayer(_ name: String) {
        LeaderboardAPI.shared.loginAs(name)
        leaderboardUsername = name
    }

    func resetCamera() {
        renderer?.resetCameraOrientation()
    }

    private func updateBraidTargets() {
        guard let renderer = renderer, renderer.isBraidMode else {
            braidTargetEntries = []
            return
        }
        braidTargetEntries = renderer.ropes.indices.compactMap { i in
            guard i < renderer.braidTargets.count else { return nil }
            let targetHole = renderer.braidTargets[i]
            let slot = targetHole - renderer.braidBottomHoleStart
            return BraidTargetEntry(strandColor: renderer.ropes[i].color, targetSlot: slot)
        }
    }

    func loadLevel(_ id: Int) {
        guard let renderer = renderer else { return }
        renderer.loadLevel(levelId: id)
        currentLevel = id
        if ropePalette != .original { applyCurrentPalette() }
        levelStartTime = Date()
        percentile = nil
        updateBraidTargets()
    }

    func loadLevelDefinition(_ def: LevelDefinition) {
        guard let renderer = renderer else { return }
        renderer.loadLevelDefinition(def)
        if ropePalette != .original { applyCurrentPalette() }
        currentLevel = def.id
        levelStartTime = Date()
        percentile = nil
        updateBraidTargets()
    }

    func dumpSettingsToClipboard() -> Bool {
        let dict: [String: Any] = [
            "particleCount": Int(particleCount),
            "gravity": gravity,
            "damping": damping,
            "constraintIterations": Int(constraintIterations),
            "settleSteps": Int(settleSteps),
            "bendCompliance": bendCompliance,
            "bendVelocityCoupling": bendVelocityCoupling,
            "dragHeight": dragHeight,
            "liftHeight": liftHeight,
            "ropeTension": ropeTension,
            "frictionCoefficient": frictionCoefficient,
            "frictionDampingRatio": frictionDampingRatio,
            "maxFrictionCap": maxFrictionCap,
            "boardFrictionRatio": boardFrictionRatio,
            "collisionResponse": collisionResponse,
            "zSeparation": zSeparation,
            "twistStiffness": twistStiffness,
            "twistDamping": twistDamping,
            "gravityTorque": gravityTorque,
            "dragPickupDuration": dragPickupDuration,
            "dragMinSubsteps": Int(dragMinSubsteps),
            "fadeOutSpeed": fadeOutSpeed,
            "lowerAnimDuration": lowerAnimDuration,
            "idleTimeout": idleTimeout,
            "profileSegments": Int(profileSegments),
            "holeRadiusScale": holeRadiusScale,
            "holeSegments": Int(holeSegments),
            "ropeRadiusScale": ropeRadiusScale,
            "stretchThinning": stretchThinning,
            "holeTintR": holeTintR,
            "holeTintG": holeTintG,
            "holeTintB": holeTintB,
            "holeTintAmount": holeTintAmount,
            "exposure": exposure,
            "bloomStrength": bloomStrength,
            "shadowType": shadowType.label,
            "shadowsEnabled": shadowsEnabled,
            "lightIntensity": lightIntensity,
            "lightDirX": lightDirX,
            "lightDirY": lightDirY,
            "lightDirZ": lightDirZ,
            "ambient": ambient,
            "shadowBias": shadowBias,
            "shadowDarkness": shadowDarkness,
            "lightSize": lightSize,
            "cartoonShaderEnabled": cartoonShaderEnabled,
            "cartoonExposure": cartoonExposure,
            "cartoonBloom": cartoonBloom,
            "cartoonEdgeStrength": cartoonEdgeStrength,
            "cartoonLevels": Int(cartoonLevels),
            "tableStyle": tableStyle.label,
            "tableColor1R": tableColor1R,
            "tableColor1G": tableColor1G,
            "tableColor1B": tableColor1B,
            "tableColor2R": tableColor2R,
            "tableColor2G": tableColor2G,
            "tableColor2B": tableColor2B,
            "woodSeed": woodSeed,
            "woodBrightness": woodBrightness,
            "woodPatternScale": woodPatternScale,
            "capRadiusScale": capRadiusScale,
            "capSegments": Int(capSegments),
            "capRings": Int(capRings),
            "capDarken": capDarken,
            "frictionSoundEnabled": frictionSoundEnabled,
            "zoomScale": zoomScale,
            "currentLevel": currentLevel,
            "ropeMatte": ropeMatte,
            "ropeGloss": ropeGloss,
            "ropeDiffuseWrap": ropeDiffuseWrap,
            "ropeSubsurface": ropeSubsurface,
            "ropeEdgeLight": ropeEdgeLight,
            "ropeSaturation": ropeSaturation,
            "ropeMicroBump": ropeMicroBump,
            "ropeBumpScale": ropeBumpScale,
            "ropeContactAO": ropeContactAO,
            "ropeLiftGlow": ropeLiftGlow,
            "ropeStretchGloss": ropeStretchGloss,
            "ropeStretchSpec": ropeStretchSpec,
            "ropeEnvReflect": ropeEnvReflect,
            "ropeEnvSpread": ropeEnvSpread,
            "ropeOpacity": ropeOpacity,
            "ropeSeamEnabled": ropeSeamEnabled,
            "ropeSeamWidth": ropeSeamWidth,
            "ropeSeamDepth": ropeSeamDepth,
            "ropeSeamDarkness": ropeSeamDarkness,
            "ropeSeamHighlight": ropeSeamHighlight,
            "ropeSeamCrackAmount": ropeSeamCrackAmount,
            "ropeSeamCrackScale": ropeSeamCrackScale,
            "ropeSeamRandomize": ropeSeamRandomize,
            "ropeSeamPosition": ropeSeamPosition,
            "ropeCracksEnabled": ropeCracksEnabled,
            "ropeCrackAmount": ropeCrackAmount,
            "ropeCrackWidth": ropeCrackWidth,
            "ropeCrackDepth": ropeCrackDepth,
            "squareCrossSection": squareCrossSection,
            "chainMode": chainMode,
            "chainLinkLength": chainLinkLength,
            "chainLinkThickness": chainLinkThickness,
            "chainLinkWidth": chainLinkWidth,
            "wormMode": wormMode,
            "renderScale": renderScale,
            "ropePalette": ropePalette.rawValue
        ]
        guard let data = try? JSONSerialization.data(withJSONObject: dict, options: [.prettyPrinted, .sortedKeys]),
              let str = String(data: data, encoding: .utf8) else { return false }
        #if os(iOS)
        UIPasteboard.general.string = str
        #elseif os(macOS)
        NSPasteboard.general.clearContents()
        NSPasteboard.general.setString(str, forType: .string)
        #endif
        return true
    }

    func dumpGeometryToClipboard() -> Bool {
        guard let sim = renderer?.simulator else { return false }
        var lines: [String] = []
        lines.append("// Geometry dump: \(sim.bands.count) bands")
        for (bi, band) in sim.bands.enumerated() {
            guard band.active else { continue }
            lines.append("// Band \(bi): \(band.positions.count) particles, pinStart=\(band.pinStart ?? -1) pinEnd=\(band.pinEnd ?? -1) segLen=\(String(format: "%.4f", band.segmentLength))")
            for (pi, pos) in band.positions.enumerated() {
                lines.append("  [\(pi)] x=\(String(format: "%.4f", pos.x)) y=\(String(format: "%.4f", pos.y)) z=\(String(format: "%.4f", pos.z))")
            }
        }
        let holes = sim.holePositions
        lines.append("// Holes: \(holes.count)")
        for (hi, h) in holes.enumerated() {
            lines.append("  [\(hi)] x=\(String(format: "%.4f", h.x)) y=\(String(format: "%.4f", h.y))")
        }
        let str = lines.joined(separator: "\n")
        #if os(iOS)
        UIPasteboard.general.string = str
        #elseif os(macOS)
        NSPasteboard.general.clearContents()
        NSPasteboard.general.setString(str, forType: .string)
        #endif
        return true
    }

    func importSettingsFromClipboard() -> Bool {
        #if os(iOS)
        guard let str = UIPasteboard.general.string else { return false }
        #elseif os(macOS)
        guard let str = NSPasteboard.general.string(forType: .string) else { return false }
        #else
        return false
        #endif
        guard let data = str.data(using: .utf8),
              let dict = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return false }

        func f(_ key: String) -> Float? {
            if let v = dict[key] as? Double { return Float(v) }
            if let v = dict[key] as? Int { return Float(v) }
            return nil
        }
        func b(_ key: String) -> Bool? { dict[key] as? Bool }

        if let v = f("particleCount") { particleCount = v }
        if let v = f("gravity") { gravity = v }
        if let v = f("damping") { damping = v }
        if let v = f("constraintIterations") { constraintIterations = v }
        if let v = f("settleSteps") { settleSteps = v }
        if let v = f("bendCompliance") { bendCompliance = v }
        if let v = f("bendVelocityCoupling") { bendVelocityCoupling = v }
        if let v = f("dragHeight") { dragHeight = v }
        if let v = f("liftHeight") { liftHeight = v }
        if let v = f("ropeTension") { ropeTension = v }
        if let v = f("frictionCoefficient") { frictionCoefficient = v }
        if let v = f("frictionDampingRatio") { frictionDampingRatio = v }
        if let v = f("maxFrictionCap") { maxFrictionCap = v }
        if let v = f("boardFrictionRatio") { boardFrictionRatio = v }
        if let v = f("collisionResponse") { collisionResponse = v }
        if let v = f("zSeparation") { zSeparation = v }
        if let v = f("twistStiffness") { twistStiffness = v }
        if let v = f("twistDamping") { twistDamping = v }
        if let v = f("gravityTorque") { gravityTorque = v }
        if let v = f("dragPickupDuration") { dragPickupDuration = v }
        if let v = f("dragMinSubsteps") { dragMinSubsteps = v }
        if let v = f("fadeOutSpeed") { fadeOutSpeed = v }
        if let v = f("lowerAnimDuration") { lowerAnimDuration = v }
        if let v = f("idleTimeout") { idleTimeout = v }
        if let v = f("profileSegments") { profileSegments = v }
        if let v = f("holeRadiusScale") { holeRadiusScale = v }
        if let v = f("holeSegments") { holeSegments = v }
        if let v = f("ropeRadiusScale") { ropeRadiusScale = v }
        if let v = f("stretchThinning") { stretchThinning = v }
        if let v = f("holeTintR") { holeTintR = v }
        if let v = f("holeTintG") { holeTintG = v }
        if let v = f("holeTintB") { holeTintB = v }
        if let v = f("holeTintAmount") { holeTintAmount = v }
        if let v = f("exposure") { exposure = v }
        if let v = f("bloomStrength") { bloomStrength = v }
        if let s = dict["shadowType"] as? String {
            for t in ShadowType.allCases where t.label == s { shadowType = t; break }
        }
        if let v = b("shadowsEnabled") { shadowsEnabled = v }
        if let v = f("lightIntensity") { lightIntensity = v }
        if let v = f("lightDirX") { lightDirX = v }
        if let v = f("lightDirY") { lightDirY = v }
        if let v = f("lightDirZ") { lightDirZ = v }
        if let v = f("ambient") { ambient = v }
        if let v = f("shadowBias") { shadowBias = v }
        if let v = f("shadowDarkness") { shadowDarkness = v }
        if let v = f("lightSize") { lightSize = v }
        if let v = b("cartoonShaderEnabled") { cartoonShaderEnabled = v }
        if let v = f("cartoonExposure") { cartoonExposure = v }
        if let v = f("cartoonBloom") { cartoonBloom = v }
        if let v = f("cartoonEdgeStrength") { cartoonEdgeStrength = v }
        if let v = f("cartoonLevels") { cartoonLevels = v }
        if let s = dict["tableStyle"] as? String {
            for t in TableStyle.allCases where t.label == s { tableStyle = t; break }
        }
        if let v = f("tableColor1R") { tableColor1R = v }
        if let v = f("tableColor1G") { tableColor1G = v }
        if let v = f("tableColor1B") { tableColor1B = v }
        if let v = f("tableColor2R") { tableColor2R = v }
        if let v = f("tableColor2G") { tableColor2G = v }
        if let v = f("tableColor2B") { tableColor2B = v }
        if let v = f("woodSeed") { woodSeed = v }
        if let v = f("woodBrightness") { woodBrightness = v }
        if let v = f("woodPatternScale") { woodPatternScale = v }
        if let v = f("capRadiusScale") { capRadiusScale = v }
        if let v = f("capSegments") { capSegments = v }
        if let v = f("capRings") { capRings = v }
        if let v = f("capDarken") { capDarken = v }
        if let v = b("frictionSoundEnabled") { frictionSoundEnabled = v }
        if let v = f("zoomScale") { zoomScale = v }
        if let v = f("ropeMatte") { ropeMatte = v }
        if let v = f("ropeGloss") { ropeGloss = v }
        if let v = f("ropeDiffuseWrap") { ropeDiffuseWrap = v }
        if let v = f("ropeSubsurface") { ropeSubsurface = v }
        if let v = f("ropeEdgeLight") { ropeEdgeLight = v }
        if let v = f("ropeSaturation") { ropeSaturation = v }
        if let v = f("ropeMicroBump") { ropeMicroBump = v }
        if let v = f("ropeBumpScale") { ropeBumpScale = v }
        if let v = f("ropeContactAO") { ropeContactAO = v }
        if let v = f("ropeLiftGlow") { ropeLiftGlow = v }
        if let v = f("ropeStretchGloss") { ropeStretchGloss = v }
        if let v = f("ropeStretchSpec") { ropeStretchSpec = v }
        if let v = f("ropeEnvReflect") { ropeEnvReflect = v }
        if let v = f("ropeOpacity") { ropeOpacity = v }
        if let v = f("ropeEnvSpread") { ropeEnvSpread = v }
        if let v = b("ropeSeamEnabled") { ropeSeamEnabled = v }
        if let v = f("ropeSeamWidth") { ropeSeamWidth = v }
        if let v = f("ropeSeamDepth") { ropeSeamDepth = v }
        if let v = f("ropeSeamDarkness") { ropeSeamDarkness = v }
        if let v = f("ropeSeamHighlight") { ropeSeamHighlight = v }
        if let v = f("ropeSeamCrackAmount") { ropeSeamCrackAmount = v }
        if let v = f("ropeSeamCrackScale") { ropeSeamCrackScale = v }
        if let v = b("ropeSeamRandomize") { ropeSeamRandomize = v }
        if let v = f("ropeSeamPosition") { ropeSeamPosition = v }
        if let v = b("ropeCracksEnabled") { ropeCracksEnabled = v }
        if let v = f("ropeCrackAmount") { ropeCrackAmount = v }
        if let v = f("ropeCrackWidth") { ropeCrackWidth = v }
        if let v = f("ropeCrackDepth") { ropeCrackDepth = v }
        if let v = b("squareCrossSection") { squareCrossSection = v }
        if let v = b("chainMode") { chainMode = v }
        if let v = f("chainLinkLength") { chainLinkLength = v }
        if let v = f("chainLinkThickness") { chainLinkThickness = v }
        if let v = f("chainLinkWidth") { chainLinkWidth = v }
        if let v = b("wormMode") { wormMode = v }
        if let v = f("renderScale") { renderScale = v }
        if let v = f("ropePalette") { ropePalette = RopePalette(rawValue: Int(v)) ?? .original }
        if let v = f("wormSegFreq") { wormSegFreq = v }
        if let v = f("wormSegBulge") { wormSegBulge = v }
        if let v = f("wormThickness") { wormThickness = v }
        if let v = f("wormTaperLen") { wormTaperLen = v }
        if let v = f("wormGrooveDepth") { wormGrooveDepth = v }
        if let v = f("wormBellyBright") { wormBellyBright = v }
        if let v = f("wormBackDark") { wormBackDark = v }
        if let v = f("wormSkinNoise") { wormSkinNoise = v }
        if let v = f("wormSSS") { wormSSS = v }
        if let v = f("wormRoughness") { wormRoughness = v }
        if let v = f("wormSpecular") { wormSpecular = v }
        if let v = f("wormRimStrength") { wormRimStrength = v }
        if let v = f("wormEyeSize") { wormEyeSize = v }
        if let v = f("wormPulseSpeed") { wormPulseSpeed = v }
        if let v = f("wormPulseAmp") { wormPulseAmp = v }
        if let v = f("wormCrawlSpeed") { wormCrawlSpeed = v }
        if let v = f("wormCrawlAmp") { wormCrawlAmp = v }
        if let v = f("wormSideAmp") { wormSideAmp = v }

        if let lvl = dict["currentLevel"] as? Int, lvl >= 1 {
            loadLevel(lvl)
        }

        return true
    }

}

@MainActor
private func configureGameView(_ view: GameMTKView, controller: GameController, coordinator: GameViewCoordinator) {
    let renderer = Renderer(view: view)
    coordinator.renderer = renderer
    controller.renderer = renderer
    renderer.onLevelComplete = { [weak controller] in
        DispatchQueue.main.async {
            controller?.onLevelComplete()
        }
    }
    renderer.onMoveCountChanged = { [weak controller] count in
        DispatchQueue.main.async {
            controller?.moveCount = count
        }
    }
    renderer.onUndoStackChanged = { [weak controller] hasEntries in
        DispatchQueue.main.async {
            controller?.canUndo = hasEntries
        }
    }

    renderer.cameraZoomScale = controller.zoomScale
    renderer.onZoomChanged = { [weak controller] newZoom in
        DispatchQueue.main.async {
            controller?.zoomScale = newZoom
        }
    }

    renderer.physicsParticleCount = Int(controller.particleCount)
    renderer.physicsGravity = controller.gravity
    renderer.physicsDamping = controller.damping
    renderer.physicsConstraintIterations = Int(controller.constraintIterations)
    renderer.physicsBroadphaseInterval = Int(controller.broadphaseRebuildInterval)
    renderer.physicsSettleSteps = Int(controller.settleSteps)
    renderer.physicsBendCompliance = controller.bendCompliance
    renderer.physicsBendVelocityCoupling = controller.bendVelocityCoupling
    renderer.dragHeight = controller.dragHeight
    renderer.physicsLiftHeight = controller.liftHeight
    renderer.physicsRopeTension = controller.ropeTension
    renderer.physicsFriction = controller.frictionCoefficient
    renderer.physicsFrictionDampingRatio = controller.frictionDampingRatio
    renderer.physicsMaxFrictionCap = controller.maxFrictionCap
    renderer.physicsBoardFrictionRatio = controller.boardFrictionRatio
    renderer.physicsCollisionResponse = controller.collisionResponse
    renderer.physicsZSeparation = controller.zSeparation
    renderer.physicsTwistStiffness = controller.twistStiffness
    renderer.physicsTwistDamping = controller.twistDamping
    renderer.physicsGravityTorque = controller.gravityTorque
    renderer.physicsDragPickupDuration = controller.dragPickupDuration
    renderer.physicsDragMinSubsteps = Int(controller.dragMinSubsteps)
    renderer.physicsFadeOutSpeed = controller.fadeOutSpeed
    renderer.physicsLowerAnimDuration = controller.lowerAnimDuration
    renderer.physicsIdleTimeout = controller.idleTimeout
    renderer.boardElevation = controller.boardElevation
    renderer.frictionSound.enabled = controller.frictionSoundEnabled
    renderer.profileSegments = Int(controller.profileSegments)
    renderer.holeRadiusScale = controller.holeRadiusScale
    renderer.shaderParams.holeTint = SIMD4<Float>(controller.holeTintR, controller.holeTintG, controller.holeTintB, controller.holeTintAmount)
    renderer.holeSegments = Int(controller.holeSegments)
    renderer.ropeRadiusScale = controller.ropeRadiusScale
    renderer.stretchThinning = controller.stretchThinning
    renderer.shaderParams.exposure = controller.exposure
    renderer.shaderParams.lightIntensity = controller.lightIntensity
    var ld = SIMD3<Float>(controller.lightDirX, controller.lightDirY, controller.lightDirZ)
    if simd_length_squared(ld) < 1e-6 { ld = SIMD3<Float>(-0.65, -0.35, 0.67) }
    renderer.shaderParams.lightDir = simd_normalize(ld)
    renderer.shaderParams.shadowType = controller.shadowType.rawValue
    renderer.shaderParams.ambient = controller.ambient
    renderer.shaderParams.shadowBias = controller.shadowBias
    renderer.shaderParams.shadowDarkness = controller.shadowDarkness
    renderer.shaderParams.lightSize = controller.lightSize
    renderer.shaderParams.shadowsEnabled = controller.shadowsEnabled
    renderer.shadowMapSize = controller.shadowMapSize
    renderer.shaderParams.bloomEnabled = controller.bloomEnabled
    renderer.shaderParams.bloomStrength = controller.bloomStrength
    renderer.shaderParams.cartoonShaderEnabled = controller.cartoonShaderEnabled
    renderer.shaderParams.cartoonExposure = controller.cartoonExposure
    renderer.shaderParams.cartoonBloom = controller.cartoonBloom
    renderer.shaderParams.cartoonEdgeStrength = controller.cartoonEdgeStrength
    renderer.shaderParams.cartoonLevels = Int(controller.cartoonLevels)
    renderer.shaderParams.cartoonShadowBright = controller.cartoonShadowBright
    renderer.shaderParams.cartoonWrap = controller.cartoonWrap
    renderer.shaderParams.cartoonEdgeSmooth = controller.cartoonEdgeSmooth
    renderer.shaderParams.ropeFlatNormals = controller.ropeFlatNormals
    renderer.shaderParams.pcssPenumbraScale = controller.pcssPenumbraScale
    renderer.shaderParams.tableStyle = controller.tableStyle.rawValue
    renderer.shaderParams.tableColor1 = SIMD3<Float>(controller.tableColor1R, controller.tableColor1G, controller.tableColor1B)
    renderer.shaderParams.tableColor2 = SIMD3<Float>(controller.tableColor2R, controller.tableColor2G, controller.tableColor2B)
    renderer.woodSeed = controller.woodSeed
    renderer.woodBrightness = controller.woodBrightness
    renderer.woodPatternScale = controller.woodPatternScale
    renderer.shaderParams.capRadiusScale = controller.capRadiusScale
    renderer.shaderParams.capSegments = Int(controller.capSegments)
    renderer.shaderParams.capRings = Int(controller.capRings)
    renderer.shaderParams.capDarken = controller.capDarken
    renderer.shaderParams.ropeMatte = controller.ropeMatte
    renderer.shaderParams.ropeGloss = controller.ropeGloss
    renderer.shaderParams.ropeDiffuseWrap = controller.ropeDiffuseWrap
    renderer.shaderParams.ropeSubsurface = controller.ropeSubsurface
    renderer.shaderParams.ropeEdgeLight = controller.ropeEdgeLight
    renderer.shaderParams.ropeSaturation = controller.ropeSaturation
    renderer.shaderParams.ropeMicroBump = controller.ropeMicroBump
    renderer.shaderParams.ropeBumpScale = controller.ropeBumpScale
    renderer.shaderParams.ropeContactAO = controller.ropeContactAO
    renderer.shaderParams.ropeLiftGlow = controller.ropeLiftGlow
    renderer.shaderParams.ropeStretchGloss = controller.ropeStretchGloss
    renderer.shaderParams.ropeStretchSpec = controller.ropeStretchSpec
    renderer.shaderParams.ropeEnvReflect = controller.ropeEnvReflect
    renderer.shaderParams.ropeEnvSpread = controller.ropeEnvSpread
    renderer.shaderParams.ropeOpacity = controller.ropeOpacity
    renderer.shaderParams.ropeSeamEnabled = controller.ropeSeamEnabled
    renderer.shaderParams.ropeSeamWidth = controller.ropeSeamWidth
    renderer.shaderParams.ropeSeamDepth = controller.ropeSeamDepth
    renderer.shaderParams.ropeSeamDarkness = controller.ropeSeamDarkness
    renderer.shaderParams.ropeSeamHighlight = controller.ropeSeamHighlight
    renderer.shaderParams.ropeSeamCrackAmount = controller.ropeSeamCrackAmount
    renderer.shaderParams.ropeSeamCrackScale = controller.ropeSeamCrackScale
    renderer.shaderParams.ropeSeamRandomize = controller.ropeSeamRandomize
    renderer.shaderParams.ropeSeamPosition = controller.ropeSeamPosition
    renderer.shaderParams.ropeCracksEnabled = controller.ropeCracksEnabled
    renderer.shaderParams.ropeCrackAmount = controller.ropeCrackAmount
    renderer.shaderParams.ropeCrackWidth = controller.ropeCrackWidth
    renderer.shaderParams.ropeCrackDepth = controller.ropeCrackDepth
    renderer.shaderParams.ropeEnvDebug = controller.ropeEnvDebug
    renderer.shaderParams.shadowDebugMode = controller.shadowDebugMode
    renderer.squareCrossSection = controller.squareCrossSection
    renderer.shaderParams.chainMode = controller.chainMode
    renderer.shaderParams.chainLinkLength = controller.chainLinkLength
    renderer.shaderParams.chainLinkThickness = controller.chainLinkThickness
    renderer.shaderParams.chainLinkWidth = controller.chainLinkWidth
    renderer.shaderParams.wormMode = controller.wormMode
    renderer.shaderParams.wormSegFreq = controller.wormSegFreq
    renderer.shaderParams.wormSegBulge = controller.wormSegBulge
    renderer.shaderParams.wormThickness = controller.wormThickness
    renderer.shaderParams.wormTaperLen = controller.wormTaperLen
    renderer.shaderParams.wormGrooveDepth = controller.wormGrooveDepth
    renderer.shaderParams.wormBellyBright = controller.wormBellyBright
    renderer.shaderParams.wormBackDark = controller.wormBackDark
    renderer.shaderParams.wormSkinNoise = controller.wormSkinNoise
    renderer.shaderParams.wormSSS = controller.wormSSS
    renderer.shaderParams.wormRoughness = controller.wormRoughness
    renderer.shaderParams.wormSpecular = controller.wormSpecular
    renderer.shaderParams.wormRimStrength = controller.wormRimStrength
    renderer.shaderParams.wormEyeSize = controller.wormEyeSize
    renderer.shaderParams.wormPulseSpeed = controller.wormPulseSpeed
    renderer.shaderParams.wormPulseAmp = controller.wormPulseAmp
    renderer.shaderParams.wormCrawlSpeed = controller.wormCrawlSpeed
    renderer.shaderParams.wormCrawlAmp = controller.wormCrawlAmp
    renderer.shaderParams.wormSideAmp = controller.wormSideAmp
    renderer.renderScale = controller.renderScale

    view.delegate = renderer
    view.onTouch = { phase, location in
        renderer.handleTouch(phase: phase, location: location, in: view)
    }
    view.onCameraPan = { translation in
        renderer.handleCameraPan(translation: translation, in: view)
    }
    view.onCameraRotation = { delta in
        renderer.handleCameraRotation(delta: delta)
    }
    view.onCameraZoom = { scale in
        renderer.handleCameraZoom(scale: scale)
    }
    view.onCameraSpin = { delta in
        renderer.handleCameraSpin(delta: delta)
    }
    renderer.loadLevel(levelId: controller.currentLevel)
    if controller.ropePalette != .original {
        controller.applyCurrentPalette()
    }
}

final class GameViewCoordinator {
    var renderer: Renderer?
}

#if os(iOS)
struct GameView: UIViewRepresentable {
    @ObservedObject var controller: GameController

    func makeUIView(context: Context) -> MTKView {
        let view = GameMTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float_stencil8
        view.clearColor = MTLClearColor(red: 0.07, green: 0.08, blue: 0.12, alpha: 1.0)
        view.preferredFramesPerSecond = 120
        view.framebufferOnly = false
        view.isOpaque = true
        view.backgroundColor = .black
        view.alpha = 1.0

        configureGameView(view, controller: controller, coordinator: context.coordinator)
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}

    func makeCoordinator() -> GameViewCoordinator { GameViewCoordinator() }
}

#elseif os(macOS)
struct GameView: NSViewRepresentable {
    @ObservedObject var controller: GameController

    func makeNSView(context: Context) -> MTKView {
        let view = GameMTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float_stencil8
        view.clearColor = MTLClearColor(red: 0.07, green: 0.08, blue: 0.12, alpha: 1.0)
        view.preferredFramesPerSecond = 120
        view.framebufferOnly = false
        view.layer?.isOpaque = true

        configureGameView(view, controller: controller, coordinator: context.coordinator)
        return view
    }

    func updateNSView(_ nsView: MTKView, context: Context) {}

    func makeCoordinator() -> GameViewCoordinator { GameViewCoordinator() }
}
#endif
