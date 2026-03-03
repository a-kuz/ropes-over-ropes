#if os(iOS)
import UIKit
#elseif os(macOS)
import AppKit
#endif
import MetalKit
import SwiftUI

class GameController: ObservableObject {
    weak var renderer: Renderer?

    struct Defaults {
        // Rope
        static let particleCount: Float = 122
        static let gravity: Float = -14.79
        static let damping: Float = 0.809
        // Solver
        static let constraintIterations: Float = 7
        static let settleSteps: Float = 19
        static let bendCompliance: Float = 0.0089
        static let bendVelocityCoupling: Float = 0.65
        // Drag
        static let dragHeight: Float = 0.35
        static let liftHeight: Float = 0.3
        static let ropeTension: Float = 0.5
        static let boardElevation: Float = 0.12
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

    @Published var boardElevation: Float = Defaults.boardElevation {
        didSet { renderer?.boardElevation = boardElevation; persist("p.bel", boardElevation) }
    }

    @Published var profileSegments: Float = 10 {
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
    @Published var ropeRadiusScale: Float = 0.767 {
        didSet { renderer?.ropeRadiusScale = ropeRadiusScale; persist("v.rrs", ropeRadiusScale) }
    }
    @Published var stretchThinning: Float = 0.027 {
        didSet { renderer?.stretchThinning = stretchThinning; persist("v.stn", stretchThinning) }
    }
    @Published var exposure: Float = 0.735 {
        didSet { renderer?.exposure = exposure; persist("v.exp", exposure) }
    }
    @Published var lightIntensity: Float = 1.24 {
        didSet { renderer?.lightIntensity = lightIntensity; persist("v.lit", lightIntensity) }
    }
    @Published var lightDirX: Float = -0.131 {
        didSet { updateLightDir(); persist("v.ldx", lightDirX) }
    }
    @Published var lightDirY: Float = -0.156 {
        didSet { updateLightDir(); persist("v.ldy", lightDirY) }
    }
    @Published var lightDirZ: Float = 0.125 {
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

    @Published var shadowType: ShadowType = .pcss {
        didSet { renderer?.shadowType = shadowType.rawValue; persist("v.stp", Float(shadowType.rawValue)) }
    }
    @Published var ambient: Float = 0.064 {
        didSet { renderer?.ambient = ambient; persist("v.amb", ambient) }
    }
    @Published var shadowBias: Float = 0.0001 {
        didSet { renderer?.shadowBias = shadowBias; persist("v.sb", shadowBias) }
    }
    @Published var shadowDarkness: Float = 0 {
        didSet { renderer?.shadowDarkness = shadowDarkness; persist("v.sd", shadowDarkness) }
    }
    @Published var lightSize: Float = 0.0069 {
        didSet { renderer?.lightSize = lightSize; persist("v.lsz", lightSize) }
    }
    @Published var shadowsEnabled: Bool = true {
        didSet { renderer?.shadowsEnabled = shadowsEnabled; persist("v.sen", shadowsEnabled ? 1 : 0) }
    }

    private func updateLightDir() {
        var d = SIMD3<Float>(lightDirX, lightDirY, lightDirZ)
        if simd_length_squared(d) < 1e-6 { d = SIMD3<Float>(-0.65, -0.35, 0.67) }
        renderer?.lightDir = simd_normalize(d)
    }
    @Published var bloomStrength: Float = 0 {
        didSet { renderer?.bloomStrength = bloomStrength; persist("v.blm", bloomStrength) }
    }
    @Published var cartoonShaderEnabled: Bool = false {
        didSet { renderer?.cartoonShaderEnabled = cartoonShaderEnabled; persist("v.crt", cartoonShaderEnabled ? 1 : 0) }
    }
    @Published var cartoonExposure: Float = 1.33 {
        didSet { renderer?.cartoonExposure = cartoonExposure; persist("v.cex", cartoonExposure) }
    }
    @Published var cartoonBloom: Float = 0 {
        didSet { renderer?.cartoonBloom = cartoonBloom; persist("v.cbl", cartoonBloom) }
    }
    @Published var cartoonEdgeStrength: Float = 1 {
        didSet { renderer?.cartoonEdgeStrength = cartoonEdgeStrength; persist("v.ced", cartoonEdgeStrength) }
    }
    @Published var cartoonLevels: Float = 2 {
        didSet { renderer?.cartoonLevels = Int(cartoonLevels); persist("v.clv", cartoonLevels) }
    }
    @Published var cartoonShadowBright: Float = 0.38 {
        didSet { renderer?.cartoonShadowBright = cartoonShadowBright; persist("v.csb", cartoonShadowBright) }
    }
    @Published var cartoonWrap: Float = 0.15 {
        didSet { renderer?.cartoonWrap = cartoonWrap; persist("v.cwp", cartoonWrap) }
    }
    @Published var cartoonEdgeSmooth: Float = 0.5 {
        didSet { renderer?.cartoonEdgeSmooth = cartoonEdgeSmooth; persist("v.ces", cartoonEdgeSmooth) }
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

    @Published var tableStyle: TableStyle = .wood {
        didSet { renderer?.tableStyle = tableStyle.rawValue; persist("v.tst", Float(tableStyle.rawValue)) }
    }
    @Published var tableColor1R: Float = 0.08 { didSet { updateTableColor(); persist("v.tc1r", tableColor1R) } }
    @Published var tableColor1G: Float = 0.09 { didSet { updateTableColor(); persist("v.tc1g", tableColor1G) } }
    @Published var tableColor1B: Float = 0.13 { didSet { updateTableColor(); persist("v.tc1b", tableColor1B) } }
    @Published var tableColor2R: Float = 0.12 { didSet { updateTableColor(); persist("v.tc2r", tableColor2R) } }
    @Published var tableColor2G: Float = 0.13 { didSet { updateTableColor(); persist("v.tc2g", tableColor2G) } }
    @Published var tableColor2B: Float = 0.2 { didSet { updateTableColor(); persist("v.tc2b", tableColor2B) } }
    @Published var woodSeed: Float = 0.613 {
        didSet { renderer?.woodSeed = woodSeed; persist("v.wsd", woodSeed) }
    }
    @Published var woodBrightness: Float = 1.18 {
        didSet { renderer?.woodBrightness = woodBrightness; persist("v.wbr", woodBrightness) }
    }
    @Published var woodPatternScale: Float = 3.95 {
        didSet { renderer?.woodPatternScale = woodPatternScale; persist("v.wps", woodPatternScale) }
    }
    @Published var capRadiusScale: Float = 0.91 {
        didSet { renderer?.capRadiusScale = capRadiusScale; persist("v.crs", capRadiusScale) }
    }
    @Published var capSegments: Float = 12 {
        didSet { renderer?.capSegments = Int(capSegments); persist("v.csg", capSegments) }
    }
    @Published var capRings: Float = 6 {
        didSet { renderer?.capRings = Int(capRings); persist("v.crg", capRings) }
    }
    @Published var capDarken: Float = 0 {
        didSet { renderer?.capDarken = capDarken; persist("v.cdk", capDarken) }
    }

    @Published var ropeMatte: Float = 0.69 {
        didSet { renderer?.ropeMatte = ropeMatte; persist("v.rmat", ropeMatte) }
    }
    @Published var ropeGloss: Float = 1.17 {
        didSet { renderer?.ropeGloss = ropeGloss; persist("v.rgls", ropeGloss) }
    }
    @Published var ropeDiffuseWrap: Float = 0.045 {
        didSet { renderer?.ropeDiffuseWrap = ropeDiffuseWrap; persist("v.rdwp", ropeDiffuseWrap) }
    }
    @Published var ropeSubsurface: Float = 0.0072 {
        didSet { renderer?.ropeSubsurface = ropeSubsurface; persist("v.rsss", ropeSubsurface) }
    }
    @Published var ropeEdgeLight: Float = 0.009 {
        didSet { renderer?.ropeEdgeLight = ropeEdgeLight; persist("v.redg", ropeEdgeLight) }
    }
    @Published var ropeSaturation: Float = 0.979 {
        didSet { renderer?.ropeSaturation = ropeSaturation; persist("v.rsat", ropeSaturation) }
    }
    @Published var ropeMicroBump: Float = 1.5 {
        didSet { renderer?.ropeMicroBump = ropeMicroBump; persist("v.rmbp", ropeMicroBump) }
    }
    @Published var ropeBumpScale: Float = 7.37 {
        didSet { renderer?.ropeBumpScale = ropeBumpScale; persist("v.rbsc", ropeBumpScale) }
    }
    @Published var ropeContactAO: Float = 0.649 {
        didSet { renderer?.ropeContactAO = ropeContactAO; persist("v.rcao", ropeContactAO) }
    }
    @Published var ropeLiftGlow: Float = 0 {
        didSet { renderer?.ropeLiftGlow = ropeLiftGlow; persist("v.rlgw", ropeLiftGlow) }
    }
    @Published var ropeStretchGloss: Float = 0.879 {
        didSet { renderer?.ropeStretchGloss = ropeStretchGloss; persist("v.rsg", ropeStretchGloss) }
    }
    @Published var ropeStretchSpec: Float = 0.371 {
        didSet { renderer?.ropeStretchSpec = ropeStretchSpec; persist("v.rss", ropeStretchSpec) }
    }
    @Published var wormMode: Bool = false {
        didSet { renderer?.wormMode = wormMode; persist("v.wrm", wormMode ? 1 : 0) }
    }
    @Published var wormSegFreq: Float = 28.0 {
        didSet { renderer?.wormSegFreq = wormSegFreq; persist("w.sf", wormSegFreq) }
    }
    @Published var wormSegBulge: Float = 0.12 {
        didSet { renderer?.wormSegBulge = wormSegBulge; persist("w.sb", wormSegBulge) }
    }
    @Published var wormThickness: Float = 1.35 {
        didSet { renderer?.wormThickness = wormThickness; persist("w.th", wormThickness) }
    }
    @Published var wormTaperLen: Float = 0.12 {
        didSet { renderer?.wormTaperLen = wormTaperLen; persist("w.tl", wormTaperLen) }
    }
    @Published var wormGrooveDepth: Float = 0.35 {
        didSet { renderer?.wormGrooveDepth = wormGrooveDepth; persist("w.gd", wormGrooveDepth) }
    }
    @Published var wormBellyBright: Float = 1.15 {
        didSet { renderer?.wormBellyBright = wormBellyBright; persist("w.bb", wormBellyBright) }
    }
    @Published var wormBackDark: Float = 0.7 {
        didSet { renderer?.wormBackDark = wormBackDark; persist("w.bd", wormBackDark) }
    }
    @Published var wormSkinNoise: Float = 0.08 {
        didSet { renderer?.wormSkinNoise = wormSkinNoise; persist("w.sn", wormSkinNoise) }
    }
    @Published var wormSSS: Float = 0.25 {
        didSet { renderer?.wormSSS = wormSSS; persist("w.ss", wormSSS) }
    }
    @Published var wormRoughness: Float = 0.25 {
        didSet { renderer?.wormRoughness = wormRoughness; persist("w.rg", wormRoughness) }
    }
    @Published var wormSpecular: Float = 0.8 {
        didSet { renderer?.wormSpecular = wormSpecular; persist("w.sp", wormSpecular) }
    }
    @Published var wormRimStrength: Float = 0.08 {
        didSet { renderer?.wormRimStrength = wormRimStrength; persist("w.rm", wormRimStrength) }
    }
    @Published var wormEyeSize: Float = 0.015 {
        didSet { renderer?.wormEyeSize = wormEyeSize; persist("w.es", wormEyeSize) }
    }
    @Published var wormPulseSpeed: Float = 2.5 {
        didSet { renderer?.wormPulseSpeed = wormPulseSpeed; persist("w.ps", wormPulseSpeed) }
    }
    @Published var wormPulseAmp: Float = 0.02 {
        didSet { renderer?.wormPulseAmp = wormPulseAmp; persist("w.pa", wormPulseAmp) }
    }
    @Published var wormCrawlSpeed: Float = 3.5 {
        didSet { renderer?.wormCrawlSpeed = wormCrawlSpeed; persist("w.cs", wormCrawlSpeed) }
    }
    @Published var wormCrawlAmp: Float = 0.012 {
        didSet { renderer?.wormCrawlAmp = wormCrawlAmp; persist("w.ca", wormCrawlAmp) }
    }
    @Published var wormSideAmp: Float = 0.008 {
        didSet { renderer?.wormSideAmp = wormSideAmp; persist("w.sa", wormSideAmp) }
    }
    @Published var squareCrossSection: Bool = true {
        didSet { renderer?.squareCrossSection = squareCrossSection; persist("v.sqcs", squareCrossSection ? 1 : 0) }
    }
    @Published var renderScale: Float = 1.0 {
        didSet { renderer?.renderScale = renderScale; persist("v.rsc", renderScale) }
    }

    private func updateHoleTint() {
        renderer?.holeTint = SIMD4<Float>(holeTintR, holeTintG, holeTintB, holeTintAmount)
    }

    private func updateTableColor() {
        renderer?.tableColor1 = SIMD3<Float>(tableColor1R, tableColor1G, tableColor1B)
        renderer?.tableColor2 = SIMD3<Float>(tableColor2R, tableColor2G, tableColor2B)
    }

    @Published var zoomScale: Float = 1.0 {
        didSet { renderer?.cameraZoomScale = zoomScale; persist("p.zoom", zoomScale) }
    }

    @Published var fps: Float = 0
    @Published var currentLevel: Int = 1 {
        didSet { persist("p.lvl", Float(currentLevel)) }
    }
    @Published var showLevelComplete: Bool = false
    @Published var canUndo: Bool = false
    @Published var frictionSoundEnabled: Bool = true {
        didSet { renderer?.frictionSound.enabled = frictionSoundEnabled; persist("p.snd", frictionSoundEnabled ? 1 : 0) }
    }
    @Published var profilerActive: Bool = false {
        didSet { PhysicsProfiler.shared.enabled = profilerActive }
    }
    @Published var profilerSummary: String = ""

    private var fpsTimer: Timer?

    private func persist(_ key: String, _ v: Float) { UserDefaults.standard.set(v, forKey: key) }

    init() {
        loadSaved()
        fpsTimer = Timer.scheduledTimer(withTimeInterval: 0.25, repeats: true) { [weak self] _ in
            guard let self, let r = self.renderer else { return }
            self.fps = r.currentFPS
            if self.currentLevel != r.currentLevelId {
                self.currentLevel = r.currentLevelId
            }
            if self.profilerActive {
                self.profilerSummary = PhysicsProfiler.shared.summaryString()
            }
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
        if let v = f("p.drg") { dragHeight = v }
        if let v = f("p.lft") { liftHeight = v }
        if let v = f("p.rtn") { ropeTension = v }
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
        if let v = f("v.blm") { bloomStrength = v }
        if let v = f("v.crt") { cartoonShaderEnabled = v > 0.5 }
        if let v = f("v.cex") { cartoonExposure = v }
        if let v = f("v.cbl") { cartoonBloom = v }
        if let v = f("v.ced") { cartoonEdgeStrength = v }
        if let v = f("v.clv") { cartoonLevels = v }
        if let v = f("v.csb") { cartoonShadowBright = v }
        if let v = f("v.cwp") { cartoonWrap = v }
        if let v = f("v.ces") { cartoonEdgeSmooth = v }
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
        if let v = f("v.sqcs") { squareCrossSection = v > 0.5 }
        if let v = f("v.rsc"), v > 0 { renderScale = v }
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
        updateHoleTint()
    }

    private static let allKeys = [
        "p.ptc", "p.grv", "p.dmp", "p.cit", "p.stl", "p.bcp", "p.bvc", "p.drg", "p.lft", "p.rtn", "p.bel", "p.snd", "p.zoom", "p.lvl",
        "v.prf", "v.hrs", "v.htr", "v.htg", "v.htb", "v.hta", "v.hsg", "v.rrs", "v.stn", "v.exp", "v.lit", "v.ldx", "v.ldy", "v.ldz",
        "v.stp", "v.amb", "v.sb", "v.sd", "v.lsz", "v.sen", "v.blm", "v.crt", "v.cex", "v.cbl", "v.ced", "v.clv", "v.csb", "v.cwp", "v.ces",
        "v.tst", "v.tc1r", "v.tc1g", "v.tc1b", "v.tc2r", "v.tc2g", "v.tc2b",
        "v.wsd", "v.wbr", "v.wps", "v.crs", "v.csg", "v.crg", "v.cdk",
        "v.rmat", "v.rgls", "v.rdwp", "v.rsss", "v.redg", "v.rsat",
        "v.rmbp", "v.rbsc", "v.rcao", "v.rlgw", "v.rsg", "v.rss", "v.sqcs", "v.rsc", "v.wrm",
        "w.sf", "w.sb", "w.th", "w.tl", "w.gd", "w.bb", "w.bd", "w.sn",
        "w.ss", "w.rg", "w.sp", "w.rm", "w.es", "w.ps", "w.pa", "w.cs", "w.ca", "w.sa"
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
        ropeSubsurface = 0.0072
        ropeEdgeLight = 0.009
        ropeSaturation = 0.979
        ropeMicroBump = 1.5
        ropeBumpScale = 7.37
        ropeContactAO = 0.649
        ropeLiftGlow = 0
        ropeStretchGloss = 0.879
        ropeStretchSpec = 0.371
        squareCrossSection = true
        renderScale = 1.0
        wormMode = false
        wormSegFreq = 28.0; wormSegBulge = 0.12; wormThickness = 1.35; wormTaperLen = 0.12
        wormGrooveDepth = 0.35; wormBellyBright = 1.15; wormBackDark = 0.7; wormSkinNoise = 0.08
        wormSSS = 0.25; wormRoughness = 0.25; wormSpecular = 0.8; wormRimStrength = 0.08
        wormEyeSize = 0.015; wormPulseSpeed = 2.5; wormPulseAmp = 0.02
        wormCrawlSpeed = 3.5; wormCrawlAmp = 0.012; wormSideAmp = 0.008
        Self.allKeys.forEach { UserDefaults.standard.removeObject(forKey: $0) }
    }

    func undo() {
        guard let renderer = renderer else { return }
        renderer.performUndo()
    }

    func restartLevel() {
        guard let renderer = renderer else { return }
        renderer.loadLevel(levelId: renderer.currentLevelId)
    }

    func loadLevel(_ id: Int) {
        guard let renderer = renderer else { return }
        renderer.loadLevel(levelId: id)
        currentLevel = id
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
            "squareCrossSection": squareCrossSection,
            "wormMode": wormMode,
            "renderScale": renderScale
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

    func dumpTopology() -> URL? {
        guard let renderer = renderer,
              let topology = renderer.topology else { return nil }

        let holes = renderer.holePositions.map {
            LevelDefinition.Vec2(xPosition: $0.x, yPosition: $0.y)
        }

        let levelRopes = renderer.ropes.map { rope in
            LevelDefinition.Rope(
                startHole: rope.startHole,
                endHole: rope.endHole,
                color: LevelDefinition.Color(
                    redChannel: rope.color.x,
                    greenChannel: rope.color.y,
                    blueChannel: rope.color.z
                ),
                radius: rope.radius
            )
        }

        var levelHooks: [LevelDefinition.Hook] = []
        for (_, hook) in topology.hooks.sorted(by: { $0.key < $1.key }) {
            guard hook.N != 0 else { continue }
            levelHooks.append(LevelDefinition.Hook(
                ropeA: .init(fromType: "hole", index: hook.ropeA, hookIndex: nil),
                ropeB: .init(fromType: "hole", index: hook.ropeB, hookIndex: nil),
                N: hook.N,
                ropeAStartIsOver: hook.ropeAStartIsOver
            ))
        }

        let def = LevelDefinition(
            id: renderer.currentLevelId,
            holeRadius: renderer.holeRadius,
            particlesPerRope: renderer.physicsParticleCount,
            holes: holes,
            ropes: levelRopes,
            hooks: levelHooks.isEmpty ? nil : levelHooks,
            actions: nil,
            boards: nil
        )

        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        guard let data = try? encoder.encode(def) else { return nil }

        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        let ts = Int(Date().timeIntervalSince1970)
        let url = docs.appendingPathComponent("level_\(renderer.currentLevelId)_\(ts).json")
        try? data.write(to: url)
        return url
    }
}

@MainActor
private func configureGameView(_ view: GameMTKView, controller: GameController, coordinator: GameViewCoordinator) {
    let renderer = Renderer(view: view)
    coordinator.renderer = renderer
    controller.renderer = renderer
    renderer.onLevelComplete = { [weak controller] in
        DispatchQueue.main.async {
            controller?.showLevelComplete = true
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
    renderer.physicsSettleSteps = Int(controller.settleSteps)
    renderer.physicsBendCompliance = controller.bendCompliance
    renderer.physicsBendVelocityCoupling = controller.bendVelocityCoupling
    renderer.dragHeight = controller.dragHeight
    renderer.physicsLiftHeight = controller.liftHeight
    renderer.physicsRopeTension = controller.ropeTension
    renderer.boardElevation = controller.boardElevation
    renderer.frictionSound.enabled = controller.frictionSoundEnabled
    renderer.profileSegments = Int(controller.profileSegments)
    renderer.holeRadiusScale = controller.holeRadiusScale
    renderer.holeTint = SIMD4<Float>(controller.holeTintR, controller.holeTintG, controller.holeTintB, controller.holeTintAmount)
    renderer.holeSegments = Int(controller.holeSegments)
    renderer.ropeRadiusScale = controller.ropeRadiusScale
    renderer.stretchThinning = controller.stretchThinning
    renderer.exposure = controller.exposure
    renderer.lightIntensity = controller.lightIntensity
    var ld = SIMD3<Float>(controller.lightDirX, controller.lightDirY, controller.lightDirZ)
    if simd_length_squared(ld) < 1e-6 { ld = SIMD3<Float>(-0.65, -0.35, 0.67) }
    renderer.lightDir = simd_normalize(ld)
    renderer.shadowType = controller.shadowType.rawValue
    renderer.ambient = controller.ambient
    renderer.shadowBias = controller.shadowBias
    renderer.shadowDarkness = controller.shadowDarkness
    renderer.lightSize = controller.lightSize
    renderer.shadowsEnabled = controller.shadowsEnabled
    renderer.bloomStrength = controller.bloomStrength
    renderer.cartoonShaderEnabled = controller.cartoonShaderEnabled
    renderer.cartoonExposure = controller.cartoonExposure
    renderer.cartoonBloom = controller.cartoonBloom
    renderer.cartoonEdgeStrength = controller.cartoonEdgeStrength
    renderer.cartoonLevels = Int(controller.cartoonLevels)
    renderer.cartoonShadowBright = controller.cartoonShadowBright
    renderer.cartoonWrap = controller.cartoonWrap
    renderer.cartoonEdgeSmooth = controller.cartoonEdgeSmooth
    renderer.tableStyle = controller.tableStyle.rawValue
    renderer.tableColor1 = SIMD3<Float>(controller.tableColor1R, controller.tableColor1G, controller.tableColor1B)
    renderer.tableColor2 = SIMD3<Float>(controller.tableColor2R, controller.tableColor2G, controller.tableColor2B)
    renderer.woodSeed = controller.woodSeed
    renderer.woodBrightness = controller.woodBrightness
    renderer.woodPatternScale = controller.woodPatternScale
    renderer.capRadiusScale = controller.capRadiusScale
    renderer.capSegments = Int(controller.capSegments)
    renderer.capRings = Int(controller.capRings)
    renderer.capDarken = controller.capDarken
    renderer.ropeMatte = controller.ropeMatte
    renderer.ropeGloss = controller.ropeGloss
    renderer.ropeDiffuseWrap = controller.ropeDiffuseWrap
    renderer.ropeSubsurface = controller.ropeSubsurface
    renderer.ropeEdgeLight = controller.ropeEdgeLight
    renderer.ropeSaturation = controller.ropeSaturation
    renderer.ropeMicroBump = controller.ropeMicroBump
    renderer.ropeBumpScale = controller.ropeBumpScale
    renderer.ropeContactAO = controller.ropeContactAO
    renderer.ropeLiftGlow = controller.ropeLiftGlow
    renderer.ropeStretchGloss = controller.ropeStretchGloss
    renderer.ropeStretchSpec = controller.ropeStretchSpec
    renderer.squareCrossSection = controller.squareCrossSection
    renderer.wormMode = controller.wormMode
    renderer.wormSegFreq = controller.wormSegFreq
    renderer.wormSegBulge = controller.wormSegBulge
    renderer.wormThickness = controller.wormThickness
    renderer.wormTaperLen = controller.wormTaperLen
    renderer.wormGrooveDepth = controller.wormGrooveDepth
    renderer.wormBellyBright = controller.wormBellyBright
    renderer.wormBackDark = controller.wormBackDark
    renderer.wormSkinNoise = controller.wormSkinNoise
    renderer.wormSSS = controller.wormSSS
    renderer.wormRoughness = controller.wormRoughness
    renderer.wormSpecular = controller.wormSpecular
    renderer.wormRimStrength = controller.wormRimStrength
    renderer.wormEyeSize = controller.wormEyeSize
    renderer.wormPulseSpeed = controller.wormPulseSpeed
    renderer.wormPulseAmp = controller.wormPulseAmp
    renderer.wormCrawlSpeed = controller.wormCrawlSpeed
    renderer.wormCrawlAmp = controller.wormCrawlAmp
    renderer.wormSideAmp = controller.wormSideAmp
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
    view.onCameraDebugToggle = {
        renderer.cameraDebugMode.toggle()
    }

    renderer.loadLevel(levelId: controller.currentLevel)
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
