import MetalKit
import SwiftUI

class GameController: ObservableObject {
    weak var renderer: Renderer?

    struct Defaults {
        static let stepMultiplier: Float = 0.0200
        static let hookRadiusMultiplier: Float = 1.0
        static let stepLimitMultiplier: Float = 0.300
        static let maturityDistance: Float = 0.08
        static let debugSegmentColors = false
        static let smoothSubdivisions: Float = 1
        static let smoothIterations: Float = 2
        static let smoothStrength: Float = 0.4
        static let dragHeight: Float = 0.35
        static let simpleMode = true
        static let disableHoleDeform = true
    }

    @Published var stepMultiplier: Float = Defaults.stepMultiplier {
        didSet { renderer?.hookStepMultiplier = stepMultiplier; persist("p.step", stepMultiplier) }
    }
    @Published var hookRadiusMultiplier: Float = Defaults.hookRadiusMultiplier {
        didSet { renderer?.hookRadiusMultiplier = hookRadiusMultiplier; persist("p.rad", hookRadiusMultiplier) }
    }
    @Published var stepLimitMultiplier: Float = Defaults.stepLimitMultiplier {
        didSet { renderer?.hookStepLimitMultiplier = stepLimitMultiplier; persist("p.lim", stepLimitMultiplier) }
    }
    @Published var maturityDistance: Float = Defaults.maturityDistance {
        didSet { renderer?.maturityDistance = maturityDistance; persist("p.mat", maturityDistance) }
    }
    @Published var debugSegmentColors: Bool = Defaults.debugSegmentColors {
        didSet { renderer?.debugSegmentColors = debugSegmentColors; persist("p.dbg", debugSegmentColors) }
    }
    @Published var smoothSubdivisions: Float = Defaults.smoothSubdivisions {
        didSet { renderer?.smoothSubdivisions = Int(smoothSubdivisions); persist("p.sub", smoothSubdivisions) }
    }
    @Published var smoothIterations: Float = Defaults.smoothIterations {
        didSet { renderer?.smoothIterations = Int(smoothIterations); persist("p.itr", smoothIterations) }
    }
    @Published var smoothStrength: Float = Defaults.smoothStrength {
        didSet { renderer?.smoothStrength = smoothStrength; persist("p.str", smoothStrength) }
    }
    @Published var dragHeight: Float = Defaults.dragHeight {
        didSet { renderer?.dragHeight = dragHeight; persist("p.drg", dragHeight) }
    }
    @Published var simpleMode: Bool = Defaults.simpleMode {
        didSet { renderer?.ropeRenderSimpleMode = simpleMode; persist("p.sim", simpleMode) }
    }
    @Published var disableHoleDeform: Bool = Defaults.disableHoleDeform {
        didSet { renderer?.ropeRenderDisableHoleDeform = disableHoleDeform; persist("p.hol", disableHoleDeform) }
    }

    private func persist(_ key: String, _ v: Float) { UserDefaults.standard.set(v, forKey: key) }
    private func persist(_ key: String, _ v: Bool) { UserDefaults.standard.set(v, forKey: key) }

    init() {
        loadSaved()
    }

    private func loadSaved() {
        let ud = UserDefaults.standard
        func f(_ k: String) -> Float? { ud.object(forKey: k) != nil ? ud.float(forKey: k) : nil }
        func b(_ k: String) -> Bool? { ud.object(forKey: k) != nil ? ud.bool(forKey: k) : nil }

        if let v = f("p.step") { stepMultiplier = v }
        if let v = f("p.rad") { hookRadiusMultiplier = v }
        if let v = f("p.lim") { stepLimitMultiplier = v }
        if let v = f("p.mat") { maturityDistance = v }
        if let v = b("p.dbg") { debugSegmentColors = v }
        if let v = f("p.sub") { smoothSubdivisions = v }
        if let v = f("p.itr") { smoothIterations = v }
        if let v = f("p.str") { smoothStrength = v }
        if let v = f("p.drg") { dragHeight = v }
        if let v = b("p.sim") { simpleMode = v }
        if let v = b("p.hol") { disableHoleDeform = v }
    }

    private static let allKeys = [
        "p.step", "p.rad", "p.lim", "p.mat", "p.dbg",
        "p.sub", "p.itr", "p.str",
        "p.drg", "p.sim", "p.hol"
    ]

    func resetToDefaults() {
        stepMultiplier = Defaults.stepMultiplier
        hookRadiusMultiplier = Defaults.hookRadiusMultiplier
        stepLimitMultiplier = Defaults.stepLimitMultiplier
        maturityDistance = Defaults.maturityDistance
        debugSegmentColors = Defaults.debugSegmentColors
        smoothSubdivisions = Defaults.smoothSubdivisions
        smoothIterations = Defaults.smoothIterations
        smoothStrength = Defaults.smoothStrength
        dragHeight = Defaults.dragHeight
        simpleMode = Defaults.simpleMode
        disableHoleDeform = Defaults.disableHoleDeform
        Self.allKeys.forEach { UserDefaults.standard.removeObject(forKey: $0) }
    }

    func restartLevel() {
        guard let renderer = renderer else { return }
        renderer.loadLevel(levelId: renderer.currentLevelId)
    }
}

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

        let renderer = Renderer(view: view)
        context.coordinator.renderer = renderer
        controller.renderer = renderer

        renderer.hookStepMultiplier = controller.stepMultiplier
        renderer.hookRadiusMultiplier = controller.hookRadiusMultiplier
        renderer.hookStepLimitMultiplier = controller.stepLimitMultiplier
        renderer.maturityDistance = controller.maturityDistance
        renderer.debugSegmentColors = controller.debugSegmentColors
        renderer.smoothSubdivisions = Int(controller.smoothSubdivisions)
        renderer.smoothIterations = Int(controller.smoothIterations)
        renderer.smoothStrength = controller.smoothStrength
        renderer.dragHeight = controller.dragHeight
        renderer.ropeRenderSimpleMode = controller.simpleMode
        renderer.ropeRenderDisableHoleDeform = controller.disableHoleDeform

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
        view.onCameraDebugToggle = {
            renderer.cameraDebugMode.toggle()
        }

        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {
        _ = uiView
        _ = context
    }

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    final class Coordinator {
        var renderer: Renderer?
    }
}
