import MetalKit
import SwiftUI

class GameController: ObservableObject {
    weak var renderer: Renderer?

    struct Defaults {
        static let particleCount: Float = 6
        static let gravity: Float = -5.0
        static let damping: Float = 0.97
        static let constraintIterations: Float = 20
        static let dragHeight: Float = 0.35
    }

    @Published var particleCount: Float = Defaults.particleCount {
        didSet { renderer?.physicsParticleCount = Int(particleCount); persist("p.ptc", particleCount) }
    }
    @Published var gravity: Float = Defaults.gravity {
        didSet { renderer?.physicsGravity = gravity; persist("p.grv", gravity) }
    }
    @Published var damping: Float = Defaults.damping {
        didSet { renderer?.physicsDamping = damping; persist("p.dmp", damping) }
    }
    @Published var constraintIterations: Float = Defaults.constraintIterations {
        didSet { renderer?.physicsConstraintIterations = Int(constraintIterations); persist("p.cit", constraintIterations) }
    }
    @Published var dragHeight: Float = Defaults.dragHeight {
        didSet { renderer?.dragHeight = dragHeight; persist("p.drg", dragHeight) }
    }

    private func persist(_ key: String, _ v: Float) { UserDefaults.standard.set(v, forKey: key) }

    init() {
        loadSaved()
    }

    private func loadSaved() {
        let ud = UserDefaults.standard
        func f(_ k: String) -> Float? { ud.object(forKey: k) != nil ? ud.float(forKey: k) : nil }

        if let v = f("p.ptc") { particleCount = v }
        if let v = f("p.grv") { gravity = v }
        if let v = f("p.dmp") { damping = v }
        if let v = f("p.cit") { constraintIterations = v }
        if let v = f("p.drg") { dragHeight = v }
    }

    private static let allKeys = [
        "p.ptc", "p.grv", "p.dmp", "p.cit", "p.drg"
    ]

    func resetToDefaults() {
        particleCount = Defaults.particleCount
        gravity = Defaults.gravity
        damping = Defaults.damping
        constraintIterations = Defaults.constraintIterations
        dragHeight = Defaults.dragHeight
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

        renderer.physicsParticleCount = Int(controller.particleCount)
        renderer.physicsGravity = controller.gravity
        renderer.physicsDamping = controller.damping
        renderer.physicsConstraintIterations = Int(controller.constraintIterations)
        renderer.dragHeight = controller.dragHeight

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
