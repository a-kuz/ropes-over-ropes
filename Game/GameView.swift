import MetalKit
import SwiftUI

class GameController: ObservableObject {
    weak var renderer: Renderer?
    
    @Published var stepMultiplier: Float = 1.0 {
        didSet { renderer?.hookStepMultiplier = stepMultiplier }
    }
    
    @Published var hookRadiusMultiplier: Float = 1.0 {
        didSet { renderer?.hookRadiusMultiplier = hookRadiusMultiplier }
    }
    
    @Published var stepLimitMultiplier: Float = 1.0 {
        didSet { renderer?.hookStepLimitMultiplier = stepLimitMultiplier }
    }
    
    @Published var debugSegmentColors: Bool = false {
        didSet { renderer?.debugSegmentColors = debugSegmentColors }
    }
    
    @Published var smoothSubdivisions: Float = 6 {
        didSet { renderer?.smoothSubdivisions = Int(smoothSubdivisions) }
    }
    
    @Published var smoothIterations: Float = 6 {
        didSet { renderer?.smoothIterations = Int(smoothIterations) }
    }
    
    @Published var smoothStrength: Float = 0.4 {
        didSet { renderer?.smoothStrength = smoothStrength }
    }
    
    @Published var smoothZone: Float = 1.0 {
        didSet { renderer?.smoothZone = smoothZone }
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
