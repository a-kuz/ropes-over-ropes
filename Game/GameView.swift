import MetalKit
import SwiftUI

struct GameView: UIViewRepresentable {
    func makeUIView(context: Context) -> MTKView {
        let view = GameMTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float_stencil8
        view.clearColor = MTLClearColor(red: 0.07, green: 0.08, blue: 0.12, alpha: 1.0)
        view.preferredFramesPerSecond = 120
        view.framebufferOnly = false

        let renderer = Renderer(view: view)
        context.coordinator.renderer = renderer
        view.delegate = renderer
        view.onTouch = { phase, location in
            renderer.handleTouch(phase: phase, location: location, in: view)
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

