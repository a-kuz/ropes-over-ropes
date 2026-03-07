import SwiftUI
import MetalKit

// MARK: - FireworksView

struct FireworksView: View {
    let variant: Int   // 0, 1, 2, or 3

    var body: some View {
        _FireworksRepresentable(variant: variant)
    }
}

// MARK: - Platform wrappers

#if canImport(UIKit)
private struct _FireworksRepresentable: UIViewRepresentable {
    let variant: Int
    func makeUIView(context: Context) -> MTKView {
        let view = MTKView()
        _setupMTKView(view, coordinator: context.coordinator)
        return view
    }
    func updateUIView(_ uiView: MTKView, context: Context) {}
    func makeCoordinator() -> FireworksCoordinator { FireworksCoordinator(variant: variant) }
}
#else
private struct _FireworksRepresentable: NSViewRepresentable {
    let variant: Int
    func makeNSView(context: Context) -> MTKView {
        let view = MTKView()
        _setupMTKView(view, coordinator: context.coordinator)
        return view
    }
    func updateNSView(_ nsView: MTKView, context: Context) {}
    func makeCoordinator() -> FireworksCoordinator { FireworksCoordinator(variant: variant) }
}
#endif

private func _setupMTKView(_ view: MTKView, coordinator: FireworksCoordinator) {
    view.device = MTLCreateSystemDefaultDevice()
    view.delegate = coordinator
    view.framebufferOnly = true
    view.isPaused = false
    view.enableSetNeedsDisplay = false
    view.preferredFramesPerSecond = 60
    view.clearColor = MTLClearColorMake(0, 0, 0, 1)
#if canImport(UIKit)
    view.contentScaleFactor = 0.5
#endif
    coordinator.setup(view: view)
}

// MARK: - Coordinator

final class FireworksCoordinator: NSObject, MTKViewDelegate, @unchecked Sendable {

    private let variant: Int
    private var pipeline: MTLRenderPipelineState?
    private var commandQueue: MTLCommandQueue?
    private let startTime: Double = CACurrentMediaTime()

    init(variant: Int) { self.variant = variant }

    func setup(view: MTKView) {
        guard let device = view.device else { return }
        commandQueue = device.makeCommandQueue()

        guard let lib  = device.makeDefaultLibrary(),
              let vert = lib.makeFunction(name: "fireworks_vertex") else { return }

        let fragName: String
        switch variant {
        case 1:  fragName = "fireworks2_fragment"
        case 2:  fragName = "fireworks3_fragment"
        case 3:  fragName = "fireworks4_fragment"
        default: fragName = "fireworks_fragment"
        }

        guard let frag = lib.makeFunction(name: fragName) else { return }

        let desc = MTLRenderPipelineDescriptor()
        desc.vertexFunction   = vert
        desc.fragmentFunction = frag
        desc.colorAttachments[0].pixelFormat = view.colorPixelFormat
        pipeline = try? device.makeRenderPipelineState(descriptor: desc)
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        guard let pipeline,
              let commandQueue,
              let drawable   = view.currentDrawable,
              let renderPass = view.currentRenderPassDescriptor,
              let cmdBuf     = commandQueue.makeCommandBuffer(),
              let enc        = cmdBuf.makeRenderCommandEncoder(descriptor: renderPass)
        else { return }

        let size = view.drawableSize
        var uniforms = FWUniforms(
            resolution: SIMD2<Float>(Float(size.width), Float(size.height)),
            time: Float(CACurrentMediaTime() - startTime)
        )

        enc.setRenderPipelineState(pipeline)
        enc.setFragmentBytes(&uniforms, length: MemoryLayout<FWUniforms>.stride, index: 0)
        enc.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
        enc.endEncoding()
        cmdBuf.present(drawable)
        cmdBuf.commit()
    }
}

// MARK: - Uniforms (matches Fireworks.metal)

private struct FWUniforms {
    var resolution: SIMD2<Float>
    var time: Float
}
