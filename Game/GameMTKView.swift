import MetalKit

final class GameMTKView: MTKView {
    var onTouch: ((UITouch.Phase, CGPoint) -> Void)?

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        _ = touches
        if let currentTouch = event?.allTouches?.first(where: { $0.view === self }) {
            onTouch?(.began, currentTouch.location(in: self))
        }
    }

    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        _ = touches
        if let currentTouch = event?.allTouches?.first(where: { $0.view === self }) {
            onTouch?(.moved, currentTouch.location(in: self))
        }
    }

    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        _ = touches
        if let currentTouch = event?.allTouches?.first(where: { $0.view === self }) {
            onTouch?(.ended, currentTouch.location(in: self))
        }
    }

    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        _ = touches
        if let currentTouch = event?.allTouches?.first(where: { $0.view === self }) {
            onTouch?(.cancelled, currentTouch.location(in: self))
        }
    }
}

