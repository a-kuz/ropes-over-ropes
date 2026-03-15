import MetalKit

#if os(iOS)
import UIKit

final class GameMTKView: MTKView {
    var onTouch: ((InputPhase, CGPoint) -> Void)?
    var onCameraPan: ((SIMD2<Float>) -> Void)?
    var onCameraRotation: ((Float) -> Void)?
    var onCameraZoom: ((Float) -> Void)?
    var onCameraSpin: ((Float) -> Void)?

    private var multiTouchActive = false
    private var singleTouchCancelled = false
    private var threeFingerSpan: CGFloat = 0

    override func didMoveToSuperview() {
        super.didMoveToSuperview()
        isMultipleTouchEnabled = true
    }

    private func averageSpan(_ touches: [UITouch]) -> CGFloat {
        guard touches.count >= 3 else { return 0 }
        let pts = touches.map { $0.location(in: self) }
        var maxDist: CGFloat = 0
        for i in 0..<pts.count {
            for j in (i+1)..<pts.count {
                let dx = pts[i].x - pts[j].x
                let dy = pts[i].y - pts[j].y
                let d = sqrt(dx * dx + dy * dy)
                if d > maxDist { maxDist = d }
            }
        }
        return maxDist
    }

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self && $0.phase != .ended && $0.phase != .cancelled }

        if viewTouches.count >= 3 {
            multiTouchActive = true
            if !singleTouchCancelled {
                singleTouchCancelled = true
                onTouch?(.cancelled, .zero)
            }
            threeFingerSpan = averageSpan(Array(viewTouches))
        } else if viewTouches.count >= 2 {
            multiTouchActive = true
            if !singleTouchCancelled {
                singleTouchCancelled = true
                onTouch?(.cancelled, .zero)
            }
        } else if viewTouches.count == 1 && !multiTouchActive {
            if let currentTouch = viewTouches.first {
                singleTouchCancelled = false
                onTouch?(.began, currentTouch.location(in: self))
            }
        }
    }

    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self && $0.phase != .ended && $0.phase != .cancelled }

        if viewTouches.count >= 3 {
            if !multiTouchActive {
                multiTouchActive = true
                if !singleTouchCancelled {
                    singleTouchCancelled = true
                    onTouch?(.cancelled, .zero)
                }
                threeFingerSpan = averageSpan(Array(viewTouches))
            } else {
                let newSpan = averageSpan(Array(viewTouches))
                if threeFingerSpan > 1 {
                    let scale = Float(threeFingerSpan / newSpan)
                    onCameraZoom?(scale)
                }
                threeFingerSpan = newSpan
            }
        } else if viewTouches.count >= 2 {
            if !multiTouchActive {
                multiTouchActive = true
                if !singleTouchCancelled {
                    singleTouchCancelled = true
                    onTouch?(.cancelled, .zero)
                }
            }
        } else if viewTouches.count == 1 && !multiTouchActive && !singleTouchCancelled {
            if let currentTouch = viewTouches.first {
                onTouch?(.moved, currentTouch.location(in: self))
            }
        }
    }

    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let remaining = allTouches.filter { $0.view === self && $0.phase != .ended && $0.phase != .cancelled }

        if remaining.isEmpty {
            if multiTouchActive || singleTouchCancelled {
                multiTouchActive = false
                singleTouchCancelled = false
                threeFingerSpan = 0
                return
            }
            if let currentTouch = touches.first {
                onTouch?(.ended, currentTouch.location(in: self))
            }
        }
    }

    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        multiTouchActive = false
        singleTouchCancelled = false
        threeFingerSpan = 0
        if let currentTouch = touches.first {
            onTouch?(.cancelled, currentTouch.location(in: self))
        }
    }
}

#elseif os(macOS)
import AppKit

final class GameMTKView: MTKView {
    var onTouch: ((InputPhase, CGPoint) -> Void)?
    var onCameraPan: ((SIMD2<Float>) -> Void)?
    var onCameraRotation: ((Float) -> Void)?
    var onCameraZoom: ((Float) -> Void)?
    var onCameraSpin: ((Float) -> Void)?

    private var isDragging = false
    private var lastRightDragLocation: CGPoint?

    override var acceptsFirstResponder: Bool { true }

    private func flippedLocation(for event: NSEvent) -> CGPoint {
        let loc = convert(event.locationInWindow, from: nil)
        return CGPoint(x: loc.x, y: bounds.height - loc.y)
    }

    override func mouseDown(with event: NSEvent) {
        let loc = flippedLocation(for: event)
        isDragging = true
        onTouch?(.began, loc)
    }

    override func mouseDragged(with event: NSEvent) {
        guard isDragging else { return }
        let loc = flippedLocation(for: event)
        onTouch?(.moved, loc)
    }

    override func mouseUp(with event: NSEvent) {
        let loc = flippedLocation(for: event)
        isDragging = false
        onTouch?(.ended, loc)
    }

    override func rightMouseDown(with event: NSEvent) {
        lastRightDragLocation = flippedLocation(for: event)
    }

    override func rightMouseDragged(with event: NSEvent) {
        let loc = flippedLocation(for: event)
        if let last = lastRightDragLocation {
            let dx = Float(loc.x - last.x)
            let dy = Float(loc.y - last.y)
            onCameraPan?(SIMD2<Float>(dx, dy))
            let rotationDelta = -dy / Float(bounds.height) * Float.pi * 0.5
            onCameraRotation?(rotationDelta)
        }
        lastRightDragLocation = loc
    }

    override func rightMouseUp(with event: NSEvent) {
        lastRightDragLocation = nil
    }

    override func scrollWheel(with event: NSEvent) {
        let delta = Float(event.scrollingDeltaY)
        if abs(delta) > 0.01 {
            let scale = 1.0 - delta * 0.01
            onCameraZoom?(scale)
        }
    }

    override func magnify(with event: NSEvent) {
        let scale = 1.0 - Float(event.magnification)
        onCameraZoom?(scale)
    }

}
#endif
