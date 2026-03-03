import MetalKit

#if os(iOS)
import UIKit

final class GameMTKView: MTKView, UIGestureRecognizerDelegate {
    var onTouch: ((InputPhase, CGPoint) -> Void)?
    var onCameraPan: ((SIMD2<Float>) -> Void)?
    var onCameraRotation: ((Float) -> Void)?
    var onCameraZoom: ((Float) -> Void)?
    var onCameraSpin: ((Float) -> Void)?
    var onCameraDebugToggle: (() -> Void)?

    private var tapCount = 0
    private var lastTapTime: Date?
    private var twoFingerActive = false
    private var singleTouchCancelled = false

    private lazy var pinchGesture: UIPinchGestureRecognizer = {
        let g = UIPinchGestureRecognizer(target: self, action: #selector(handlePinch(_:)))
        g.delegate = self
        return g
    }()

    private lazy var rotationGesture: UIRotationGestureRecognizer = {
        let g = UIRotationGestureRecognizer(target: self, action: #selector(handleRotation(_:)))
        g.delegate = self
        return g
    }()

    override func didMoveToSuperview() {
        super.didMoveToSuperview()
        if pinchGesture.view == nil {
            isMultipleTouchEnabled = true
            addGestureRecognizer(pinchGesture)
            addGestureRecognizer(rotationGesture)
        }
    }

    @objc private func handlePinch(_ gesture: UIPinchGestureRecognizer) {
        switch gesture.state {
        case .began:
            twoFingerActive = true
            if !singleTouchCancelled {
                singleTouchCancelled = true
                onTouch?(.cancelled, .zero)
            }
        case .changed:
            let scale = Float(gesture.scale)
            let zoomScale = 1.0 / scale
            onCameraZoom?(zoomScale)
            gesture.scale = 1.0
        case .ended, .cancelled, .failed:
            twoFingerActive = false
            singleTouchCancelled = false
        default:
            break
        }
    }

    @objc private func handleRotation(_ gesture: UIRotationGestureRecognizer) {
        switch gesture.state {
        case .began:
            twoFingerActive = true
            if !singleTouchCancelled {
                singleTouchCancelled = true
                onTouch?(.cancelled, .zero)
            }
        case .changed:
            onCameraSpin?(Float(gesture.rotation))
            gesture.rotation = 0
        case .ended, .cancelled, .failed:
            twoFingerActive = false
            singleTouchCancelled = false
        default:
            break
        }
    }

    func gestureRecognizer(_ gestureRecognizer: UIGestureRecognizer, shouldRecognizeSimultaneouslyWith otherGestureRecognizer: UIGestureRecognizer) -> Bool {
        true
    }

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self }

        if viewTouches.count >= 2 {
            twoFingerActive = true
            if !singleTouchCancelled {
                singleTouchCancelled = true
                onTouch?(.cancelled, .zero)
            }
        } else if viewTouches.count == 1 && !twoFingerActive {
            if let currentTouch = viewTouches.first {
                singleTouchCancelled = false
                onTouch?(.began, currentTouch.location(in: self))
            }
        }
    }

    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self }

        if viewTouches.count >= 2 {
            if !twoFingerActive {
                twoFingerActive = true
                if !singleTouchCancelled {
                    singleTouchCancelled = true
                    onTouch?(.cancelled, .zero)
                }
            }
        } else if viewTouches.count == 1 && !twoFingerActive && !singleTouchCancelled {
            if let currentTouch = viewTouches.first {
                onTouch?(.moved, currentTouch.location(in: self))
            }
        }
    }

    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let remaining = allTouches.filter { $0.view === self && $0.phase != .ended && $0.phase != .cancelled }

        if remaining.isEmpty {
            if twoFingerActive || singleTouchCancelled {
                twoFingerActive = false
                singleTouchCancelled = false
                return
            }
            if let currentTouch = touches.first {
                let now = Date()
                if let lastTime = lastTapTime, now.timeIntervalSince(lastTime) < 0.5 {
                    tapCount += 1
                } else {
                    tapCount = 1
                }
                lastTapTime = now

                if tapCount >= 3 {
                    tapCount = 0
                    lastTapTime = nil
                    onCameraDebugToggle?()
                } else {
                    onTouch?(.ended, currentTouch.location(in: self))
                }
            }
        }
    }

    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        twoFingerActive = false
        singleTouchCancelled = false
        tapCount = 0
        lastTapTime = nil
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
    var onCameraDebugToggle: (() -> Void)?

    private var isDragging = false
    private var lastRightDragLocation: CGPoint?
    private var clickCount = 0
    private var lastClickTime: Date?

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

        let now = Date()
        if let lastTime = lastClickTime, now.timeIntervalSince(lastTime) < 0.4 {
            clickCount += 1
        } else {
            clickCount = 1
        }
        lastClickTime = now

        if clickCount >= 3 {
            clickCount = 0
            lastClickTime = nil
            onCameraDebugToggle?()
        } else {
            onTouch?(.ended, loc)
        }
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
