import MetalKit

#if os(iOS)
import UIKit

final class GameMTKView: MTKView {
    var onTouch: ((InputPhase, CGPoint) -> Void)?
    var onCameraPan: ((SIMD2<Float>) -> Void)?
    var onCameraRotation: ((Float) -> Void)?
    var onCameraZoom: ((Float) -> Void)?
    var onCameraDebugToggle: (() -> Void)?

    private var lastPanLocation: CGPoint?
    private var lastPinchScale: CGFloat = 1.0
    private var tapCount = 0
    private var lastTapTime: Date?

    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self }
        
        if viewTouches.count == 2 {
            if let touch1 = viewTouches.first, let touch2 = viewTouches.dropFirst().first {
                let loc1 = touch1.location(in: self)
                let loc2 = touch2.location(in: self)
                let center = CGPoint(x: (loc1.x + loc2.x) / 2, y: (loc1.y + loc2.y) / 2)
                lastPanLocation = center
                
                let dx = loc2.x - loc1.x
                let dy = loc2.y - loc1.y
                let distance = sqrt(dx * dx + dy * dy)
                lastPinchScale = distance
            }
        } else if viewTouches.count == 1 {
            if let currentTouch = viewTouches.first {
                onTouch?(.began, currentTouch.location(in: self))
            }
        }
    }

    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self }
        
        if viewTouches.count == 2 {
            if let touch1 = viewTouches.first, let touch2 = viewTouches.dropFirst().first {
                let loc1 = touch1.location(in: self)
                let loc2 = touch2.location(in: self)
                let center = CGPoint(x: (loc1.x + loc2.x) / 2, y: (loc1.y + loc2.y) / 2)
                
                if let lastPan = lastPanLocation {
                    let panDeltaX = Float(center.x - lastPan.x)
                    let panDeltaY = Float(center.y - lastPan.y)
                    onCameraPan?(SIMD2<Float>(panDeltaX, panDeltaY))
                    let rotationDelta = -panDeltaY / Float(bounds.height) * Float.pi * 0.5
                    onCameraRotation?(rotationDelta)
                    lastPanLocation = center
                }
                
                let dx = loc2.x - loc1.x
                let dy = loc2.y - loc1.y
                let distance = sqrt(dx * dx + dy * dy)
                if lastPinchScale > 0 {
                    let scale = Float(distance / lastPinchScale)
                    onCameraZoom?(scale)
                    lastPinchScale = distance
                }
            }
        } else if viewTouches.count == 1 {
            if let currentTouch = viewTouches.first {
                onTouch?(.moved, currentTouch.location(in: self))
            }
        }
    }

    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self }
        
        if viewTouches.count == 2 {
            lastPanLocation = nil
            lastPinchScale = 1.0
        } else if viewTouches.count == 1 {
            if let currentTouch = viewTouches.first {
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
        guard let event = event, let allTouches = event.allTouches else { return }
        let viewTouches = allTouches.filter { $0.view === self }
        
        if viewTouches.count == 2 {
            lastPanLocation = nil 
            lastPinchScale = 1.0
        } else if viewTouches.count == 1 {
            tapCount = 0
            lastTapTime = nil
            if let currentTouch = viewTouches.first {
                onTouch?(.cancelled, currentTouch.location(in: self))
            }
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
