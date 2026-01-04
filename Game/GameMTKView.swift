import MetalKit

final class GameMTKView: MTKView {
    var onTouch: ((UITouch.Phase, CGPoint) -> Void)?
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

