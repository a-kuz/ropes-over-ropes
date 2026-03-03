import MetalKit
import simd

extension Renderer {
    func handleCameraPan(translation: SIMD2<Float>, in view: MTKView) {
        guard cameraDebugMode else { return }
        let width = max(1.0, Float(view.bounds.size.width))
        let height = max(1.0, Float(view.bounds.size.height))
        let aspect = width / height
        let halfHeight = camera.orthoHalfHeight
        let halfWidth = halfHeight * aspect

        let worldDeltaX = (translation.x / width) * 2 * halfWidth
        let worldDeltaY = -(translation.y / height) * 2 * halfHeight

        camera.center.x -= worldDeltaX
        camera.center.y -= worldDeltaY
    }

    func handleCameraRotation(delta: Float) {
        guard cameraDebugMode else { return }
        camera.tiltAngle += delta
        camera.tiltAngle = max(-Float.pi / 2 + 0.1, min(Float.pi / 2 - 0.1, camera.tiltAngle))
    }

    func handleCameraSpin(delta: Float) {
        camera.rotationAngle += delta
    }

    func handleCameraZoom(scale: Float) {
        cameraZoomScale *= scale
        cameraZoomScale = max(0.3, min(3.0, cameraZoomScale))
        camera.orthoHalfHeight = cameraBaseOrthoHalfHeight / cameraZoomScale
        onZoomChanged?(cameraZoomScale)
    }
}
