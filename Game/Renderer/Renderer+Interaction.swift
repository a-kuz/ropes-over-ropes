import MetalKit
import simd
import os.log

extension Renderer {
    private static let interactionLogger = Logger(subsystem: "com.uzls.four", category: "Interaction")

    private struct DragCandidate {
        var ropeIndex: Int
        var endIndex: Int
        var holeIndex: Int
        var score: Float
    }

    @MainActor
    func handleTouch(phase: UITouch.Phase, location: CGPoint, in view: MTKView) {
        if cameraDebugMode {
            switch phase {
            case .began:
                cameraDebugTouchStart = location
            case .moved:
                if let start = cameraDebugTouchStart {
                    let deltaX = Float(location.x - start.x)
                    let deltaY = Float(location.y - start.y)
                    let width = max(1.0, Float(view.bounds.size.width))
                    let height = max(1.0, Float(view.bounds.size.height))
                    let aspect = width / height
                    let halfHeight = camera.orthoHalfHeight
                    let halfWidth = halfHeight * aspect

                    let worldDeltaX = (deltaX / width) * 2 * halfWidth
                    camera.center.x -= worldDeltaX

                    let rotationDelta = -deltaY / height * Float.pi * 0.5
                    camera.tiltAngle += rotationDelta
                    camera.tiltAngle = max(-Float.pi / 2 + 0.1, min(Float.pi / 2 - 0.1, camera.tiltAngle))

                    cameraDebugTouchStart = location
                }
            case .ended, .cancelled:
                cameraDebugTouchStart = nil
            default:
                break
            }
            return
        }
        let worldPosition = screenToWorld(location, view: view)
        switch phase {
        case .began:
            beginDrag(world: worldPosition)
        case .moved:
            updateDrag(world: worldPosition)
        case .ended, .cancelled:
            endDrag(world: worldPosition)
        default:
            break
        }
    }

    @MainActor
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

    @MainActor
    func handleCameraRotation(delta: Float) {
        guard cameraDebugMode else { return }
        camera.tiltAngle += delta
        camera.tiltAngle = max(-Float.pi / 2 + 0.1, min(Float.pi / 2 - 0.1, camera.tiltAngle))
    }

    @MainActor
    func handleCameraZoom(scale: Float) {
        guard cameraDebugMode else { return }
        camera.orthoHalfHeight *= scale
        camera.orthoHalfHeight = max(0.1, min(10.0, camera.orthoHalfHeight))
    }

    private func beginDrag(world: SIMD2<Float>) {
        let hitRadius = holeRadius * 1.65
        var best: DragCandidate?

        for ropeIndex in ropes.indices {
            let endpoints = ropes[ropeIndex]
            let startHoleIndex = endpoints.startHole
            let endHoleIndex = endpoints.endHole
            guard let startHolePosition = holePositions[safe: startHoleIndex],
                  let endHolePosition = holePositions[safe: endHoleIndex] else { continue }

            // Use z-coordinate from simulator to determine which endpoint is on top
            let startZ = simulator?.endpointZ(bandIndex: ropeIndex, endIndex: 0) ?? 0
            let endZ = simulator?.endpointZ(bandIndex: ropeIndex, endIndex: 1) ?? 0

            let startDistance = simd_length(world - startHolePosition)
            let startTopAllowed = startZ >= endZ  // higher z = on top
            let startScore = startDistance + (startTopAllowed ? 0 : hitRadius * 0.75)
            if startDistance < hitRadius && (best == nil || startScore < best!.score) {
                best = DragCandidate(ropeIndex: ropeIndex, endIndex: 0, holeIndex: startHoleIndex, score: startScore)
            }

            let endDistance = simd_length(world - endHolePosition)
            let endTopAllowed = endZ >= startZ
            let endScore = endDistance + (endTopAllowed ? 0 : hitRadius * 0.75)
            if endDistance < hitRadius && (best == nil || endScore < best!.score) {
                best = DragCandidate(ropeIndex: ropeIndex, endIndex: 1, holeIndex: endHoleIndex, score: endScore)
            }
        }

        if let best {
            guard let _ = holePositions[safe: best.holeIndex],
                  holeOccupied.indices.contains(best.holeIndex) else {
                dragState = nil
                return
            }
            dragWorld = holePositions[best.holeIndex]
            lastDragWorld = dragWorld
            holeOccupied[best.holeIndex] = false

            dragState = DragState(ropeIndex: best.ropeIndex, endIndex: best.endIndex, originalHoleIndex: best.holeIndex)

            // Physics: begin drag (lift endpoint)
            simulator?.beginDrag(bandIndex: best.ropeIndex, endIndex: best.endIndex, worldPosition: dragWorld)
            Haptics.light()
        }
    }

    private func updateDrag(world: SIMD2<Float>) {
        guard dragState != nil else { return }

        dragWorld = world

        // Physics: move dragged endpoint
        simulator?.updateDrag(worldPosition: world)
    }

    private func endDrag(world: SIMD2<Float>) {
        guard let dragState else { return }
        guard ropes.indices.contains(dragState.ropeIndex) else {
            self.dragState = nil
            return
        }

        let snapRadius = holeRadius * 1.9
        var bestIndex: Int?
        var bestDistance: Float = .greatestFiniteMagnitude

        for holeIndex in holePositions.indices {
            guard holeOccupied.indices.contains(holeIndex) else { continue }
            if holeOccupied[holeIndex] { continue }
            let distance = simd_length(holePositions[holeIndex] - world)
            if distance < snapRadius && distance < bestDistance {
                bestIndex = holeIndex
                bestDistance = distance
            }
        }

        let snappedHoleIndex = bestIndex ?? dragState.originalHoleIndex

        // Physics: end drag (lower into hole + settle)
        simulator?.endDrag(targetHoleIndex: snappedHoleIndex)
        Haptics.medium()

        // Update rope endpoints
        if let _ = bestIndex {
            if dragState.endIndex == 0 {
                ropes[dragState.ropeIndex].startHole = snappedHoleIndex
            } else {
                ropes[dragState.ropeIndex].endHole = snappedHoleIndex
            }
            if holeOccupied.indices.contains(snappedHoleIndex) {
                holeOccupied[snappedHoleIndex] = true
            }
        } else {
            if holeOccupied.indices.contains(dragState.originalHoleIndex) {
                holeOccupied[dragState.originalHoleIndex] = true
            }
        }

        // Delay win check so rope has time to settle after drag
        settleCheckTimer = settleCheckDelay
        self.dragState = nil
    }

    @MainActor
    private func screenToWorld(_ location: CGPoint, view: MTKView) -> SIMD2<Float> {
        let width = max(1.0, Float(view.bounds.size.width))
        let height = max(1.0, Float(view.bounds.size.height))
        let aspect = width / height
        let halfHeight = camera.orthoHalfHeight
        let halfWidth = halfHeight * aspect

        let ndcX = (Float(location.x) / width) * 2 - 1
        let ndcY = (Float(location.y) / height) * 2 - 1

        if abs(camera.tiltAngle) < 0.01 {
            return SIMD2<Float>(ndcX * halfWidth, ndcY * halfHeight)
        }

        let yOffset = camera.distance * sin(camera.tiltAngle)
        let zOffset = camera.distance * cos(camera.tiltAngle)
        let eye = camera.center + SIMD3<Float>(0, yOffset, zOffset)
        let viewMatrix = simd_float4x4.lookAt(eye: eye, center: camera.center, up: SIMD3<Float>(0, 1, 0))

        let right = SIMD3<Float>(viewMatrix[0].x, viewMatrix[0].y, viewMatrix[0].z)
        let up = SIMD3<Float>(viewMatrix[1].x, viewMatrix[1].y, viewMatrix[1].z)

        let viewX = ndcX * halfWidth
        let viewY = ndcY * halfHeight

        let worldPoint = camera.center + right * viewX + up * viewY

        return SIMD2<Float>(worldPoint.x, worldPoint.y)
    }
}
