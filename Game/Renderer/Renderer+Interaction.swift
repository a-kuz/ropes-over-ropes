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

            let startDistance = simd_length(world - startHolePosition)
            let startTopAllowed = isEndTopForDrag(ropeIndex: ropeIndex, endIndex: 0)
            let startScore = startDistance + (startTopAllowed ? 0 : hitRadius * 0.75)
            if startDistance < hitRadius && (best == nil || startScore < best!.score) {
                best = DragCandidate(ropeIndex: ropeIndex, endIndex: 0, holeIndex: startHoleIndex, score: startScore)
            }

            let endDistance = simd_length(world - endHolePosition)
            let endTopAllowed = isEndTopForDrag(ropeIndex: ropeIndex, endIndex: 1)
            let endScore = endDistance + (endTopAllowed ? 0 : hitRadius * 0.75)
            if endDistance < hitRadius && (best == nil || endScore < best!.score) {
                best = DragCandidate(ropeIndex: ropeIndex, endIndex: 1, holeIndex: endHoleIndex, score: endScore)
            }
        }

        if let best {
            guard let initial = holePositions[safe: best.holeIndex],
                  holeOccupied.indices.contains(best.holeIndex) else {
                dragState = nil
                return
            }
            dragWorld = initial
            dragWorldLazy = initial
            dragWorldTarget = initial
            dragStartWorld = initial
            lastDragWorld = initial
            dragSagProgress = 0.0
            holeOccupied[best.holeIndex] = false
            topology?.beginDrag(ropeIndex: best.ropeIndex, endIndex: best.endIndex, position: dragWorldLazy)
            let snapshot = topology?.snapshot() ?? TopologySnapshot(ropes: [], hooks: [:], nextHookId: 1)
            
            let endpoints = ropes[best.ropeIndex]
            let startHoleIndex = endpoints.startHole
            let endHoleIndex = endpoints.endHole
            let restLength: Float
            if let startPos = holePositions[safe: startHoleIndex],
               let endPos = holePositions[safe: endHoleIndex] {
                restLength = simd_length(endPos - startPos)
            } else {
                restLength = 1.0
            }
            
            dragState = DragState(ropeIndex: best.ropeIndex, endIndex: best.endIndex, originalHoleIndex: best.holeIndex, topologySnapshot: snapshot, restLength: restLength)
            dragStretchRatio = 1.0
            dragOscillationPhase = 0.0
            dragOscillationVelocity = 0.0
            dragOscillationRopeIndex = nil
        }
    }

    private func updateDrag(world: SIMD2<Float>) {
        guard let dragState else { return }
        
        dragWorldTarget = world
        dragWorld = world
        
        let endpoints = ropes[dragState.ropeIndex]
        let fixedHoleIndex = (dragState.endIndex == 0) ? endpoints.endHole : endpoints.startHole
        guard let fixedPos = holePositions[safe: fixedHoleIndex] else { return }
        
        let targetLength = simd_length(world - fixedPos)
        let currentLazyLength = simd_length(dragWorldLazy - fixedPos)
        let restLength = dragState.restLength
        
        dragStretchRatio = targetLength / max(1e-6, restLength)
        
        let deltaTime = Float(1.0 / 60.0)
        let velocity = (world - dragWorld) / max(1e-4, deltaTime)
        let speed = simd_length(velocity)
        
        if speed > 0.5 {
            let impulse = speed * 0.12
            dragOscillationVelocity += impulse
            dragOscillationRopeIndex = dragState.ropeIndex
        }
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
        
        guard let targetPos = holePositions[safe: snappedHoleIndex] else {
            self.dragState = nil
            return
        }

        topology?.restore(dragState.topologySnapshot)
        
        guard let fromPos = holePositions[safe: dragState.originalHoleIndex] else {
            self.dragState = nil
            return
        }
        
        topology?.processCanonicalMove(
            ropeIndex: dragState.ropeIndex,
            endIndex: dragState.endIndex,
            from: fromPos,
            to: targetPos
        )

        if let snappedHoleIndex = bestIndex {
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

        let draggedRopeIndex = dragState.ropeIndex
        let endpoints = ropes[draggedRopeIndex]
        
        let velocity = (dragWorldLazy - lastDragWorld) / max(1e-4, Float(1.0 / 60.0))
        let speed = simd_length(velocity)
        dragOscillationVelocity = speed * 0.15
        dragOscillationPhase = 0.0
        dragOscillationRopeIndex = draggedRopeIndex

        snapAnimationState = SnapAnimationState(
            ropeIndex: dragState.ropeIndex,
            endIndex: dragState.endIndex,
            targetHoleIndex: snappedHoleIndex,
            startPosition: dragWorldLazy,
            targetPosition: targetPos,
            startZ: dragLiftCurrent,
            progress: 0.0,
            topologySnapshot: dragState.topologySnapshot,
            originalHoleIndex: dragState.originalHoleIndex
        )
        
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
    
    private func isEndTopForDrag(ropeIndex: Int, endIndex: Int) -> Bool {
        guard let topology else { return true }
        for (_, hook) in topology.hooks {
            if hook.ropeA == ropeIndex || hook.ropeB == ropeIndex {
                return topology.isEndTop(ropeIndex: ropeIndex, endIndex: endIndex, hook: hook)
            }
        }
        return true
    }
}

