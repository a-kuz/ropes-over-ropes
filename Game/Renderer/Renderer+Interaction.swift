import MetalKit
import simd

extension Renderer {
    func handleTouch(phase: UITouch.Phase, location: CGPoint, in view: MTKView) {
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

    private func beginDrag(world: SIMD2<Float>) {
        let hitRadius = holeRadius * 1.65
        var best: (ropeIndex: Int, endIndex: Int, holeIndex: Int, distance: Float)?

        for ropeIndex in ropes.indices {
            let endpoints = ropes[ropeIndex]
            let startHoleIndex = endpoints.startHole
            let endHoleIndex = endpoints.endHole
            guard let startHolePosition = holePositions[safe: startHoleIndex],
                  let endHolePosition = holePositions[safe: endHoleIndex] else { continue }

            let startDistance = simd_length(world - startHolePosition)
            let startTopAllowed = topology?.isEndTop(ropeIndex: ropeIndex, endIndex: 0) ?? true
            if startTopAllowed, startDistance < hitRadius && (best == nil || startDistance < best!.distance) {
                best = (ropeIndex, 0, startHoleIndex, startDistance)
            }

            let endDistance = simd_length(world - endHolePosition)
            let endTopAllowed = topology?.isEndTop(ropeIndex: ropeIndex, endIndex: 1) ?? true
            if endTopAllowed, endDistance < hitRadius && (best == nil || endDistance < best!.distance) {
                best = (ropeIndex, 1, endHoleIndex, endDistance)
            }
        }

        if let best {
            dragState = DragState(ropeIndex: best.ropeIndex, endIndex: best.endIndex, originalHoleIndex: best.holeIndex)
            guard let initial = holePositions[safe: best.holeIndex],
                  holeOccupied.indices.contains(best.holeIndex) else {
                dragState = nil
                return
            }
            dragWorld = initial
            lastDragWorld = dragWorld
            holeOccupied[best.holeIndex] = false
            topology?.beginDrag(ropeIndex: best.ropeIndex, endIndex: best.endIndex, floatingPosition: dragWorld)
        }
    }

    private func updateDrag(world: SIMD2<Float>) {
        guard dragState != nil else { return }
        if let state = dragState {
            topology?.setFloating(ropeIndex: state.ropeIndex, position: world)
            topology?.processDragSegment(ropeIndex: state.ropeIndex, from: lastDragWorld, to: world)
            lastDragWorld = world
        }
        dragWorld = world
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

        if let snappedHoleIndex = bestIndex {
            if dragState.endIndex == 0 {
                ropes[dragState.ropeIndex].startHole = snappedHoleIndex
            } else {
                ropes[dragState.ropeIndex].endHole = snappedHoleIndex
            }
            if holeOccupied.indices.contains(snappedHoleIndex) {
                holeOccupied[snappedHoleIndex] = true
            }
            topology?.endDrag(ropeIndex: dragState.ropeIndex, endIndex: dragState.endIndex, holeIndex: snappedHoleIndex)
        } else {
            if holeOccupied.indices.contains(dragState.originalHoleIndex) {
                holeOccupied[dragState.originalHoleIndex] = true
            }
            topology?.endDrag(ropeIndex: dragState.ropeIndex, endIndex: dragState.endIndex, holeIndex: dragState.originalHoleIndex)
        }

        self.dragState = nil

        let endpoints = ropes[dragState.ropeIndex]
        if let pinStart = holePositions[safe: endpoints.startHole],
           let pinEnd = holePositions[safe: endpoints.endHole] {
            simulation.setPins(
                ropeIndex: dragState.ropeIndex,
                pinStart: SIMD3<Float>(pinStart.x, pinStart.y, 0),
                pinEnd: SIMD3<Float>(pinEnd.x, pinEnd.y, 0)
            )
        } else {
            simulation.deactivateRope(ropeIndex: dragState.ropeIndex)
            topology?.deactivateRope(ropeIndex: dragState.ropeIndex)
        }

        removeUntangledRopes()
    }

    private func screenToWorld(_ location: CGPoint, view: MTKView) -> SIMD2<Float> {
        let width = max(1.0, Float(view.bounds.size.width))
        let height = max(1.0, Float(view.bounds.size.height))
        let aspect = width / height
        let halfHeight = camera.orthoHalfHeight
        let halfWidth = halfHeight * aspect

        let ndcX = (Float(location.x) / width) * 2 - 1
        let ndcY = (Float(location.y) / height) * 2 - 1
        return SIMD2<Float>(ndcX * halfWidth, ndcY * halfHeight)
    }
}

