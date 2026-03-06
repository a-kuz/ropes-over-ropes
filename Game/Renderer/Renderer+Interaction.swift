import MetalKit
import simd
import os.log

enum InputPhase {
    case began, moved, ended, cancelled
}

#if os(iOS)
import UIKit
extension InputPhase {
    init(_ phase: UITouch.Phase) {
        switch phase {
        case .began: self = .began
        case .moved: self = .moved
        case .ended: self = .ended
        case .cancelled: self = .cancelled
        default: self = .cancelled
        }
    }
}
#endif

extension Renderer {
    private static let interactionLogger = Logger(subsystem: "com.uzls.four", category: "Interaction")

    private struct DragCandidate {
        var ropeIndex: Int
        var endIndex: Int
        var holeIndex: Int
        var score: Float
    }

    func handleTouch(phase: InputPhase, location: CGPoint, in view: MTKView) {
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
            }
            return
        }
        let worldPosition = screenToWorld(location, view: view)
        switch phase {
        case .began:
            beginDrag(world: worldPosition)
            if dragState == nil {
                cameraDragActive = true
                cameraDragStart = location
            }
        case .moved:
            if cameraDragActive {
                if let start = cameraDragStart {
                    let dy = Float(location.y - start.y)
                    let height = max(1.0, Float(view.bounds.size.height))
                    let rotationDelta = dy / height * Float.pi * 0.15
                    camera.tiltAngle += rotationDelta
                    camera.tiltAngle = max(0.0, min(Float.pi / 2 - 0.1, camera.tiltAngle))
                    cameraDragStart = location
                }
            } else {
                updateDrag(world: worldPosition)
            }
        case .ended, .cancelled:
            if cameraDragActive {
                cameraDragActive = false
                cameraDragStart = nil
            } else {
                endDrag(world: worldPosition)
            }
        }
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

            pushUndoState()

            dragWorld = holePositions[best.holeIndex]
            lastDragWorld = dragWorld
            holeOccupied[best.holeIndex] = false
            Self.logger.info("[WIN-DIAG] beginDrag rope=\(best.ropeIndex) end=\(best.endIndex) originalHole=\(best.holeIndex) occupiedAfterFree=\(self.holeOccupied.enumerated().compactMap { idx, value in value ? String(idx) : nil }.joined(separator: ","))")

            dragState = DragState(ropeIndex: best.ropeIndex, endIndex: best.endIndex, originalHoleIndex: best.holeIndex)

            simulator?.beginDrag(bandIndex: best.ropeIndex, endIndex: best.endIndex, worldPosition: dragWorld)
            Haptics.light()
        }
    }

    private func updateDrag(world: SIMD2<Float>) {
        guard dragState != nil else { return }

        dragWorld = world
        simulator?.updateDrag(worldPosition: world)

        let snapRadius = holeRadius * 1.9
        var bestIndex: Int = -1
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
        highlightHoleIndex = bestIndex
    }

    private func endDrag(world: SIMD2<Float>) {
        guard let dragState else { return }
        guard ropes.indices.contains(dragState.ropeIndex) else {
            self.dragState = nil
            return
        }
        guard let sim = simulator, sim.bands.indices.contains(dragState.ropeIndex) else {
            self.dragState = nil
            highlightHoleIndex = -1
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

        let band = sim.bands[dragState.ropeIndex]
        guard band.active, band.fadeOut == 0 else {
            Self.logger.warning("[WIN-DIAG] endDrag skipped writeback for fading/inactive rope=\(dragState.ropeIndex) end=\(dragState.endIndex) target=\(snappedHoleIndex) active=\(band.active) fade=\(String(format: "%.3f", band.fadeOut))")
            levelFlow.scheduleSettleCheck()
            self.dragState = nil
            highlightHoleIndex = -1
            return
        }

        sim.endDrag(targetHoleIndex: snappedHoleIndex)
        Haptics.medium()

        if snappedHoleIndex != dragState.originalHoleIndex {
            moveCount += 1
            onMoveCountChanged?(moveCount)
        }

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

        Self.logger.info("[WIN-DIAG] endDrag rope=\(dragState.ropeIndex) end=\(dragState.endIndex) target=\(snappedHoleIndex) original=\(dragState.originalHoleIndex) snapped=\(bestIndex != nil) occupiedAfterSet=\(self.holeOccupied.enumerated().compactMap { idx, value in value ? String(idx) : nil }.joined(separator: ","))")

        levelFlow.scheduleSettleCheck()
        self.dragState = nil
        highlightHoleIndex = -1
    }

    private func screenToWorld(_ location: CGPoint, view: MTKView) -> SIMD2<Float> {
        let width = max(1.0, Float(view.bounds.size.width))
        let height = max(1.0, Float(view.bounds.size.height))
        let aspect = width / height

        let ndcX = (Float(location.x) / width) * 2 - 1
        let ndcY = (Float(location.y) / height) * 2 - 1

        let vp = camera.viewProj(aspect: aspect)
        let inv = vp.inverse

        let nearClip = SIMD4<Float>(ndcX, ndcY, 0, 1)
        let farClip = SIMD4<Float>(ndcX, ndcY, 1, 1)
        let nearW = inv * nearClip
        let farW = inv * farClip
        let nearP = SIMD3<Float>(nearW.x, nearW.y, nearW.z) / nearW.w
        let farP = SIMD3<Float>(farW.x, farW.y, farW.z) / farW.w
        let dir = farP - nearP

        guard abs(dir.z) > 1e-8 else {
            return SIMD2<Float>(nearP.x, nearP.y)
        }

        for board in boards.sorted(by: { $0.elevation > $1.elevation }) {
            let tBoard = (board.elevation - nearP.z) / dir.z
            if tBoard > 0 {
                let hitX = nearP.x + dir.x * tBoard
                let hitY = nearP.y + dir.y * tBoard
                let hw = board.width * 0.5
                let hh = board.height * 0.5
                if hitX >= board.centerX - hw && hitX <= board.centerX + hw &&
                   hitY >= board.centerY - hh && hitY <= board.centerY + hh {
                    return SIMD2<Float>(hitX, hitY)
                }
            }
        }

        let t = -nearP.z / dir.z
        return SIMD2<Float>(nearP.x + dir.x * t, nearP.y + dir.y * t)
    }
}
