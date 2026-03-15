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
        guard !victoryOrbitActive && !victoryReplayActive && !victoryReplayPending else { return }
        let worldPosition = screenToWorld(location, view: view)
        switch phase {
        case .began:
            beginDrag(world: worldPosition, screenLocation: location, view: view)
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

    private func beginDrag(world: SIMD2<Float>, screenLocation: CGPoint, view: MTKView) {
        let screenPt = SIMD2<Float>(Float(screenLocation.x), Float(screenLocation.y))
        let viewW = max(1.0, Float(view.bounds.size.width))
        let viewH = max(1.0, Float(view.bounds.size.height))
        let aspect = viewW / viewH
        let vp = camera.viewProj(aspect: aspect)

        let screenHitRadius = viewW * 0.045
        var best: DragCandidate?

        for ropeIndex in ropes.indices {
            let endpoints = ropes[ropeIndex]
            let startHoleIndex = endpoints.startHole
            let endHoleIndex = endpoints.endHole

            if isTensionMode {
                let startIsWeight = startHoleIndex >= holePositions.count
                let endIsWeight = endHoleIndex >= holePositions.count
                if startIsWeight && endIsWeight { continue }
            }

            let startZ = simulator?.endpointZ(bandIndex: ropeIndex, endIndex: 0) ?? 0
            let endZ = simulator?.endpointZ(bandIndex: ropeIndex, endIndex: 1) ?? 0

            let startIsAnchor = isBraidMode && startHoleIndex < braidBottomHoleStart

            let startIsWeight = isTensionMode && startHoleIndex >= holePositions.count
            if !startIsWeight && !startIsAnchor {
                let startScreen = worldToScreen(endpointPosition3D(holeIndex: startHoleIndex, z: startZ), vp: vp, viewW: viewW, viewH: viewH)
                let startDistance = simd_length(screenPt - startScreen)
                let startTopAllowed = startZ >= endZ
                let startScore = startDistance + (startTopAllowed ? 0 : screenHitRadius * 0.75)
                if startDistance < screenHitRadius && (best == nil || startScore < best!.score) {
                    best = DragCandidate(ropeIndex: ropeIndex, endIndex: 0, holeIndex: startHoleIndex, score: startScore)
                }
            }

            let endIsWeight = isTensionMode && endHoleIndex >= holePositions.count
            if !endIsWeight {
                let endScreen = worldToScreen(endpointPosition3D(holeIndex: endHoleIndex, z: endZ), vp: vp, viewW: viewW, viewH: viewH)
                let endDistance = simd_length(screenPt - endScreen)
                let endTopAllowed = endZ >= startZ
                let endScore = endDistance + (endTopAllowed ? 0 : screenHitRadius * 0.75)
                if endDistance < screenHitRadius && (best == nil || endScore < best!.score) {
                    best = DragCandidate(ropeIndex: ropeIndex, endIndex: 1, holeIndex: endHoleIndex, score: endScore)
                }
            }
        }

        if best == nil, let sim = simulator {
            let bodyScreenHitRadius = viewW * 0.06
            var bestBodyDist: Float = .greatestFiniteMagnitude

            for ropeIndex in ropes.indices {
                let band = sim.bands[ropeIndex]
                guard band.active, band.fadeOut == 0 else { continue }

                let endpoints = ropes[ropeIndex]
                let startHoleIndex = endpoints.startHole
                let endHoleIndex = endpoints.endHole

                if isTensionMode {
                    let startIsWeight = startHoleIndex >= holePositions.count
                    let endIsWeight = endHoleIndex >= holePositions.count
                    if startIsWeight && endIsWeight { continue }
                }

                let positions = band.positions
                guard positions.count >= 2 else { continue }

                var minSegDist: Float = .greatestFiniteMagnitude
                for i in 0..<(positions.count - 1) {
                    let a = worldToScreen(positions[i], vp: vp, viewW: viewW, viewH: viewH)
                    let b = worldToScreen(positions[i + 1], vp: vp, viewW: viewW, viewH: viewH)
                    let d = pointToSegmentDistance2D(screenPt, a: a, b: b)
                    minSegDist = min(minSegDist, d)
                }

                guard minSegDist < bodyScreenHitRadius, minSegDist < bestBodyDist else { continue }

                let startIsAnchor = isBraidMode && startHoleIndex < braidBottomHoleStart
                let startIsWeight = isTensionMode && startHoleIndex >= holePositions.count
                let endIsWeight = isTensionMode && endHoleIndex >= holePositions.count

                let startScreen = worldToScreen(endpointPosition3D(holeIndex: startHoleIndex, z: 0), vp: vp, viewW: viewW, viewH: viewH)
                let endScreen = worldToScreen(endpointPosition3D(holeIndex: endHoleIndex, z: 0), vp: vp, viewW: viewW, viewH: viewH)

                let distToStart = simd_length(screenPt - startScreen)
                let distToEnd = simd_length(screenPt - endScreen)

                let chosenEnd: Int
                let chosenHole: Int
                if startIsWeight || startIsAnchor {
                    if endIsWeight { continue }
                    chosenEnd = 1
                    chosenHole = endHoleIndex
                } else if endIsWeight {
                    chosenEnd = 0
                    chosenHole = startHoleIndex
                } else {
                    if distToStart <= distToEnd {
                        chosenEnd = 0
                        chosenHole = startHoleIndex
                    } else {
                        chosenEnd = 1
                        chosenHole = endHoleIndex
                    }
                }

                bestBodyDist = minSegDist
                best = DragCandidate(ropeIndex: ropeIndex, endIndex: chosenEnd, holeIndex: chosenHole, score: minSegDist)
            }
        }

        if let best {
            guard holePositions.indices.contains(best.holeIndex),
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

        let snapRadius = holeRadius * 1.9
        var bestIndex: Int = -1
        var bestDistance: Float = .greatestFiniteMagnitude
        for holeIndex in holePositions.indices {
            guard holeOccupied.indices.contains(holeIndex) else { continue }
            // In braid mode: allow snapping to occupied bottom holes (swap), skip top holes
            if isBraidMode {
                if holeIndex < braidBottomHoleStart { continue } // skip top holes
                // Allow occupied holes for swap
            } else {
                if holeOccupied[holeIndex] { continue }
            }
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

        if isBraidMode {
            // Braid mode: snap to nearest bottom hole (occupied or not), skip top holes
            for holeIndex in holePositions.indices {
                if holeIndex < braidBottomHoleStart { continue } // skip top holes
                let distance = simd_length(holePositions[holeIndex] - world)
                if distance < snapRadius && distance < bestDistance {
                    bestIndex = holeIndex
                    bestDistance = distance
                }
            }
        } else {
            for holeIndex in holePositions.indices {
                guard holeOccupied.indices.contains(holeIndex) else { continue }
                if holeOccupied[holeIndex] { continue }
                let distance = simd_length(holePositions[holeIndex] - world)
                if distance < snapRadius && distance < bestDistance {
                    bestIndex = holeIndex
                    bestDistance = distance
                }
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

        // Braid mode: handle swap when target hole is occupied
        if isBraidMode, snappedHoleIndex != dragState.originalHoleIndex,
           holeOccupied.indices.contains(snappedHoleIndex), holeOccupied[snappedHoleIndex] {
            // Find which rope occupies the target hole
            if let otherRopeIndex = ropes.indices.first(where: { ri in
                ri != dragState.ropeIndex &&
                ((dragState.endIndex == 0 && ropes[ri].startHole == snappedHoleIndex) ||
                 (dragState.endIndex == 1 && ropes[ri].endHole == snappedHoleIndex) ||
                 ropes[ri].startHole == snappedHoleIndex || ropes[ri].endHole == snappedHoleIndex)
            }) {
                // Determine which end of the other rope is at the target hole
                let otherEndIndex: Int
                if ropes[otherRopeIndex].endHole == snappedHoleIndex {
                    otherEndIndex = 1
                } else {
                    otherEndIndex = 0
                }

                // Move the other rope's end to the original hole of the dragged strand
                sim.swapEndToHole(bandIndex: otherRopeIndex, endIndex: otherEndIndex, holeIndex: dragState.originalHoleIndex)
                if otherEndIndex == 0 {
                    ropes[otherRopeIndex].startHole = dragState.originalHoleIndex
                } else {
                    ropes[otherRopeIndex].endHole = dragState.originalHoleIndex
                }
                // The original hole is now occupied by the swapped rope
                if holeOccupied.indices.contains(dragState.originalHoleIndex) {
                    holeOccupied[dragState.originalHoleIndex] = true
                }
                Self.logger.info("[BRAID] Swap: rope \(dragState.ropeIndex) end=\(dragState.endIndex) to hole \(snappedHoleIndex), rope \(otherRopeIndex) end=\(otherEndIndex) to hole \(dragState.originalHoleIndex)")
            }
        }

        sim.endDrag(targetHoleIndex: snappedHoleIndex)
        Haptics.medium()

        if snappedHoleIndex != dragState.originalHoleIndex {
            moveCount += 1
            onMoveCountChanged?(moveCount)

            // Always record for victory replay
            moveHistory.append(ReplayMove(
                ropeIndex: dragState.ropeIndex,
                endIndex: dragState.endIndex,
                fromHole: dragState.originalHoleIndex,
                toHole: snappedHoleIndex
            ))

            // Record the action if recording is active
            if isRecording {
                recordedActions.append((ropeIndex: dragState.ropeIndex, endIndex: dragState.endIndex,
                                        fromHole: dragState.originalHoleIndex, toHole: snappedHoleIndex))
                Self.logger.info("[REC] drag rope=\(dragState.ropeIndex) end=\(dragState.endIndex) \(dragState.originalHoleIndex)→\(snappedHoleIndex)")
            }
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

    private func endpointPosition3D(holeIndex: Int, z: Float) -> SIMD3<Float> {
        if holePositions.indices.contains(holeIndex) {
            let p = holePositions[holeIndex]
            let elev = holeElevations.indices.contains(holeIndex) ? holeElevations[holeIndex] : Float(0)
            return SIMD3<Float>(p.x, p.y, elev)
        }
        let wi = holeIndex - holePositions.count
        if weightRenderInfos.indices.contains(wi) {
            let p = weightRenderInfos[wi].position
            return SIMD3<Float>(p.x, p.y, z)
        }
        return SIMD3<Float>(0, 0, z)
    }

    private func worldToScreen(_ pos: SIMD3<Float>, vp: simd_float4x4, viewW: Float, viewH: Float) -> SIMD2<Float> {
        let clip = vp * SIMD4<Float>(pos.x, pos.y, pos.z, 1)
        let ndcX = clip.x / clip.w
        let ndcY = clip.y / clip.w
        return SIMD2<Float>((ndcX + 1) * 0.5 * viewW, (ndcY + 1) * 0.5 * viewH)
    }

    private func pointToSegmentDistance2D(_ p: SIMD2<Float>, a: SIMD2<Float>, b: SIMD2<Float>) -> Float {
        let ab = b - a
        let ap = p - a
        let lenSq = simd_length_squared(ab)
        if lenSq < 1e-12 { return simd_length(ap) }
        let t = simd_clamp(simd_dot(ap, ab) / lenSq, 0, 1)
        return simd_length(p - (a + ab * t))
    }
}
