import Metal
import simd
import os.log

extension Renderer {
    private static let meshLogger = Logger(subsystem: "com.uzls.four", category: "RopeMesh")
    
    func updateRopeMesh() {
        topology?.updateHookPositions(deltaTime: lastDeltaTime)
        
        var allVertices: [RopeVertex] = []
        var allIndices: [UInt32] = []
        allVertices.reserveCapacity(ropes.count * 6400 * 8)
        allIndices.reserveCapacity(ropes.count * 6399 * 24)

        var baseVertex: UInt32 = 0

        let dragLift: Float = dragLiftCurrent
        let repulsorsBase = ropeRenderSimpleMode ? [] : makeRepulsorsBase()
        let repulsorsWithTopology = ropeRenderSimpleMode ? [] : makeRepulsorsWithTopology()

        let ropeRadiusForIndex: (Int) -> Float = { idx in
            self.ropes[safe: idx]?.radius ?? 0.0425
        }
        
        var allRopePoints: [[SIMD3<Float>]] = []
        var allSegmentStarts: [[Int]] = []
        var ropeRadii: [Float] = []
        
        let deltaTime = lastDeltaTime
        
        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 {
                allRopePoints.append([])
                allSegmentStarts.append([])
                ropeRadii.append(0)
                previousRopePoints[ropeIndex] = nil
                ropePointVelocities[ropeIndex] = nil
                continue
            }
            
            let ropeRadius = ropes[ropeIndex].radius    
            ropeRadii.append(ropeRadius)
            
            guard let topology else {
                allRopePoints.append([])
                allSegmentStarts.append([0])
                continue
            }
            
            let lift = max(ropeRadius * 2.7, 0.02)
            let result = TopologySampler.sampleRopeRender(
                engine: topology,
                ropeIndex: ropeIndex,
                lift: lift,
                dragLift: dragLift,
                ropeRadius: ropeRadius,
                ropeRadiusForIndex: ropeRadiusForIndex,
                holeRadius: holeRadius
            )
            
            var targetPoints = result.points
            if let tensionState = ropeTensionStates[ropeIndex], globalTensionActive {
                let restLength = ropeRestLengths[ropeIndex] ?? 1.0
                let sagRatio = (tensionState.currentLength - restLength) / max(0.001, restLength)
                targetPoints = Self.applySagToPoints(points: targetPoints, sagRatio: sagRatio, ropeRadius: ropeRadius)
            }
            
            var smoothedPoints = targetPoints
            let isDragged = dragState?.ropeIndex == ropeIndex
            
            if let previousPoints = previousRopePoints[ropeIndex], !previousPoints.isEmpty, !targetPoints.isEmpty {
                var velocities = ropePointVelocities[ropeIndex] ?? Array(repeating: SIMD3<Float>(0, 0, 0), count: targetPoints.count)
                
                if previousPoints.count != targetPoints.count {
                    smoothedPoints = interpolateRopePoints(
                        from: previousPoints,
                        to: targetPoints,
                        deltaTime: deltaTime,
                        isDragged: isDragged,
                        velocities: &velocities
                    )
                    ropePointVelocities[ropeIndex] = velocities
                } else {
                    let dt = min(deltaTime, 1.0 / 30.0)
                    let pointCount = targetPoints.count
                    
                    for i in 0..<pointCount {
                        let target = targetPoints[i]
                        let current = previousPoints[i]
                        let displacement = target - current
                        let dist = simd_length(displacement)
                        
                        let t = Float(i) / Float(max(1, pointCount - 1))
                        let dragEndT = isDragged ? (dragState?.endIndex == 0 ? 0.0 : 1.0) : -1.0
                        let distFromDragEnd = isDragged ? abs(Double(t) - dragEndT) : 1.0
                        
                        let responsiveness: Float
                        if isDragged && distFromDragEnd < 0.3 {
                            responsiveness = 0.98
                        } else if isDragged {
                            let falloff = min(1.0, distFromDragEnd * 1.5)
                            responsiveness = Float(0.98 - falloff * 0.7)
                        } else {
                            responsiveness = 0.25
                        }
                        
                        var velocity = velocities.count > i ? velocities[i] : SIMD3<Float>(0, 0, 0)
                        
                        let nonlinearK: Float = 150.0
                        let distFactor = 1.0 + dist * 8.0
                        let springForce = displacement * nonlinearK * distFactor
                        
                        let criticalDamping = 2.0 * sqrt(nonlinearK * distFactor)
                        let dampingForce = velocity * criticalDamping * 0.9
                        
                        let acceleration = springForce - dampingForce
                        velocity += acceleration * dt
                        
                        var newPos = current + velocity * dt
                        newPos = newPos * (1.0 - responsiveness) + target * responsiveness
                        
                        smoothedPoints[i] = newPos
                        
                        if velocities.count <= i {
                            velocities.append(velocity)
                        } else {
                            velocities[i] = velocity
                        }
                    }
                    
                    ropePointVelocities[ropeIndex] = velocities
                }
            } else {
                ropePointVelocities[ropeIndex] = Array(repeating: SIMD3<Float>(0, 0, 0), count: targetPoints.count)
            }
            
            previousRopePoints[ropeIndex] = smoothedPoints
            allRopePoints.append(smoothedPoints)
            allSegmentStarts.append(result.segmentStarts)
        }
        
        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 { continue }
            if allRopePoints[ropeIndex].isEmpty { continue }
            
            let ropeColor = ropes[ropeIndex].color
            let ropeRadius = ropes[ropeIndex].radius
            
            var renderPoints = allRopePoints[ropeIndex]
            
            if !ropeRenderDisableHoleDeform {
                renderPoints = Self.applyHoleBendVisual(
                    points: renderPoints,
                    topology: topology,
                    ropeIndex: ropeIndex,
                    holeRadius: holeRadius,
                    holePositions: holePositions,
                    ropes: ropes
                )
            }
            
            let isDragged = dragState?.ropeIndex == ropeIndex
            let stretchRatio = isDragged ? dragStretchRatio : 1.0
            let oscillation = (dragOscillationRopeIndex == ropeIndex) ? dragOscillationPhase : 0.0
            let segmentStarts = allSegmentStarts[ropeIndex]
            
            let ropeRestLength = ropeEffectiveRestLengths[ropeIndex] ?? ropeRestLengths[ropeIndex] ?? 0
            
            let ropeMesh = renderPoints.withUnsafeBufferPointer { pointsBuffer in
                let events = Self.makeTwistEvents(topology: topology, ropeIndex: ropeIndex, points: pointsBuffer)
                let taut = ropeRenderSimpleMode ? 0 : Self.tautness(points: pointsBuffer)
                let repulsors = ropeRenderSimpleMode ? [] : (topology != nil ? repulsorsWithTopology : repulsorsBase)
                return RopeMeshBuilder.buildRect(
                    points: pointsBuffer,
                    radius: ropeRadius,
                    color: ropeColor,
                    twistEvents: events,
                    tautness: taut,
                    repulsors: repulsors,
                    stretchRatio: ropeRenderSimpleMode ? 1.0 : stretchRatio,
                    oscillation: ropeRenderSimpleMode ? 0.0 : oscillation,
                    segmentStarts: TopologySampler.debugSegmentColors ? segmentStarts : [],
                    restLength: ropeRenderSimpleMode ? 0 : ropeRestLength
                )
            }
            
            allVertices.append(contentsOf: ropeMesh.vertices)
            allIndices.append(contentsOf: ropeMesh.indices.map { $0 + baseVertex })
            baseVertex += UInt32(ropeMesh.vertices.count)
        }
        ropeIndexCount = allIndices.count

        let vertexBytes = allVertices.count * MemoryLayout<RopeVertex>.stride
        if ropeVB == nil || ropeVB!.length < vertexBytes {
            ropeVB = device.makeBuffer(length: max(1, vertexBytes), options: [.storageModeShared])
        }
        if vertexBytes > 0 {
            ropeVB?.contents().copyMemory(from: allVertices, byteCount: vertexBytes)
        }

        let indexBytes = allIndices.count * MemoryLayout<UInt32>.stride
        if ropeIB == nil || ropeIB!.length < indexBytes {
            ropeIB = device.makeBuffer(length: max(1, indexBytes), options: [.storageModeShared])
        }
        if indexBytes > 0 {
            ropeIB?.contents().copyMemory(from: allIndices, byteCount: indexBytes)
        }
        
        if let topology {
            var loggableRopes: [(index: Int, points: [SIMD3<Float>])] = []
            for ropeIndex in ropes.indices {
                if allRopePoints[ropeIndex].isEmpty { continue }
                loggableRopes.append((index: ropeIndex, points: allRopePoints[ropeIndex]))
            }
            ropePhysicsLogger.logStateIfNeeded(
                time: Double(time),
                ropes: loggableRopes,
                hooks: topology.hooks
            )
        }
        
        let currentMeshStats = MeshStats(vertices: allVertices.count, indices: allIndices.count, ropeCount: self.ropes.count)
        if currentMeshStats != lastMeshStats {
            lastMeshStats = currentMeshStats
            Self.meshLogger.info("Mesh stats: vertices=\(allVertices.count) indices=\(allIndices.count) ropeCount=\(self.ropes.count)")
        }
    }
    
    private func makeRepulsorsBase() -> [SIMD4<Float>] {
        let r = holeRadius * 1.02
        let strength = holeRadius * 0.08
        var repulsors: [SIMD4<Float>] = []
        repulsors.reserveCapacity(holePositions.count)
        for p in holePositions {
            repulsors.append(SIMD4<Float>(p.x, p.y, r, strength))
        }
        return repulsors
    }

    private func makeRepulsorsWithTopology() -> [SIMD4<Float>] {
        var repulsors = makeRepulsorsBase()
        
        guard let topology else { return repulsors }
        
        let hookCenters = TopologySampler.hookCenters(engine: topology, ropeRadiusForIndex: { idx in
            self.ropes[safe: idx]?.radius ?? 0.0425
        })
        
        for c in hookCenters {
            let strength: Float = 0.006
            let r: Float = 0.04
            repulsors.append(SIMD4<Float>(c.x, c.y, r, strength))
        }
        
        return repulsors
    }
    
    private static func makeTwistEvents(topology: TopologyEngine?, ropeIndex: Int, points: UnsafeBufferPointer<SIMD3<Float>>) -> [RopeMeshBuilder.TwistEvent] {
        guard let topology else { return [] }
        
        var events: [RopeMeshBuilder.TwistEvent] = []
        
        for (_, hook) in topology.hooks {
            guard hook.ropeA == ropeIndex || hook.ropeB == ropeIndex else { continue }
            
            let isRopeA = (ropeIndex == hook.ropeA)
            let firstIsOver = isRopeA ? hook.ropeAStartIsOver : !hook.ropeAStartIsOver
            
            let A1 = topology.ropeStart(hook.ropeA)
            let A2 = topology.ropeEnd(hook.ropeA)
            let B1 = topology.ropeStart(hook.ropeB)
            let B2 = topology.ropeEnd(hook.ropeB)
            
            let mid = (A1 + A2 + B1 + B2) * 0.25
            
            let angle: Float = firstIsOver ? Float.pi / 4 : -Float.pi / 4
            
            if let dist = findParameterForPosition(points: points, target: mid) {
                events.append(RopeMeshBuilder.TwistEvent(dist: dist, angle: angle, window: 0.15))
            }
        }
        
        return events.sorted { $0.dist < $1.dist }
    }
    
    private static func findParameterForPosition(points: UnsafeBufferPointer<SIMD3<Float>>, target: SIMD2<Float>) -> Float? {
        guard points.count >= 2 else { return nil }
        
        var bestParam: Float = 0
        var bestDist: Float = .greatestFiniteMagnitude
        
        var cumLength: Float = 0
        var lengths: [Float] = [0]
        
        for i in 1..<points.count {
            cumLength += simd_length(points[i] - points[i-1])
            lengths.append(cumLength)
        }
        
        let totalLength = cumLength
        if totalLength < 1e-6 { return nil }
        
        for i in 0..<points.count {
            let p = SIMD2<Float>(points[i].x, points[i].y)
            let dist = simd_length(p - target)
            if dist < bestDist {
                bestDist = dist
                bestParam = lengths[i] / totalLength
            }
        }
        
        return bestParam
    }
    
    private static func tautness(points: UnsafeBufferPointer<SIMD3<Float>>) -> Float {
        guard points.count >= 2 else { return 1.0 }
        let start = SIMD2<Float>(points.first!.x, points.first!.y)
        let end = SIMD2<Float>(points.last!.x, points.last!.y)
        let straight = simd_length(end - start)
        var arc: Float = 0
        for i in 1..<points.count {
            arc += simd_length(SIMD2<Float>(points[i].x, points[i].y) - SIMD2<Float>(points[i-1].x, points[i-1].y))
        }
        let stretch = straight / max(1e-6, arc)
        return smoothstep(edge0: 0.83, edge1: 0.985, value: stretch)
    }

    private static func smoothstep(edge0: Float, edge1: Float, value: Float) -> Float {
        let normalized = max(0, min(1, (value - edge0) / (edge1 - edge0)))
        return normalized * normalized * (3 - 2 * normalized)
    }
    
    private static func applySagToPoints(points: [SIMD3<Float>], sagRatio: Float, ropeRadius: Float) -> [SIMD3<Float>] {
        guard points.count >= 2 else { return points }
        guard sagRatio > 0.001 else { return points }
        
        var result = points
        let count = result.count
        
        var cumLen: [Float] = [0]
        for i in 1..<count {
            cumLen.append(cumLen[i-1] + simd_length(points[i] - points[i-1]))
        }
        let totalLen = cumLen.last ?? 1.0
        
        let baseZ = ropeRadius
        var minPositiveZ: Float = .greatestFiniteMagnitude
        for p in points where p.z > 0 {
            minPositiveZ = min(minPositiveZ, p.z)
        }
        if minPositiveZ == .greatestFiniteMagnitude {
            minPositiveZ = baseZ
        }
        
        let maxSagDrop = sagRatio * 0.35
        
        for i in 0..<count {
            let originalZ = points[i].z
            if originalZ < 0 { continue }
            
            let t = cumLen[i] / max(0.001, totalLen)
            let parabola = 4.0 * t * (1.0 - t)
            
            let heightAboveMin = originalZ - minPositiveZ
            let sagDrop = maxSagDrop * parabola
            let newMinZ = max(baseZ, minPositiveZ - sagDrop)
            
            result[i].z = newMinZ + heightAboveMin
        }
        
        return result
    }
    
    private func interpolateRopePoints(
        from: [SIMD3<Float>],
        to: [SIMD3<Float>],
        deltaTime: Float,
        isDragged: Bool,
        velocities: inout [SIMD3<Float>]
    ) -> [SIMD3<Float>] {
        if to.isEmpty {
            return from
        }
        if from.isEmpty {
            return to
        }
        
        let dt = min(deltaTime, 1.0 / 30.0)
        var result: [SIMD3<Float>] = []
        result.reserveCapacity(to.count)
        
        let fromLength = cumulativeLength(points: from)
        let toLength = cumulativeLength(points: to)
        
        if fromLength < 1e-6 || toLength < 1e-6 {
            return to
        }
        
        var newVelocities: [SIMD3<Float>] = []
        newVelocities.reserveCapacity(to.count)
        
        let dragEndT = isDragged ? (dragState?.endIndex == 0 ? 0.0 : 1.0) : -1.0
        
        for i in 0..<to.count {
            let t = Float(i) / Float(max(1, to.count - 1))
            let targetDist = t * toLength
            let targetPoint = samplePoint(points: to, targetDist: targetDist, totalLength: toLength)
            
            let fromDist = t * fromLength
            let currentPoint = samplePoint(points: from, targetDist: fromDist, totalLength: fromLength)
            
            let distFromDragEnd = isDragged ? abs(Double(t) - dragEndT) : 1.0
            let responsiveness: Float
            if isDragged && distFromDragEnd < 0.3 {
                responsiveness = 0.98
            } else if isDragged {
                let falloff = min(1.0, distFromDragEnd * 1.5)
                responsiveness = Float(0.98 - falloff * 0.7)
            } else {
                responsiveness = 0.25
            }
            
            let velocity = velocities.count > i ? velocities[i] : SIMD3<Float>(0, 0, 0)
            let displacement = targetPoint - currentPoint
            let dist = simd_length(displacement)
            
            let nonlinearK: Float = 150.0
            let distFactor = 1.0 + dist * 8.0
            let springForce = displacement * nonlinearK * distFactor
            
            let criticalDamping = 2.0 * sqrt(nonlinearK * distFactor)
            let dampingForce = velocity * criticalDamping * 0.9
            
            let acceleration = springForce - dampingForce
            var newVelocity = velocity + acceleration * dt
            
            var newPos = currentPoint + newVelocity * dt
            newPos = newPos * (1.0 - responsiveness) + targetPoint * responsiveness
            
            result.append(newPos)
            newVelocities.append(newVelocity)
        }
        
        velocities = newVelocities
        return result
    }
    
    private func cumulativeLength(points: [SIMD3<Float>]) -> Float {
        guard points.count >= 2 else { return 0 }
        var total: Float = 0
        for i in 1..<points.count {
            total += simd_length(points[i] - points[i-1])
        }
        return total
    }
    
    private func samplePoint(points: [SIMD3<Float>], targetDist: Float, totalLength: Float) -> SIMD3<Float> {
        guard points.count >= 2, totalLength > 1e-6 else {
            return points.first ?? .zero
        }
        
        var cumDist: Float = 0
        for i in 1..<points.count {
            let segLen = simd_length(points[i] - points[i-1])
            let nextCumDist = cumDist + segLen
            
            if targetDist <= nextCumDist || i == points.count - 1 {
                let t = segLen > 1e-6 ? (targetDist - cumDist) / segLen : 0
                let clampedT = max(0, min(1, t))
                return points[i-1] + (points[i] - points[i-1]) * clampedT
            }
            
            cumDist = nextCumDist
        }
        
        return points.last ?? .zero
    }
    
    private static func applyHoleBendVisual(
        points: [SIMD3<Float>],
        topology: TopologyEngine?,
        ropeIndex: Int,
        holeRadius: Float,
        holePositions: [SIMD2<Float>],
        ropes: [RopeEndpoints]
    ) -> [SIMD3<Float>] {
        guard points.count >= 2 else { return points }
        guard let topology else { return points }
        guard topology.ropes.indices.contains(ropeIndex) else { return points }
        
        let rope = topology.ropes[ropeIndex]
        guard rope.active else { return points }
        
        let startHoleIndex = rope.startHole
        let endHoleIndex = rope.endHole
        
        guard let startHolePos = holePositions[safe: startHoleIndex],
              let endHolePos = holePositions[safe: endHoleIndex] else {
            return points
        }
        
        let bendRadius = holeRadius * 0.85
        let holeDepth = holeRadius * 1.25
        
        var result = points
        let count = result.count
        
        for i in 0..<count {
            let p = result[i]
            let p2 = SIMD2<Float>(p.x, p.y)
            
            let distToStart = simd_length(p2 - startHolePos)
            if distToStart < bendRadius {
                let t = distToStart / bendRadius
                let smoothT = t * t * (3 - 2 * t)
                let targetZ = -holeDepth * (1 - smoothT)
                result[i].z = min(p.z, targetZ)
            }
            
            let distToEnd = simd_length(p2 - endHolePos)
            if distToEnd < bendRadius {
                let t = distToEnd / bendRadius
                let smoothT = t * t * (3 - 2 * t)
                let targetZ = -holeDepth * (1 - smoothT)
                result[i].z = min(p.z, targetZ)
            }
        }
        
        return result
    }
}
