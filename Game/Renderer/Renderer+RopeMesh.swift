import Metal
import simd
import os.log

extension Renderer {
    private static let meshLogger = Logger(subsystem: "com.uzls.four", category: "RopeMesh")
    
    func updateRopeMesh() {
        var allVertices: [RopeVertex] = []
        var allIndices: [UInt32] = []
        allVertices.reserveCapacity(ropes.count * 6400 * 8)
        allIndices.reserveCapacity(ropes.count * 6399 * 24)

        var baseVertex: UInt32 = 0

        let dragLift: Float = dragLiftCurrent
        let repulsorsBase = makeRepulsorsBase()
        let repulsorsWithTopology = makeRepulsorsWithTopology()

        let ropeWidthForIndex: (Int) -> Float = { idx in
            self.ropes[safe: idx]?.width ?? 0.085
        }
        let ropeHeightForIndex: (Int) -> Float = { idx in
            self.ropes[safe: idx]?.height ?? 0.03
        }
        
        var allRopePoints: [[SIMD3<Float>]] = []
        var ropeWidths: [Float] = []
        var ropeHeights: [Float] = []
        
        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 {
                allRopePoints.append([])
                ropeWidths.append(0)
                ropeHeights.append(0)
                continue
            }
            
            let ropeWidth = ropes[ropeIndex].width
            let ropeHeight = ropes[ropeIndex].height
            ropeWidths.append(ropeWidth)
            ropeHeights.append(ropeHeight)
            
            if let topology {
                let lift = max(ropeHeight * 1.35, 0.02)
                let points = TopologySampler.sampleRopeRender(
                    engine: topology,
                    ropeIndex: ropeIndex,
                    lift: lift,
                    dragLift: dragLift,
                    ropeWidth: ropeWidth,
                    ropeWidthForIndex: ropeWidthForIndex,
                    ropeHeightForIndex: ropeHeightForIndex,
                    holeRadius: holeRadius
                )
                allRopePoints.append(points)
            } else {
                var points: [SIMD3<Float>] = []
                simulation.withRopePositions(ropeIndex: ropeIndex) { buffer in
                    points = Array(buffer)
                }
                allRopePoints.append(points)
            }
        }
        
        if let dragState, dragLift > 0.01, let topology {
            let draggedRopeIndex = dragState.ropeIndex
            if allRopePoints.indices.contains(draggedRopeIndex) && !allRopePoints[draggedRopeIndex].isEmpty {
                
                let draggedRope = topology.ropes[draggedRopeIndex]
                var crossingsToUpdate: [(crossingId: Int, crossing: TopologyCrossing, otherRopeIndex: Int)] = []
                
                for node in draggedRope.nodes {
                    if case .crossing(let cid) = node,
                       let crossing = topology.crossings[cid] {
                        let otherIndex = (crossing.ropeA == draggedRopeIndex) ? crossing.ropeB : crossing.ropeA
                        
                        if crossing.ropeOver != draggedRopeIndex {
                            crossingsToUpdate.append((cid, crossing, otherIndex))
                        }
                    }
                }
                
                for crossingInfo in crossingsToUpdate {
                    let otherRopeIndex = crossingInfo.otherRopeIndex
                    guard allRopePoints.indices.contains(otherRopeIndex) else { continue }
                    guard !allRopePoints[otherRopeIndex].isEmpty else { continue }
                    
                    let draggedRopeEndpoints = ropes[draggedRopeIndex]
                    let otherRopeEndpoints = ropes[otherRopeIndex]
                    
                    guard let draggedStart = holePositions[safe: draggedRopeEndpoints.startHole],
                          let draggedEnd = holePositions[safe: draggedRopeEndpoints.endHole],
                          let otherStart = holePositions[safe: otherRopeEndpoints.startHole],
                          let otherEnd = holePositions[safe: otherRopeEndpoints.endHole] else { continue }
                    
                    var prevCrossingXY: SIMD2<Float>? = nil
                    if let currentCrossingIndex = draggedRope.nodes.firstIndex(where: { 
                        if case .crossing(let cid) = $0 { return cid == crossingInfo.crossingId }
                        return false
                    }) {
                        for i in (0..<currentCrossingIndex).reversed() {
                            if case .crossing(let prevCid) = draggedRope.nodes[i],
                               let prevCrossing = topology.crossings[prevCid],
                               prevCrossing.ropeOver != draggedRopeIndex {
                                prevCrossingXY = prevCrossing.position
                                break
                            }
                        }
                    }
                    
                    let fixedAnchor = (dragState.endIndex == 0) ? draggedEnd : draggedStart
                    let actualStartAnchor = prevCrossingXY ?? fixedAnchor
                    
                    var crossingPos = crossingInfo.crossing.position
                    var lowerPoints = allRopePoints[draggedRopeIndex]
                    var upperPoints = allRopePoints[otherRopeIndex]
                    
                    RopePhysics.computeDragCrossingPhysics(
                        lowerRopePoints: &lowerPoints,
                        upperRopePoints: &upperPoints,
                        crossingPosition: &crossingPos,
                        lowerRopeStartAnchor: actualStartAnchor,
                        lowerRopeEndAnchor: fixedAnchor,
                        upperRopeStartAnchor: otherStart,
                        upperRopeEndAnchor: otherEnd,
                        dragPosition: dragWorld,
                        dragZ: dragLift,
                        dragEndIndex: dragState.endIndex,
                        lowerRopeWidth: ropeWidths[draggedRopeIndex],
                        upperRopeHeight: ropeHeights[otherRopeIndex]
                    )
                    
                    allRopePoints[draggedRopeIndex] = lowerPoints
                    allRopePoints[otherRopeIndex] = upperPoints
                }
            }
        }
        
        if let topology, ropes.count >= 2 {
            RopePhysics.applyBandRepulsion(
                allRopePoints: &allRopePoints,
                ropeWidths: ropeWidths,
                ropeHeights: ropeHeights,
                topology: topology,
                iterations: 5
            )
        }
        
        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 { continue }
            if allRopePoints[ropeIndex].isEmpty { continue }
            
            let ropeColor = ropes[ropeIndex].color
            let ropeWidth = ropes[ropeIndex].width
            let ropeHeight = ropes[ropeIndex].height
            
            var renderPoints = allRopePoints[ropeIndex]
            
            if let topology {
                renderPoints = Self.applyCrossingVisualDeform(
                    points: renderPoints,
                    topology: topology,
                    ropeIndex: ropeIndex,
                    ropeHeight: ropeHeight,
                    ropeHeightForIndex: ropeHeightForIndex,
                    ropeWidth: ropeWidth,
                    ropeWidthForIndex: ropeWidthForIndex
                )
            }
            
            renderPoints = Self.applyHoleBendVisual(
                points: renderPoints,
                topology: topology,
                ropeIndex: ropeIndex,
                holeRadius: holeRadius,
                holePositions: holePositions,
                ropes: ropes
            )
            
            let isDragged = dragState?.ropeIndex == ropeIndex
            let stretchRatio = isDragged ? dragStretchRatio : 1.0
            let oscillation = (dragOscillationRopeIndex == ropeIndex) ? dragOscillationPhase : 0.0
            
            if oscillation != 0 && Int(time * 10) % 10 == 0 {
                Self.meshLogger.info("🎯 Applying oscillation to rope[\(ropeIndex)]: phase=\(String(format: "%.4f", oscillation)) amplitude=\(String(format: "%.4f", oscillation * 0.025))")
            }
            
            let ropeMesh = renderPoints.withUnsafeBufferPointer { pointsBuffer in
                let events = Self.makeTwistEvents(topology: topology, ropeIndex: ropeIndex, points: pointsBuffer)
                let taut = Self.tautness(points: pointsBuffer)
                let repulsors = topology != nil ? repulsorsWithTopology : repulsorsBase
                return RopeMeshBuilder.buildRect(
                    points: pointsBuffer,
                    width: ropeWidth,
                    height: ropeHeight,
                    color: ropeColor,
                    twistEvents: events,
                    tautness: taut,
                    repulsors: repulsors,
                    stretchRatio: stretchRatio,
                    oscillation: oscillation
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
                crossings: topology.crossings
            )
        }
        
        if Int(time) % 2 == 0 && time - Float(Int(time)) < 0.02 {
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
        let avgWidth: Float = ropes.isEmpty ? 0.08 : ropes.reduce(0) { $0 + $1.width } / Float(ropes.count)
        let hookR = (avgWidth * 0.35) * 1.06
        let hookStrength = hookR * 0.55
        let hookCenters = TopologySampler.hookCenters(engine: topology)
        repulsors.reserveCapacity(repulsors.count + hookCenters.count)
        for c in hookCenters {
            repulsors.append(SIMD4<Float>(c.x, c.y, hookR, hookStrength))
        }
        return repulsors
    }

    private static func tautness(points: UnsafeBufferPointer<SIMD3<Float>>) -> Float {
        if points.count < 2 { return 0 }
        var len: Float = 0
        for i in 1..<points.count {
            len += simd_length(points[i] - points[i - 1])
        }
        let chord = simd_length(points[points.count - 1] - points[0])
        let stretch = chord / max(1e-6, len)
        return smoothstep(edge0: 0.83, edge1: 0.985, value: stretch)
    }

    private static func smoothstep(edge0: Float, edge1: Float, value: Float) -> Float {
        let normalized = max(0, min(1, (value - edge0) / (edge1 - edge0)))
        return normalized * normalized * (3 - 2 * normalized)
    }
    
    private static func applyCrossingVisualDeform(points: [SIMD3<Float>], topology: TopologyEngine, ropeIndex: Int, ropeHeight: Float, ropeHeightForIndex: (Int) -> Float, ropeWidth: Float, ropeWidthForIndex: (Int) -> Float) -> [SIMD3<Float>] {
        if points.count < 2 { return points }
        if ropeIndex < 0 || ropeIndex >= topology.ropes.count { return points }
        
        let nodes = topology.ropes[ropeIndex].nodes
        var crossingIds: [Int] = []
        crossingIds.reserveCapacity(nodes.count)
        for n in nodes {
            if case .crossing(let cid) = n {
                crossingIds.append(cid)
            }
        }
        if crossingIds.isEmpty { return points }
        
        let hookPairs = topology.findHookPairs()
        var hookCrossingIds = Set<Int>()
        hookCrossingIds.reserveCapacity(hookPairs.count * 2)
        for hook in hookPairs {
            hookCrossingIds.insert(hook.crossingIdA)
            hookCrossingIds.insert(hook.crossingIdB)
        }
        
        struct CrossingInfluence {
            let position: SIMD2<Float>
            let radius: Float
            let targetZ: Float
            let isOver: Bool
        }
        
        var influences: [CrossingInfluence] = []
        influences.reserveCapacity(crossingIds.count)
        
        for cid in crossingIds {
            if hookCrossingIds.contains(cid) { continue }
            guard let cross = topology.crossings[cid] else { continue }
            let isOver = cross.ropeOver == ropeIndex
            let otherIndex = (cross.ropeA == ropeIndex) ? cross.ropeB : cross.ropeA
            let otherHeight = ropeHeightForIndex(otherIndex)
            let otherWidth = ropeWidthForIndex(otherIndex)
            let base = max(ropeWidth, otherWidth)
            let radius = max(base * 1.5, 0.06)
            let targetZ: Float = isOver ? (otherHeight + 0.002) : 0
            influences.append(CrossingInfluence(position: cross.position, radius: radius, targetZ: targetZ, isOver: isOver))
        }
        
        if influences.isEmpty { return points }
        
        let count = points.count
        var out = points
        
        for i in 0..<count {
            let u = Float(i) / Float(max(1, count - 1))
            let endFade = smoothstep(edge0: 0.08, edge1: 0.20, value: u) * smoothstep(edge0: 0.08, edge1: 0.20, value: 1 - u)
            if endFade <= 1e-5 { continue }

            var position = out[i]
            let p2 = SIMD2<Float>(position.x, position.y)
            var z = position.z

            for inf in influences {
                let dist = simd_length(p2 - inf.position)
                if dist >= inf.radius { continue }
                
                let weight = 1.0 - (dist / inf.radius)
                let smoothWeight = weight * weight * (3.0 - 2.0 * weight) * endFade
                
                if inf.isOver {
                    z = max(z, inf.targetZ * smoothWeight)
                }
            }

            position.z = z
            out[i] = position
        }
        
        return out
    }

    static func makeTwistEvents(topology: TopologyEngine?, ropeIndex: Int, points: UnsafeBufferPointer<SIMD3<Float>>) -> [RopeMeshBuilder.TwistEvent] {
        return []
    }
    
    private static func applyHoleBendVisual(
        points: [SIMD3<Float>],
        topology: TopologyEngine?,
        ropeIndex: Int,
        holeRadius: Float,
        holePositions: [SIMD2<Float>],
        ropes: [RopeEndpoints]
    ) -> [SIMD3<Float>] {
        guard let topology, !points.isEmpty else { return points }
        guard ropeIndex >= 0 && ropeIndex < topology.ropes.count else { return points }
        
        let rope = topology.ropes[ropeIndex]
        guard rope.active && rope.nodes.count >= 2 else { return points }
        
        guard let startNode = rope.nodes.first,
              let endNode = rope.nodes.last else { return points }
        
        guard ropeIndex < ropes.count else { return points }
        let ropeEndpoints = ropes[ropeIndex]
        
        let holeDepth = holeRadius * 1.25
        let holeInnerRadius = holeRadius * 0.76
        
        var startHolePos: SIMD2<Float>?
        var endHolePos: SIMD2<Float>?
        
        if case .hole(let startHoleIndex) = startNode,
           ropeEndpoints.startHole == startHoleIndex {
            startHolePos = holePositions[safe: startHoleIndex]
        }
        
        if case .hole(let endHoleIndex) = endNode,
           ropeEndpoints.endHole == endHoleIndex {
            endHolePos = holePositions[safe: endHoleIndex]
        }
        
        guard startHolePos != nil || endHolePos != nil else { return points }
        
        var result = points
        
        for i in 0..<result.count {
            let p = result[i]
            let xy = SIMD2<Float>(p.x, p.y)
            var z = p.z
            
            let wasRaisedByCrossing = z > 0.001
            
            if wasRaisedByCrossing {
                result[i] = p
                continue
            }
            
            let u = Float(i) / Float(max(1, result.count - 1))
            let nearStartWeight = 1.0 - smoothstep(edge0: 0.0, edge1: 0.25, value: u)
            let nearEndWeight = 1.0 - smoothstep(edge0: 0.0, edge1: 0.25, value: 1.0 - u)
            
            if let startHole = startHolePos, nearStartWeight > 0.01 {
                let distFromHoleCenter = simd_length(xy - startHole)
                
                if distFromHoleCenter <= holeInnerRadius {
                    let depthFactor: Float = 1.0
                    let holeBendZ = -holeDepth * depthFactor * nearStartWeight
                    z = min(z, holeBendZ)
                }
            }
            
            if let endHole = endHolePos, nearEndWeight > 0.01 {
                let distFromHoleCenter = simd_length(xy - endHole)
                
                if distFromHoleCenter <= holeInnerRadius {
                    let depthFactor: Float = 1.0
                    let holeBendZ = -holeDepth * depthFactor * nearEndWeight
                    z = min(z, holeBendZ)
                }
            }
            
            result[i] = SIMD3<Float>(xy.x, xy.y, z)
        }
        
        return result
    }

    private static func closestDistanceAlong(points: UnsafeBufferPointer<SIMD3<Float>>, toXY target: SIMD2<Float>) -> Float {
        var bestDistAlong: Float = 0
        var bestD2: Float = .greatestFiniteMagnitude

        var distanceAlong: Float = 0
        for segmentIndex in 0..<(points.count - 1) {
            let pointA3 = points[segmentIndex]
            let pointB3 = points[segmentIndex + 1]
            let pointA2 = SIMD2<Float>(pointA3.x, pointA3.y)
            let pointB2 = SIMD2<Float>(pointB3.x, pointB3.y)
            let segment = pointB2 - pointA2
            let segmentLen2 = simd_length_squared(segment)
            let segmentT: Float
            if segmentLen2 < 1e-10 {
                segmentT = 0
            } else {
                segmentT = max(0, min(1, simd_dot(target - pointA2, segment) / segmentLen2))
            }
            let projectedPoint = pointA2 + segment * segmentT
            let projectedDist2 = simd_length_squared(projectedPoint - target)
            if projectedDist2 < bestD2 {
                bestD2 = projectedDist2
                bestDistAlong = distanceAlong + simd_length(pointB3 - pointA3) * segmentT
            }
            distanceAlong += simd_length(pointB3 - pointA3)
        }

        return bestDistAlong
    }
}
