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
        let repulsorsBase = ropeRenderSimpleMode ? [] : makeRepulsorsBase()
        let repulsorsWithTopology = ropeRenderSimpleMode ? [] : makeRepulsorsWithTopology()

        let ropeWidthForIndex: (Int) -> Float = { idx in
            self.ropes[safe: idx]?.width ?? 0.085
        }
        let ropeHeightForIndex: (Int) -> Float = { idx in
            self.ropes[safe: idx]?.height ?? 0.03
        }
        
        var allRopePoints: [[SIMD3<Float>]] = []
        var allSegmentStarts: [[Int]] = []
        var ropeWidths: [Float] = []
        var ropeHeights: [Float] = []
        
        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 {
                allRopePoints.append([])
                allSegmentStarts.append([])
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
                let result = TopologySampler.sampleRopeRender(
                    engine: topology,
                    ropeIndex: ropeIndex,
                    lift: lift,
                    dragLift: dragLift,
                    ropeWidth: ropeWidth,
                    ropeWidthForIndex: ropeWidthForIndex,
                    ropeHeightForIndex: ropeHeightForIndex,
                    holeRadius: holeRadius
                )
                allRopePoints.append(result.points)
                allSegmentStarts.append(result.segmentStarts)
            } else {
                var points: [SIMD3<Float>] = []
                simulation.withRopePositions(ropeIndex: ropeIndex) { buffer in
                    points = Array(buffer)
                }
                allRopePoints.append(points)
                allSegmentStarts.append([0])
            }
        }
        
        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 { continue }
            if allRopePoints[ropeIndex].isEmpty { continue }
            
            let ropeColor = ropes[ropeIndex].color
            let ropeWidth = ropes[ropeIndex].width
            let ropeHeight = ropes[ropeIndex].height
            
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
            
            let ropeMesh = renderPoints.withUnsafeBufferPointer { pointsBuffer in
                let events = Self.makeTwistEvents(topology: topology, ropeIndex: ropeIndex, points: pointsBuffer)
                let taut = ropeRenderSimpleMode ? 0 : Self.tautness(points: pointsBuffer)
                let repulsors = ropeRenderSimpleMode ? [] : (topology != nil ? repulsorsWithTopology : repulsorsBase)
                return RopeMeshBuilder.buildRect(
                    points: pointsBuffer,
                    width: ropeWidth,
                    height: ropeHeight,
                    color: ropeColor,
                    twistEvents: events,
                    tautness: taut,
                    repulsors: repulsors,
                    stretchRatio: ropeRenderSimpleMode ? 1.0 : stretchRatio,
                    oscillation: ropeRenderSimpleMode ? 0.0 : oscillation,
                    segmentStarts: TopologySampler.debugSegmentColors ? segmentStarts : []
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
        
        let hookCenters = TopologySampler.hookCenters(engine: topology, ropeWidthForIndex: { idx in
            self.ropes[safe: idx]?.width ?? 0.085
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
