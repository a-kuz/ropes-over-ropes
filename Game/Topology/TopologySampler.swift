import simd

enum TopologySampler {
    static func sampleRope(engine: TopologyEngine, ropeIndex: Int, count: Int, lift: Float, dragLift: Float, ropeWidth: Float, ropeWidthForIndex: (Int) -> Float, ropeHeightForIndex: (Int) -> Float, holeRadius: Float) -> [SIMD3<Float>] {
        let ropeHeight = ropeHeightForIndex(ropeIndex)
        let poly = buildPolyWithPhysics(
            engine: engine,
            ropeIndex: ropeIndex,
            lift: lift,
            dragLift: dragLift,
            ropeWidth: ropeWidth,
            ropeHeight: ropeHeight,
            ropeWidthForIndex: ropeWidthForIndex,
            ropeHeightForIndex: ropeHeightForIndex,
            holeRadius: holeRadius
        )
        return resample(poly: poly, count: count)
    }
    
    static func sampleRopeRender(engine: TopologyEngine, ropeIndex: Int, lift: Float, dragLift: Float, ropeWidth: Float, ropeWidthForIndex: (Int) -> Float, ropeHeightForIndex: (Int) -> Float, holeRadius: Float) -> [SIMD3<Float>] {
        let ropeHeight = ropeHeightForIndex(ropeIndex)
        return buildPolyWithPhysics(
            engine: engine,
            ropeIndex: ropeIndex,
            lift: lift,
            dragLift: dragLift,
            ropeWidth: ropeWidth,
            ropeHeight: ropeHeight,
            ropeWidthForIndex: ropeWidthForIndex,
            ropeHeightForIndex: ropeHeightForIndex,
            holeRadius: holeRadius
        )
    }
    
    static func hookCenters(engine: TopologyEngine) -> [SIMD2<Float>] {
        var centers: [SIMD2<Float>] = []
        var processed = Set<Int>()
        
        for ropeIndex in engine.ropes.indices {
            let rope = engine.ropes[ropeIndex]
            if !rope.active { continue }
            let nodes = rope.nodes
            
            for i in 0..<nodes.count {
                guard case .crossing(let idA) = nodes[i],
                      let crossingA = engine.crossings[idA],
                      !processed.contains(idA) else { continue }
                
                for j in (i + 1)..<nodes.count {
                    guard case .crossing(let idB) = nodes[j],
                          let crossingB = engine.crossings[idB],
                          !processed.contains(idB),
                          isHookPair(crossingA: crossingA, crossingB: crossingB, ropeIndex: ropeIndex) else { continue }
                    
                    let prevPos = (i > 0) ? engine.position(of: nodes[i - 1]) : .zero
                    let nextPos = (j + 1 < nodes.count) ? engine.position(of: nodes[j + 1]) : .zero
                    
                    let otherRopeIndex = (ropeIndex == crossingA.ropeA) ? crossingA.ropeB : crossingA.ropeA
                    let otherPrevNext = otherRopeNeighbors(engine: engine, ropeIndex: otherRopeIndex, crossingA: crossingA, crossingB: crossingB)
                    
                    let center = (prevPos + nextPos + otherPrevNext.0 + otherPrevNext.1) * 0.25
                    
                    centers.append(center)
                    processed.insert(idA)
                    processed.insert(idB)
                    break
                }
            }
        }
        return centers
    }
    
    private static func buildPolyWithPhysics(
        engine: TopologyEngine,
        ropeIndex: Int,
        lift: Float,
        dragLift: Float,
        ropeWidth: Float,
        ropeHeight: Float,
        ropeWidthForIndex: (Int) -> Float,
        ropeHeightForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> [SIMD3<Float>] {
        let rope = engine.ropes[ropeIndex]
        if !rope.active { return [] }
        
        let nodes = rope.nodes
        if nodes.count < 2 { return [] }
        
        guard let startNode = nodes.first,
              let endNode = nodes.last else { return [] }
        
        let holeInnerRadius = holeRadius * 0.76
        
        let (startAnchor, startZ): (SIMD2<Float>, Float)
        switch startNode {
        case .floating:
            startAnchor = engine.position(of: startNode)
            startZ = dragLift
        case .hole(let holeIndex):
            if let holeCenter = engine.holePositions[safe: holeIndex] {
                let endPos = engine.position(of: endNode)
                let dir = endPos - holeCenter
                let dirLen = simd_length(dir)
                if dirLen > 1e-6 {
                    let dirNorm = dir / dirLen
                    startAnchor = holeCenter + dirNorm * holeInnerRadius
                } else {
                    startAnchor = holeCenter
                }
            } else {
                startAnchor = engine.position(of: startNode)
            }
            startZ = 0
        default:
            startAnchor = engine.position(of: startNode)
            startZ = 0
        }
        
        let (endAnchor, endZ): (SIMD2<Float>, Float)
        switch endNode {
        case .floating:
            endAnchor = engine.position(of: endNode)
            endZ = dragLift
        case .hole(let holeIndex):
            if let holeCenter = engine.holePositions[safe: holeIndex] {
                let startPos = startAnchor
                let dir = holeCenter - startPos
                let dirLen = simd_length(dir)
                if dirLen > 1e-6 {
                    let dirNorm = dir / dirLen
                    endAnchor = holeCenter - dirNorm * holeInnerRadius
                } else {
                    endAnchor = holeCenter
                }
            } else {
                endAnchor = engine.position(of: endNode)
            }
            endZ = 0
        default:
            endAnchor = engine.position(of: endNode)
            endZ = 0
        }
        
        let hookPairs = findHookPairsForRope(engine: engine, ropeIndex: ropeIndex, nodes: nodes)
        
        if !hookPairs.isEmpty {
            return buildHookPoly(
                engine: engine,
                ropeIndex: ropeIndex,
                nodes: nodes,
                hookPairs: hookPairs,
                lift: lift,
                dragLift: dragLift,
                ropeWidth: ropeWidth,
                ropeWidthForIndex: ropeWidthForIndex,
                ropeHeightForIndex: ropeHeightForIndex,
                holeRadius: holeRadius
            )
        }
        
        return buildCrossingPoly(
            engine: engine,
            ropeIndex: ropeIndex,
            nodes: nodes,
            startAnchor: startAnchor,
            endAnchor: endAnchor,
            startZ: startZ,
            endZ: endZ,
            lift: lift,
            dragLift: dragLift,
            ropeWidth: ropeWidth,
            ropeHeight: ropeHeight,
            ropeWidthForIndex: ropeWidthForIndex,
            ropeHeightForIndex: ropeHeightForIndex,
            holeRadius: holeRadius
        )
    }
    
    private static func buildCrossingPoly(
        engine: TopologyEngine,
        ropeIndex: Int,
        nodes: [TopologyNode],
        startAnchor: SIMD2<Float>,
        endAnchor: SIMD2<Float>,
        startZ: Float,
        endZ: Float,
        lift: Float,
        dragLift: Float,
        ropeWidth: Float,
        ropeHeight: Float,
        ropeWidthForIndex: (Int) -> Float,
        ropeHeightForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> [SIMD3<Float>] {
        var poly: [SIMD3<Float>] = []
        poly.reserveCapacity(128)
        
        let segmentCount = 64
        let dir = endAnchor - startAnchor
        let length = simd_length(dir)
        
        if length < 1e-6 {
            return [SIMD3<Float>(startAnchor.x, startAnchor.y, startZ)]
        }
        
        guard let startNode = nodes.first,
              let endNode = nodes.last else { return [] }
        
        let startHolePos: SIMD2<Float>?
        let endHolePos: SIMD2<Float>?
        
        if case .hole(let startHoleIndex) = startNode {
            startHolePos = engine.holePositions[safe: startHoleIndex]
        } else {
            startHolePos = nil
        }
        
        if case .hole(let endHoleIndex) = endNode {
            endHolePos = engine.holePositions[safe: endHoleIndex]
        } else {
            endHolePos = nil
        }
        
        let holeDepth = holeRadius * 1.25
        let holeBendRadius = holeRadius * 0.85
        
        var crossingInfos: [(position: SIMD2<Float>, isOver: Bool, otherHeight: Float, otherWidth: Float)] = []
        
        for node in nodes {
            if case .crossing(let crossingId) = node,
               let crossing = engine.crossings[crossingId] {
                let isOver = crossing.ropeOver == ropeIndex
                let otherIndex = (crossing.ropeA == ropeIndex) ? crossing.ropeB : crossing.ropeA
                let otherHeight = ropeHeightForIndex(otherIndex)
                let otherWidth = ropeWidthForIndex(otherIndex)
                crossingInfos.append((crossing.position, isOver, otherHeight, otherWidth))
            }
        }
        
        for i in 0...segmentCount {
            let t = Float(i) / Float(segmentCount)
            let xy = startAnchor + dir * t
            var z = startZ + (endZ - startZ) * t
            
            if startZ > 0 || endZ > 0 {
                z = max(0, z)
            }
            
            let endFade = smoothstep(edge0: 0.05, edge1: 0.15, value: t) *
                          smoothstep(edge0: 0.05, edge1: 0.15, value: 1.0 - t)
            
            for info in crossingInfos {
                let dist = simd_length(xy - info.position)
                let influenceRadius = max(ropeWidth, info.otherWidth) * 1.5
                
                if dist < influenceRadius {
                    let weight = (1.0 - dist / influenceRadius)
                    let smoothWeight = weight * weight * (3.0 - 2.0 * weight) * endFade
                    
                    if info.isOver {
                        let targetZ = info.otherHeight + 0.002
                        z = max(z, targetZ * smoothWeight)
                    }
                }
            }
            
            poly.append(SIMD3<Float>(xy.x, xy.y, z))
        }
        
        return poly
    }
    
    private static func findHookPairsForRope(
        engine: TopologyEngine,
        ropeIndex: Int,
        nodes: [TopologyNode]
    ) -> [(indexA: Int, indexB: Int, crossingA: TopologyCrossing, crossingB: TopologyCrossing)] {
        var hookPairs: [(indexA: Int, indexB: Int, crossingA: TopologyCrossing, crossingB: TopologyCrossing)] = []
        
        for i in 0..<nodes.count {
            guard case .crossing(let crossingIdA) = nodes[i],
                  let crossingA = engine.crossings[crossingIdA] else { continue }
            
            for j in (i + 1)..<nodes.count {
                guard case .crossing(let crossingIdB) = nodes[j],
                      let crossingB = engine.crossings[crossingIdB],
                      isHookPair(crossingA: crossingA, crossingB: crossingB, ropeIndex: ropeIndex) else { continue }
                
                hookPairs.append((indexA: i, indexB: j, crossingA: crossingA, crossingB: crossingB))
                break
            }
        }
        
        return hookPairs
    }
    
    private static func buildHookPoly(
        engine: TopologyEngine,
        ropeIndex: Int,
        nodes: [TopologyNode],
        hookPairs: [(indexA: Int, indexB: Int, crossingA: TopologyCrossing, crossingB: TopologyCrossing)],
        lift: Float,
        dragLift: Float,
        ropeWidth: Float,
        ropeWidthForIndex: (Int) -> Float,
        ropeHeightForIndex: (Int) -> Float,
        holeRadius: Float
    ) -> [SIMD3<Float>] {
        var poly: [SIMD3<Float>] = []
        poly.reserveCapacity(nodes.count * 24)
        
        var processedNodeIndices = Set<Int>()
        
        guard let startNode = nodes.first else { return [] }
        let holeDepth = holeRadius * 1.25
        let holeInnerRadius = holeRadius * 0.76
        
        let (startAnchor, startZ): (SIMD2<Float>, Float)
        switch startNode {
        case .floating:
            startAnchor = engine.position(of: startNode)
            startZ = dragLift
        case .hole(let holeIndex):
            if let holeCenter = engine.holePositions[safe: holeIndex],
               let endNode = nodes.last {
                let endPos = engine.position(of: endNode)
                let dir = endPos - holeCenter
                let dirLen = simd_length(dir)
                if dirLen > 1e-6 {
                    let dirNorm = dir / dirLen
                    startAnchor = holeCenter + dirNorm * holeInnerRadius
                } else {
                    startAnchor = holeCenter
                }
            } else {
                startAnchor = engine.position(of: startNode)
            }
            startZ = -holeDepth
        default:
            startAnchor = engine.position(of: startNode)
            startZ = 0
        }
        poly.append(SIMD3<Float>(startAnchor.x, startAnchor.y, startZ))
        processedNodeIndices.insert(0)
        
        var nodeIndex = 1
        while nodeIndex < nodes.count {
            if processedNodeIndices.contains(nodeIndex) {
                nodeIndex += 1
                continue
            }
            
            let node = nodes[nodeIndex]
            
            if case .crossing = node,
               let hookPair = hookPairs.first(where: { $0.indexA == nodeIndex || $0.indexB == nodeIndex }) {
                
                let crossingA = hookPair.crossingA
                let indexA = hookPair.indexA
                let indexB = hookPair.indexB
                
                let otherRopeIndex = (ropeIndex == crossingA.ropeA) ? crossingA.ropeB : crossingA.ropeA
                let otherWidth = ropeWidthForIndex(otherRopeIndex)
                let otherHeight = ropeHeightForIndex(otherRopeIndex)
                let hookRadius = max(otherWidth * 0.5, 0.04)
                
                let prevPos = (indexA > 0) ? engine.position(of: nodes[indexA - 1]) : startAnchor
                let nextPos = (indexB + 1 < nodes.count) ? engine.position(of: nodes[indexB + 1]) : prevPos
                
                let otherPrevNext = otherRopeNeighbors(engine: engine, ropeIndex: otherRopeIndex, crossingA: crossingA, crossingB: hookPair.crossingB)
                
                let hookCenter = (prevPos + nextPos + otherPrevNext.0 + otherPrevNext.1) * 0.25
                
                let aIsUnder = crossingA.ropeOver != ropeIndex
                
                let overZ: Float = otherHeight + 0.003
                let underZ: Float = 0
                
                let lineDir: SIMD2<Float>
                let lineDirRaw = nextPos - prevPos
                if simd_length_squared(lineDirRaw) > 1e-8 {
                    lineDir = simd_normalize(lineDirRaw)
                } else {
                    lineDir = SIMD2<Float>(1, 0)
                }
                
                let otherMid = (otherPrevNext.0 + otherPrevNext.1) * 0.5
                let toOther = otherMid - prevPos
                let perpDist = toOther.x * (-lineDir.y) + toOther.y * lineDir.x
                let farSide: Float = perpDist >= 0 ? 1 : -1
                let farDir = SIMD2<Float>(-lineDir.y, lineDir.x) * farSide
                
                let touch1 = tangentPointOnSide(from: prevPos, center: hookCenter, radius: hookRadius, sideDir: farDir)
                let touch2 = tangentPointOnSide(from: nextPos, center: hookCenter, radius: hookRadius, sideDir: farDir)
                
                let transitionSteps = 8
                for step in 1...transitionSteps {
                    let t = Float(step) / Float(transitionSteps)
                    let smoothT = t * t * (3.0 - 2.0 * t)
                    let xy = prevPos + (touch1 - prevPos) * smoothT
                    let targetZ = aIsUnder ? underZ : overZ
                    let z = targetZ * smoothT
                    poly.append(SIMD3<Float>(xy.x, xy.y, z))
                }
                
                let angle1 = atan2(touch1.y - hookCenter.y, touch1.x - hookCenter.x)
                let angle2 = atan2(touch2.y - hookCenter.y, touch2.x - hookCenter.x)
                
                var angleDiff = angle2 - angle1
                if angleDiff > Float.pi { angleDiff -= 2 * Float.pi }
                if angleDiff < -Float.pi { angleDiff += 2 * Float.pi }
                
                let arcSteps = max(12, Int(abs(angleDiff) * hookRadius / 0.006))
                for step in 1..<arcSteps {
                    let t = Float(step) / Float(arcSteps)
                    let angle = angle1 + angleDiff * t
                    let arcX = hookCenter.x + cos(angle) * hookRadius
                    let arcY = hookCenter.y + sin(angle) * hookRadius
                    
                    let arcZ: Float
                    if aIsUnder {
                        arcZ = underZ + (overZ - underZ) * t
                    } else {
                        arcZ = overZ + (underZ - overZ) * t
                    }
                    poly.append(SIMD3<Float>(arcX, arcY, arcZ))
                }
                
                for step in 1...transitionSteps {
                    let t = Float(step) / Float(transitionSteps)
                    let smoothT = t * t * (3.0 - 2.0 * t)
                    let xy = touch2 + (nextPos - touch2) * smoothT
                    let startZ = aIsUnder ? overZ : underZ
                    let z = startZ * (1.0 - smoothT)
                    poly.append(SIMD3<Float>(xy.x, xy.y, z))
                }
                
                for idx in indexA...indexB {
                    processedNodeIndices.insert(idx)
                }
                
                nodeIndex = indexB + 1
                continue
            }
            
            let positionXY = engine.position(of: node)
            let positionZ = baseZ(engine: engine, ropeIndex: ropeIndex, node: node, lift: lift, dragLift: dragLift)
            poly.append(SIMD3<Float>(positionXY.x, positionXY.y, positionZ))
            processedNodeIndices.insert(nodeIndex)
            nodeIndex += 1
        }
        
        guard let endNode = nodes.last else { 
            return applyHoleBend(poly: poly, engine: engine, nodes: nodes, holeRadius: holeRadius)
        }
        if !processedNodeIndices.contains(nodes.count - 1) {
            let (endAnchor, endZ): (SIMD2<Float>, Float)
            switch endNode {
            case .floating:
                endAnchor = engine.position(of: endNode)
                endZ = dragLift
            case .hole(let holeIndex):
                if let holeCenter = engine.holePositions[safe: holeIndex] {
                    let dir = holeCenter - startAnchor
                    let dirLen = simd_length(dir)
                    if dirLen > 1e-6 {
                        let dirNorm = dir / dirLen
                        endAnchor = holeCenter - dirNorm * holeInnerRadius
                    } else {
                        endAnchor = holeCenter
                    }
                } else {
                    endAnchor = engine.position(of: endNode)
                }
                endZ = -holeRadius * 1.25
            default:
                endAnchor = engine.position(of: endNode)
                endZ = 0
            }
            poly.append(SIMD3<Float>(endAnchor.x, endAnchor.y, endZ))
        }
        
        return applyHoleBend(poly: poly, engine: engine, nodes: nodes, holeRadius: holeRadius)
    }

    private static func baseZ(engine: TopologyEngine, ropeIndex: Int, node: TopologyNode, lift: Float, dragLift: Float) -> Float {
        switch node {
        case .floating:
            return dragLift
        case .crossing:
            return 0
        default:
            return 0
        }
    }

    private static func isHookPair(crossingA: TopologyCrossing, crossingB: TopologyCrossing, ropeIndex: Int) -> Bool {
        if crossingA.id == crossingB.id { return false }
        if crossingA.ropeA != crossingB.ropeA || crossingA.ropeB != crossingB.ropeB { return false }
        if crossingA.ropeA != ropeIndex && crossingA.ropeB != ropeIndex { return false }
        
        let aIsOver = crossingA.ropeOver == ropeIndex
        let bIsOver = crossingB.ropeOver == ropeIndex
        return aIsOver != bIsOver
    }
    
    private static func otherRopeNeighbors(engine: TopologyEngine, ropeIndex: Int, crossingA: TopologyCrossing, crossingB: TopologyCrossing) -> (SIMD2<Float>, SIMD2<Float>) {
        guard engine.ropes.indices.contains(ropeIndex) else {
            return (.zero, .zero)
        }
        let otherNodes = engine.ropes[ropeIndex].nodes
        if otherNodes.count < 2 {
            return (.zero, .zero)
        }
        
        var firstCrossingIdx: Int?
        var lastCrossingIdx: Int?
        
        for (idx, node) in otherNodes.enumerated() {
            if case .crossing(let id) = node {
                if id == crossingA.id || id == crossingB.id {
                    if firstCrossingIdx == nil {
                        firstCrossingIdx = idx
                    }
                    lastCrossingIdx = idx
                }
            }
        }
        
        guard let firstIdx = firstCrossingIdx, let lastIdx = lastCrossingIdx else {
            let first = engine.position(of: otherNodes.first!)
            let last = engine.position(of: otherNodes.last!)
            return (first, last)
        }
        
        let prevIdx = max(0, firstIdx - 1)
        let nextIdx = min(otherNodes.count - 1, lastIdx + 1)
        
        let prev = engine.position(of: otherNodes[prevIdx])
        let next = engine.position(of: otherNodes[nextIdx])
        return (prev, next)
    }
    
    private static func tangentPointOnSide(from point: SIMD2<Float>, center: SIMD2<Float>, radius: Float, sideDir: SIMD2<Float>) -> SIMD2<Float> {
        let d = center - point
        let dist = simd_length(d)
        
        if dist <= radius {
            return center + simd_normalize(sideDir) * radius
        }
        
        let theta = acos(radius / dist)
        let baseAngle = atan2(d.y, d.x)
        
        let tangentAnglePlus = baseAngle + theta
        let tangentAngleMinus = baseAngle - theta
        
        let touchPlus = center - SIMD2<Float>(cos(tangentAnglePlus), sin(tangentAnglePlus)) * radius
        let touchMinus = center - SIMD2<Float>(cos(tangentAngleMinus), sin(tangentAngleMinus)) * radius
        
        let dotPlus = simd_dot(touchPlus - center, sideDir)
        let dotMinus = simd_dot(touchMinus - center, sideDir)
        
        return dotPlus > dotMinus ? touchPlus : touchMinus
    }
    
    private static func optimalHookPoint(
        ropeSegmentStart: SIMD2<Float>,
        ropeSegmentEnd: SIMD2<Float>,
        hookFrom: SIMD2<Float>,
        hookTo: SIMD2<Float>
    ) -> SIMD2<Float> {
        let segDir = ropeSegmentEnd - ropeSegmentStart
        let segLen2 = simd_length_squared(segDir)
        
        if segLen2 < 1e-8 {
            return ropeSegmentStart
        }
        
        let segLen = sqrt(segLen2)
        let segNorm = segDir / segLen
        let segPerp = SIMD2<Float>(-segNorm.y, segNorm.x)
        
        let toHookTo = hookTo - ropeSegmentStart
        let perpDist = simd_dot(toHookTo, segPerp)
        let hookToReflected = hookTo - 2 * perpDist * segPerp
        
        let rayDir = hookToReflected - hookFrom
        let rayDirLen2 = simd_length_squared(rayDir)
        
        if rayDirLen2 < 1e-8 {
            let t = simd_dot(hookFrom - ropeSegmentStart, segDir) / segLen2
            let tClamped = max(0, min(1, t))
            return ropeSegmentStart + segDir * tClamped
        }
        
        let cross = rayDir.x * segDir.y - rayDir.y * segDir.x
        
        if abs(cross) < 1e-8 {
            let t = simd_dot(hookFrom - ropeSegmentStart, segDir) / segLen2
            let tClamped = max(0, min(1, t))
            return ropeSegmentStart + segDir * tClamped
        }
        
        let d = ropeSegmentStart - hookFrom
        let u = (d.x * rayDir.y - d.y * rayDir.x) / cross
        
        let uClamped = max(0, min(1, u))
        return ropeSegmentStart + segDir * uClamped
    }
    
    private static func applyHoleBend(poly: [SIMD3<Float>], engine: TopologyEngine, nodes: [TopologyNode], holeRadius: Float) -> [SIMD3<Float>] {
        guard !poly.isEmpty, let startNode = nodes.first, let endNode = nodes.last else { return poly }
        
        let holeDepth = holeRadius * 1.25
        let holeBendRadius = holeRadius * 0.85
        let bendZoneOuter = holeRadius * 2.5
        
        var result = poly
        
        let startHolePos: SIMD2<Float>?
        let endHolePos: SIMD2<Float>?
        let startZ: Float
        let endZ: Float
        
        if case .hole(let startHoleIndex) = startNode {
            startHolePos = engine.holePositions[safe: startHoleIndex]
            startZ = -holeDepth
        } else {
            startHolePos = nil
            startZ = 0
        }
        
        if case .hole(let endHoleIndex) = endNode {
            endHolePos = engine.holePositions[safe: endHoleIndex]
            endZ = -holeDepth
        } else {
            endHolePos = nil
            endZ = 0
        }
        
        for i in 0..<result.count {
            let p = result[i]
            let xy = SIMD2<Float>(p.x, p.y)
            var z = p.z
            
            var holeBendZ: Float = z
            let u = Float(i) / Float(max(1, result.count - 1))
            
            if let startHole = startHolePos, startZ < 0 {
                let distFromHoleCenter = simd_length(xy - startHole)
                let holeInnerRadius = holeRadius * 0.76
                let bendZoneOuter = holeRadius * 2.0
                let nearAnchorWeight = 1.0 - smoothstep(edge0: 0.0, edge1: 0.25, value: u)
                
                if distFromHoleCenter <= holeInnerRadius && nearAnchorWeight > 0.01 {
                    let depthFactor: Float = 1.0
                    holeBendZ = -holeDepth * depthFactor * nearAnchorWeight
                } else if distFromHoleCenter < bendZoneOuter && nearAnchorWeight > 0.01 {
                    let distFromEdge = distFromHoleCenter - holeInnerRadius
                    let bendZoneWidth = bendZoneOuter - holeInnerRadius
                    let distT = distFromEdge / bendZoneWidth
                    let depthFactor = 1.0 - smoothstep(edge0: 0.0, edge1: 1.0, value: distT)
                    let anchorDepth = -holeDepth * depthFactor * nearAnchorWeight
                    holeBendZ = min(holeBendZ, anchorDepth)
                } else if nearAnchorWeight > 0.1 {
                    let anchorDepth = -holeDepth * nearAnchorWeight
                    holeBendZ = min(holeBendZ, anchorDepth)
                }
            }
            
            if let endHole = endHolePos, endZ < 0 {
                let distFromHoleCenter = simd_length(xy - endHole)
                let holeInnerRadius = holeRadius * 0.76
                let bendZoneOuter = holeRadius * 2.0
                let nearAnchorWeight = 1.0 - smoothstep(edge0: 0.0, edge1: 0.25, value: 1.0 - u)
                
                if distFromHoleCenter <= holeInnerRadius && nearAnchorWeight > 0.01 {
                    let depthFactor: Float = 1.0
                    let endHoleBendZ = -holeDepth * depthFactor * nearAnchorWeight
                    holeBendZ = min(holeBendZ, endHoleBendZ)
                } else if distFromHoleCenter < bendZoneOuter && nearAnchorWeight > 0.01 {
                    let distFromEdge = distFromHoleCenter - holeInnerRadius
                    let bendZoneWidth = bendZoneOuter - holeInnerRadius
                    let distT = distFromEdge / bendZoneWidth
                    let depthFactor = 1.0 - smoothstep(edge0: 0.0, edge1: 1.0, value: distT)
                    let anchorDepth = -holeDepth * depthFactor * nearAnchorWeight
                    holeBendZ = min(holeBendZ, anchorDepth)
                } else if nearAnchorWeight > 0.1 {
                    let anchorDepth = -holeDepth * nearAnchorWeight
                    holeBendZ = min(holeBendZ, anchorDepth)
                }
            }
            
            z = min(z, holeBendZ)
            result[i] = SIMD3<Float>(xy.x, xy.y, z)
        }
        
        return result
    }
    
    private static func smoothstep(edge0: Float, edge1: Float, value: Float) -> Float {
        let normalized = max(0, min(1, (value - edge0) / (edge1 - edge0)))
        return normalized * normalized * (3 - 2 * normalized)
    }

    private static func cumulativeLengths(poly: [SIMD3<Float>]) -> [Float] {
        if poly.count < 2 { return [0] }
        var cum: [Float] = [0]
        cum.reserveCapacity(poly.count)
        for pointIndex in 1..<poly.count {
            let pointA = poly[pointIndex - 1]
            let pointB = poly[pointIndex]
            cum.append(cum[pointIndex - 1] + simd_length(pointB - pointA))
        }
        return cum
    }

    private static func sample(poly: [SIMD3<Float>], cum: [Float], dist: Float) -> SIMD3<Float> {
        if poly.count <= 1 { return poly.first ?? .zero }
        var lowerIndex = 0
        var upperIndex = cum.count - 1
        while lowerIndex + 1 < upperIndex {
            let midIndex = (lowerIndex + upperIndex) / 2
            if cum[midIndex] <= dist {
                lowerIndex = midIndex
            } else {
                upperIndex = midIndex
            }
        }

        let pointA = poly[lowerIndex]
        let pointB = poly[min(lowerIndex + 1, poly.count - 1)]
        let startDist = cum[lowerIndex]
        let endDist = cum[min(lowerIndex + 1, cum.count - 1)]
        let segmentSpan = max(1e-6, endDist - startDist)
        let tValue = (dist - startDist) / segmentSpan
        return pointA + (pointB - pointA) * tValue
    }

    private static func resample(poly: [SIMD3<Float>], count: Int) -> [SIMD3<Float>] {
        let safeCount = max(0, count)
        if safeCount == 0 { return [] }
        if poly.isEmpty { return Array(repeating: .zero, count: safeCount) }
        if poly.count == 1 { return Array(repeating: poly[0], count: safeCount) }

        let lengths = cumulativeLengths(poly: poly)
        let total = lengths.last ?? 0
        if total <= 1e-6 { return Array(repeating: poly[0], count: safeCount) }

        var out: [SIMD3<Float>] = []
        out.reserveCapacity(safeCount)
        for sampleIndex in 0..<safeCount {
            let tValue = Float(sampleIndex) / Float(max(1, safeCount - 1))
            let distance = total * tValue
            out.append(sample(poly: poly, cum: lengths, dist: distance))
        }
        return out
    }
}
