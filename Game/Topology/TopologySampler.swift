import simd

enum TopologySampler {
    static func sampleRope(engine: TopologyEngine, ropeIndex: Int, count: Int, lift: Float, dragLift: Float, ropeWidth: Float, ropeWidthForIndex: (Int) -> Float) -> [SIMD3<Float>] {
        let poly = buildPoly(engine: engine, ropeIndex: ropeIndex, lift: lift, dragLift: dragLift, ropeWidth: ropeWidth, ropeWidthForIndex: ropeWidthForIndex)
        return resample(poly: poly, count: count)
    }
    
    static func sampleRopeRender(engine: TopologyEngine, ropeIndex: Int, lift: Float, dragLift: Float, ropeWidth: Float, ropeWidthForIndex: (Int) -> Float) -> [SIMD3<Float>] {
        return buildPoly(engine: engine, ropeIndex: ropeIndex, lift: lift, dragLift: dragLift, ropeWidth: ropeWidth, ropeWidthForIndex: ropeWidthForIndex)
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
    
    private static func buildPoly(engine: TopologyEngine, ropeIndex: Int, lift: Float, dragLift: Float, ropeWidth: Float, ropeWidthForIndex: (Int) -> Float) -> [SIMD3<Float>] {
        let rope = engine.ropes[ropeIndex]
        if !rope.active { return [] }

        let nodes = rope.nodes
        if nodes.count < 2 { return [] }

        var poly: [SIMD3<Float>] = []
        poly.reserveCapacity(nodes.count * 16)

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
        
        var processedNodeIndices = Set<Int>()
        
        var nodeIndex = 0
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
                let hookRadius = otherWidth * 0.5
                
                let prevPos = (indexA > 0) ? engine.position(of: nodes[indexA - 1]) : .zero
                let nextPos = (indexB + 1 < nodes.count) ? engine.position(of: nodes[indexB + 1]) : .zero
                
                let otherPrevNext = otherRopeNeighbors(engine: engine, ropeIndex: otherRopeIndex, crossingA: crossingA, crossingB: hookPair.crossingB)
                
                let hookCenter = (prevPos + nextPos + otherPrevNext.0 + otherPrevNext.1) * 0.25
                
                let aIsUnder = crossingA.ropeOver != ropeIndex
                
                let overZ: Float = lift
                let underZ: Float = -lift * 0.25
                
                let lineDir = simd_normalize(nextPos - prevPos)
                let otherMid = (otherPrevNext.0 + otherPrevNext.1) * 0.5
                let toOther = otherMid - prevPos
                let perpDist = toOther.x * (-lineDir.y) + toOther.y * lineDir.x
                let farSide: Float = perpDist >= 0 ? 1 : -1
                let farDir = SIMD2<Float>(-lineDir.y, lineDir.x) * farSide
                
                let touch1 = tangentPointOnSide(from: prevPos, center: hookCenter, radius: hookRadius, sideDir: farDir)
                let touch2 = tangentPointOnSide(from: nextPos, center: hookCenter, radius: hookRadius, sideDir: farDir)
                
                let angle1 = atan2(touch1.y - hookCenter.y, touch1.x - hookCenter.x)
                let angle2 = atan2(touch2.y - hookCenter.y, touch2.x - hookCenter.x)
                
                var angleDiff = angle2 - angle1
                if angleDiff > Float.pi { angleDiff -= 2 * Float.pi }
                if angleDiff < -Float.pi { angleDiff += 2 * Float.pi }
                
                poly.append(SIMD3<Float>(touch1.x, touch1.y, aIsUnder ? underZ : overZ))
                
                let arcSteps = max(4, Int(abs(angleDiff) * hookRadius / 0.01))
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
                
                poly.append(SIMD3<Float>(touch2.x, touch2.y, aIsUnder ? overZ : underZ))
                poly.append(SIMD3<Float>(touch2.x, touch2.y, aIsUnder ? overZ : underZ))

                for idx in indexA...indexB {
                    processedNodeIndices.insert(idx)
                }

                nodeIndex = indexB + 1
                continue
            }

            if case .crossing(let crossingId) = node,
               let crossing = engine.crossings[crossingId] {
                let isOver = crossing.ropeOver == ropeIndex
                let otherRopeIndex = (ropeIndex == crossing.ropeA) ? crossing.ropeB : crossing.ropeA
                let otherWidth = ropeWidthForIndex(otherRopeIndex)
                
                let bumpHeight: Float = isOver ? (lift + otherWidth * 0.3) : (-lift * 0.15)
                let bumpRadius = max(ropeWidth, otherWidth) * 1.5
                
                var prevIdx = nodeIndex - 1
                while prevIdx >= 0 {
                    let n = nodes[prevIdx]
                    if case .hole = n { break }
                    if case .floating = n { break }
                    prevIdx -= 1
                }
                let prevAnchor = prevIdx >= 0 ? engine.position(of: nodes[prevIdx]) : crossing.position
                
                var nextIdx = nodeIndex + 1
                while nextIdx < nodes.count {
                    let n = nodes[nextIdx]
                    if case .hole = n { break }
                    if case .floating = n { break }
                    nextIdx += 1
                }
                let nextAnchor = nextIdx < nodes.count ? engine.position(of: nodes[nextIdx]) : crossing.position
                
                let line = nextAnchor - prevAnchor
                let lineLen = simd_length(line)
                let lineDir = lineLen > 1e-6 ? line / lineLen : SIMD2<Float>(1, 0)
                
                let t = lineLen > 1e-6 ? simd_dot(crossing.position - prevAnchor, lineDir) / lineLen : 0.5
                let crossingPosOnLine = prevAnchor + line * max(0.01, min(0.99, t))
                
                let distFromPrev = t * lineLen
                let distToNext = (1 - t) * lineLen
                
                let rampBefore = min(bumpRadius, distFromPrev * 0.7)
                let rampAfter = min(bumpRadius, distToNext * 0.7)
                
                let steps = 8
                for i in 0...steps {
                    let s = Float(i) / Float(steps)
                    let offset = (s - 0.5) * 2.0
                    let ramp = offset < 0 ? rampBefore : rampAfter
                    let posXY = crossingPosOnLine + lineDir * offset * ramp
                    let curve = 1.0 - offset * offset
                    let z = bumpHeight * curve
                    poly.append(SIMD3<Float>(posXY.x, posXY.y, z))
                }
                
                nodeIndex += 1
                continue
            }
            
            let positionXY = engine.position(of: node)
            let positionZ = baseZ(engine: engine, ropeIndex: ropeIndex, node: node, lift: lift, dragLift: dragLift)
            poly.append(SIMD3<Float>(positionXY.x, positionXY.y, positionZ))
            nodeIndex += 1
        }
        return poly
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
