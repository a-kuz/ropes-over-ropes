import simd
import os.log

struct CrossingConstraint {
    let crossingId: Int
    let position: SIMD2<Float>
    let overRopeIndex: Int
    let underRopeIndex: Int
    let overRopeHeight: Float
    let underRopeHeight: Float
    let overRopeWidth: Float
    let underRopeWidth: Float
}

struct RopePhysicsState {
    var points: [SIMD3<Float>]
    var ropeIndex: Int
    var width: Float
    var height: Float
    var startAnchor: SIMD2<Float>
    var endAnchor: SIMD2<Float>
    var startZ: Float
    var endZ: Float
}

final class RopePhysics {
    private static let logger = Logger(subsystem: "com.uzls.four", category: "RopePhysics")
    
    static let tableZ: Float = 0.0
    static let contactGap: Float = 0.001
    
    private var lastLogTime: Double = 0
    private var logInterval: Double = 1.0
    
    struct CrossingZone {
        let crossingId: Int
        let centerXY: SIMD2<Float>
        let ropeIsOver: Bool
        let influenceRadius: Float
        let targetZ: Float
        let otherRopeWidth: Float
        let otherRopeHeight: Float
        let transitionWidth: Float
    }
    
    static func computeCrossingZones(
        ropeIndex: Int,
        topology: TopologyEngine,
        ropeHeight: Float,
        ropeWidth: Float,
        ropeHeightForIndex: (Int) -> Float,
        ropeWidthForIndex: (Int) -> Float,
        dragLift: Float
    ) -> [CrossingZone] {
        guard ropeIndex >= 0 && ropeIndex < topology.ropes.count else { return [] }
        let rope = topology.ropes[ropeIndex]
        guard rope.active else { return [] }
        
        var zones: [CrossingZone] = []
        
        let hookPairs = topology.findHookPairs()
        var hookCrossingIds = Set<Int>()
        for hook in hookPairs {
            hookCrossingIds.insert(hook.crossingIdA)
            hookCrossingIds.insert(hook.crossingIdB)
        }
        
        for node in rope.nodes {
            guard case .crossing(let crossingId) = node else { continue }
            if hookCrossingIds.contains(crossingId) { continue }
            guard let crossing = topology.crossings[crossingId] else { continue }
            
            let isOver = crossing.ropeOver == ropeIndex
            let otherIndex = (crossing.ropeA == ropeIndex) ? crossing.ropeB : crossing.ropeA
            let otherHeight = ropeHeightForIndex(otherIndex)
            let otherWidth = ropeWidthForIndex(otherIndex)
            
            let targetZ: Float
            if isOver {
                targetZ = otherHeight + contactGap
            } else {
                targetZ = tableZ
            }
            
            let transitionWidth = max(ropeWidth, otherWidth) * 0.5
            let influenceRadius = otherWidth * 0.5 + transitionWidth * 2.0
            
            zones.append(CrossingZone(
                crossingId: crossingId,
                centerXY: crossing.position,
                ropeIsOver: isOver,
                influenceRadius: influenceRadius,
                targetZ: targetZ,
                otherRopeWidth: otherWidth,
                otherRopeHeight: otherHeight,
                transitionWidth: transitionWidth
            ))
        }
        
        return zones
    }
    
    static func applyCrossingPhysics(
        points: inout [SIMD3<Float>],
        zones: [CrossingZone],
        ropeHeight: Float,
        ropeWidth: Float,
        startAnchor: SIMD2<Float>,
        endAnchor: SIMD2<Float>,
        startZ: Float,
        endZ: Float
    ) {
        guard points.count >= 2 else { return }
        guard !zones.isEmpty else { return }
        
        let count = points.count
        
        var lengths: [Float] = [0]
        for i in 1..<count {
            lengths.append(lengths[i-1] + simd_length(points[i] - points[i-1]))
        }
        let totalLength = lengths.last ?? 1.0
        
        for i in 0..<count {
            let p = points[i]
            let pos2D = SIMD2<Float>(p.x, p.y)
            let u = lengths[i] / max(1e-6, totalLength)
            
            let endMask = Self.smoothstep(edge0: 0.08, edge1: 0.20, value: u) *
                          Self.smoothstep(edge0: 0.08, edge1: 0.20, value: 1.0 as Float - u)
            
            var zAdjustment: Float = 0
            var maxWeight: Float = 0
            
            for zone in zones {
                let toCenter = pos2D - zone.centerXY
                let dist = simd_length(toCenter)
                
                if dist > zone.influenceRadius { continue }
                
                let halfOtherWidth = zone.otherRopeWidth * 0.5
                
                if zone.ropeIsOver {
                    let distFromCore = max(0, dist - halfOtherWidth)
                    let transitionT = 1.0 - min(1.0, distFromCore / zone.transitionWidth)
                    let smoothT = transitionT * transitionT * (3.0 - 2.0 * transitionT)
                    
                    let targetZ: Float
                    if dist < halfOtherWidth {
                        targetZ = zone.targetZ
                    } else {
                        targetZ = zone.targetZ * smoothT
                    }
                    
                    let weight = smoothT * endMask
                    if weight > maxWeight {
                        maxWeight = weight
                        zAdjustment = targetZ
                    }
                } else {
                    let coreWeight = Self.smoothstep(edge0: zone.influenceRadius, edge1: halfOtherWidth, value: dist)
                    let weight = coreWeight * endMask
                    if weight > 0.01 {
                        let pushDown = tableZ - p.z
                        zAdjustment = min(zAdjustment, pushDown * weight)
                    }
                }
            }
            
            var newZ = p.z + zAdjustment
            if startZ >= 0 && endZ >= 0 {
                newZ = max(tableZ, newZ)
            }
            points[i] = SIMD3<Float>(p.x, p.y, newZ)
        }
        
        points[0] = SIMD3<Float>(startAnchor.x, startAnchor.y, startZ)
        points[count-1] = SIMD3<Float>(endAnchor.x, endAnchor.y, endZ)
    }
    
    static func generateCrossingSegments(
        startAnchor: SIMD2<Float>,
        endAnchor: SIMD2<Float>,
        startZ: Float,
        endZ: Float,
        zones: [CrossingZone],
        baseSegmentCount: Int,
        ropeWidth: Float,
        ropeHeight: Float
    ) -> [SIMD3<Float>] {
        let dir = endAnchor - startAnchor
        let length = simd_length(dir)
        guard length > 1e-6 else {
            return [SIMD3<Float>(startAnchor.x, startAnchor.y, startZ)]
        }
        let dirNorm = dir / length
        
        var keyParams: [Float] = [0.0, 1.0]
        
        for zone in zones {
            let toZone = zone.centerXY - startAnchor
            let projT = simd_dot(toZone, dirNorm) / length
            
            if projT > 0.01 && projT < 0.99 {
                let halfWidth = zone.otherRopeWidth * 0.5
                let transW = zone.transitionWidth
                
                let offsetNear = (halfWidth + transW * 1.5) / length
                let offsetFar = halfWidth / length
                
                keyParams.append(max(0.01, projT - offsetNear))
                keyParams.append(max(0.01, projT - offsetFar))
                keyParams.append(projT)
                keyParams.append(min(0.99, projT + offsetFar))
                keyParams.append(min(0.99, projT + offsetNear))
            }
        }
        
        keyParams.sort()
        var uniqueParams: [Float] = []
        for p in keyParams {
            if uniqueParams.isEmpty || abs(p - uniqueParams.last!) > 0.001 {
                uniqueParams.append(p)
            }
        }
        
        var allParams: [Float] = []
        for i in 0..<(uniqueParams.count - 1) {
            let tStart = uniqueParams[i]
            let tEnd = uniqueParams[i+1]
            let span = tEnd - tStart
            
            let segCount = max(2, Int(ceil(span * Float(baseSegmentCount) * 2.0)))
            for j in 0..<segCount {
                let t = tStart + span * Float(j) / Float(segCount)
                allParams.append(t)
            }
        }
        allParams.append(1.0)
        
        var points: [SIMD3<Float>] = []
        points.reserveCapacity(allParams.count)
        
        for t in allParams {
            let xy = startAnchor + dir * t
            let baseZ = startZ + (endZ - startZ) * t
            
            var zOffset: Float = 0
            let endMask = Self.smoothstep(edge0: 0.08, edge1: 0.20, value: t) *
                          Self.smoothstep(edge0: 0.08, edge1: 0.20, value: 1.0 as Float - t)
            
            for zone in zones {
                let toZone = xy - zone.centerXY
                let dist = simd_length(toZone)
                
                if dist > zone.influenceRadius { continue }
                
                let halfWidth = zone.otherRopeWidth * 0.5
                
                if zone.ropeIsOver {
                    let distFromCore = max(0, dist - halfWidth)
                    let transitionT = 1.0 - min(1.0, distFromCore / zone.transitionWidth)
                    let smoothT = transitionT * transitionT * (3.0 - 2.0 * transitionT)
                    
                    let targetZ = zone.targetZ * smoothT * endMask
                    zOffset = max(zOffset, targetZ)
                } else {
                    zOffset = min(zOffset, 0)
                }
            }
            
            let finalZ = baseZ + zOffset
            points.append(SIMD3<Float>(xy.x, xy.y, finalZ))
        }
        
        return points
    }
    
    static func applyBandRepulsion(
        allRopePoints: inout [[SIMD3<Float>]],
        ropeWidths: [Float],
        ropeHeights: [Float],
        topology: TopologyEngine?,
        iterations: Int = 4
    ) {
        guard allRopePoints.count >= 2 else { return }
        
        var crossingInfo: [Int: (ropeA: Int, ropeB: Int, ropeOver: Int, position: SIMD2<Float>)] = [:]
        var hookCrossingIds = Set<Int>()
        
        if let topology = topology {
            let hookPairs = topology.findHookPairs()
            for hook in hookPairs {
                hookCrossingIds.insert(hook.crossingIdA)
                hookCrossingIds.insert(hook.crossingIdB)
            }
            
            for (cid, crossing) in topology.crossings {
                if !hookCrossingIds.contains(cid) {
                    crossingInfo[cid] = (crossing.ropeA, crossing.ropeB, crossing.ropeOver, crossing.position)
                }
            }
        }
        
        for _ in 0..<iterations {
            for ropeA in 0..<allRopePoints.count {
                guard allRopePoints[ropeA].count >= 2 else { continue }
                let widthA = ropeWidths[ropeA]
                let heightA = ropeHeights[ropeA]
                
                for ropeB in (ropeA+1)..<allRopePoints.count {
                    guard allRopePoints[ropeB].count >= 2 else { continue }
                    let widthB = ropeWidths[ropeB]
                    let heightB = ropeHeights[ropeB]
                    
                    var isOverRelation: Int? = nil
                    var crossingPos: SIMD2<Float>? = nil
                    
                    for (_, info) in crossingInfo {
                        if (info.ropeA == ropeA && info.ropeB == ropeB) || (info.ropeA == ropeB && info.ropeB == ropeA) {
                            isOverRelation = (info.ropeOver == ropeA) ? 1 : -1
                            crossingPos = info.position
                            break
                        }
                    }
                    
                    let minSep = (widthA + widthB) * 0.5
                    let minSepZ = max(heightA, heightB) + contactGap
                    
                    for i in 1..<(allRopePoints[ropeA].count - 1) {
                        let pA = allRopePoints[ropeA][i]
                        
                        for j in 1..<(allRopePoints[ropeB].count - 1) {
                            let pB = allRopePoints[ropeB][j]
                            
                            let dXY = SIMD2<Float>(pA.x - pB.x, pA.y - pB.y)
                            let distXY = simd_length(dXY)
                            
                            if distXY < minSep && distXY > 1e-6 {
                                let dZ = pA.z - pB.z
                                
                                if let isOver = isOverRelation {
                                    if isOver > 0 {
                                        if pA.z < pB.z + minSepZ {
                                            allRopePoints[ropeA][i].z = pB.z + minSepZ
                                        }
                                    } else {
                                        if pB.z < pA.z + minSepZ {
                                            allRopePoints[ropeB][j].z = pA.z + minSepZ
                                        }
                                    }
                                } else if abs(dZ) < minSepZ {
                                    let pushXY = (dXY / distXY) * (minSep - distXY) * 0.2
                                    
                                    allRopePoints[ropeA][i].x += pushXY.x
                                    allRopePoints[ropeA][i].y += pushXY.y
                                    allRopePoints[ropeB][j].x -= pushXY.x
                                    allRopePoints[ropeB][j].y -= pushXY.y
                                }
                            }
                        }
                    }
                }
            }
            
        }
    }
    
    static func applyTensionConstraints(
        points: inout [SIMD3<Float>],
        startAnchor: SIMD3<Float>,
        endAnchor: SIMD3<Float>,
        iterations: Int = 8,
        stiffness: Float = 0.85
    ) {
        guard points.count >= 2 else { return }
        
        let totalDist = simd_length(endAnchor - startAnchor)
        let restSegmentLength = totalDist / Float(points.count - 1)
        
        points[0] = startAnchor
        points[points.count - 1] = endAnchor
        
        for _ in 0..<iterations {
            for i in 0..<(points.count - 1) {
                let p0 = points[i]
                let p1 = points[i + 1]
                let delta = p1 - p0
                let dist = simd_length(delta)
                
                if dist > 1e-6 {
                    let diff = (dist - restSegmentLength) / dist
                    let correction = delta * (diff * 0.5 * stiffness)
                    
                    let isPin0 = (i == 0)
                    let isPin1 = (i + 1 == points.count - 1)
                    
                    if isPin0 && !isPin1 {
                        points[i + 1] = p1 - correction * 2.0
                    } else if !isPin0 && isPin1 {
                        points[i] = p0 + correction * 2.0
                    } else if !isPin0 && !isPin1 {
                        points[i] = p0 + correction
                        points[i + 1] = p1 - correction
                    }
                }
            }
            
            points[0] = startAnchor
            points[points.count - 1] = endAnchor
        }
    }
    
    static func applyHookTension(
        ropeAPoints: inout [SIMD3<Float>],
        ropeBPoints: inout [SIMD3<Float>],
        hookCenter: SIMD2<Float>,
        hookRadius: Float,
        ropeAWidth: Float,
        ropeBWidth: Float,
        ropeAHeight: Float,
        ropeBHeight: Float,
        ropeAStartAnchor: SIMD2<Float>,
        ropeAEndAnchor: SIMD2<Float>,
        ropeBStartAnchor: SIMD2<Float>,
        ropeBEndAnchor: SIMD2<Float>,
        iterations: Int = 12,
        tensionStiffness: Float = 0.9,
        contactStiffness: Float = 0.8,
        smoothingAlpha: Float = 0.3,
        elasticModulus: Float = 0.7
    ) {
        guard ropeAPoints.count >= 2 && ropeBPoints.count >= 2 else { return }
        
        let halfWidthA = ropeAWidth * 0.5
        let halfWidthB = ropeBWidth * 0.5
        let contactDistance = halfWidthA + halfWidthB
        let hookInfluenceRadius = hookRadius * 1.5
        
        let restLengthA = simd_length(ropeAEndAnchor - ropeAStartAnchor)
        let restLengthB = simd_length(ropeBEndAnchor - ropeBStartAnchor)
        
        for iteration in 0..<iterations {
            let progress = Float(iteration) / Float(iterations)
            let currentTension = tensionStiffness * (0.5 + 0.5 * progress)
            let currentContact = contactStiffness * (0.5 + 0.5 * progress)
            
            processRopeInHook(
                points: &ropeAPoints,
                otherPoints: ropeBPoints,
                hookCenter: hookCenter,
                hookRadius: hookRadius,
                hookInfluenceRadius: hookInfluenceRadius,
                width: ropeAWidth,
                halfWidth: halfWidthA,
                otherWidth: ropeBWidth,
                otherHalfWidth: halfWidthB,
                height: ropeAHeight,
                otherHeight: ropeBHeight,
                startAnchor: ropeAStartAnchor,
                endAnchor: ropeAEndAnchor,
                restLength: restLengthA,
                contactDistance: contactDistance,
                currentTension: currentTension,
                currentContact: currentContact,
                smoothingAlpha: smoothingAlpha,
                elasticModulus: elasticModulus
            )
            
            processRopeInHook(
                points: &ropeBPoints,
                otherPoints: ropeAPoints,
                hookCenter: hookCenter,
                hookRadius: hookRadius,
                hookInfluenceRadius: hookInfluenceRadius,
                width: ropeBWidth,
                halfWidth: halfWidthB,
                otherWidth: ropeAWidth,
                otherHalfWidth: halfWidthA,
                height: ropeBHeight,
                otherHeight: ropeAHeight,
                startAnchor: ropeBStartAnchor,
                endAnchor: ropeBEndAnchor,
                restLength: restLengthB,
                contactDistance: contactDistance,
                currentTension: currentTension,
                currentContact: currentContact,
                smoothingAlpha: smoothingAlpha,
                elasticModulus: elasticModulus
            )
            
            ropeAPoints[0] = SIMD3<Float>(ropeAStartAnchor.x, ropeAStartAnchor.y, tableZ)
            ropeAPoints[ropeAPoints.count - 1] = SIMD3<Float>(ropeAEndAnchor.x, ropeAEndAnchor.y, tableZ)
            ropeBPoints[0] = SIMD3<Float>(ropeBStartAnchor.x, ropeBStartAnchor.y, tableZ)
            ropeBPoints[ropeBPoints.count - 1] = SIMD3<Float>(ropeBEndAnchor.x, ropeBEndAnchor.y, tableZ)
        }
    }
    
    private static func processRopeInHook(
        points: inout [SIMD3<Float>],
        otherPoints: [SIMD3<Float>],
        hookCenter: SIMD2<Float>,
        hookRadius: Float,
        hookInfluenceRadius: Float,
        width: Float,
        halfWidth: Float,
        otherWidth: Float,
        otherHalfWidth: Float,
        height: Float,
        otherHeight: Float,
        startAnchor: SIMD2<Float>,
        endAnchor: SIMD2<Float>,
        restLength: Float,
        contactDistance: Float,
        currentTension: Float,
        currentContact: Float,
        smoothingAlpha: Float,
        elasticModulus: Float
    ) {
        let anchorDiff = endAnchor - startAnchor
        let anchorLength = simd_length(anchorDiff)
        let targetSegmentLength = anchorLength / Float(points.count - 1)
        let restSegmentLength = restLength / Float(points.count - 1)
        
        for i in 1..<(points.count - 1) {
            let p = points[i]
            let p2D = SIMD2<Float>(p.x, p.y)
            let distToHook = simd_length(p2D - hookCenter)
            
            if distToHook > hookInfluenceRadius { continue }
            
            let hookInfluence = 1.0 - min(1.0, distToHook / hookInfluenceRadius)
            let influence = hookInfluence * hookInfluence
            
            if influence < 0.01 { continue }
            
            var tensionForce = SIMD2<Float>(0, 0)
            var contactForce = SIMD2<Float>(0, 0)
            var smoothingForce = SIMD2<Float>(0, 0)
            
            let prev = points[i - 1]
            let next = points[i + 1]
            let prev2D = SIMD2<Float>(prev.x, prev.y)
            let next2D = SIMD2<Float>(next.x, next.y)
            
            let toPrev = prev2D - p2D
            let toNext = next2D - p2D
            let prevLen = simd_length(toPrev)
            let nextLen = simd_length(toNext)
            
            if prevLen > 1e-6 && nextLen > 1e-6 {
                let avgLength = (prevLen + nextLen) * 0.5
                let stretch = avgLength - restSegmentLength
                
                if stretch > 0 {
                    let tensionDir = normalize2(toNext + toPrev)
                    let elasticForce = stretch * elasticModulus
                    tensionForce = tensionDir * elasticForce * currentTension
                } else if avgLength > targetSegmentLength * 1.05 {
                    let tensionDir = normalize2(toNext + toPrev)
                    let lengthDiff = avgLength - targetSegmentLength
                    tensionForce = tensionDir * lengthDiff * currentTension * 0.5
                }
            }
            
            var closestContact: (point: SIMD2<Float>, segmentIndex: Int, distance: Float)?
            var minContactDist: Float = .greatestFiniteMagnitude
            
            for j in 0..<(otherPoints.count - 1) {
                let otherP0 = otherPoints[j]
                let otherP1 = otherPoints[j + 1]
                let other2D0 = SIMD2<Float>(otherP0.x, otherP0.y)
                let other2D1 = SIMD2<Float>(otherP1.x, otherP1.y)
                
                let segmentDir = other2D1 - other2D0
                let segmentLen = simd_length(segmentDir)
                
                if segmentLen < 1e-6 { continue }
                
                let segmentNorm = segmentDir / segmentLen
                let segmentPerp = SIMD2<Float>(-segmentNorm.y, segmentNorm.x)
                
                let toSegment = p2D - other2D0
                let projT = simd_dot(toSegment, segmentNorm) / segmentLen
                let clampedT = max(0.0, min(1.0, projT))
                let closestOnSegment = other2D0 + segmentDir * clampedT
                
                let toClosest = p2D - closestOnSegment
                let distToSegment = simd_length(toClosest)
                
                let effectiveWidth = halfWidth + otherHalfWidth
                
                if distToSegment < effectiveWidth * 1.2 && distToSegment < minContactDist {
                    minContactDist = distToSegment
                    closestContact = (closestOnSegment, j, distToSegment)
                }
            }
            
            if let contact = closestContact {
                let toOther = p2D - contact.point
                let dist = contact.distance
                
                if dist < 1e-6 {
                    let randomX = Float.random(in: -1...1)
                    let randomY = Float.random(in: -1...1)
                    let randomDir = normalize2(SIMD2<Float>(randomX, randomY))
                    let pushAmount = contactDistance * 0.1
                    contactForce = randomDir * pushAmount
                } else {
                    let dir = toOther / dist
                    let effectiveWidth = halfWidth + otherHalfWidth
                    
                    if dist < effectiveWidth {
                        let overlap = effectiveWidth - dist
                        let push = overlap * currentContact
                        contactForce = dir * push
                    } else if dist > effectiveWidth * 1.1 {
                        let gap = dist - effectiveWidth
                        let pull = gap * currentContact * 0.2
                        contactForce = -dir * pull
                    }
                }
            }
            
            if i > 1 && i < points.count - 2 {
                let prev2D = SIMD2<Float>(points[i - 1].x, points[i - 1].y)
                let next2D = SIMD2<Float>(points[i + 1].x, points[i + 1].y)
                let smooth = (prev2D + next2D) * 0.5
                let toSmooth = smooth - p2D
                smoothingForce = toSmooth * smoothingAlpha
            }
            
            let forceSum = tensionForce + contactForce + smoothingForce
            let totalForce = forceSum * influence
            let newP2D = p2D + totalForce
            
            let newZ: Float
            if distToHook < hookRadius {
                let zWeight = 1.0 - (distToHook / hookRadius)
                let overZ = otherHeight + contactGap
                let zPart1 = p.z * (1.0 - zWeight)
                let zPart2 = overZ * zWeight
                newZ = max(tableZ, zPart1 + zPart2)
            } else {
                newZ = max(tableZ, p.z)
            }
            
            points[i] = SIMD3<Float>(newP2D.x, newP2D.y, newZ)
        }
    }
    
    private static func normalize2(_ v: SIMD2<Float>) -> SIMD2<Float> {
        let l2 = simd_length_squared(v)
        if l2 < 1e-12 { return .zero }
        return v / sqrt(l2)
    }
    
    static func computeDragCrossingPhysics(
        lowerRopePoints: inout [SIMD3<Float>],
        upperRopePoints: inout [SIMD3<Float>],
        crossingPosition: inout SIMD2<Float>,
        lowerRopeStartAnchor: SIMD2<Float>,
        lowerRopeEndAnchor: SIMD2<Float>,
        upperRopeStartAnchor: SIMD2<Float>,
        upperRopeEndAnchor: SIMD2<Float>,
        dragPosition: SIMD2<Float>,
        dragZ: Float,
        dragEndIndex: Int,
        lowerRopeWidth: Float,
        upperRopeHeight: Float
    ) {
        guard lowerRopePoints.count >= 2 else { return }
        guard upperRopePoints.count >= 2 else { return }
        guard dragZ > 0.01 else { return }
        
        let fixedAnchor = (dragEndIndex == 0) ? lowerRopeEndAnchor : lowerRopeStartAnchor
        let lowerRopeStart = lowerRopeStartAnchor
        let lowerRopeEnd = fixedAnchor
        
        if let intersection = SegmentIntersection.intersect(
            a0: lowerRopeStart,
            a1: lowerRopeEnd,
            b0: upperRopeStartAnchor,
            b1: upperRopeEndAnchor
        ) {
            crossingPosition = intersection.p
            
            let tOnLower = intersection.t
            let tClamped = max(0.05, min(0.95, tOnLower))
            
            let z_at_crossing_on_lower = dragZ * tClamped
            
            let lowerPullStrength: Float = 0.4
            let z_crossing_lower = z_at_crossing_on_lower * lowerPullStrength
            let z_crossing_upper = z_crossing_lower + upperRopeHeight
            
            let flatRadiusLower = lowerRopeWidth * 0.5
            let minBendRadiusLower = upperRopeHeight * 2.5
            let transitionRadiusLower = max(minBendRadiusLower, sqrt(z_crossing_lower * lowerRopeWidth * 4.0))
            
            let flatRadiusUpper = lowerRopeWidth * 0.6
            let minBendRadiusUpper = upperRopeHeight * 2.5
            let heightDrop = max(0.001, upperRopeHeight - z_crossing_lower)
            let transitionRadiusUpper = max(minBendRadiusUpper, sqrt(heightDrop * lowerRopeWidth * 4.0))
            
            let cX = crossingPosition.x
            let cY = crossingPosition.y
            logger.info("🔧 Drag crossing: XY=(\(String(format: "%.3f", cX)), \(String(format: "%.3f", cY))) z_low=\(String(format: "%.3f", z_crossing_lower)) z_up=\(String(format: "%.3f", z_crossing_upper)) dragZ=\(String(format: "%.3f", dragZ)) flatR_low=\(String(format: "%.3f", flatRadiusLower)) transR_low=\(String(format: "%.3f", transitionRadiusLower))")
            
            updateLowerRopeWithCrossing(
                points: &lowerRopePoints,
                crossingXY: crossingPosition,
                crossingZ: z_crossing_lower,
                startAnchor: lowerRopeStartAnchor,
                fixedAnchor: fixedAnchor,
                dragPosition: dragPosition,
                dragZ: dragZ,
                flatRadius: flatRadiusLower,
                transitionRadius: transitionRadiusLower
            )
            
            updateUpperRopeWithCrossing(
                points: &upperRopePoints,
                crossingXY: crossingPosition,
                crossingZ: z_crossing_upper,
                upperHeight: upperRopeHeight,
                startAnchor: upperRopeStartAnchor,
                endAnchor: upperRopeEndAnchor,
                flatRadius: flatRadiusUpper,
                transitionRadius: transitionRadiusUpper
            )
        }
    }
    
    private static func updateLowerRopeWithCrossing(
        points: inout [SIMD3<Float>],
        crossingXY: SIMD2<Float>,
        crossingZ: Float,
        startAnchor: SIMD2<Float>,
        fixedAnchor: SIMD2<Float>,
        dragPosition: SIMD2<Float>,
        dragZ: Float,
        flatRadius: Float,
        transitionRadius: Float
    ) {
        guard points.count >= 2 else { return }
        
        let totalLength = Float(points.count - 1)
        
        let lineStart = startAnchor
        let lineEnd = fixedAnchor
        let lineDir = lineEnd - lineStart
        let lineLen = simd_length(lineDir)
        
        if lineLen < 1e-6 {
            for i in 0..<points.count {
                points[i] = SIMD3<Float>(lineStart.x, lineStart.y, 0)
            }
            return
        }
        
        let lineDirNorm = lineDir / lineLen
        let toCrossing = crossingXY - lineStart
        let crossingTOnLine = simd_dot(toCrossing, lineDirNorm) / lineLen
        
        for i in 0..<points.count {
            let t = Float(i) / totalLength
            
            let xy = lineStart + lineDir * t
            
            let distFromCrossing = abs(t - crossingTOnLine) * lineLen
            let crossingInfluence = flatRadius * 1.5
            
            let z: Float
            if distFromCrossing < crossingInfluence {
                let weight = 1.0 - (distFromCrossing / crossingInfluence)
                let smoothWeight = weight * weight * (3.0 - 2.0 * weight)
                z = crossingZ * smoothWeight
            } else {
                z = 0
            }
            
            points[i] = SIMD3<Float>(xy.x, xy.y, z)
        }
    }
    
    private static func smoothRise(t: Float) -> Float {
        let clamped = max(0, min(1, t))
        return clamped * clamped * clamped * (clamped * (clamped * 6.0 - 15.0) + 10.0)
    }
    
    private static func smoothBend(t: Float) -> Float {
        let clamped = max(0, min(1, t))
        return clamped * clamped * clamped * (clamped * (clamped * 6.0 - 15.0) + 10.0)
    }
    
    private static func updateUpperRopeWithCrossing(
        points: inout [SIMD3<Float>],
        crossingXY: SIMD2<Float>,
        crossingZ: Float,
        upperHeight: Float,
        startAnchor: SIMD2<Float>,
        endAnchor: SIMD2<Float>,
        flatRadius: Float,
        transitionRadius: Float
    ) {
        guard points.count >= 2 else { return }
        
        var closestIndex = 0
        var closestDist: Float = .greatestFiniteMagnitude
        
        for i in 0..<points.count {
            let dist = simd_length(SIMD2<Float>(points[i].x, points[i].y) - crossingXY)
            if dist < closestDist {
                closestDist = dist
                closestIndex = i
            }
        }
        
        let totalLength = Float(points.count - 1)
        let crossingT = Float(closestIndex) / totalLength
        
        let flatStartT = max(0.0, crossingT - flatRadius)
        let flatEndT = min(1.0, crossingT + flatRadius)
        let transitionStartT = max(0.0, crossingT - transitionRadius)
        let transitionEndT = min(1.0, crossingT + transitionRadius)
        
        for i in 0..<points.count {
            let t = Float(i) / totalLength
            
            let originalXY = startAnchor + (endAnchor - startAnchor) * t
            let newXY: SIMD2<Float>
            let targetZ: Float
            
            if t < transitionStartT {
                newXY = originalXY
                targetZ = upperHeight
            } else if t < flatStartT {
                let localT = (t - transitionStartT) / max(0.01, flatStartT - transitionStartT)
                let smoothWeight = smoothBend(t: localT)
                
                newXY = originalXY + (crossingXY - originalXY) * smoothWeight
                targetZ = crossingZ * smoothWeight + upperHeight * (1.0 - smoothWeight)
            } else if t <= flatEndT {
                newXY = crossingXY
                targetZ = crossingZ
            } else if t < transitionEndT {
                let localT = (t - flatEndT) / max(0.01, transitionEndT - flatEndT)
                let smoothWeight = smoothBend(t: localT)
                
                newXY = originalXY + (crossingXY - originalXY) * (1.0 - smoothWeight)
                targetZ = crossingZ * (1.0 - smoothWeight) + upperHeight * smoothWeight
            } else {
                newXY = originalXY
                targetZ = upperHeight
            }
            
            points[i].x = newXY.x
            points[i].y = newXY.y
            points[i].z = targetZ
        }
    }
    
    static func computeDragLiftInfluence(
        points: inout [SIMD3<Float>],
        dragPosition: SIMD2<Float>,
        dragZ: Float,
        dragEndIndex: Int,
        crossingZones: [CrossingZone]
    ) {
        guard points.count >= 2 else { return }
        guard dragZ > 0.01 else { return }
        
        for zone in crossingZones {
            if !zone.ropeIsOver { continue }
            
            let liftInfluence = dragZ * 0.5
            let liftRadius = zone.influenceRadius * 1.5
            
            for i in 0..<points.count {
                let p = points[i]
                let dist = simd_length(SIMD2<Float>(p.x, p.y) - zone.centerXY)
                
                if dist < liftRadius {
                    let weight = 1.0 - (dist / liftRadius)
                    let smoothWeight = weight * weight * (3.0 - 2.0 * weight)
                    let additionalLift = liftInfluence * smoothWeight
                    points[i].z = max(points[i].z, zone.targetZ + additionalLift)
                }
            }
        }
    }
    
    private static func smoothstep(edge0: Float, edge1: Float, value: Float) -> Float {
        let t = max(0, min(1, (value - edge0) / (edge1 - edge0)))
        return t * t * (3.0 - 2.0 * t)
    }
    
    func logStateIfNeeded(
        time: Double,
        ropes: [(index: Int, points: [SIMD3<Float>])],
        crossings: [Int: TopologyCrossing]
    ) {
        guard time - self.lastLogTime >= self.logInterval else { return }
        self.lastLogTime = time
        
        Self.logger.info("═══════════════════════════════════════════")
        Self.logger.info("ROPE PHYSICS STATE @ t=\(String(format: "%.2f", time))s")
        
        for rope in ropes {
            let first = rope.points.first ?? .zero
            let last = rope.points.last ?? .zero
            let mid = rope.points.count > 2 ? rope.points[rope.points.count / 2] : first
            
            Self.logger.info("  Rope[\(rope.index)]: \(rope.points.count) pts")
            Self.logger.info("    start: (\(String(format: "%.3f", first.x)), \(String(format: "%.3f", first.y)), \(String(format: "%.3f", first.z)))")
            Self.logger.info("    mid:   (\(String(format: "%.3f", mid.x)), \(String(format: "%.3f", mid.y)), \(String(format: "%.3f", mid.z)))")
            Self.logger.info("    end:   (\(String(format: "%.3f", last.x)), \(String(format: "%.3f", last.y)), \(String(format: "%.3f", last.z)))")
        }
        
        Self.logger.info("  Crossings: \(crossings.count)")
        for (id, c) in crossings.sorted(by: { $0.key < $1.key }) {
            Self.logger.info("    [\(id)]: pos=(\(String(format: "%.3f", c.position.x)), \(String(format: "%.3f", c.position.y))) rA=\(c.ropeA) rB=\(c.ropeB) over=\(c.ropeOver)")
        }
        Self.logger.info("═══════════════════════════════════════════")
    }
}
