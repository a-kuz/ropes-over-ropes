import Metal
import simd

extension Renderer {
    func updateRopeMesh() {
        var allVertices: [RopeVertex] = []
        var allIndices: [UInt32] = []
        allVertices.reserveCapacity(ropes.count * 6400 * 8)
        allIndices.reserveCapacity(ropes.count * 6399 * 24)

        var baseVertex: UInt32 = 0

        let dragLift: Float = dragLiftCurrent

        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 { continue }
            let ropeColor = ropes[ropeIndex].color
            let ropeWidth = ropes[ropeIndex].width
            let ropeHeight = ropes[ropeIndex].height
            
            let ropeMesh: RopeMesh
            if let topology {
                let lift = max(ropeHeight * 1.35, 0.02)
                let points = TopologySampler.sampleRopeRender(
                    engine: topology,
                    ropeIndex: ropeIndex,
                    lift: lift,
                    dragLift: dragLift,
                    ropeWidth: ropeWidth,
                    ropeWidthForIndex: { idx in
                        return ropes[safe: idx]?.width ?? ropeWidth
                    }
                )
                ropeMesh = points.withUnsafeBufferPointer { pointsBuffer in
                    let events = Self.makeTwistEvents(topology: topology, ropeIndex: ropeIndex, points: pointsBuffer)
                    return RopeMeshBuilder.buildRect(points: pointsBuffer, width: ropeWidth, height: ropeHeight, color: ropeColor, twistEvents: events)
                }
            } else {
                ropeMesh = simulation.withRopePositions(ropeIndex: ropeIndex) { points in
                    let events = Self.makeTwistEvents(topology: topology, ropeIndex: ropeIndex, points: points)
                    return RopeMeshBuilder.buildRect(points: points, width: ropeWidth, height: ropeHeight, color: ropeColor, twistEvents: events)
                }
            }

            allVertices.append(contentsOf: ropeMesh.vertices)
            allIndices.append(contentsOf: ropeMesh.indices.map { $0 + baseVertex })
            baseVertex += UInt32(ropeMesh.vertices.count)
        }
        
        if let topology {
            let hookCenters = TopologySampler.hookCenters(engine: topology)
            let avgWidth: Float = ropes.isEmpty ? 0.08 : ropes.reduce(0) { $0 + $1.width } / Float(ropes.count)
            let cylinderRadius = avgWidth * 0.35
            let cylinderHeight: Float = 0.12
            let cylinderColor = SIMD3<Float>(0.95, 0.75, 0.2)
            
            for center in hookCenters {
                let (verts, inds) = Self.buildCylinder(
                    center: SIMD3<Float>(center.x, center.y, 0),
                    radius: cylinderRadius,
                    height: cylinderHeight,
                    color: cylinderColor,
                    segments: 16
                )
                allVertices.append(contentsOf: verts)
                allIndices.append(contentsOf: inds.map { $0 + baseVertex })
                baseVertex += UInt32(verts.count)
            }
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
    }
    
    private static func buildCylinder(center: SIMD3<Float>, radius: Float, height: Float, color: SIMD3<Float>, segments: Int) -> ([RopeVertex], [UInt32]) {
        var vertices: [RopeVertex] = []
        var indices: [UInt32] = []
        
        let bottomZ = center.z - height * 0.5
        let topZ = center.z + height * 0.5
        
        for i in 0...segments {
            let angle = Float(i) / Float(segments) * Float.pi * 2
            let x = center.x + cos(angle) * radius
            let y = center.y + sin(angle) * radius
            let normal = SIMD3<Float>(cos(angle), sin(angle), 0)
            
            vertices.append(RopeVertex(
                position: SIMD3<Float>(x, y, bottomZ),
                normal: normal,
                color: color,
                texCoord: SIMD2<Float>(Float(i) / Float(segments), 0)
            ))
            vertices.append(RopeVertex(
                position: SIMD3<Float>(x, y, topZ),
                normal: normal,
                color: color,
                texCoord: SIMD2<Float>(Float(i) / Float(segments), 1)
            ))
        }
        
        for i in 0..<segments {
            let b0 = UInt32(i * 2)
            let t0 = b0 + 1
            let b1 = UInt32((i + 1) * 2)
            let t1 = b1 + 1
            
            indices.append(contentsOf: [b0, b1, t1, b0, t1, t0])
        }
        
        let bottomCenter = UInt32(vertices.count)
        vertices.append(RopeVertex(
            position: SIMD3<Float>(center.x, center.y, bottomZ),
            normal: SIMD3<Float>(0, 0, -1),
            color: color,
            texCoord: SIMD2<Float>(0.5, 0.5)
        ))
        
        let topCenter = UInt32(vertices.count)
        vertices.append(RopeVertex(
            position: SIMD3<Float>(center.x, center.y, topZ),
            normal: SIMD3<Float>(0, 0, 1),
            color: color,
            texCoord: SIMD2<Float>(0.5, 0.5)
        ))
        
        for i in 0..<segments {
            let b0 = UInt32(i * 2)
            let b1 = UInt32((i + 1) * 2)
            indices.append(contentsOf: [bottomCenter, b1, b0])
            
            let t0 = b0 + 1
            let t1 = b1 + 1
            indices.append(contentsOf: [topCenter, t0, t1])
        }
        
        return (vertices, indices)
    }

    static func makeTwistEvents(topology: TopologyEngine?, ropeIndex: Int, points: UnsafeBufferPointer<SIMD3<Float>>) -> [RopeMeshBuilder.TwistEvent] {
        return []
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
