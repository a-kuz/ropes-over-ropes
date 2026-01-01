import Metal
import simd

extension Renderer {
    func updateRopeMesh() {
        var allVertices: [RopeVertex] = []
        var allIndices: [UInt32] = []
        allVertices.reserveCapacity(ropes.count * 64 * 8)
        allIndices.reserveCapacity(ropes.count * 63 * 24)

        var baseVertex: UInt32 = 0

        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 { continue }
            let ropeColor = ropes[ropeIndex].color
            let ropeWidth = ropes[ropeIndex].width
            let ropeHeight = ropes[ropeIndex].height
            let ropeMesh = simulation.withRopePositions(ropeIndex: ropeIndex) { points in
                let events = Self.makeTwistEvents(topology: topology, ropeIndex: ropeIndex, points: points)
                return RopeMeshBuilder.buildRect(points: points, width: ropeWidth, height: ropeHeight, color: ropeColor, twistEvents: events)
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
