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
        guard let topology else { return [] }
        if points.count < 2 { return [] }

        var events: [RopeMeshBuilder.TwistEvent] = []
        events.reserveCapacity(topology.crossings.count)

        for (_, crossing) in topology.crossings {
            if crossing.ropeOver == ropeIndex { continue }
            if crossing.ropeA != ropeIndex && crossing.ropeB != ropeIndex { continue }

            let base = Float(crossing.handedness)
            let sign: Float = (ropeIndex == crossing.ropeA) ? base : -base
            let angle = (Float.pi * 0.5) * sign
            let dist = closestDistanceAlong(points: points, toXY: crossing.position)
            events.append(RopeMeshBuilder.TwistEvent(dist: dist, angle: angle, window: 0.16))
        }

        return events
    }

    private static func closestDistanceAlong(points: UnsafeBufferPointer<SIMD3<Float>>, toXY target: SIMD2<Float>) -> Float {
        var bestDistAlong: Float = 0
        var bestD2: Float = .greatestFiniteMagnitude

        var s: Float = 0
        for i in 0..<(points.count - 1) {
            let a3 = points[i]
            let b3 = points[i + 1]
            let a = SIMD2<Float>(a3.x, a3.y)
            let b = SIMD2<Float>(b3.x, b3.y)
            let ab = b - a
            let ab2 = simd_length_squared(ab)
            let t: Float
            if ab2 < 1e-10 {
                t = 0
            } else {
                t = max(0, min(1, simd_dot(target - a, ab) / ab2))
            }
            let p = a + ab * t
            let d2 = simd_length_squared(p - target)
            if d2 < bestD2 {
                bestD2 = d2
                bestDistAlong = s + simd_length(b3 - a3) * t
            }
            s += simd_length(b3 - a3)
        }

        return bestDistAlong
    }
}

