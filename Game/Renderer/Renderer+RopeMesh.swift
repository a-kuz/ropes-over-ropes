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
        var allRopePoints: [[SIMD3<Float>]] = []

        for ropeIndex in ropes.indices {
            if ropes[ropeIndex].startHole < 0 || ropes[ropeIndex].endHole < 0 {
                allRopePoints.append([])
                continue
            }

            guard let sim = simulator, sim.bands.indices.contains(ropeIndex), sim.bands[ropeIndex].active else {
                allRopePoints.append([])
                continue
            }

            let points = sim.bands[ropeIndex].positions
            allRopePoints.append(points)

            let ropeColor = ropes[ropeIndex].color
            let ropeRadius = ropes[ropeIndex].radius

            let ropeMesh = points.withUnsafeBufferPointer { pointsBuffer in
                RopeMeshBuilder.buildRect(
                    points: pointsBuffer,
                    radius: ropeRadius,
                    color: ropeColor,
                    twistEvents: [],
                    tautness: 1.0,
                    repulsors: [],
                    stretchRatio: 1.0,
                    oscillation: 0.0,
                    segmentStarts: [],
                    restLength: 0
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
}
