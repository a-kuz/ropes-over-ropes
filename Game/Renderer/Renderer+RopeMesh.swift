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
        var allRopePoints: [ContiguousArray<SIMD3<Float>>] = []

        for ropeIndex in ropes.indices {
            guard let sim = simulator, sim.bands.indices.contains(ropeIndex), sim.bands[ropeIndex].active else {
                allRopePoints.append([])
                continue
            }

            // Skip non-fading ropes that have been removed (startHole == -1)
            if ropes[ropeIndex].startHole < 0 && sim.bands[ropeIndex].fadeOut == 0 {
                allRopePoints.append([])
                continue
            }

            let points = sim.bands[ropeIndex].positions
            allRopePoints.append(points)

            let ropeColor = ropes[ropeIndex].color
            let fadeOut = sim.bands[ropeIndex].fadeOut
            let ropeRadius = ropes[ropeIndex].radius * (1 - fadeOut)

            let band = sim.bands[ropeIndex]
            let restLength = band.segmentLength * Float(points.count - 1)

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
                    restLength: restLength
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

        if let sim = simulator {
            var loggableRopes: [(index: Int, points: ContiguousArray<SIMD3<Float>>)] = []
            for ropeIndex in ropes.indices {
                if allRopePoints[ropeIndex].isEmpty { continue }
                if !sim.bands[ropeIndex].active { continue }
                loggableRopes.append((index: ropeIndex, points: allRopePoints[ropeIndex]))
            }
            ropePhysicsLogger.logStateIfNeeded(
                time: Double(time),
                ropes: loggableRopes
            )
        }

        let currentMeshStats = MeshStats(vertices: allVertices.count, indices: allIndices.count, ropeCount: self.ropes.count)
        if currentMeshStats != lastMeshStats {
            lastMeshStats = currentMeshStats
            Self.meshLogger.info("Mesh stats: vertices=\(allVertices.count) indices=\(allIndices.count) ropeCount=\(self.ropes.count)")
        }
    }
}
