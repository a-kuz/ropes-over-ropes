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
            let ropeRadius = ropes[ropeIndex].radius * 1.3
            let cs = ropes[ropeIndex].crossSection

            let band = sim.bands[ropeIndex]

            let frames: [MaterialFrame]?
            if cs.isRectangular && sim.cachedFrames.indices.contains(ropeIndex) && !sim.cachedFrames[ropeIndex].isEmpty {
                frames = sim.cachedFrames[ropeIndex]
            } else {
                frames = nil
            }

            var visiblePoints = points
            var visibleFrames = frames

            if fadeOut > 0, let suckHole = band.suckHole {
                let n = points.count
                let clipZ: Float = -ropeRadius * 1.5
                var lo = 0
                var hi = n - 1

                while lo < n && points[lo].z < clipZ { lo += 1 }
                while hi >= 0 && points[hi].z < clipZ { hi -= 1 }

                if hi - lo >= 1 {
                    var clipped = ContiguousArray(points[lo...hi])
                    var clippedFrames: [MaterialFrame]? = nil
                    if let f = frames, f.count == n {
                        clippedFrames = Array(f[lo...hi])
                    }

                    let holeXY = holePositions[suckHole]
                    let sinkZ: Float = -ropeRadius * 2.5

                    if lo > 0 {
                        let entryPt = SIMD3<Float>(holeXY.x, holeXY.y, sinkZ)
                        clipped.insert(entryPt, at: 0)
                        if var cf = clippedFrames, !cf.isEmpty {
                            cf.insert(cf[0], at: 0)
                            clippedFrames = cf
                        }
                    }
                    if hi < n - 1 {
                        let entryPt = SIMD3<Float>(holeXY.x, holeXY.y, sinkZ)
                        clipped.append(entryPt)
                        if var cf = clippedFrames, !cf.isEmpty {
                            cf.append(cf[cf.count - 1])
                            clippedFrames = cf
                        }
                    }

                    visiblePoints = clipped
                    visibleFrames = clippedFrames
                } else {
                    continue
                }
            }

            let restLength = band.segmentLength * Float(visiblePoints.count - 1)

            let ropeMesh = visiblePoints.withUnsafeBufferPointer { pointsBuffer in
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
                    restLength: restLength,
                    crossSection: cs,
                    materialFrames: visibleFrames
                )
            }

            allVertices.append(contentsOf: ropeMesh.vertices)
            allIndices.append(contentsOf: ropeMesh.indices.map { $0 + baseVertex })
            baseVertex += UInt32(ropeMesh.vertices.count)

            let sphereRadius = holeRadius * 0.72
            if visiblePoints.count >= 2 {
                let isSucking = fadeOut > 0 && band.suckHole != nil

                let drawStartSphere = !isSucking
                let drawEndSphere = !isSucking

                if drawStartSphere {
                    let startPos = visiblePoints[0]
                    let startTangent = simd_normalize(visiblePoints[1] - visiblePoints[0])
                    let startSphere = RopeMeshBuilder.buildHemisphere(
                        center: startPos, radius: sphereRadius, facing: -startTangent,
                        color: ropeColor, segments: 12, rings: 6
                    )
                    allVertices.append(contentsOf: startSphere.vertices)
                    allIndices.append(contentsOf: startSphere.indices.map { $0 + baseVertex })
                    baseVertex += UInt32(startSphere.vertices.count)
                }

                if drawEndSphere {
                    let endPos = visiblePoints[visiblePoints.count - 1]
                    let endTangent = simd_normalize(visiblePoints[visiblePoints.count - 1] - visiblePoints[visiblePoints.count - 2])
                    let endSphere = RopeMeshBuilder.buildHemisphere(
                        center: endPos, radius: sphereRadius, facing: endTangent,
                        color: ropeColor, segments: 12, rings: 6
                    )
                    allVertices.append(contentsOf: endSphere.vertices)
                    allIndices.append(contentsOf: endSphere.indices.map { $0 + baseVertex })
                    baseVertex += UInt32(endSphere.vertices.count)
                }
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
