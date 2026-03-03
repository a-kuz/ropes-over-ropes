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
            if ropes[ropeIndex].startHole < 0 && sim.bands[ropeIndex].fadeOut == 0 {
                allRopePoints.append([])
                continue
            }
            allRopePoints.append(sim.bands[ropeIndex].positions)
        }

        for ropeIndex in ropes.indices {
            guard let sim = simulator, sim.bands.indices.contains(ropeIndex), sim.bands[ropeIndex].active else { continue }
            if ropes[ropeIndex].startHole < 0 && sim.bands[ropeIndex].fadeOut == 0 { continue }

            let points = sim.bands[ropeIndex].positions
            let ropeColor = ropes[ropeIndex].color
            let fadeOut = sim.bands[ropeIndex].fadeOut
            let ropeRadius = ropes[ropeIndex].radius * 1.3
            let cs = ropes[ropeIndex].crossSection

            let band = sim.bands[ropeIndex]

            let frames: [MaterialFrame]?
            if (cs.isRectangular || squareCrossSection) && sim.cachedFrames.indices.contains(ropeIndex) && !sim.cachedFrames[ropeIndex].isEmpty {
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
            let scaledRadius = ropeRadius * ropeRadiusScale

            var ropeContactPoints: [SIMD2<Float>] = []
            for otherIdx in ropes.indices where otherIdx != ropeIndex && !allRopePoints[otherIdx].isEmpty {
                let otherPoints = allRopePoints[otherIdx]
                let rj = ropes[otherIdx].radius * 1.3 * ropeRadiusScale
                let threshold = (scaledRadius + rj) * 1.5
                for s in 0..<(otherPoints.count - 1) {
                    let mid = (otherPoints[s] + otherPoints[s + 1]) * 0.5
                    var minDist2: Float = .greatestFiniteMagnitude
                    for p in visiblePoints {
                        let d2 = simd_length_squared(SIMD2<Float>(p.x, p.y) - SIMD2<Float>(mid.x, mid.y))
                        minDist2 = min(minDist2, d2)
                    }
                    if minDist2 < threshold * threshold {
                        ropeContactPoints.append(SIMD2<Float>(mid.x, mid.y))
                    }
                }
            }

            let wmp = RopeMeshBuilder.WormMeshParams(
                segFreq: wormSegFreq, segBulge: wormSegBulge,
                thickness: wormThickness, taperLen: wormTaperLen
            )
            let ropeMesh = visiblePoints.withUnsafeBufferPointer { pointsBuffer in
                RopeMeshBuilder.buildRect(
                    points: pointsBuffer,
                    radius: scaledRadius,
                    color: ropeColor,
                    twistEvents: [],
                    tautness: 1.0,
                    repulsors: [],
                    stretchRatio: 1.0,
                    oscillation: 0.0,
                    segmentStarts: [],
                    restLength: restLength,
                    crossSection: cs,
                    materialFrames: visibleFrames,
                    profileSegments: profileSegments,
                    ropeContactPoints: ropeContactPoints,
                    ropeContactRadius: scaledRadius,
                    stretchThinning: stretchThinning,
                    wormMode: wormMode,
                    wormTime: time,
                    wormMeshParams: wmp,
                    squareCrossSection: squareCrossSection
                )
            }

            allVertices.append(contentsOf: ropeMesh.vertices)
            allIndices.append(contentsOf: ropeMesh.indices.map { $0 + baseVertex })
            baseVertex += UInt32(ropeMesh.vertices.count)

            if squareCrossSection {
                if visiblePoints.count >= 2 {
                    let isSucking = fadeOut > 0 && band.suckHole != nil
                    let bandHalf = scaledRadius
                    let hr = holeRadius * holeRadiusScale

                    func swivelFrame(at idx: Int) -> (d1: SIMD3<Float>, d2: SIMD3<Float>) {
                        if let fr = visibleFrames, fr.count > idx {
                            return (fr[idx].d1, fr[idx].d2)
                        }
                        return (SIMD3<Float>(1, 0, 0), SIMD3<Float>(0, 1, 0))
                    }

                    if !isSucking || band.suckFromEnd == 0 {
                        let pos = visiblePoints[0]
                        let tan = simd_normalize(visiblePoints[1] - visiblePoints[0])
                        let (d1, d2) = swivelFrame(at: 0)
                        let swivel = RopeMeshBuilder.buildSwivel(
                            center: pos, tangent: -tan,
                            holeRadius: hr, bandHalf: bandHalf,
                            d1: d1, d2: d2, color: ropeColor
                        )
                        allVertices.append(contentsOf: swivel.vertices)
                        allIndices.append(contentsOf: swivel.indices.map { $0 + baseVertex })
                        baseVertex += UInt32(swivel.vertices.count)
                    }

                    if !isSucking || band.suckFromEnd == 1 {
                        let lastIdx = visiblePoints.count - 1
                        let pos = visiblePoints[lastIdx]
                        let tan = simd_normalize(visiblePoints[lastIdx] - visiblePoints[lastIdx - 1])
                        let (d1, d2) = swivelFrame(at: lastIdx)
                        let swivel = RopeMeshBuilder.buildSwivel(
                            center: pos, tangent: tan,
                            holeRadius: hr, bandHalf: bandHalf,
                            d1: d1, d2: d2, color: ropeColor
                        )
                        allVertices.append(contentsOf: swivel.vertices)
                        allIndices.append(contentsOf: swivel.indices.map { $0 + baseVertex })
                        baseVertex += UInt32(swivel.vertices.count)
                    }
                }
            } else {
                let sphereRadius = holeRadius * holeRadiusScale * capRadiusScale
                if visiblePoints.count >= 2 {
                    let isSucking = fadeOut > 0 && band.suckHole != nil
                    let suckFromEnd = band.suckFromEnd
                    let drawStartSphere = !isSucking || suckFromEnd == 0
                    let drawEndSphere = !isSucking || suckFromEnd == 1

                    if drawStartSphere {
                        let startPos = visiblePoints[0]
                        let startTangent = simd_normalize(visiblePoints[1] - visiblePoints[0])
                        let startCap = RopeMeshBuilder.buildHemisphere(
                            center: startPos, radius: sphereRadius, facing: -startTangent,
                            color: ropeColor, segments: capSegments, rings: capRings, darken: capDarken, wormMode: wormMode
                        )
                        allVertices.append(contentsOf: startCap.vertices)
                        allIndices.append(contentsOf: startCap.indices.map { $0 + baseVertex })
                        baseVertex += UInt32(startCap.vertices.count)
                    }

                    if drawEndSphere {
                        let endPos = visiblePoints[visiblePoints.count - 1]
                        let endTangent = simd_normalize(visiblePoints[visiblePoints.count - 1] - visiblePoints[visiblePoints.count - 2])
                        let endCap = RopeMeshBuilder.buildHemisphere(
                            center: endPos, radius: sphereRadius, facing: endTangent,
                            color: ropeColor, segments: capSegments, rings: capRings, darken: capDarken, wormMode: wormMode
                        )
                        allVertices.append(contentsOf: endCap.vertices)
                        allIndices.append(contentsOf: endCap.indices.map { $0 + baseVertex })
                        baseVertex += UInt32(endCap.vertices.count)
                    }
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

        let currentMeshStats = MeshStats(vertices: allVertices.count, indices: allIndices.count, ropeCount: self.ropes.count)
        if currentMeshStats != lastMeshStats {
            lastMeshStats = currentMeshStats
            Self.meshLogger.info("Mesh stats: vertices=\(allVertices.count) indices=\(allIndices.count) ropeCount=\(self.ropes.count)")
        }
    }
}
