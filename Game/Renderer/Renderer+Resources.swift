import MetalKit
import simd

extension Renderer {
    private func updateWoodBakeBounds() -> Bool {
        guard !holePositions.isEmpty else { return false }
        var minP = SIMD2<Float>(Float.greatestFiniteMagnitude, Float.greatestFiniteMagnitude)
        var maxP = SIMD2<Float>(-Float.greatestFiniteMagnitude, -Float.greatestFiniteMagnitude)
        for h in holePositions {
            minP = min(minP, h)
            maxP = max(maxP, h)
        }
        let boardSize = max(maxP.x - minP.x, maxP.y - minP.y)
        let padding = max(boardSize * 0.6, 4.0)
        let center = (minP + maxP) * 0.5
        let half = (maxP - minP) * 0.5 + SIMD2<Float>(padding, padding)
        woodBoundsMin = center - half
        woodBoundsMax = center + half
        return true
    }

    func rebuildHoleInstances() {
        guard !holePositions.isEmpty else { return }
        let visualRadius = holeRadius * holeRadiusScale
        let tintColors = computeHoleTintColors()
        holeInstances = Self.makeHoleInstances(device: device, positions: holePositions, elevations: holeElevations, radius: visualRadius, tintColors: tintColors)
    }

    func updateHoleTintColors() {
        guard let buf = holeInstances, !holePositions.isEmpty else { return }
        let count = holePositions.count
        let stride = MemoryLayout<HoleInstance>.stride
        guard buf.length >= count * stride else { return }
        let ptr = buf.contents().bindMemory(to: HoleInstance.self, capacity: count)
        guard let sim = simulator else {
            for i in 0..<count { ptr[i].tintColor = .zero }
            return
        }
        for i in 0..<count { ptr[i].tintColor = .zero }
        for i in ropes.indices {
            guard i < sim.bands.count, sim.bands[i].active, sim.bands[i].fadeOut == 0 else { continue }
            let c = ropes[i].color
            let tint = SIMD4<Float>(c.x, c.y, c.z, 1)
            if let pinS = sim.bands[i].pinStart, pinS >= 0, pinS < count { ptr[pinS].tintColor = tint }
            if let pinE = sim.bands[i].pinEnd, pinE >= 0, pinE < count { ptr[pinE].tintColor = tint }
        }
    }

    private func computeHoleTintColors() -> [SIMD4<Float>] {
        var colors = [SIMD4<Float>](repeating: .zero, count: holePositions.count)
        guard let sim = simulator else { return colors }
        for i in ropes.indices {
            guard i < sim.bands.count, sim.bands[i].active, sim.bands[i].fadeOut == 0 else { continue }
            let ropeColor = ropes[i].color
            let tint = SIMD4<Float>(ropeColor.x, ropeColor.y, ropeColor.z, 1)
            if let pinS = sim.bands[i].pinStart, pinS >= 0, pinS < holePositions.count {
                colors[pinS] = tint
            }
            if let pinE = sim.bands[i].pinEnd, pinE >= 0, pinE < holePositions.count {
                colors[pinE] = tint
            }
        }
        return colors
    }

    func rebuildHoleInstancesIfNeeded() {
        rebuildHoleInstances()
    }

    func rebuildHoleMeshIfNeeded() {
        Self.buildHoleMeshBuffers(device: device, segments: holeSegments, square: squareCrossSection, padMode: shaderParams.padMode, padHeight: shaderParams.padHeight, vertexBuffer: &holeVB, indexBuffer: &holeIB, indexCount: &holeIndexCount)
    }

    func rebuildBoardMesh() {
        guard !boards.isEmpty else {
            boardMeshVB = nil
            boardMeshIB = nil
            boardMeshIndexCount = 0
            return
        }

        var vertices: [BoardVertex] = []
        var indices: [UInt32] = []

        for board in boards {
            let hw = board.width * 0.5
            let hh = board.height * 0.5
            let z = board.elevation
            let cx = board.centerX
            let cy = board.centerY
            let base: UInt32 = UInt32(vertices.count)

            let topN = SIMD3<Float>(0, 0, 1)
            vertices.append(BoardVertex(position: SIMD3(cx - hw, cy - hh, z), normal: topN, worldXY: SIMD2(cx - hw, cy - hh)))
            vertices.append(BoardVertex(position: SIMD3(cx + hw, cy - hh, z), normal: topN, worldXY: SIMD2(cx + hw, cy - hh)))
            vertices.append(BoardVertex(position: SIMD3(cx + hw, cy + hh, z), normal: topN, worldXY: SIMD2(cx + hw, cy + hh)))
            vertices.append(BoardVertex(position: SIMD3(cx - hw, cy + hh, z), normal: topN, worldXY: SIMD2(cx - hw, cy + hh)))
            indices.append(contentsOf: [base, base+1, base+2, base, base+2, base+3])

            let sides: [(SIMD3<Float>, SIMD3<Float>, SIMD3<Float>, SIMD3<Float>, SIMD3<Float>)] = [
                (SIMD3(cx - hw, cy - hh, z), SIMD3(cx + hw, cy - hh, z), SIMD3(cx + hw, cy - hh, 0), SIMD3(cx - hw, cy - hh, 0), SIMD3(0, -1, 0)),
                (SIMD3(cx + hw, cy - hh, z), SIMD3(cx + hw, cy + hh, z), SIMD3(cx + hw, cy + hh, 0), SIMD3(cx + hw, cy - hh, 0), SIMD3(1, 0, 0)),
                (SIMD3(cx + hw, cy + hh, z), SIMD3(cx - hw, cy + hh, z), SIMD3(cx - hw, cy + hh, 0), SIMD3(cx + hw, cy + hh, 0), SIMD3(0, 1, 0)),
                (SIMD3(cx - hw, cy + hh, z), SIMD3(cx - hw, cy - hh, z), SIMD3(cx - hw, cy - hh, 0), SIMD3(cx - hw, cy + hh, 0), SIMD3(-1, 0, 0)),
            ]
            for (p0, p1, p2, p3, n) in sides {
                let sb = UInt32(vertices.count)
                vertices.append(BoardVertex(position: p0, normal: n, worldXY: SIMD2(p0.x, p0.y)))
                vertices.append(BoardVertex(position: p1, normal: n, worldXY: SIMD2(p1.x, p1.y)))
                vertices.append(BoardVertex(position: p2, normal: n, worldXY: SIMD2(p2.x, p2.y)))
                vertices.append(BoardVertex(position: p3, normal: n, worldXY: SIMD2(p3.x, p3.y)))
                indices.append(contentsOf: [sb, sb+1, sb+2, sb, sb+2, sb+3])
            }
        }

        boardMeshIndexCount = indices.count
        guard boardMeshIndexCount > 0 else { return }
        boardMeshVB = device.makeBuffer(bytes: vertices, length: vertices.count * MemoryLayout<BoardVertex>.stride, options: [.storageModeShared])
        boardMeshIB = device.makeBuffer(bytes: indices, length: indices.count * MemoryLayout<UInt32>.stride, options: [.storageModeShared])
    }

    // MARK: - Platform mesh (rescue mode)

    /// Build platform mesh from 4 corner positions. Generates a box with thickness.
    /// Called every frame to track moving platform corners.
    func updatePlatformMesh() {
        guard isRescueMode, let sim = simulator, let plat = sim.platform else {
            platformVB = nil
            platformIB = nil
            platformIndexCount = 0
            return
        }

        let corners = plat.corners  // TL(0), TR(1), BR(2), BL(3)
        let thickness: Float = 0.06

        var vertices: [BoardVertex] = []
        var indices: [UInt32] = []
        vertices.reserveCapacity(24)
        indices.reserveCapacity(36)

        // Top face normal — cross product of two edges
        let edge01 = corners[1] - corners[0]
        let edge03 = corners[3] - corners[0]
        let topNormal = simd_normalize(simd_cross(edge01, edge03))

        // Top face (4 verts, 2 tris)
        let base: UInt32 = 0
        for i in 0..<4 {
            let p = corners[i]
            vertices.append(BoardVertex(position: p, normal: topNormal, worldXY: SIMD2(p.x, p.y)))
        }
        indices.append(contentsOf: [base, base+1, base+2, base, base+2, base+3])

        // Bottom face
        let botBase = UInt32(vertices.count)
        let botNormal = -topNormal
        for i in 0..<4 {
            let p = corners[i] - topNormal * thickness
            vertices.append(BoardVertex(position: p, normal: botNormal, worldXY: SIMD2(p.x, p.y)))
        }
        indices.append(contentsOf: [botBase, botBase+2, botBase+1, botBase, botBase+3, botBase+2])

        // 4 side faces: edges (0,1), (1,2), (2,3), (3,0)
        let edgePairs: [(Int, Int)] = [(0,1), (1,2), (2,3), (3,0)]
        for (a, b) in edgePairs {
            let topA = corners[a]
            let topB = corners[b]
            let botA = topA - topNormal * thickness
            let botB = topB - topNormal * thickness

            let edgeDir = topB - topA
            let sideNormal = simd_normalize(simd_cross(edgeDir, topNormal))

            let sb = UInt32(vertices.count)
            vertices.append(BoardVertex(position: topA, normal: sideNormal, worldXY: SIMD2(topA.x, topA.y)))
            vertices.append(BoardVertex(position: topB, normal: sideNormal, worldXY: SIMD2(topB.x, topB.y)))
            vertices.append(BoardVertex(position: botB, normal: sideNormal, worldXY: SIMD2(botB.x, botB.y)))
            vertices.append(BoardVertex(position: botA, normal: sideNormal, worldXY: SIMD2(botA.x, botA.y)))
            indices.append(contentsOf: [sb, sb+1, sb+2, sb, sb+2, sb+3])
        }

        platformIndexCount = indices.count
        guard platformIndexCount > 0 else { return }

        let vbSize = vertices.count * MemoryLayout<BoardVertex>.stride
        let ibSize = indices.count * MemoryLayout<UInt32>.stride

        if platformVB == nil || platformVB!.length < vbSize {
            platformVB = device.makeBuffer(length: vbSize, options: [.storageModeShared])
        }
        if platformIB == nil || platformIB!.length < ibSize {
            platformIB = device.makeBuffer(bytes: indices, length: ibSize, options: [.storageModeShared])
        } else {
            platformIB?.contents().copyMemory(from: indices, byteCount: ibSize)
        }
        platformVB?.contents().copyMemory(from: vertices, byteCount: vbSize)
    }

    func bakeWoodTexture() {
        guard updateWoodBakeBounds() else { return }
        let minBake = woodBoundsMin
        let maxBake = woodBoundsMax

        let texSize = 8192
        let desc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .rgba16Float, width: texSize, height: texSize, mipmapped: false)
        desc.usage = [.shaderRead, .shaderWrite]
        guard let tex = device.makeTexture(descriptor: desc) else { return }
        bakedWoodTex = tex
        guard let cmdBuf = commandQueue.makeCommandBuffer(),
              let encoder = cmdBuf.makeComputeCommandEncoder() else { return }

        let effectiveSeed = Float(currentLevelId) + woodSeed * 100
        var params = BakeWoodParams(worldMin: minBake, worldMax: maxBake, seed: effectiveSeed, brightness: woodBrightness, patternScale: woodPatternScale)
        encoder.setComputePipelineState(bakeWoodPipeline)
        encoder.setTexture(tex, index: 0)
        encoder.setBytes(&params, length: MemoryLayout<BakeWoodParams>.stride, index: 0)

        let threadWidth = bakeWoodPipeline.threadExecutionWidth
        let threadHeight = max(1, bakeWoodPipeline.maxTotalThreadsPerThreadgroup / threadWidth)
        let threadsPerGroup = MTLSize(width: threadWidth, height: threadHeight, depth: 1)
        let groupsW = (texSize + threadWidth - 1) / threadWidth
        let groupsH = (texSize + threadHeight - 1) / threadHeight
        encoder.dispatchThreadgroups(MTLSize(width: groupsW, height: groupsH, depth: 1), threadsPerThreadgroup: threadsPerGroup)
        encoder.endEncoding()
        cmdBuf.commit()
        cmdBuf.waitUntilCompleted()
    }

    func bakeHoleMaskTexture() {
        guard updateWoodBakeBounds() else {
            bakedHoleMaskTex = nil
            return
        }

        let texSize = 2048
        let desc = MTLTextureDescriptor.texture2DDescriptor(pixelFormat: .r8Unorm, width: texSize, height: texSize, mipmapped: false)
        desc.usage = .shaderRead
        guard let tex = device.makeTexture(descriptor: desc) else { return }
        bakedHoleMaskTex = tex

        let visualRadius = holeRadius * holeRadiusScale
        let radiusSq = visualRadius * visualRadius
        let minBake = woodBoundsMin
        let maxBake = woodBoundsMax
        let span = maxBake - minBake
        let useSquareHoles = squareCrossSection
        var pixels = [UInt8](repeating: 0, count: texSize * texSize)

        for y in 0..<texSize {
            let v = (Float(y) + 0.5) / Float(texSize)
            let worldY = minBake.y + span.y * v
            for x in 0..<texSize {
                let u = (Float(x) + 0.5) / Float(texSize)
                let worldX = minBake.x + span.x * u
                var isHole: UInt8 = 0
                for i in holePositions.indices {
                    if holeElevations.indices.contains(i), holeElevations[i] > 0.01 { continue }
                    let delta = SIMD2<Float>(worldX, worldY) - holePositions[i]
                    if useSquareHoles {
                        if max(abs(delta.x), abs(delta.y)) < visualRadius {
                            isHole = 255
                            break
                        }
                    } else if simd_length_squared(delta) < radiusSq {
                        isHole = 255
                        break
                    }
                }
                pixels[y * texSize + x] = isHole
            }
        }

        tex.replace(region: MTLRegionMake2D(0, 0, texSize, texSize), mipmapLevel: 0, withBytes: pixels, bytesPerRow: texSize)
    }

    func bakeBoardWoodVolumeTexture() {
        guard !boards.isEmpty, !holePositions.isEmpty else {
            bakedBoardWoodVolumeTex = nil
            return
        }

        let texW = 128
        let texH = 128
        let texD = 64
        let desc = MTLTextureDescriptor()
        desc.textureType = .type3D
        desc.pixelFormat = .rgba16Float
        desc.width = texW
        desc.height = texH
        desc.depth = texD
        desc.mipmapLevelCount = 1
        desc.usage = [.shaderRead, .shaderWrite]
        guard let tex = device.makeTexture(descriptor: desc) else { return }
        bakedBoardWoodVolumeTex = tex
        boardWoodZMin = min(0, boards.map(\.elevation).min() ?? boardElevation)
        boardWoodZMax = max(boards.map(\.elevation).max() ?? boardElevation, 0.001)

        guard let cmdBuf = commandQueue.makeCommandBuffer(),
              let encoder = cmdBuf.makeComputeCommandEncoder() else { return }
        let effectiveSeed = Float(currentLevelId) + woodSeed * 100
        var params = BakeBoardWoodVolumeParams(
            worldMin: SIMD4<Float>(woodBoundsMin.x, woodBoundsMin.y, boardWoodZMin, 0),
            worldMax: SIMD4<Float>(woodBoundsMax.x, woodBoundsMax.y, boardWoodZMax, 0),
            seed: effectiveSeed,
            brightness: woodBrightness
        )
        encoder.setComputePipelineState(bakeBoardWoodVolumePipeline)
        encoder.setTexture(tex, index: 0)
        encoder.setBytes(&params, length: MemoryLayout<BakeBoardWoodVolumeParams>.stride, index: 0)

        let maxThreads = bakeBoardWoodVolumePipeline.maxTotalThreadsPerThreadgroup
        let tgDepth = max(1, min(4, maxThreads / 16))
        let threadsPerGroup = MTLSize(width: 4, height: 4, depth: tgDepth)
        let groupsW = (texW + threadsPerGroup.width - 1) / threadsPerGroup.width
        let groupsH = (texH + threadsPerGroup.height - 1) / threadsPerGroup.height
        let groupsD = (texD + threadsPerGroup.depth - 1) / threadsPerGroup.depth
        encoder.dispatchThreadgroups(MTLSize(width: groupsW, height: groupsH, depth: groupsD), threadsPerThreadgroup: threadsPerGroup)
        encoder.endEncoding()
        cmdBuf.commit()
        cmdBuf.waitUntilCompleted()
    }

    func invalidateOffscreenTextures() {
        hdrTex = nil
    }

    func scaledOffscreenSize(from drawableSize: CGSize) -> CGSize {
        let s = CGFloat(renderScale)
        return CGSize(
            width: max(1, (drawableSize.width * s).rounded()),
            height: max(1, (drawableSize.height * s).rounded())
        )
    }
}
