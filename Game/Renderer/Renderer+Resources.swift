import MetalKit
import simd

extension Renderer {
    func rebuildHoleInstances() {
        guard !holePositions.isEmpty else { return }
        let visualRadius = holeRadius * holeRadiusScale
        holeInstances = Self.makeHoleInstances(device: device, positions: holePositions, elevations: holeElevations, radius: visualRadius)
    }

    func rebuildHoleInstancesIfNeeded() {
        rebuildHoleInstances()
    }

    func rebuildHoleMeshIfNeeded() {
        Self.buildHoleMeshBuffers(device: device, segments: holeSegments, square: squareCrossSection, vertexBuffer: &holeVB, indexBuffer: &holeIB, indexCount: &holeIndexCount)
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

    func bakeWoodTexture() {
        guard !holePositions.isEmpty else { return }
        var minP = SIMD2<Float>(Float.greatestFiniteMagnitude, Float.greatestFiniteMagnitude)
        var maxP = SIMD2<Float>(-Float.greatestFiniteMagnitude, -Float.greatestFiniteMagnitude)
        for h in holePositions {
            minP = min(minP, h)
            maxP = max(maxP, h)
        }
        let boardSize = max(maxP.x - minP.x, maxP.y - minP.y)
        let padding = max(boardSize * 3.0, 10.0)
        let center = (minP + maxP) * 0.5
        let half = (maxP - minP) * 0.5 + SIMD2<Float>(padding, padding)
        let minBake = center - half
        let maxBake = center + half
        woodBoundsMin = minBake
        woodBoundsMax = maxBake

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
