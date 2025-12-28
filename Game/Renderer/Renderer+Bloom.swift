import Metal

extension Renderer {
    func encodeBloom(encoder: MTLComputeCommandEncoder, hdrTexture: MTLTexture, bloomTextureA: MTLTexture, bloomTextureB: MTLTexture) {
        encoder.setComputePipelineState(bloomThreshold)
        encoder.setTexture(hdrTexture, index: 0)
        encoder.setTexture(bloomTextureA, index: 1)
        dispatch2D(
            encoder: encoder,
            pipeline: bloomThreshold,
            width: bloomTextureA.width,
            height: bloomTextureA.height
        )

        encoder.setComputePipelineState(bloomBlurH)
        encoder.setTexture(bloomTextureA, index: 0)
        encoder.setTexture(bloomTextureB, index: 1)
        dispatch2D(
            encoder: encoder,
            pipeline: bloomBlurH,
            width: bloomTextureB.width,
            height: bloomTextureB.height
        )

        encoder.setComputePipelineState(bloomBlurV)
        encoder.setTexture(bloomTextureB, index: 0)
        encoder.setTexture(bloomTextureA, index: 1)
        dispatch2D(
            encoder: encoder,
            pipeline: bloomBlurV,
            width: bloomTextureA.width,
            height: bloomTextureA.height
        )
    }

    func dispatch2D(encoder: MTLComputeCommandEncoder, pipeline: MTLComputePipelineState, width: Int, height: Int) {
        let threadWidth = pipeline.threadExecutionWidth
        let threadHeight = max(1, pipeline.maxTotalThreadsPerThreadgroup / threadWidth)
        let threadsPerThreadgroup = MTLSize(width: threadWidth, height: threadHeight, depth: 1)
        let groupsWidth = (width + threadWidth - 1) / threadWidth
        let groupsHeight = (height + threadHeight - 1) / threadHeight
        encoder.dispatchThreadgroups(
            MTLSize(width: groupsWidth, height: groupsHeight, depth: 1),
            threadsPerThreadgroup: threadsPerThreadgroup
        )
    }
}

