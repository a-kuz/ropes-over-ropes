package com.uzls.four.renderer

import android.opengl.GLES30
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * UBO layout matching Metal's FrameUniforms
 * 3 mat4 (192) + 22 vec4 (352) = 544 bytes
 */
class FrameUniforms {
    private val floatCount = 3 * 16 + 23 * 4 // 140
    private val byteSize = floatCount * 4 // 560
    private val buffer: ByteBuffer = ByteBuffer.allocateDirect(byteSize).order(ByteOrder.nativeOrder())
    private val floatBuffer = buffer.asFloatBuffer()

    var uboId: Int = -1
        private set

    fun create() {
        val ids = IntArray(1)
        GLES30.glGenBuffers(1, ids, 0)
        uboId = ids[0]
        GLES30.glBindBuffer(GLES30.GL_UNIFORM_BUFFER, uboId)
        GLES30.glBufferData(GLES30.GL_UNIFORM_BUFFER, byteSize, null, GLES30.GL_DYNAMIC_DRAW)
        GLES30.glBindBuffer(GLES30.GL_UNIFORM_BUFFER, 0)
    }

    fun upload(
        viewProj: FloatArray,          // mat4 [0]
        invViewProj: FloatArray,       // mat4 [16]
        lightViewProj: FloatArray,     // mat4 [32]
        lightDirIntensity: FloatArray, // vec4 [48]
        ambientColor: FloatArray,      // vec4 [52]
        cameraPos: FloatArray,         // vec4 [56]
        orthoHalfSizeShadowBias: FloatArray, // vec4 [60]
        shadowInvSize: FloatArray,     // vec4 [64]
        timeDrag: FloatArray,          // vec4 [68]
        woodBoundsMin: FloatArray,     // vec4 [72]
        woodBoundsMax: FloatArray,     // vec4 [76]
        holeTint: FloatArray,          // vec4 [80]
        visualParams: FloatArray,      // vec4 [84]
        lightingParams: FloatArray,    // vec4 [88]
        tableParams: FloatArray,       // vec4 [92]
        tableParams2: FloatArray,      // vec4 [96]
        ropeMatParams: FloatArray,     // vec4 [100]
        ropeMatParams2: FloatArray,    // vec4 [104]
        ropeMatParams3: FloatArray,    // vec4 [108]
        cartoonParams: FloatArray,     // vec4 [112]
        wormParams1: FloatArray,       // vec4 [116]
        wormParams2: FloatArray,       // vec4 [120]
        wormParams3: FloatArray,       // vec4 [124]
        wormParams4: FloatArray,        // vec4 [128]
        ropeMatParams4: FloatArray     // vec4 [132]
    ) {
        floatBuffer.position(0)
        floatBuffer.put(viewProj, 0, 16)
        floatBuffer.put(invViewProj, 0, 16)
        floatBuffer.put(lightViewProj, 0, 16)
        floatBuffer.put(lightDirIntensity, 0, 4)
        floatBuffer.put(ambientColor, 0, 4)
        floatBuffer.put(cameraPos, 0, 4)
        floatBuffer.put(orthoHalfSizeShadowBias, 0, 4)
        floatBuffer.put(shadowInvSize, 0, 4)
        floatBuffer.put(timeDrag, 0, 4)
        floatBuffer.put(woodBoundsMin, 0, 4)
        floatBuffer.put(woodBoundsMax, 0, 4)
        floatBuffer.put(holeTint, 0, 4)
        floatBuffer.put(visualParams, 0, 4)
        floatBuffer.put(lightingParams, 0, 4)
        floatBuffer.put(tableParams, 0, 4)
        floatBuffer.put(tableParams2, 0, 4)
        floatBuffer.put(ropeMatParams, 0, 4)
        floatBuffer.put(ropeMatParams2, 0, 4)
        floatBuffer.put(ropeMatParams3, 0, 4)
        floatBuffer.put(cartoonParams, 0, 4)
        floatBuffer.put(wormParams1, 0, 4)
        floatBuffer.put(wormParams2, 0, 4)
        floatBuffer.put(wormParams3, 0, 4)
        floatBuffer.put(wormParams4, 0, 4)
        floatBuffer.put(ropeMatParams4, 0, 4)

        buffer.position(0)
        GLES30.glBindBuffer(GLES30.GL_UNIFORM_BUFFER, uboId)
        GLES30.glBufferSubData(GLES30.GL_UNIFORM_BUFFER, 0, byteSize, buffer)
        GLES30.glBindBuffer(GLES30.GL_UNIFORM_BUFFER, 0)
    }

    fun bind(bindingPoint: Int) {
        GLES30.glBindBufferBase(GLES30.GL_UNIFORM_BUFFER, bindingPoint, uboId)
    }

    fun delete() {
        if (uboId != -1) {
            GLES30.glDeleteBuffers(1, intArrayOf(uboId), 0)
            uboId = -1
        }
    }
}
