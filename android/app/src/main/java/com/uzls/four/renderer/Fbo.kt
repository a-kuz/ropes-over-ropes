package com.uzls.four.renderer

import android.opengl.GLES30

class Fbo private constructor(
    val width: Int,
    val height: Int,
    val fboId: Int,
    val colorTexId: Int,
    val depthTexId: Int
) {
    fun bind() {
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, fboId)
        GLES30.glViewport(0, 0, width, height)
    }

    fun unbind() {
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0)
    }

    fun delete() {
        GLES30.glDeleteFramebuffers(1, intArrayOf(fboId), 0)
        if (colorTexId != -1) GLES30.glDeleteTextures(1, intArrayOf(colorTexId), 0)
        if (depthTexId != -1) GLES30.glDeleteTextures(1, intArrayOf(depthTexId), 0)
    }

    companion object {
        fun makeHdr(w: Int, h: Int): Fbo {
            val fbo = IntArray(1)
            GLES30.glGenFramebuffers(1, fbo, 0)
            GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, fbo[0])

            // Color: RGBA16F
            val colorTex = IntArray(1)
            GLES30.glGenTextures(1, colorTex, 0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, colorTex[0])
            GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_RGBA16F, w, h, 0,
                GLES30.GL_RGBA, GLES30.GL_HALF_FLOAT, null)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE)
            GLES30.glFramebufferTexture2D(GLES30.GL_FRAMEBUFFER, GLES30.GL_COLOR_ATTACHMENT0,
                GLES30.GL_TEXTURE_2D, colorTex[0], 0)

            // Depth+Stencil: DEPTH24_STENCIL8
            val depthTex = IntArray(1)
            GLES30.glGenTextures(1, depthTex, 0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, depthTex[0])
            GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_DEPTH24_STENCIL8, w, h, 0,
                GLES30.GL_DEPTH_STENCIL, GLES30.GL_UNSIGNED_INT_24_8, null)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_NEAREST)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_NEAREST)
            GLES30.glFramebufferTexture2D(GLES30.GL_FRAMEBUFFER, GLES30.GL_DEPTH_STENCIL_ATTACHMENT,
                GLES30.GL_TEXTURE_2D, depthTex[0], 0)

            checkFramebuffer("HDR")
            GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0)

            return Fbo(w, h, fbo[0], colorTex[0], depthTex[0])
        }

        fun makeBloom(w: Int, h: Int): Fbo {
            val fbo = IntArray(1)
            GLES30.glGenFramebuffers(1, fbo, 0)
            GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, fbo[0])

            val colorTex = IntArray(1)
            GLES30.glGenTextures(1, colorTex, 0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, colorTex[0])
            GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_RGBA16F, w, h, 0,
                GLES30.GL_RGBA, GLES30.GL_HALF_FLOAT, null)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE)
            GLES30.glFramebufferTexture2D(GLES30.GL_FRAMEBUFFER, GLES30.GL_COLOR_ATTACHMENT0,
                GLES30.GL_TEXTURE_2D, colorTex[0], 0)

            checkFramebuffer("Bloom")
            GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0)

            return Fbo(w, h, fbo[0], colorTex[0], -1)
        }

        fun makeShadow(size: Int): Fbo {
            val fbo = IntArray(1)
            GLES30.glGenFramebuffers(1, fbo, 0)
            GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, fbo[0])

            val depthTex = IntArray(1)
            GLES30.glGenTextures(1, depthTex, 0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, depthTex[0])
            GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_DEPTH_COMPONENT32F, size, size, 0,
                GLES30.GL_DEPTH_COMPONENT, GLES30.GL_FLOAT, null)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_COMPARE_MODE, GLES30.GL_COMPARE_REF_TO_TEXTURE)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_COMPARE_FUNC, GLES30.GL_LEQUAL)
            GLES30.glFramebufferTexture2D(GLES30.GL_FRAMEBUFFER, GLES30.GL_DEPTH_ATTACHMENT,
                GLES30.GL_TEXTURE_2D, depthTex[0], 0)

            // No color attachment
            GLES30.glDrawBuffers(1, intArrayOf(GLES30.GL_NONE), 0)
            GLES30.glReadBuffer(GLES30.GL_NONE)

            checkFramebuffer("Shadow")
            GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0)

            return Fbo(size, size, fbo[0], -1, depthTex[0])
        }

        private fun checkFramebuffer(name: String) {
            val status = GLES30.glCheckFramebufferStatus(GLES30.GL_FRAMEBUFFER)
            if (status != GLES30.GL_FRAMEBUFFER_COMPLETE) {
                throw IllegalStateException("$name FBO incomplete: 0x${status.toString(16)}")
            }
        }
    }
}
