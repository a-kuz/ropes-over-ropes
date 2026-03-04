package com.uzls.four.renderer

import android.opengl.GLES30
import android.util.Log

class ShaderProgram(vertSrc: String, fragSrc: String) {
    val id: Int

    init {
        val vert = compileShader(GLES30.GL_VERTEX_SHADER, vertSrc)
        val frag = compileShader(GLES30.GL_FRAGMENT_SHADER, fragSrc)

        id = GLES30.glCreateProgram()
        GLES30.glAttachShader(id, vert)
        GLES30.glAttachShader(id, frag)
        GLES30.glLinkProgram(id)

        val status = IntArray(1)
        GLES30.glGetProgramiv(id, GLES30.GL_LINK_STATUS, status, 0)
        if (status[0] == 0) {
            val log = GLES30.glGetProgramInfoLog(id)
            GLES30.glDeleteProgram(id)
            throw IllegalStateException("Program link failed: $log")
        }

        GLES30.glDeleteShader(vert)
        GLES30.glDeleteShader(frag)
    }

    fun use() = GLES30.glUseProgram(id)

    fun setMat4(name: String, m: FloatArray) {
        GLES30.glUniformMatrix4fv(loc(name), 1, false, m, 0)
    }

    fun setVec4(name: String, x: Float, y: Float, z: Float, w: Float) {
        GLES30.glUniform4f(loc(name), x, y, z, w)
    }

    fun setVec2(name: String, x: Float, y: Float) {
        GLES30.glUniform2f(loc(name), x, y)
    }

    fun setVec3(name: String, x: Float, y: Float, z: Float) {
        GLES30.glUniform3f(loc(name), x, y, z)
    }

    fun setFloat(name: String, v: Float) {
        GLES30.glUniform1f(loc(name), v)
    }

    fun setInt(name: String, v: Int) {
        GLES30.glUniform1i(loc(name), v)
    }

    fun bindUbo(blockName: String, bindingPoint: Int) {
        val idx = GLES30.glGetUniformBlockIndex(id, blockName)
        if (idx != GLES30.GL_INVALID_INDEX) {
            GLES30.glUniformBlockBinding(id, idx, bindingPoint)
        }
    }

    fun delete() = GLES30.glDeleteProgram(id)

    private val locCache = HashMap<String, Int>()

    private fun loc(name: String): Int =
        locCache.getOrPut(name) { GLES30.glGetUniformLocation(id, name) }

    companion object {
        private fun compileShader(type: Int, src: String): Int {
            val shader = GLES30.glCreateShader(type)
            GLES30.glShaderSource(shader, src)
            GLES30.glCompileShader(shader)

            val status = IntArray(1)
            GLES30.glGetShaderiv(shader, GLES30.GL_COMPILE_STATUS, status, 0)
            if (status[0] == 0) {
                val log = GLES30.glGetShaderInfoLog(shader)
                GLES30.glDeleteShader(shader)
                val typeName = if (type == GLES30.GL_VERTEX_SHADER) "vertex" else "fragment"
                throw IllegalStateException("$typeName shader compile failed: $log")
            }
            return shader
        }
    }
}
