package com.uzls.four.renderer

import com.uzls.four.simulation.*
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

class Camera {
    var centerX = 0f
    var centerY = 0f
    var centerZ = 0f
    var distance = 2.8f
    var orthoHalfHeight = 2.05f
    var tiltAngle = 0f
    var rotationAngle = 0f

    fun viewProj(aspect: Float): FloatArray {
        val halfW = orthoHalfHeight * aspect
        val halfH = orthoHalfHeight

        val eyeX = centerX
        val eyeY = centerY + distance * sin(tiltAngle)
        val eyeZ = centerZ + distance * cos(tiltAngle)

        // iOS uses up = (0, 1, 0) — Y is screen-up
        val view = mat4LookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, 0f, 1f, 0f)
        val proj = mat4Ortho(-halfW, halfW, -halfH, halfH, 0.01f, 10f)
        val rot = mat4RotationZ(rotationAngle)

        return mat4Multiply(proj, mat4Multiply(rot, view))
    }

    fun lightViewProj(lightDirX: Float, lightDirY: Float, lightDirZ: Float, aspect: Float): FloatArray {
        // Match iOS makeLightViewProj exactly:
        // eye = target + lightDir * 4.9, up = (0,1,0)
        // ortho uses halfW*1.08, halfH*1.08, near=0.01, far=12.0
        val halfH = orthoHalfHeight
        val halfW = halfH * aspect
        val eyeX = centerX + lightDirX * 4.9f
        val eyeY = centerY + lightDirY * 4.9f
        val eyeZ = centerZ + lightDirZ * 4.9f

        // Choose up vector; avoid degeneracy when light is nearly parallel to up
        var upX = 0f; var upY = 1f; var upZ = 0f
        val dotUp = upX * lightDirX + upY * lightDirY + upZ * lightDirZ
        if (kotlin.math.abs(dotUp) > 0.95f) {
            upX = 1f; upY = 0f; upZ = 0f
        }

        val view = mat4LookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ)
        val proj = mat4Ortho(-halfW * 1.08f, halfW * 1.08f, -halfH * 1.08f, halfH * 1.08f, 0.01f, 12f)

        return mat4Multiply(proj, view)
    }

    fun fitToHoles(holePositions: FloatArray, holeRadius: Float, aspect: Float, maxElevation: Float) {
        if (holePositions.isEmpty()) return

        var minX = Float.MAX_VALUE; var maxX = -Float.MAX_VALUE
        var minY = Float.MAX_VALUE; var maxY = -Float.MAX_VALUE
        val count = holePositions.size / 2
        for (i in 0 until count) {
            val x = holePositions[i * 2]
            val y = holePositions[i * 2 + 1]
            if (x < minX) minX = x; if (x > maxX) maxX = x
            if (y < minY) minY = y; if (y > maxY) maxY = y
        }

        centerX = (minX + maxX) / 2f
        centerY = (minY + maxY) / 2f
        // iOS: center = SIMD3<Float>(centerX, centerY, 0)
        centerZ = 0f

        val margin = holeRadius * 2.5f
        val contentW = (maxX - minX) + margin * 2f
        val contentH = (maxY - minY) + margin * 2f
        val halfHFromHeight = contentH * 0.5f
        val halfHFromWidth = (contentW * 0.5f) / max(aspect, 0.01f)
        val elevationPadding = if (maxElevation > 0.01f) maxElevation * 1.5f else 0f
        // iOS: orthoHalfHeight = max(halfHFromHeight, halfHFromWidth) * 1.2 + elevationPadding
        orthoHalfHeight = max(halfHFromHeight, halfHFromWidth) * 1.2f + elevationPadding

        rotationAngle = 0f

        if (maxElevation > 0.01f && tiltAngle < 0.15f) {
            tiltAngle = 0.25f
        }
    }

    fun screenToWorld(
        screenX: Float, screenY: Float,
        viewWidth: Int, viewHeight: Int,
        boards: List<Board>? = null
    ): FloatArray {
        val aspect = viewWidth.toFloat() / viewHeight
        val vp = viewProj(aspect)
        val inv = mat4Inverse(vp)

        val ndcX = (screenX / viewWidth) * 2f - 1f
        val ndcY = 1f - (screenY / viewHeight) * 2f // Flip Y for Android

        val nearClip = mat4MultiplyVec4(inv, floatArrayOf(ndcX, ndcY, -1f, 1f))
        val farClip = mat4MultiplyVec4(inv, floatArrayOf(ndcX, ndcY, 1f, 1f))

        // Perspective divide
        val nw = nearClip[3]; val fw = farClip[3]
        val nx = nearClip[0] / nw; val ny = nearClip[1] / nw; val nz = nearClip[2] / nw
        val fx = farClip[0] / fw; val fy = farClip[1] / fw; val fz = farClip[2] / fw

        // Ray direction
        val dx = fx - nx; val dy = fy - ny; val dz = fz - nz

        // Check boards (highest elevation first)
        boards?.sortedByDescending { it.elevation }?.forEach { board ->
            if (kotlin.math.abs(dz) > 1e-6f) {
                val t = (board.elevation - nz) / dz
                val hitX = nx + dx * t
                val hitY = ny + dy * t
                val hw = board.width / 2f
                val hh = board.height / 2f
                if (hitX >= board.centerX - hw && hitX <= board.centerX + hw &&
                    hitY >= board.centerY - hh && hitY <= board.centerY + hh
                ) {
                    return floatArrayOf(hitX, hitY)
                }
            }
        }

        // Fallback: intersect Z=0 plane
        if (kotlin.math.abs(dz) > 1e-6f) {
            val t = -nz / dz
            return floatArrayOf(nx + dx * t, ny + dy * t)
        }
        return floatArrayOf(nx, ny)
    }
}

data class Board(
    val centerX: Float,
    val centerY: Float,
    val width: Float,
    val height: Float,
    val elevation: Float
)
