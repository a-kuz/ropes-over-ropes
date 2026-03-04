package com.uzls.four.simulation

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

// Column-major 4x4 matrix stored as FloatArray(16)
// Index: col * 4 + row

fun mat4Identity(): FloatArray {
    val m = FloatArray(16)
    m[0] = 1f; m[5] = 1f; m[10] = 1f; m[15] = 1f
    return m
}

fun mat4Ortho(
    left: Float, right: Float,
    bottom: Float, top: Float,
    near: Float, far: Float
): FloatArray {
    val m = FloatArray(16)
    val rl = right - left
    val tb = top - bottom
    val fn = far - near
    m[0] = 2f / rl
    m[5] = 2f / tb
    m[10] = -2f / fn
    m[12] = -(right + left) / rl
    m[13] = -(top + bottom) / tb
    m[14] = -(far + near) / fn
    m[15] = 1f
    return m
}

fun mat4LookAt(
    eyeX: Float, eyeY: Float, eyeZ: Float,
    centerX: Float, centerY: Float, centerZ: Float,
    upX: Float, upY: Float, upZ: Float
): FloatArray {
    var fx = centerX - eyeX
    var fy = centerY - eyeY
    var fz = centerZ - eyeZ
    val fLen = vec3Length(fx, fy, fz)
    if (fLen > 1e-9f) { fx /= fLen; fy /= fLen; fz /= fLen }

    // s = f × up
    var sx = vec3CrossX(fy, fz, upY, upZ)
    var sy = vec3CrossY(fx, fz, upX, upZ)
    var sz = vec3CrossZ(fx, fy, upX, upY)
    val sLen = vec3Length(sx, sy, sz)
    if (sLen > 1e-9f) { sx /= sLen; sy /= sLen; sz /= sLen }

    // u = s × f
    val ux = vec3CrossX(sy, sz, fy, fz)
    val uy = vec3CrossY(sx, sz, fx, fz)
    val uz = vec3CrossZ(sx, sy, fx, fy)

    val m = FloatArray(16)
    m[0] = sx; m[1] = ux; m[2] = -fx; m[3] = 0f
    m[4] = sy; m[5] = uy; m[6] = -fy; m[7] = 0f
    m[8] = sz; m[9] = uz; m[10] = -fz; m[11] = 0f
    m[12] = -(sx * eyeX + sy * eyeY + sz * eyeZ)
    m[13] = -(ux * eyeX + uy * eyeY + uz * eyeZ)
    m[14] = (fx * eyeX + fy * eyeY + fz * eyeZ)
    m[15] = 1f
    return m
}

fun mat4RotationZ(angle: Float): FloatArray {
    val c = cos(angle)
    val s = sin(angle)
    val m = FloatArray(16)
    m[0] = c; m[1] = s
    m[4] = -s; m[5] = c
    m[10] = 1f; m[15] = 1f
    return m
}

fun mat4Multiply(a: FloatArray, b: FloatArray): FloatArray {
    val m = FloatArray(16)
    for (col in 0..3) {
        for (row in 0..3) {
            var sum = 0f
            for (k in 0..3) {
                sum += a[k * 4 + row] * b[col * 4 + k]
            }
            m[col * 4 + row] = sum
        }
    }
    return m
}

fun mat4MultiplyVec4(m: FloatArray, v: FloatArray): FloatArray {
    val out = FloatArray(4)
    for (row in 0..3) {
        out[row] = m[row] * v[0] + m[4 + row] * v[1] + m[8 + row] * v[2] + m[12 + row] * v[3]
    }
    return out
}

fun mat4Inverse(m: FloatArray): FloatArray {
    val inv = FloatArray(16)

    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] -
            m[9] * m[6] * m[15] + m[9] * m[7] * m[14] +
            m[13] * m[6] * m[11] - m[13] * m[7] * m[10]

    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] +
            m[8] * m[6] * m[15] - m[8] * m[7] * m[14] -
            m[12] * m[6] * m[11] + m[12] * m[7] * m[10]

    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] -
            m[8] * m[5] * m[15] + m[8] * m[7] * m[13] +
            m[12] * m[5] * m[11] - m[12] * m[7] * m[9]

    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] +
            m[8] * m[5] * m[14] - m[8] * m[6] * m[13] -
            m[12] * m[5] * m[10] + m[12] * m[6] * m[9]

    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] +
            m[9] * m[2] * m[15] - m[9] * m[3] * m[14] -
            m[13] * m[2] * m[11] + m[13] * m[3] * m[10]

    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] -
            m[8] * m[2] * m[15] + m[8] * m[3] * m[14] +
            m[12] * m[2] * m[11] - m[12] * m[3] * m[10]

    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] +
            m[8] * m[1] * m[15] - m[8] * m[3] * m[13] -
            m[12] * m[1] * m[11] + m[12] * m[3] * m[9]

    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] -
            m[8] * m[1] * m[14] + m[8] * m[2] * m[13] +
            m[12] * m[1] * m[10] - m[12] * m[2] * m[9]

    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] -
            m[5] * m[2] * m[15] + m[5] * m[3] * m[14] +
            m[13] * m[2] * m[7] - m[13] * m[3] * m[6]

    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] +
            m[4] * m[2] * m[15] - m[4] * m[3] * m[14] -
            m[12] * m[2] * m[7] + m[12] * m[3] * m[6]

    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] -
            m[4] * m[1] * m[15] + m[4] * m[3] * m[13] +
            m[12] * m[1] * m[7] - m[12] * m[3] * m[5]

    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] +
            m[4] * m[1] * m[14] - m[4] * m[2] * m[13] -
            m[12] * m[1] * m[6] + m[12] * m[2] * m[5]

    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] +
            m[5] * m[2] * m[11] - m[5] * m[3] * m[10] -
            m[9] * m[2] * m[7] + m[9] * m[3] * m[6]

    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] -
            m[4] * m[2] * m[11] + m[4] * m[3] * m[10] +
            m[8] * m[2] * m[7] - m[8] * m[3] * m[6]

    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] +
            m[4] * m[1] * m[11] - m[4] * m[3] * m[9] -
            m[8] * m[1] * m[7] + m[8] * m[3] * m[5]

    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] -
            m[4] * m[1] * m[10] + m[4] * m[2] * m[9] +
            m[8] * m[1] * m[6] - m[8] * m[2] * m[5]

    val det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12]
    if (det == 0f) return mat4Identity()

    val invDet = 1f / det
    for (i in 0..15) inv[i] *= invDet
    return inv
}
