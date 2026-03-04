@file:Suppress("NOTHING_TO_INLINE")

package com.uzls.four.simulation

import kotlin.math.sqrt

// Vec3 operations on FloatArray stride-3 layout
// All indices are byte offsets into the flat array: arr[i], arr[i+1], arr[i+2]

inline fun FloatArray.getX(i: Int) = this[i]
inline fun FloatArray.getY(i: Int) = this[i + 1]
inline fun FloatArray.getZ(i: Int) = this[i + 2]

inline fun FloatArray.setVec3(i: Int, x: Float, y: Float, z: Float) {
    this[i] = x; this[i + 1] = y; this[i + 2] = z
}

inline fun FloatArray.copyVec3(dst: Int, src: FloatArray, srcIdx: Int) {
    this[dst] = src[srcIdx]; this[dst + 1] = src[srcIdx + 1]; this[dst + 2] = src[srcIdx + 2]
}

fun vec3(x: Float, y: Float, z: Float) = floatArrayOf(x, y, z)

inline fun vec3Length(x: Float, y: Float, z: Float): Float =
    sqrt(x * x + y * y + z * z)

inline fun vec3LengthSq(x: Float, y: Float, z: Float): Float =
    x * x + y * y + z * z

inline fun vec3Dot(ax: Float, ay: Float, az: Float, bx: Float, by: Float, bz: Float): Float =
    ax * bx + ay * by + az * bz

inline fun vec3CrossX(ay: Float, az: Float, by: Float, bz: Float): Float = ay * bz - az * by
inline fun vec3CrossY(ax: Float, az: Float, bx: Float, bz: Float): Float = az * bx - ax * bz
inline fun vec3CrossZ(ax: Float, ay: Float, bx: Float, by: Float): Float = ax * by - ay * bx

fun vec3Normalize(x: Float, y: Float, z: Float): FloatArray {
    val len = vec3Length(x, y, z)
    return if (len < 1e-9f) floatArrayOf(0f, 0f, 0f)
    else floatArrayOf(x / len, y / len, z / len)
}

// Distance between two vec3 in a flat array
inline fun vec3Dist(a: FloatArray, ai: Int, b: FloatArray, bi: Int): Float {
    val dx = a[ai] - b[bi]
    val dy = a[ai + 1] - b[bi + 1]
    val dz = a[ai + 2] - b[bi + 2]
    return sqrt(dx * dx + dy * dy + dz * dz)
}

inline fun vec3DistSq(a: FloatArray, ai: Int, b: FloatArray, bi: Int): Float {
    val dx = a[ai] - b[bi]
    val dy = a[ai + 1] - b[bi + 1]
    val dz = a[ai + 2] - b[bi + 2]
    return dx * dx + dy * dy + dz * dz
}

// Lerp between two points in flat arrays
inline fun vec3Lerp(
    out: FloatArray, oi: Int,
    a: FloatArray, ai: Int,
    b: FloatArray, bi: Int,
    t: Float
) {
    val inv = 1f - t
    out[oi] = a[ai] * inv + b[bi] * t
    out[oi + 1] = a[ai + 1] * inv + b[bi + 1] * t
    out[oi + 2] = a[ai + 2] * inv + b[bi + 2] * t
}
