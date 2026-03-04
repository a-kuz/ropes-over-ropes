package com.uzls.four.simulation

import kotlin.math.abs
import kotlin.math.max

// --- CrossSection (sealed class mirroring Swift enum) ---

sealed class CrossSection {
    data class Circular(val radius: Float) : CrossSection()
    data class Rectangular(val width: Float, val height: Float) : CrossSection()

    val collisionRadius: Float
        get() = when (this) {
            is Circular -> radius
            is Rectangular -> max(width, height) * 0.5f
        }

    val halfWidth: Float
        get() = when (this) {
            is Circular -> radius
            is Rectangular -> width * 0.5f
        }

    val halfHeight: Float
        get() = when (this) {
            is Circular -> radius
            is Rectangular -> height * 0.5f
        }

    val isRectangular: Boolean
        get() = this is Rectangular

    /**
     * Effective radius along a contact normal, given the material frame directions d1 and d2.
     * For circular cross-sections this is just the radius.
     * For rectangular, it's the Minkowski support function.
     */
    fun effectiveRadius(
        normalX: Float, normalY: Float, normalZ: Float,
        d1x: Float, d1y: Float, d1z: Float,
        d2x: Float, d2y: Float, d2z: Float
    ): Float = when (this) {
        is Circular -> radius
        is Rectangular -> {
            val hw = width * 0.5f
            val hh = height * 0.5f
            abs(vec3Dot(normalX, normalY, normalZ, d1x, d1y, d1z)) * hw +
                    abs(vec3Dot(normalX, normalY, normalZ, d2x, d2y, d2z)) * hh
        }
    }
}

// --- MaterialFrame ---

class MaterialFrame(
    var tangentX: Float = 0f, var tangentY: Float = 0f, var tangentZ: Float = 0f,
    var d1x: Float = 0f, var d1y: Float = 0f, var d1z: Float = 0f,
    var d2x: Float = 0f, var d2y: Float = 0f, var d2z: Float = 0f
)

// --- DragInfo ---

data class DragInfo(
    val bandIndex: Int,
    val endIndex: Int,       // 0 = first particle, 1 = last particle
    val originalHoleIndex: Int
)

// --- LowerAnimation ---

class LowerAnimation(
    val bandIndex: Int,
    val endIndex: Int,
    val targetHole: Int,
    var startPosX: Float,
    var startPosY: Float,
    var startPosZ: Float,
    var timer: Float = 0f,
    var returnPosX: Float = 0f,
    var returnPosY: Float = 0f,
    var returnPosZ: Float = 0f,
    var hasReturnPos: Boolean = false,
    var returnDuration: Float = 0f
) {
    companion object {
        const val DURATION: Float = 0.55f
    }
}

// --- CollisionPair ---

class CollisionPair(
    val bandA: Int,
    val segA: Int,
    val bandB: Int,
    val segB: Int
)

// --- FrictionEvent ---

data class FrictionEvent(
    val intensity: Float,
    val relativeSpeed: Float,
    val positionX: Float,
    val positionY: Float,
    val positionZ: Float
)

// --- BandSnapshot ---

class BandSnapshot(
    val positions: FloatArray,
    val previousPositions: FloatArray,
    val twistAngles: FloatArray,
    val previousTwistAngles: FloatArray,
    val segmentLength: Float,
    val pinStart: Int,      // -1 means nil
    val pinEnd: Int,        // -1 means nil
    val active: Boolean,
    val fadeOut: Float,
    val suckHole: Int,      // -1 means nil
    val suckFromEnd: Int,
    val suckConsumed: Float
)

// --- Snapshot ---

class Snapshot(
    val bands: Array<BandSnapshot>
)

// --- BoardDef ---

class BoardDef(
    val centerX: Float,
    val centerY: Float,
    val width: Float,
    val height: Float,
    val elevation: Float
)

// --- RopeConfig ---

class RopeConfig(
    val startHole: Int,
    val endHole: Int,
    val radius: Float,
    val crossSection: CrossSection? = null
)

// --- LevelAction ---

class LevelAction(
    val type: ActionType,
    val ropeIndex: Int,
    val endIndex: Int,
    val holeIndex: Int
) {
    enum class ActionType { PIN, DRAG }
}

// --- Band (stride-3 FloatArray layout) ---

class Band(
    /** Flat positions array: [x0,y0,z0, x1,y1,z1, ...]. Length = particleCount * 3 */
    var positions: FloatArray,
    /** Flat previous-positions array, same layout */
    var previousPositions: FloatArray,
    /** Twist angles per particle */
    var twistAngles: FloatArray,
    /** Previous twist angles per particle */
    var previousTwistAngles: FloatArray,
    var segmentLength: Float,
    var radius: Float,
    var crossSection: CrossSection,
    /** Hole index for start pin, or -1 if unpinned */
    var pinStart: Int = -1,
    /** Hole index for end pin, or -1 if unpinned */
    var pinEnd: Int = -1,
    var active: Boolean = false,
    var fadeOut: Float = 0f,
    /** Hole index for suck animation, or -1 if none */
    var suckHole: Int = -1,
    var suckFromEnd: Int = 1,
    var suckConsumed: Float = 0f
) {
    /** Number of particles in this band */
    val particleCount: Int get() = positions.size / 3

    // Color (set after construction)
    var colorR: Float = 0.8f
    var colorG: Float = 0.3f
    var colorB: Float = 0.3f

    // Visual state computed during simulation
    var tautness: Float = 0f

    companion object {
        const val FADE_OUT_SPEED: Float = 45.0f
    }
}
