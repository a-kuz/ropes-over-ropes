package com.uzls.four.level

import com.uzls.four.simulation.CrossSection
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonTransformingSerializer
import kotlinx.serialization.json.float
import kotlinx.serialization.json.jsonPrimitive

@Serializable
data class LevelDefinition(
    val id: Int,
    val holeRadius: Float,
    val particlesPerRope: Int,
    val holes: List<Vec2>,
    val ropes: List<Rope>,
    val hooks: List<Hook>? = null,
    val actions: List<Action>? = null,
    val boards: List<Board>? = null
) {

    @Serializable
    data class Vec2(
        @SerialName("x") val xPosition: Float,
        @SerialName("y") val yPosition: Float,
        @SerialName("z") val zPosition: Float = 0f
    ) {
        val simdX: Float get() = xPosition
        val simdY: Float get() = yPosition
    }

    @Serializable
    data class Board(
        @SerialName("cx") val centerX: Float,
        @SerialName("cy") val centerY: Float,
        @SerialName("w") val width: Float,
        @SerialName("h") val height: Float,
        @SerialName("z") val elevation: Float = 0f
    )

    @Serializable
    data class Color(
        @SerialName("r") val redChannel: Float,
        @SerialName("g") val greenChannel: Float,
        @SerialName("b") val blueChannel: Float
    )

    @Serializable
    data class CrossSectionDef(
        val type: String,
        val width: Float? = null,
        val height: Float? = null
    ) {
        fun toCrossSection(fallbackRadius: Float): CrossSection {
            return when (type) {
                "rectangular" -> {
                    val w = width ?: (fallbackRadius * 2f)
                    val h = height ?: (fallbackRadius * 0.7f)
                    CrossSection.Rectangular(w, h)
                }
                else -> CrossSection.Circular(fallbackRadius)
            }
        }
    }

    @Serializable
    data class Rope(
        val startHole: Int,
        val endHole: Int,
        val color: Color,
        val radius: Float,
        @SerialName("crossSection") val crossSectionDef: CrossSectionDef? = null
    ) {
        val crossSection: CrossSection
            get() = crossSectionDef?.toCrossSection(radius) ?: CrossSection.Circular(radius)
    }

    @Serializable
    data class HookRopeRef(
        val fromType: String,   // "hole" or "hook"
        val index: Int,         // rope index (for "hole") or hook index (for "hook")
        val hookIndex: Int? = null // which side of the referenced hook (0=ropeA, 1=ropeB)
    )

    @Serializable
    data class Hook(
        val ropeA: HookRopeRef,
        val ropeB: HookRopeRef,
        val N: Int,
        val ropeAStartIsOver: Boolean
    )

    @Serializable
    data class Action(
        val type: String,       // "pin" or "drag"
        val ropeIndex: Int,
        val endIndex: Int,      // 0 = start, 1 = end
        val holeIndex: Int
    )
}

// CrossSection is defined in com.uzls.four.simulation.Band
