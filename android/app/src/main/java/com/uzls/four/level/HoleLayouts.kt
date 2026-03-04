package com.uzls.four.level

import kotlin.math.*

enum class HoleLayout {
    GRID,
    CIRCLE,
    HEXAGON,
    DIAMOND,
    CROSS,
    RINGS,
    TRIANGLE,
    STAR,
    HONEYCOMB,
    SPIRAL,
    DOUBLE_GRID,
    SCATTERED,
    COLUMNS,
    CENTER_PLUS_RING,
    ROTATED_SQUARE,
    SQUARE_IN_CIRCLE,
    ARC,
    CONCENTRIC_SQUARES,
    TRIANGLE_IN_CIRCLE,
    CRESCENT,
    LETTER_H,
    LETTER_T,
    ARROW,
    ZIGZAG,
    CLOVER,
    CLUSTERS,
    THREE_COLUMNS,
    RING_WITH_SPOKES,
    DIAMOND_OUTLINE,
    BOW_TIE;

    fun generate(n: Int): List<LevelDefinition.Vec2> = when (this) {
        GRID -> gridLayout(n)
        CIRCLE -> circleLayout(n)
        HEXAGON -> hexagonLayout(n)
        DIAMOND -> diamondLayout(n)
        CROSS -> crossLayout(n)
        RINGS -> ringsLayout(n)
        TRIANGLE -> triangleLayout(n)
        STAR -> starLayout(n)
        HONEYCOMB -> honeycombLayout(n)
        SPIRAL -> spiralLayout(n)
        DOUBLE_GRID -> doubleGridLayout(n)
        SCATTERED -> scatteredLayout(n)
        COLUMNS -> columnsLayout(n)
        CENTER_PLUS_RING -> centerPlusRingLayout(n)
        ROTATED_SQUARE -> rotatedSquareLayout(n)
        SQUARE_IN_CIRCLE -> squareInCircleLayout(n)
        ARC -> arcLayout(n)
        CONCENTRIC_SQUARES -> concentricSquaresLayout(n)
        TRIANGLE_IN_CIRCLE -> triangleInCircleLayout(n)
        CRESCENT -> crescentLayout(n)
        LETTER_H -> letterHLayout(n)
        LETTER_T -> letterTLayout(n)
        ARROW -> arrowLayout(n)
        ZIGZAG -> zigzagLayout(n)
        CLOVER -> cloverLayout(n)
        CLUSTERS -> clustersLayout(n)
        THREE_COLUMNS -> threeColumnsLayout(n)
        RING_WITH_SPOKES -> ringWithSpokesLayout(n)
        DIAMOND_OUTLINE -> diamondOutlineLayout(n)
        BOW_TIE -> bowTieLayout(n)
    }
}

// MARK: - Layout implementations

private fun vec2(x: Float, y: Float, z: Float = 0f) =
    LevelDefinition.Vec2(xPosition = x, yPosition = y, zPosition = z)

private fun gridLayout(n: Int): List<LevelDefinition.Vec2> {
    val cols = maxOf(3, ceil(sqrt(n.toFloat() * 1.25f)).toInt())
    val rows = maxOf(3, (n + cols - 1) / cols)
    val s = minOf(0.42f, 1.6f / (maxOf(cols, rows) - 1).toFloat())
    val ox = -(cols - 1).toFloat() / 2f * s
    val oy = -(rows - 1).toFloat() / 2f * s
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (row in 0 until rows) {
        for (col in 0 until cols) {
            pts.add(vec2(ox + col.toFloat() * s, oy + row.toFloat() * s))
        }
    }
    return pts
}

private fun circleLayout(n: Int): List<LevelDefinition.Vec2> {
    val count = maxOf(8, n)
    val r = minOf(0.90f, 0.40f + count.toFloat() * 0.035f)
    return (0 until count).map { i ->
        val a = i.toFloat() / count.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        vec2(r * cos(a), r * sin(a))
    }
}

private fun hexagonLayout(n: Int): List<LevelDefinition.Vec2> {
    val pts = mutableListOf(vec2(0f, 0f))
    var ring = 1
    while (pts.size < n) {
        val count = ring * 6
        val r = ring.toFloat() * minOf(0.35f, 0.95f / (ring + 1).toFloat())
        for (i in 0 until count) {
            val a = i.toFloat() / count.toFloat() * 2f * PI.toFloat() + ring.toFloat() * 0.15f
            pts.add(vec2(r * cos(a), r * sin(a)))
        }
        ring++
    }
    return pts
}

private fun diamondLayout(n: Int): List<LevelDefinition.Vec2> {
    var half = maxOf(2, ceil(sqrt(n.toFloat())).toInt())
    if (half % 2 == 0) half++
    val s = minOf(0.38f, 1.6f / half.toFloat())
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (rowIdx in 0 until (half * 2 - 1)) {
        val dist = abs(rowIdx - (half - 1))
        val count = half - dist
        val y = (rowIdx.toFloat() - (half - 1).toFloat()) * s
        val ox = -(count - 1).toFloat() / 2f * s
        for (col in 0 until count) {
            pts.add(vec2(ox + col.toFloat() * s, y))
        }
    }
    return pts
}

private fun crossLayout(n: Int): List<LevelDefinition.Vec2> {
    val arm = maxOf(2, n / 5)
    val s = minOf(0.40f, 1.6f / (arm * 2).toFloat())
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (i in -arm..arm) {
        pts.add(vec2(0f, i.toFloat() * s))
    }
    for (i in -arm..arm) {
        if (i != 0) {
            pts.add(vec2(i.toFloat() * s, 0f))
        }
    }
    val fill = maxOf(1, arm - 1)
    for (dx in 1..fill) {
        for (dy in 1..fill) {
            for (sx in listOf(-1f, 1f)) {
                for (sy in listOf(-1f, 1f)) {
                    pts.add(vec2(sx * dx.toFloat() * s, sy * dy.toFloat() * s))
                }
            }
        }
    }
    return pts
}

private fun ringsLayout(n: Int): List<LevelDefinition.Vec2> {
    val ringCount = if (n <= 14) 2 else if (n <= 22) 3 else 4
    val pts = mutableListOf<LevelDefinition.Vec2>()
    val perRing = maxOf(4, (n - if (ringCount > 2) 1 else 0) / ringCount)
    if (ringCount > 2) {
        pts.add(vec2(0f, 0f))
    }
    for (ring in 0 until ringCount) {
        val r = 0.25f + ring.toFloat() * (0.65f / (ringCount - 1).toFloat())
        val count = maxOf(4, perRing + ring * 2)
        val offset = ring.toFloat() * PI.toFloat() / count.toFloat()
        for (i in 0 until count) {
            val a = i.toFloat() / count.toFloat() * 2f * PI.toFloat() + offset
            pts.add(vec2(r * cos(a), r * sin(a)))
        }
    }
    return pts
}

private fun triangleLayout(n: Int): List<LevelDefinition.Vec2> {
    var rows = 4
    while (rows * (rows + 1) / 2 < n) rows++
    val s = minOf(0.38f, 1.6f / rows.toFloat())
    val pts = mutableListOf<LevelDefinition.Vec2>()
    val cy = (rows - 1).toFloat() / 2f
    for (row in 0 until rows) {
        val count = row + 1
        val ox = -(count - 1).toFloat() / 2f * s
        val y = (row.toFloat() - cy) * s * 0.866f
        for (col in 0 until count) {
            pts.add(vec2(ox + col.toFloat() * s, y))
        }
    }
    return pts
}

private fun starLayout(n: Int): List<LevelDefinition.Vec2> {
    val points = if (n <= 14) 5 else if (n <= 22) 6 else 8
    val pts = mutableListOf(vec2(0f, 0f))
    for (i in 0 until points) {
        val outerAngle = i.toFloat() / points.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(0.85f * cos(outerAngle), 0.85f * sin(outerAngle)))
        val innerAngle = outerAngle + PI.toFloat() / points.toFloat()
        pts.add(vec2(0.40f * cos(innerAngle), 0.40f * sin(innerAngle)))
    }
    if (pts.size < n) {
        val extra = n - pts.size
        for (i in 0 until extra) {
            val a = i.toFloat() / extra.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f + 0.3f
            pts.add(vec2(0.62f * cos(a), 0.62f * sin(a)))
        }
    }
    return pts
}

private fun honeycombLayout(n: Int): List<LevelDefinition.Vec2> {
    var halfRows = 2
    while ((2 * halfRows + 1) * (halfRows + 2) < n) halfRows++
    val s = minOf(0.36f, 1.6f / (2 * halfRows + 1).toFloat())
    val h = s * 0.866f
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (row in -halfRows..halfRows) {
        val cols = if (abs(row) % 2 == 0) (halfRows + 2) else (halfRows + 1)
        val offset = if (abs(row) % 2 == 0) 0f else s * 0.5f
        val ox = -(cols - 1).toFloat() / 2f * s + offset
        for (col in 0 until cols) {
            pts.add(vec2(ox + col.toFloat() * s, row.toFloat() * h))
        }
    }
    return pts
}

private fun spiralLayout(n: Int): List<LevelDefinition.Vec2> {
    val count = maxOf(10, n - 1)
    val pts = mutableListOf(vec2(0f, 0f))
    val turns = 2.0f + count.toFloat() * 0.12f
    for (i in 1..count) {
        val t = i.toFloat() / count.toFloat()
        val r = 0.12f + t * 0.78f
        val a = t * turns * PI.toFloat()
        pts.add(vec2(r * cos(a), r * sin(a)))
    }
    return pts
}

private fun doubleGridLayout(n: Int): List<LevelDefinition.Vec2> {
    val half = maxOf(6, n / 2)
    val cols = maxOf(3, ceil(sqrt(half.toFloat() * 1.5f)).toInt())
    val rows = maxOf(2, (half + cols - 1) / cols)
    val s = minOf(0.38f, 1.5f / maxOf(cols, rows * 2 + 1).toFloat())
    val gap = s * 0.6f
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (gy in listOf(-1f, 1f)) {
        val cy = gy * ((rows - 1).toFloat() / 2f * s + gap)
        for (row in 0 until rows) {
            for (col in 0 until cols) {
                val x = -(cols - 1).toFloat() / 2f * s + col.toFloat() * s
                val y = cy + (row.toFloat() - (rows - 1).toFloat() / 2f) * s
                pts.add(vec2(x, y))
            }
        }
    }
    return pts
}

private fun scatteredLayout(n: Int): List<LevelDefinition.Vec2> {
    val pts = mutableListOf<LevelDefinition.Vec2>()
    val cols = maxOf(3, ceil(sqrt(n.toFloat() * 1.3f)).toInt())
    val rows = maxOf(3, (n + cols - 1) / cols)
    val sx = 1.6f / cols.toFloat()
    val sy = 1.6f / rows.toFloat()
    var idx = 0
    for (row in 0 until rows) {
        for (col in 0 until cols) {
            if (idx >= n) break
            val jitterX = ((idx * 7 + 13) % 17).toFloat() / 17.0f - 0.5f
            val jitterY = ((idx * 11 + 3) % 13).toFloat() / 13.0f - 0.5f
            val x = -0.8f + col.toFloat() * sx + jitterX * sx * 0.35f
            val y = -0.8f + row.toFloat() * sy + jitterY * sy * 0.35f
            pts.add(vec2(x, y))
            idx++
        }
    }
    return pts
}

private fun columnsLayout(n: Int): List<LevelDefinition.Vec2> {
    val colCount = if (n <= 12) 2 else if (n <= 20) 3 else 4
    val rowCount = maxOf(3, (n + colCount - 1) / colCount)
    val s = minOf(0.35f, 1.5f / (rowCount - 1).toFloat())
    val xSpread = minOf(0.55f, 0.30f + colCount.toFloat() * 0.12f)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (col in 0 until colCount) {
        val x = -xSpread + col.toFloat() * (2f * xSpread / (colCount - 1).toFloat())
        for (row in 0 until rowCount) {
            val y = (row.toFloat() - (rowCount - 1).toFloat() / 2f) * s
            pts.add(vec2(x, y))
        }
    }
    return pts
}

private fun centerPlusRingLayout(n: Int): List<LevelDefinition.Vec2> {
    val centerCount = if (n <= 14) 1 else if (n <= 20) 2 else 3
    val pts = mutableListOf<LevelDefinition.Vec2>()
    if (centerCount == 1) {
        pts.add(vec2(0f, 0f))
    } else {
        val r = 0.12f
        for (i in 0 until centerCount) {
            val a = i.toFloat() / centerCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
            pts.add(vec2(r * cos(a), r * sin(a)))
        }
    }
    val ringCount = n - centerCount
    val r = minOf(0.85f, 0.55f + ringCount.toFloat() * 0.02f)
    for (i in 0 until ringCount) {
        val a = i.toFloat() / ringCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(r * cos(a), r * sin(a)))
    }
    return pts
}

private fun rotatedSquareLayout(n: Int): List<LevelDefinition.Vec2> {
    val rings = maxOf(2, (n + 3) / 4)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (ring in 1..rings) {
        val r = ring.toFloat() / rings.toFloat() * 0.85f
        val perSide = maxOf(1, ring)
        val count = perSide * 4
        for (i in 0 until count) {
            val a = i.toFloat() / count.toFloat() * 2f * PI.toFloat() + PI.toFloat() / 4f
            pts.add(vec2(r * cos(a), r * sin(a)))
        }
    }
    return pts
}

private fun squareInCircleLayout(n: Int): List<LevelDefinition.Vec2> {
    val sqSide = maxOf(2, n / 4)
    val ringCount = maxOf(8, n - sqSide * 4)
    val sq = minOf(0.50f, 0.30f + sqSide.toFloat() * 0.05f)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    val s = 2f * sq / (sqSide - 1).toFloat()
    for (row in 0 until sqSide) {
        for (col in 0 until sqSide) {
            if (row == 0 || row == sqSide - 1 || col == 0 || col == sqSide - 1) {
                pts.add(vec2(-sq + col.toFloat() * s, -sq + row.toFloat() * s))
            }
        }
    }
    val r = minOf(0.90f, sq + 0.35f)
    for (i in 0 until ringCount) {
        val a = i.toFloat() / ringCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(r * cos(a), r * sin(a)))
    }
    return pts
}

private fun arcLayout(n: Int): List<LevelDefinition.Vec2> {
    val count = maxOf(8, n)
    val r = minOf(0.85f, 0.55f + count.toFloat() * 0.02f)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (i in 0 until count) {
        val a = i.toFloat() / (count - 1).toFloat() * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(r * cos(a), r * sin(a)))
    }
    return pts
}

private fun concentricSquaresLayout(n: Int): List<LevelDefinition.Vec2> {
    val rings = maxOf(2, n / 6)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (ring in 1..rings) {
        val r = ring.toFloat() / rings.toFloat() * 0.85f
        val count = maxOf(4, 4 + (ring - 1) * 4)
        for (i in 0 until count) {
            val a = i.toFloat() / count.toFloat() * 2f * PI.toFloat() + PI.toFloat() / 4f
            pts.add(vec2(r * cos(a), r * sin(a)))
        }
    }
    return pts
}

private fun triangleInCircleLayout(n: Int): List<LevelDefinition.Vec2> {
    val innerCount = maxOf(3, n / 4)
    val outerCount = maxOf(6, n - innerCount)
    val rInner = minOf(0.40f, 0.20f + innerCount.toFloat() * 0.04f)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (i in 0 until innerCount) {
        val a = i.toFloat() / innerCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(rInner * cos(a), rInner * sin(a)))
    }
    val rOuter = minOf(0.85f, rInner + 0.35f)
    for (i in 0 until outerCount) {
        val a = i.toFloat() / outerCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(rOuter * cos(a), rOuter * sin(a)))
    }
    return pts
}

private fun crescentLayout(n: Int): List<LevelDefinition.Vec2> {
    val outerCount = maxOf(5, (n * 3 + 2) / 5)
    val innerCount = maxOf(4, n - outerCount)
    val rOut = 0.80f
    val rIn = 0.50f
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (i in 0 until outerCount) {
        val a = i.toFloat() / (outerCount - 1).toFloat() * PI.toFloat() * 0.9f - PI.toFloat() / 2f - PI.toFloat() / 8f
        pts.add(vec2(rOut * cos(a), rOut * sin(a)))
    }
    for (i in 0 until innerCount) {
        val a = i.toFloat() / (innerCount - 1).toFloat() * PI.toFloat() * 0.75f - PI.toFloat() / 2f + PI.toFloat() / 10f
        pts.add(vec2(rIn * cos(a) + 0.22f, rIn * sin(a)))
    }
    return pts
}

private fun letterHLayout(n: Int): List<LevelDefinition.Vec2> {
    val colRows = maxOf(3, (n - 1) / 2)
    val s = minOf(0.32f, 1.5f / (colRows - 1).toFloat())
    val xL = -0.45f
    val xR = 0.45f
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (row in 0 until colRows) {
        val y = (row.toFloat() - (colRows - 1).toFloat() / 2f) * s
        pts.add(vec2(xL, y))
        pts.add(vec2(xR, y))
    }
    val barCount = maxOf(1, n - colRows * 2)
    val barS = 0.9f / (barCount + 1).toFloat()
    for (i in 1..barCount) {
        pts.add(vec2(xL + i.toFloat() * barS, 0f))
    }
    return pts
}

private fun letterTLayout(n: Int): List<LevelDefinition.Vec2> {
    val topCount = maxOf(3, n / 3)
    val stemCount = maxOf(3, n - topCount)
    val topS = minOf(0.30f, 1.5f / (topCount - 1).toFloat())
    val stemS = minOf(0.30f, 1.3f / stemCount.toFloat())
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (col in 0 until topCount) {
        val x = (col.toFloat() - (topCount - 1).toFloat() / 2f) * topS
        pts.add(vec2(x, 0.65f))
    }
    for (row in 0 until stemCount) {
        pts.add(vec2(0f, 0.65f - (row + 1).toFloat() * stemS))
    }
    return pts
}

private fun arrowLayout(n: Int): List<LevelDefinition.Vec2> {
    val perSide = maxOf(3, n / 2)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (i in 0 until perSide) {
        val t = i.toFloat() / (perSide - 1).toFloat()
        pts.add(vec2(-0.60f * (1f - t), -0.60f + t * 1.2f))
        pts.add(vec2(0.60f * (1f - t), -0.60f + t * 1.2f))
    }
    return pts
}

private fun zigzagLayout(n: Int): List<LevelDefinition.Vec2> {
    val count = maxOf(8, n)
    val s = minOf(0.28f, 1.5f / (count - 1).toFloat())
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (i in 0 until count) {
        val x = ((i % 2).toFloat() - 0.5f) * 0.65f
        val y = (i.toFloat() - (count - 1).toFloat() / 2f) * s
        pts.add(vec2(x, y))
    }
    return pts
}

private fun cloverLayout(n: Int): List<LevelDefinition.Vec2> {
    val petals = if (n <= 14) 4 else if (n <= 22) 5 else 6
    val perPetal = maxOf(3, n / petals)
    val r = 0.22f
    val bigR = 0.55f
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (petal in 0 until petals) {
        val ca = petal.toFloat() / petals.toFloat() * 2f * PI.toFloat()
        val cx = bigR * cos(ca)
        val cy = bigR * sin(ca)
        for (i in 0 until perPetal) {
            val a = i.toFloat() / perPetal.toFloat() * 2f * PI.toFloat() + ca
            pts.add(vec2(cx + r * cos(a), cy + r * sin(a)))
        }
    }
    return pts
}

private fun clustersLayout(n: Int): List<LevelDefinition.Vec2> {
    val clusterCount = if (n <= 14) 3 else if (n <= 22) 4 else 5
    val perCluster = maxOf(3, n / clusterCount)
    val bigR = 0.65f
    val r = minOf(0.18f, 0.10f + perCluster.toFloat() * 0.02f)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (c in 0 until clusterCount) {
        val ca = c.toFloat() / clusterCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        val cx = bigR * cos(ca)
        val cy = bigR * sin(ca)
        for (i in 0 until perCluster) {
            val a = i.toFloat() / perCluster.toFloat() * 2f * PI.toFloat()
            pts.add(vec2(cx + r * cos(a), cy + r * sin(a)))
        }
    }
    return pts
}

private fun threeColumnsLayout(n: Int): List<LevelDefinition.Vec2> {
    val rowCount = maxOf(3, (n + 2) / 3)
    val s = minOf(0.35f, 1.5f / (rowCount - 1).toFloat())
    val xPos = listOf(-0.55f, 0f, 0.55f)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (x in xPos) {
        for (row in 0 until rowCount) {
            val y = (row.toFloat() - (rowCount - 1).toFloat() / 2f) * s
            pts.add(vec2(x, y))
        }
    }
    return pts
}

private fun ringWithSpokesLayout(n: Int): List<LevelDefinition.Vec2> {
    val spokes = maxOf(3, n / 4)
    val ringCount = maxOf(6, n - spokes - 1)
    val pts = mutableListOf(vec2(0f, 0f))
    for (i in 0 until spokes) {
        val a = i.toFloat() / spokes.toFloat() * 2f * PI.toFloat()
        pts.add(vec2(0.35f * cos(a), 0.35f * sin(a)))
    }
    for (i in 0 until ringCount) {
        val a = i.toFloat() / ringCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(0.78f * cos(a), 0.78f * sin(a)))
    }
    return pts
}

private fun diamondOutlineLayout(n: Int): List<LevelDefinition.Vec2> {
    val rings = maxOf(2, n / 4)
    val pts = mutableListOf<LevelDefinition.Vec2>()
    for (ring in 1..rings) {
        val r = ring.toFloat() / rings.toFloat() * 0.78f
        val count = maxOf(4, ring * 4)
        for (i in 0 until count) {
            val a = i.toFloat() / count.toFloat() * 2f * PI.toFloat() + PI.toFloat() / 4f
            pts.add(vec2(r * cos(a), r * sin(a)))
        }
    }
    return pts
}

private fun bowTieLayout(n: Int): List<LevelDefinition.Vec2> {
    val perSide = maxOf(3, (n - 1) / 2)
    val pts = mutableListOf(vec2(0f, 0f))
    val r = 0.70f
    for (i in 0 until perSide) {
        val a = i.toFloat() / perSide.toFloat() * PI.toFloat() - PI.toFloat() / 2f
        pts.add(vec2(r * cos(a), r * sin(a)))
    }
    for (i in 0 until perSide) {
        val a = i.toFloat() / perSide.toFloat() * PI.toFloat() + PI.toFloat() / 2f
        pts.add(vec2(r * cos(a), r * sin(a)))
    }
    return pts
}
