package com.uzls.four.level

import kotlin.math.*

object LevelGenerator {

    // MARK: - Public

    fun generate(levelId: Int, boardElevation: Float = 0.12f): LevelDefinition {
        val rng = SeededRNG(seed = levelId.toULong() * 2654435761UL)

        val diff = difficulty(levelId)
        val use3D = levelId >= 10 && levelId % 3 == 0
        val boardLayout: BoardLayout? = if (use3D) pickBoardLayout(levelId) else null

        val holes: List<LevelDefinition.Vec2>
        val boards: List<LevelDefinition.Board>?

        if (boardLayout != null) {
            val result = generateBoardLayout(boardLayout, minHoles(levelId, diff.ropeCount), boardElevation)
            holes = result.holes
            boards = result.boards
        } else {
            val layout = pickLayout(levelId)
            holes = layout.generate(minHoles(levelId, diff.ropeCount))
            boards = null
        }

        val maxRopes = maxOf(1, (holes.size - 3) / 2)
        val ropeCount = minOf(diff.ropeCount, maxRopes)

        val shortCount = minOf(diff.shortRopeCount, ropeCount - 1)
        val ropePairs = pickStructuredPairs(holes, ropeCount, shortCount, rng)
        val ropes = ropePairs.mapIndexed { i, pair ->
            LevelDefinition.Rope(
                startHole = pair.first,
                endHole = pair.second,
                color = colors[i % colors.size],
                radius = 0.038f
            )
        }

        val actions = buildActions(ropes, holes, diff.totalDrags, shortCount)

        return LevelDefinition(
            id = levelId,
            holeRadius = 0.08f,
            particlesPerRope = 32,
            holes = holes,
            ropes = ropes,
            hooks = null,
            actions = actions,
            boards = boards
        )
    }

    // MARK: - 3D Board layouts

    enum class BoardLayout {
        TWO_SIDES,
        BRIDGE,
        STAIRCASE,
        PLATFORM,
        VALLEY;
    }

    data class BoardLayoutResult(
        val holes: List<LevelDefinition.Vec2>,
        val boards: List<LevelDefinition.Board>
    )

    private fun pickBoardLayout(levelId: Int): BoardLayout {
        val layouts = BoardLayout.entries
        return layouts[(levelId / 3) % layouts.size]
    }

    // MARK: - Difficulty

    private data class Difficulty(
        val ropeCount: Int,
        val totalDrags: Int,
        val shortRopeCount: Int
    )

    private fun difficulty(levelId: Int): Difficulty = when (levelId) {
        in 1..2 -> Difficulty(ropeCount = 3, totalDrags = 3, shortRopeCount = 0)
        in 3..5 -> Difficulty(ropeCount = 4, totalDrags = 4, shortRopeCount = if (levelId >= 4) 2 else 0)
        in 6..9 -> Difficulty(ropeCount = 5, totalDrags = 6, shortRopeCount = 3)
        in 10..15 -> Difficulty(ropeCount = 6, totalDrags = 8, shortRopeCount = 4)
        in 16..25 -> Difficulty(ropeCount = 7, totalDrags = 10, shortRopeCount = 5)
        in 26..50 -> Difficulty(ropeCount = 8, totalDrags = 12 + (levelId - 25) / 5, shortRopeCount = 6)
        else -> Difficulty(
            ropeCount = minOf(10, 8 + (levelId - 50) / 25),
            totalDrags = minOf(22, 16 + (levelId - 50) / 10),
            shortRopeCount = minOf(8, 6 + (levelId - 50) / 30)
        )
    }

    private fun minHoles(levelId: Int, ropeCount: Int): Int {
        val base = ropeCount * 2 + 4
        return when (levelId) {
            in 1..5 -> maxOf(base, 10)
            in 6..15 -> maxOf(base, 14)
            in 16..30 -> maxOf(base, 18)
            in 31..60 -> maxOf(base, 20)
            in 61..100 -> maxOf(base, 22)
            else -> maxOf(base, 24)
        }
    }

    // MARK: - Layout selection

    private fun pickLayout(levelId: Int): HoleLayout {
        val layouts = HoleLayout.entries
        return layouts[levelId % layouts.size]
    }

    // MARK: - Colors

    private val colors: List<LevelDefinition.Color> = listOf(
        LevelDefinition.Color(redChannel = 0.95f, greenChannel = 0.30f, blueChannel = 0.05f),
        LevelDefinition.Color(redChannel = 0.10f, greenChannel = 0.35f, blueChannel = 0.92f),
        LevelDefinition.Color(redChannel = 0.90f, greenChannel = 0.12f, blueChannel = 0.25f),
        LevelDefinition.Color(redChannel = 0.15f, greenChannel = 0.75f, blueChannel = 0.30f),
        LevelDefinition.Color(redChannel = 0.92f, greenChannel = 0.78f, blueChannel = 0.05f),
        LevelDefinition.Color(redChannel = 0.60f, greenChannel = 0.10f, blueChannel = 0.72f),
        LevelDefinition.Color(redChannel = 0.05f, greenChannel = 0.65f, blueChannel = 0.72f),
        LevelDefinition.Color(redChannel = 0.85f, greenChannel = 0.15f, blueChannel = 0.55f),
        LevelDefinition.Color(redChannel = 0.20f, greenChannel = 0.55f, blueChannel = 0.90f),
        LevelDefinition.Color(redChannel = 0.80f, greenChannel = 0.50f, blueChannel = 0.05f),
    )

    // MARK: - Rope pair selection

    private fun pickStructuredPairs(
        holes: List<LevelDefinition.Vec2>, count: Int, shortCount: Int, rng: SeededRNG
    ): List<Pair<Int, Int>> {
        if (holes.size < 4) return emptyList()

        val cx = holes.map { it.xPosition }.sum() / holes.size.toFloat()
        val cy = holes.map { it.yPosition }.sum() / holes.size.toFloat()

        val sorted = holes.mapIndexed { idx, hole ->
            IndexedAngle(idx, atan2(hole.yPosition - cy, hole.xPosition - cx))
        }.sortedBy { it.angle }

        val offset = rng.next(sorted.size)
        val n = sorted.size
        val half = n / 2
        val quarter = maxOf(1, n / 4)

        val pairs = mutableListOf<Pair<Int, Int>>()
        val usedHoles = mutableSetOf<Int>()

        if (shortCount > 0) {
            val sectorStart = offset % n
            val sectorLen = minOf(quarter + shortCount, n / 2)

            val sectorHoles = mutableListOf<Int>()
            for (k in 0 until sectorLen) {
                sectorHoles.add(sorted[(sectorStart + k) % n].idx)
            }

            val minShortDist = 0.3f
            for (i in sectorHoles.indices) {
                if (pairs.size >= shortCount) break
                val a = sectorHoles[i]
                if (a in usedHoles) continue
                val shift = maxOf(2, minOf(quarter / 2, sectorHoles.size / 3))
                for (delta in shift until sectorHoles.size) {
                    val b = sectorHoles[(i + delta) % sectorHoles.size]
                    if (b == a || b in usedHoles) continue
                    val dx = holes[a].xPosition - holes[b].xPosition
                    val dy = holes[a].yPosition - holes[b].yPosition
                    if (sqrt(dx * dx + dy * dy) < minShortDist) continue
                    pairs.add(Pair(a, b))
                    usedHoles.add(a)
                    usedHoles.add(b)
                    break
                }
            }
        }

        for (i in 0 until count) {
            if (pairs.size >= count) break
            val aIdx = (i * 2 + offset) % n
            val a = sorted[aIdx].idx
            if (a in usedHoles) continue

            val shift = if (i % 2 == 0) 1 else -1
            val bIdx = ((aIdx + half + shift) % n + n) % n
            val b = sorted[bIdx].idx

            if (b != a && b !in usedHoles) {
                pairs.add(Pair(a, b))
                usedHoles.add(a)
                usedHoles.add(b)
            }
        }

        if (pairs.size < count) {
            for (i in 0 until n) {
                if (pairs.size >= count) break
                val a = sorted[(i + offset) % n].idx
                if (a in usedHoles) continue
                for (j in (i + 1) until n) {
                    val b = sorted[(j + offset) % n].idx
                    if (b in usedHoles) continue
                    val dx = holes[a].xPosition - holes[b].xPosition
                    val dy = holes[a].yPosition - holes[b].yPosition
                    if (sqrt(dx * dx + dy * dy) > 0.5f) {
                        pairs.add(Pair(a, b))
                        usedHoles.add(a)
                        usedHoles.add(b)
                        break
                    }
                }
            }
        }

        return pairs
    }

    private data class IndexedAngle(val idx: Int, val angle: Float)

    // MARK: - Tangle generation

    private fun buildActions(
        ropes: List<LevelDefinition.Rope>,
        holes: List<LevelDefinition.Vec2>,
        totalDrags: Int,
        shortCount: Int
    ): List<LevelDefinition.Action> {
        val actions = mutableListOf<LevelDefinition.Action>()
        val holeX = FloatArray(holes.size) { holes[it].xPosition }
        val holeY = FloatArray(holes.size) { holes[it].yPosition }

        var shortCenterX = 0f
        var shortCenterY = 0f
        var shortRadius = 0f
        if (shortCount > 0) {
            var sx = 0f; var sy = 0f; var cnt = 0f
            for (i in 0 until shortCount) {
                sx += holeX[ropes[i].startHole] + holeX[ropes[i].endHole]
                sy += holeY[ropes[i].startHole] + holeY[ropes[i].endHole]
                cnt += 2f
            }
            shortCenterX = sx / cnt; shortCenterY = sy / cnt
            var maxD = 0f
            for (i in 0 until shortCount) {
                for (h in intArrayOf(ropes[i].startHole, ropes[i].endHole)) {
                    val dx = holeX[h] - shortCenterX; val dy = holeY[h] - shortCenterY
                    maxD = maxOf(maxD, sqrt(dx * dx + dy * dy))
                }
            }
            shortRadius = maxD + 0.35f
        }

        for (i in ropes.indices) {
            actions.add(LevelDefinition.Action(type = "pin", ropeIndex = i, endIndex = 0, holeIndex = ropes[i].startHole))
            actions.add(LevelDefinition.Action(type = "pin", ropeIndex = i, endIndex = 1, holeIndex = ropes[i].endHole))
        }

        val endpoints = Array(ropes.size) { intArrayOf(ropes[it].startHole, ropes[it].endHole) }
        val usedHoles = mutableSetOf<Int>()
        for (ep in endpoints) {
            usedHoles.add(ep[0])
            usedHoles.add(ep[1])
        }

        fun tryDrag(ropeIdx: Int, endIdx: Int, targetRopeIdx: Int, unconstrained: Boolean = false): Boolean {
            val currentHole = endpoints[ropeIdx][endIdx]
            val anchorHole = endpoints[ropeIdx][1 - endIdx]
            val anchorX = holeX[anchorHole]
            val anchorY = holeY[anchorHole]
            val tSX = holeX[endpoints[targetRopeIdx][0]]
            val tSY = holeY[endpoints[targetRopeIdx][0]]
            val tEX = holeX[endpoints[targetRopeIdx][1]]
            val tEY = holeY[endpoints[targetRopeIdx][1]]
            val isShort = !unconstrained && ropeIdx < shortCount

            val otherUsed = mutableSetOf<Int>()
            for (ri in ropes.indices) {
                if (ri != ropeIdx) {
                    otherUsed.add(endpoints[ri][0])
                    otherUsed.add(endpoints[ri][1])
                }
            }

            var bestHole: Int? = null
            var bestScore = -Float.MAX_VALUE

            for (candidate in holes.indices) {
                if (candidate == currentHole || candidate == anchorHole) continue
                if (candidate in otherUsed) continue
                val cX = holeX[candidate]
                val cY = holeY[candidate]
                if (isShort) {
                    val dx = cX - shortCenterX; val dy = cY - shortCenterY
                    if (sqrt(dx * dx + dy * dy) > shortRadius) continue
                }
                if (segmentsCross(anchorX, anchorY, cX, cY, tSX, tSY, tEX, tEY)) {
                    val midX = (tSX + tEX) * 0.5f
                    val midY = (tSY + tEY) * 0.5f
                    val dx = cX - midX
                    val dy = cY - midY
                    val score = -sqrt(dx * dx + dy * dy)
                    if (score > bestScore) {
                        bestScore = score
                        bestHole = candidate
                    }
                }
            }

            val targetHole = bestHole ?: return false

            usedHoles.remove(currentHole)
            usedHoles.add(targetHole)
            endpoints[ropeIdx][endIdx] = targetHole
            actions.add(LevelDefinition.Action(type = "drag", ropeIndex = ropeIdx, endIndex = endIdx, holeIndex = targetHole))
            return true
        }

        if (shortCount >= 2) {
            for (d in 0 until shortCount) {
                val ropeIdx = d
                for (targetIdx in 0 until shortCount) {
                    if (targetIdx == ropeIdx) continue
                    val endIdx = d % 2
                    if (tryDrag(ropeIdx, endIdx, targetIdx)) break
                    if (tryDrag(ropeIdx, 1 - endIdx, targetIdx)) break
                }
            }
        }

        for (d in 0 until totalDrags) {
            val ropeIdx = d % ropes.size
            val targetRopeIdx = (ropeIdx + 1 + d / ropes.size) % ropes.size
            if (targetRopeIdx == ropeIdx) continue
            val endIdx = (d / ropes.size) % 2
            if (!tryDrag(ropeIdx, endIdx, targetRopeIdx)) {
                tryDrag(ropeIdx, 1 - endIdx, targetRopeIdx)
            }
        }

        for (attempt in 0 until ropes.size * 8) {
            val crossings = ropeCrossings(endpoints, holeX, holeY)
            val isolated = (ropes.indices).filter { crossings[it] == 0 }
            if (isolated.isEmpty()) break

            val ropeIdx = isolated[attempt % isolated.size]
            var fixed = false
            for (other in ropes.indices) {
                if (other == ropeIdx || fixed) continue
                for (endIdx in 0..1) {
                    if (fixed) continue
                    fixed = tryDrag(ropeIdx, endIdx, other, unconstrained = true)
                }
            }
            if (!fixed) {
                for (other in ropes.indices) {
                    if (other == ropeIdx || fixed) continue
                    for (endIdx in 0..1) {
                        if (fixed) continue
                        fixed = tryDrag(other, endIdx, ropeIdx, unconstrained = true)
                    }
                }
            }
            if (!fixed) {
                val allUsed = mutableSetOf<Int>()
                for (ep in endpoints) { allUsed.add(ep[0]); allUsed.add(ep[1]) }
                val freeHoles = holes.indices.filter { it !in allUsed }
                for (other in ropes.indices) {
                    if (other == ropeIdx || fixed) continue
                    val oSX = holeX[endpoints[other][0]]; val oSY = holeY[endpoints[other][0]]
                    val oEX = holeX[endpoints[other][1]]; val oEY = holeY[endpoints[other][1]]
                    for (h0 in freeHoles) {
                        if (fixed) break
                        for (h1 in freeHoles) {
                            if (h1 == h0 || fixed) continue
                            if (segmentsCross(holeX[h0], holeY[h0], holeX[h1], holeY[h1], oSX, oSY, oEX, oEY)) {
                                val old0 = endpoints[ropeIdx][0]; val old1 = endpoints[ropeIdx][1]
                                usedHoles.remove(old0); usedHoles.remove(old1)
                                endpoints[ropeIdx][0] = h0; endpoints[ropeIdx][1] = h1
                                usedHoles.add(h0); usedHoles.add(h1)
                                actions.add(LevelDefinition.Action(type = "drag", ropeIndex = ropeIdx, endIndex = 0, holeIndex = h0))
                                actions.add(LevelDefinition.Action(type = "drag", ropeIndex = ropeIdx, endIndex = 1, holeIndex = h1))
                                fixed = true
                            }
                        }
                    }
                }
            }
        }

        return actions
    }

    // MARK: - Geometry helpers

    private fun ropeCrossings(endpoints: Array<IntArray>, holeX: FloatArray, holeY: FloatArray): IntArray {
        val n = endpoints.size
        val counts = IntArray(n)
        for (i in 0 until n) {
            val a0x = holeX[endpoints[i][0]]; val a0y = holeY[endpoints[i][0]]
            val a1x = holeX[endpoints[i][1]]; val a1y = holeY[endpoints[i][1]]
            for (j in (i + 1) until n) {
                val b0x = holeX[endpoints[j][0]]; val b0y = holeY[endpoints[j][0]]
                val b1x = holeX[endpoints[j][1]]; val b1y = holeY[endpoints[j][1]]
                if (segmentsCross(a0x, a0y, a1x, a1y, b0x, b0y, b1x, b1y)) {
                    counts[i]++
                    counts[j]++
                }
            }
        }
        return counts
    }

    private fun segmentsCross(
        a0x: Float, a0y: Float, a1x: Float, a1y: Float,
        b0x: Float, b0y: Float, b1x: Float, b1y: Float
    ): Boolean {
        val d1x = a1x - a0x
        val d1y = a1y - a0y
        val d2x = b1x - b0x
        val d2y = b1y - b0y
        val cross = d1x * d2y - d1y * d2x
        if (abs(cross) < 1e-9f) return false
        val dx = b0x - a0x
        val dy = b0y - a0y
        val t = (dx * d2y - dy * d2x) / cross
        val u = (dx * d1y - dy * d1x) / cross
        return t > 0.01f && t < 0.99f && u > 0.01f && u < 0.99f
    }

    // MARK: - 3D Board layout implementations

    fun generateBoardLayout(layout: BoardLayout, n: Int, elevation: Float = 0.12f): BoardLayoutResult {
        return when (layout) {
            BoardLayout.TWO_SIDES -> twoSidesLayout(n, elevation)
            BoardLayout.BRIDGE -> bridgeLayout(n, elevation)
            BoardLayout.STAIRCASE -> staircaseLayout(n, elevation)
            BoardLayout.PLATFORM -> platformLayout(n, elevation)
            BoardLayout.VALLEY -> valleyLayout(n, elevation)
        }
    }

    private fun twoSidesLayout(n: Int, elevation: Float): BoardLayoutResult {
        val boardW = 0.55f
        val boardH = 1.4f
        val gap = 0.35f

        val leftBoard = LevelDefinition.Board(centerX = -(gap + boardW * 0.5f), centerY = 0f, width = boardW, height = boardH, elevation = elevation)
        val rightBoard = LevelDefinition.Board(centerX = gap + boardW * 0.5f, centerY = 0f, width = boardW, height = boardH, elevation = elevation)

        val centerHoles = maxOf(4, n / 3)
        val sideHoles = maxOf(3, (n - centerHoles) / 2)
        val s = 0.35f

        val holes = mutableListOf<LevelDefinition.Vec2>()

        val centerCols = maxOf(2, ceil(sqrt(centerHoles.toFloat() * 1.5f)).toInt())
        val centerRows = maxOf(2, (centerHoles + centerCols - 1) / centerCols)
        val cs = minOf(s, gap * 1.5f / centerCols.toFloat())
        for (row in 0 until centerRows) {
            for (col in 0 until centerCols) {
                val x = (col.toFloat() - (centerCols - 1).toFloat() / 2f) * cs
                val y = (row.toFloat() - (centerRows - 1).toFloat() / 2f) * cs
                holes.add(LevelDefinition.Vec2(xPosition = x, yPosition = y))
            }
        }

        val sideRows = maxOf(2, sideHoles)
        val ss = minOf(s, boardH * 0.8f / (sideRows - 1).toFloat())
        for (i in 0 until sideRows) {
            val y = (i.toFloat() - (sideRows - 1).toFloat() / 2f) * ss
            holes.add(LevelDefinition.Vec2(xPosition = leftBoard.centerX, yPosition = y, zPosition = elevation))
        }
        for (i in 0 until sideRows) {
            val y = (i.toFloat() - (sideRows - 1).toFloat() / 2f) * ss
            holes.add(LevelDefinition.Vec2(xPosition = rightBoard.centerX, yPosition = y, zPosition = elevation))
        }

        return BoardLayoutResult(holes = holes, boards = listOf(leftBoard, rightBoard))
    }

    private fun bridgeLayout(n: Int, elevation: Float): BoardLayoutResult {
        val boardW = 1.8f
        val boardH = 0.45f

        val bridge = LevelDefinition.Board(centerX = 0f, centerY = 0f, width = boardW, height = boardH, elevation = elevation)

        val topHoles = maxOf(3, n / 3)
        val bottomHoles = maxOf(3, n / 3)
        val bridgeHoles = maxOf(3, n - topHoles - bottomHoles)
        val s = 0.35f

        val holes = mutableListOf<LevelDefinition.Vec2>()

        val cols = maxOf(2, ceil(sqrt(topHoles.toFloat() * 2f)).toInt())
        val rows = maxOf(2, (topHoles + cols - 1) / cols)
        val ts = minOf(s, 1.4f / cols.toFloat())
        for (row in 0 until rows) {
            for (col in 0 until cols) {
                val x = (col.toFloat() - (cols - 1).toFloat() / 2f) * ts
                val y = boardH * 0.5f + 0.25f + row.toFloat() * ts
                holes.add(LevelDefinition.Vec2(xPosition = x, yPosition = y))
            }
        }

        for (row in 0 until rows) {
            for (col in 0 until cols) {
                if (holes.size >= topHoles + bottomHoles) break
                val x = (col.toFloat() - (cols - 1).toFloat() / 2f) * ts
                val y = -(boardH * 0.5f + 0.25f + row.toFloat() * ts)
                holes.add(LevelDefinition.Vec2(xPosition = x, yPosition = y))
            }
        }

        val bCols = maxOf(2, bridgeHoles)
        val bs = minOf(s, boardW * 0.7f / (bCols - 1).toFloat())
        for (i in 0 until bCols) {
            val x = (i.toFloat() - (bCols - 1).toFloat() / 2f) * bs
            holes.add(LevelDefinition.Vec2(xPosition = x, yPosition = 0f, zPosition = elevation))
        }

        return BoardLayoutResult(holes = holes, boards = listOf(bridge))
    }

    private fun staircaseLayout(n: Int, elevation: Float): BoardLayoutResult {
        val steps = 3
        val stepW = 0.55f
        val stepH = 1.2f
        val stepGap = 0.08f

        val boards = mutableListOf<LevelDefinition.Board>()
        val holes = mutableListOf<LevelDefinition.Vec2>()
        val holesPerStep = maxOf(3, n / (steps + 1))
        val s = 0.32f

        val floorHoles = maxOf(3, n - holesPerStep * steps)
        val floorX = -(steps.toFloat() * (stepW + stepGap)) * 0.5f - 0.3f
        val floorRows = maxOf(2, floorHoles)
        val fs = minOf(s, stepH * 0.8f / (floorRows - 1).toFloat())
        for (i in 0 until floorRows) {
            val y = (i.toFloat() - (floorRows - 1).toFloat() / 2f) * fs
            holes.add(LevelDefinition.Vec2(xPosition = floorX, yPosition = y))
        }

        for (step in 0 until steps) {
            val elev = (step + 1).toFloat() * elevation
            val cx = step.toFloat() * (stepW + stepGap) - (steps - 1).toFloat() * (stepW + stepGap) * 0.5f + stepW * 0.5f
            val board = LevelDefinition.Board(centerX = cx, centerY = 0f, width = stepW, height = stepH, elevation = elev)
            boards.add(board)

            val stepRows = maxOf(2, holesPerStep)
            val ss = minOf(s, stepH * 0.7f / (stepRows - 1).toFloat())
            for (i in 0 until stepRows) {
                val y = (i.toFloat() - (stepRows - 1).toFloat() / 2f) * ss
                holes.add(LevelDefinition.Vec2(xPosition = cx, yPosition = y, zPosition = elev))
            }
        }

        return BoardLayoutResult(holes = holes, boards = boards)
    }

    private fun platformLayout(n: Int, elevation: Float): BoardLayoutResult {
        val boardW = 0.8f
        val boardH = 0.8f

        val platform = LevelDefinition.Board(centerX = 0f, centerY = 0f, width = boardW, height = boardH, elevation = elevation)

        val platformHoles = maxOf(4, n / 3)
        val groundHoles = n - platformHoles
        val s = 0.35f

        val holes = mutableListOf<LevelDefinition.Vec2>()

        val pCols = maxOf(2, ceil(sqrt(platformHoles.toFloat())).toInt())
        val pRows = maxOf(2, (platformHoles + pCols - 1) / pCols)
        val ps = minOf(s, boardW * 0.7f / (maxOf(pCols, pRows) - 1).toFloat())
        for (row in 0 until pRows) {
            for (col in 0 until pCols) {
                val x = (col.toFloat() - (pCols - 1).toFloat() / 2f) * ps
                val y = (row.toFloat() - (pRows - 1).toFloat() / 2f) * ps
                holes.add(LevelDefinition.Vec2(xPosition = x, yPosition = y, zPosition = elevation))
            }
        }

        val ringR = boardW * 0.5f + 0.35f
        val ringCount = maxOf(8, groundHoles)
        for (i in 0 until ringCount) {
            val a = i.toFloat() / ringCount.toFloat() * 2f * PI.toFloat() - PI.toFloat() / 2f
            holes.add(LevelDefinition.Vec2(xPosition = ringR * cos(a), yPosition = ringR * sin(a)))
        }

        return BoardLayoutResult(holes = holes, boards = listOf(platform))
    }

    private fun valleyLayout(n: Int, elevation: Float): BoardLayoutResult {
        val boardW = 0.50f
        val boardH = 1.6f

        val leftBoard = LevelDefinition.Board(centerX = -0.65f, centerY = 0f, width = boardW, height = boardH, elevation = elevation)
        val rightBoard = LevelDefinition.Board(centerX = 0.65f, centerY = 0f, width = boardW, height = boardH, elevation = elevation)

        val sideHoles = maxOf(3, n / 3)
        val valleyHoles = maxOf(4, n - sideHoles * 2)
        val s = 0.32f

        val holes = mutableListOf<LevelDefinition.Vec2>()

        val sRows = maxOf(2, sideHoles)
        val ss = minOf(s, boardH * 0.7f / (sRows - 1).toFloat())
        for (i in 0 until sRows) {
            val y = (i.toFloat() - (sRows - 1).toFloat() / 2f) * ss
            holes.add(LevelDefinition.Vec2(xPosition = leftBoard.centerX, yPosition = y, zPosition = elevation))
        }
        for (i in 0 until sRows) {
            val y = (i.toFloat() - (sRows - 1).toFloat() / 2f) * ss
            holes.add(LevelDefinition.Vec2(xPosition = rightBoard.centerX, yPosition = y, zPosition = elevation))
        }

        val vCols = maxOf(2, ceil(sqrt(valleyHoles.toFloat() * 2f)).toInt())
        val vRows = maxOf(2, (valleyHoles + vCols - 1) / vCols)
        val vs = minOf(s, 0.5f / (maxOf(vCols, vRows) - 1).toFloat())
        for (row in 0 until vRows) {
            for (col in 0 until vCols) {
                val x = (col.toFloat() - (vCols - 1).toFloat() / 2f) * vs
                val y = (row.toFloat() - (vRows - 1).toFloat() / 2f) * vs
                holes.add(LevelDefinition.Vec2(xPosition = x, yPosition = y))
            }
        }

        return BoardLayoutResult(holes = holes, boards = listOf(leftBoard, rightBoard))
    }
}
