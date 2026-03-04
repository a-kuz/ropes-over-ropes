package com.uzls.four.renderer

import kotlin.math.*

data class HoleMesh(
    val vertices: FloatArray, // stride 6: pos(3)+normal(3)
    val vertexCount: Int,
    val indices: ShortArray,
    val indexCount: Int
)

object HoleMeshBuilder {

    fun build(
        segments: Int = 48,
        innerRadius: Float = 0.76f,
        outerRadius: Float = 1.0f,
        depth: Float = 1.25f,
        ringHeight: Float = 0.1f
    ): HoleMesh {
        val segCount = max(12, min(128, segments))
        val inner = max(0.05f, min(0.98f, innerRadius))
        val outer = max(inner + 0.01f, min(1.5f, outerRadius))
        val holeDepth = max(0.1f, depth)
        val rh = max(0.02f, min(0.2f, ringHeight))

        // Pre-calculate capacity
        // 6 sections: top ring, outer wall, inner wall, bottom ring, inner well, bottom cap
        // Sections 1-5: 4 verts + 6 indices each per segment
        // Section 6: 3 verts + 3 indices per segment
        val vertCap = segCount * (4 * 5 + 3) // ~23 verts per segment
        val idxCap = segCount * (6 * 5 + 3)  // ~33 indices per segment

        val vertices = FloatArray(vertCap * 6)
        val indices = ShortArray(idxCap)
        var vi = 0
        var ii = 0

        fun addVert(px: Float, py: Float, pz: Float, nx: Float, ny: Float, nz: Float) {
            val o = vi * 6
            vertices[o] = px; vertices[o + 1] = py; vertices[o + 2] = pz
            vertices[o + 3] = nx; vertices[o + 4] = ny; vertices[o + 5] = nz
            vi++
        }

        fun ringPoint(radius: Float, angle: Float, z: Float): Triple<Float, Float, Float> {
            return Triple(cos(angle) * radius, sin(angle) * radius, z)
        }

        // Section 1: Top ring (outer to inner)
        for (s in 0 until segCount) {
            val a0 = (s.toFloat() / segCount) * PI.toFloat() * 2f
            val a1 = ((s + 1).toFloat() / segCount) * PI.toFloat() * 2f

            val (o0x, o0y, o0z) = ringPoint(outer, a0, rh)
            val (o1x, o1y, o1z) = ringPoint(outer, a1, rh)
            val (i0x, i0y, i0z) = ringPoint(inner, a0, rh)
            val (i1x, i1y, i1z) = ringPoint(inner, a1, rh)

            val base = vi.toShort()
            addVert(o0x, o0y, o0z, 0f, 0f, 1f)
            addVert(o1x, o1y, o1z, 0f, 0f, 1f)
            addVert(i0x, i0y, i0z, 0f, 0f, 1f)
            addVert(i1x, i1y, i1z, 0f, 0f, 1f)

            indices[ii++] = base; indices[ii++] = (base + 2).toShort(); indices[ii++] = (base + 1).toShort()
            indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort(); indices[ii++] = (base + 3).toShort()
        }

        // Section 2: Outer wall
        for (s in 0 until segCount) {
            val a0 = (s.toFloat() / segCount) * PI.toFloat() * 2f
            val a1 = ((s + 1).toFloat() / segCount) * PI.toFloat() * 2f

            val (t0x, t0y, _) = ringPoint(outer, a0, rh)
            val (t1x, t1y, _) = ringPoint(outer, a1, rh)
            val (b0x, b0y, _) = ringPoint(outer, a0, 0f)
            val (b1x, b1y, _) = ringPoint(outer, a1, 0f)

            val n0x = -cos(a0); val n0y = -sin(a0)
            val n1x = -cos(a1); val n1y = -sin(a1)

            val base = vi.toShort()
            addVert(t0x, t0y, rh, n0x, n0y, 0f)
            addVert(b0x, b0y, 0f, n0x, n0y, 0f)
            addVert(t1x, t1y, rh, n1x, n1y, 0f)
            addVert(b1x, b1y, 0f, n1x, n1y, 0f)

            indices[ii++] = base; indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort()
            indices[ii++] = (base + 2).toShort(); indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 3).toShort()
        }

        // Section 3: Inner wall (top part)
        for (s in 0 until segCount) {
            val a0 = (s.toFloat() / segCount) * PI.toFloat() * 2f
            val a1 = ((s + 1).toFloat() / segCount) * PI.toFloat() * 2f

            val (t0x, t0y, _) = ringPoint(inner, a0, rh)
            val (t1x, t1y, _) = ringPoint(inner, a1, rh)
            val (b0x, b0y, _) = ringPoint(inner, a0, 0f)
            val (b1x, b1y, _) = ringPoint(inner, a1, 0f)

            val n0x = cos(a0); val n0y = sin(a0)
            val n1x = cos(a1); val n1y = sin(a1)

            val base = vi.toShort()
            addVert(t0x, t0y, rh, n0x, n0y, 0f)
            addVert(b0x, b0y, 0f, n0x, n0y, 0f)
            addVert(t1x, t1y, rh, n1x, n1y, 0f)
            addVert(b1x, b1y, 0f, n1x, n1y, 0f)

            indices[ii++] = base; indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort()
            indices[ii++] = (base + 2).toShort(); indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 3).toShort()
        }

        // Section 4: Bottom ring
        for (s in 0 until segCount) {
            val a0 = (s.toFloat() / segCount) * PI.toFloat() * 2f
            val a1 = ((s + 1).toFloat() / segCount) * PI.toFloat() * 2f

            val (o0x, o0y, _) = ringPoint(outer, a0, 0f)
            val (o1x, o1y, _) = ringPoint(outer, a1, 0f)
            val (i0x, i0y, _) = ringPoint(inner, a0, 0f)
            val (i1x, i1y, _) = ringPoint(inner, a1, 0f)

            val base = vi.toShort()
            addVert(o0x, o0y, 0f, 0f, 0f, -1f)
            addVert(o1x, o1y, 0f, 0f, 0f, -1f)
            addVert(i0x, i0y, 0f, 0f, 0f, -1f)
            addVert(i1x, i1y, 0f, 0f, 0f, -1f)

            indices[ii++] = base; indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort()
            indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 3).toShort(); indices[ii++] = (base + 2).toShort()
        }

        // Section 5: Inner well
        for (s in 0 until segCount) {
            val a0 = (s.toFloat() / segCount) * PI.toFloat() * 2f
            val a1 = ((s + 1).toFloat() / segCount) * PI.toFloat() * 2f

            val (t0x, t0y, _) = ringPoint(inner, a0, 0f)
            val (t1x, t1y, _) = ringPoint(inner, a1, 0f)
            val (b0x, b0y, _) = ringPoint(inner, a0, -holeDepth)
            val (b1x, b1y, _) = ringPoint(inner, a1, -holeDepth)

            val n0x = -cos(a0); val n0y = -sin(a0)
            val n1x = -cos(a1); val n1y = -sin(a1)

            val base = vi.toShort()
            addVert(t0x, t0y, 0f, n0x, n0y, 0f)
            addVert(b0x, b0y, -holeDepth, n0x, n0y, 0f)
            addVert(t1x, t1y, 0f, n1x, n1y, 0f)
            addVert(b1x, b1y, -holeDepth, n1x, n1y, 0f)

            indices[ii++] = base; indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort()
            indices[ii++] = (base + 2).toShort(); indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 3).toShort()
        }

        // Section 6: Bottom cap
        for (s in 0 until segCount) {
            val a0 = (s.toFloat() / segCount) * PI.toFloat() * 2f
            val a1 = ((s + 1).toFloat() / segCount) * PI.toFloat() * 2f

            val (p0x, p0y, _) = ringPoint(inner, a0, -holeDepth)
            val (p1x, p1y, _) = ringPoint(inner, a1, -holeDepth)

            val base = vi.toShort()
            addVert(0f, 0f, -holeDepth, 0f, 0f, 1f) // center
            addVert(p0x, p0y, -holeDepth, 0f, 0f, 1f)
            addVert(p1x, p1y, -holeDepth, 0f, 0f, 1f)

            indices[ii++] = base; indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort()
        }

        return HoleMesh(vertices, vi, indices, ii)
    }

    fun buildSquare(
        innerHalf: Float = 0.76f,
        outerHalf: Float = 1.0f,
        depth: Float = 1.25f,
        ringHeight: Float = 0.1f
    ): HoleMesh {
        val ih = max(0.05f, innerHalf)
        val oh = max(ih + 0.01f, outerHalf)
        val d = max(0.1f, depth)
        val rh = max(0.02f, min(0.2f, ringHeight))

        val vertCap = 200
        val idxCap = 300
        val vertices = FloatArray(vertCap * 6)
        val indices = ShortArray(idxCap)
        var vi = 0
        var ii = 0

        fun addVert(px: Float, py: Float, pz: Float, nx: Float, ny: Float, nz: Float) {
            val o = vi * 6
            vertices[o] = px; vertices[o + 1] = py; vertices[o + 2] = pz
            vertices[o + 3] = nx; vertices[o + 4] = ny; vertices[o + 5] = nz
            vi++
        }

        fun quad(
            p0x: Float, p0y: Float, p0z: Float,
            p1x: Float, p1y: Float, p1z: Float,
            p2x: Float, p2y: Float, p2z: Float,
            p3x: Float, p3y: Float, p3z: Float,
            nx: Float, ny: Float, nz: Float
        ) {
            val base = vi.toShort()
            addVert(p0x, p0y, p0z, nx, ny, nz)
            addVert(p1x, p1y, p1z, nx, ny, nz)
            addVert(p2x, p2y, p2z, nx, ny, nz)
            addVert(p3x, p3y, p3z, nx, ny, nz)
            indices[ii++] = base; indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort()
            indices[ii++] = base; indices[ii++] = (base + 2).toShort(); indices[ii++] = (base + 3).toShort()
        }

        val oc = arrayOf(
            floatArrayOf(oh, oh), floatArrayOf(-oh, oh),
            floatArrayOf(-oh, -oh), floatArrayOf(oh, -oh)
        )
        val ic = arrayOf(
            floatArrayOf(ih, ih), floatArrayOf(-ih, ih),
            floatArrayOf(-ih, -ih), floatArrayOf(ih, -ih)
        )

        // Top ring
        for (i in 0 until 4) {
            val j = (i + 1) % 4
            quad(oc[i][0], oc[i][1], rh, oc[j][0], oc[j][1], rh,
                ic[j][0], ic[j][1], rh, ic[i][0], ic[i][1], rh,
                0f, 0f, 1f)
        }

        // Outer walls
        val owN = arrayOf(
            floatArrayOf(0f, -1f, 0f), floatArrayOf(1f, 0f, 0f),
            floatArrayOf(0f, 1f, 0f), floatArrayOf(-1f, 0f, 0f)
        )
        val owE = arrayOf(intArrayOf(3, 0), intArrayOf(0, 1), intArrayOf(1, 2), intArrayOf(2, 3))
        for (ei in 0 until 4) {
            val a = owE[ei][0]; val b = owE[ei][1]
            quad(oc[a][0], oc[a][1], rh, oc[b][0], oc[b][1], rh,
                oc[b][0], oc[b][1], 0f, oc[a][0], oc[a][1], 0f,
                owN[ei][0], owN[ei][1], owN[ei][2])
        }

        // Inner walls (top)
        val iwN = arrayOf(
            floatArrayOf(0f, 1f, 0f), floatArrayOf(-1f, 0f, 0f),
            floatArrayOf(0f, -1f, 0f), floatArrayOf(1f, 0f, 0f)
        )
        val iwE = arrayOf(intArrayOf(3, 0), intArrayOf(0, 1), intArrayOf(1, 2), intArrayOf(2, 3))
        for (ei in 0 until 4) {
            val a = iwE[ei][0]; val b = iwE[ei][1]
            quad(ic[b][0], ic[b][1], rh, ic[a][0], ic[a][1], rh,
                ic[a][0], ic[a][1], 0f, ic[b][0], ic[b][1], 0f,
                iwN[ei][0], iwN[ei][1], iwN[ei][2])
        }

        // Bottom ring
        for (i in 0 until 4) {
            val j = (i + 1) % 4
            quad(oc[j][0], oc[j][1], 0f, oc[i][0], oc[i][1], 0f,
                ic[i][0], ic[i][1], 0f, ic[j][0], ic[j][1], 0f,
                0f, 0f, -1f)
        }

        // Inner well
        for (ei in 0 until 4) {
            val a = iwE[ei][0]; val b = iwE[ei][1]
            quad(ic[b][0], ic[b][1], 0f, ic[a][0], ic[a][1], 0f,
                ic[a][0], ic[a][1], -d, ic[b][0], ic[b][1], -d,
                iwN[ei][0], iwN[ei][1], iwN[ei][2])
        }

        // Bottom cap
        for (i in 0 until 4) {
            val j = (i + 1) % 4
            val base = vi.toShort()
            addVert(0f, 0f, -d, 0f, 0f, 1f)
            addVert(ic[i][0], ic[i][1], -d, 0f, 0f, 1f)
            addVert(ic[j][0], ic[j][1], -d, 0f, 0f, 1f)
            indices[ii++] = base; indices[ii++] = (base + 1).toShort(); indices[ii++] = (base + 2).toShort()
        }

        return HoleMesh(vertices, vi, indices, ii)
    }
}
