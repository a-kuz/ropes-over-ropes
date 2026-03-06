package com.uzls.four.renderer

import com.uzls.four.simulation.*
import kotlin.math.*

data class RopeMesh(
    val vertices: FloatArray, // stride 15: pos(3)+normal(3)+color(3)+texCoord(2)+params(4)
    val vertexCount: Int,
    val indices: IntArray,
    val indexCount: Int
)

data class TwistEvent(val dist: Float, val angle: Float, val window: Float)

data class WormMeshParams(
    var segFreq: Float = 28f,
    var segBulge: Float = 0.12f,
    var thickness: Float = 1.35f,
    var taperLen: Float = 0.12f
)

// CrossSection and MaterialFrame are imported from com.uzls.four.simulation

private data class Profile2D(
    val positions: FloatArray, // stride 2
    val normals: FloatArray,   // stride 2
    val v: FloatArray,
    val count: Int
)

object RopeMeshBuilder {
    private val debugColors = floatArrayOf(
        1f, 0.3f, 0.3f,
        0.3f, 1f, 0.3f,
        0.3f, 0.3f, 1f,
        1f, 1f, 0.3f,
        1f, 0.3f, 1f,
        0.3f, 1f, 1f
    )

    fun buildRect(
        points: FloatArray, pointCount: Int,
        radius: Float, colorR: Float, colorG: Float, colorB: Float,
        twistEvents: List<TwistEvent> = emptyList(),
        tautness: Float = 0f,
        repulsors: FloatArray = FloatArray(0), // stride 4: x,y,radius,strength
        stretchRatio: Float = 1f,
        oscillation: Float = 0f,
        segmentStarts: IntArray = IntArray(0),
        restLength: Float = 0f,
        crossSection: CrossSection = CrossSection.Circular(0f),
        materialFrames: Array<MaterialFrame>? = null,
        profileSegments: Int = 16,
        ropeContactPoints: FloatArray = FloatArray(0), // stride 2
        ropeContactRadius: Float = 0f,
        stretchThinning: Float = 0.5f,
        wormMode: Boolean = false,
        wormTime: Float = 0f,
        wormMeshParams: WormMeshParams = WormMeshParams(),
        squareCrossSection: Boolean = false
    ): RopeMesh {
        if (pointCount < 2) return RopeMesh(FloatArray(0), 0, IntArray(0), 0)

        val usePhysicsFrames = (crossSection.isRectangular || squareCrossSection) &&
                materialFrames != null && materialFrames.size == pointCount

        val r = max(0.0005f, radius)
        val profile: Profile2D
        if (squareCrossSection) {
            val side = r * 2f
            profile = squareProfile(side, side)
        } else {
            profile = when (crossSection) {
                is CrossSection.Rectangular -> rectangularProfile(crossSection.width, crossSection.height)
                else -> circularProfile(r, profileSegments)
            }
        }
        val profileCount = profile.count

        var totalLen = 0f
        for (i in 1 until pointCount) {
            val pi = i * 3; val pi1 = (i - 1) * 3
            totalLen += vec3Dist(points, pi, points, pi1)
        }
        totalLen = max(1e-6f, totalLen)

        val effectiveRestLength = if (restLength > 0f) restLength else totalLen
        val globalStretchFactor = totalLen / max(1e-6f, effectiveRestLength)

        val vertCapacity = pointCount * profileCount
        val vertices = FloatArray(vertCapacity * 15)
        var vertIdx = 0

        val idxCapacity = (pointCount - 1) * profileCount * 6
        val indices = IntArray(idxCapacity)
        var idxIdx = 0

        var distanceAlong = 0f

        for (pointIndex in 0 until pointCount) {
            val pi = pointIndex * 3
            var posX = points[pi]; var posY = points[pi + 1]; var posZ = points[pi + 2]

            if (pointIndex > 0) {
                val pi1 = (pointIndex - 1) * 3
                distanceAlong += vec3Dist(points, pi, points, pi1)
            }

            var nrmX: Float; var nrmY: Float; var nrmZ: Float
            var binX: Float; var binY: Float; var binZ: Float

            if (usePhysicsFrames) {
                val frame = materialFrames!![pointIndex]
                nrmX = frame.d1x; nrmY = frame.d1y; nrmZ = frame.d1z
                binX = frame.d2x; binY = frame.d2y; binZ = frame.d2z
            } else {
                var tX: Float; var tY: Float; var tZ: Float
                if (pointIndex == 0) {
                    tX = points[3] - points[0]; tY = points[4] - points[1]; tZ = points[5] - points[2]
                } else if (pointIndex == pointCount - 1) {
                    val last = (pointCount - 1) * 3; val prev = (pointCount - 2) * 3
                    tX = points[last] - points[prev]; tY = points[last + 1] - points[prev + 1]; tZ = points[last + 2] - points[prev + 2]
                } else {
                    val next = (pointIndex + 1) * 3; val prev = (pointIndex - 1) * 3
                    tX = points[next] - points[prev]; tY = points[next + 1] - points[prev + 1]; tZ = points[next + 2] - points[prev + 2]
                }
                val tLen = vec3Length(tX, tY, tZ)
                if (tLen > 1e-9f) { tX /= tLen; tY /= tLen; tZ /= tLen }

                // up = (0,0,1)
                val upDotT = tZ // dot((0,0,1), tangent) = tangent.z
                val upProjX = -tX * upDotT; val upProjY = -tY * upDotT; val upProjZ = 1f - tZ * upDotT
                val upProjLen2 = upProjX * upProjX + upProjY * upProjY + upProjZ * upProjZ

                if (upProjLen2 > 1e-6f) {
                    val upProjLen = sqrt(upProjLen2)
                    binX = upProjX / upProjLen; binY = upProjY / upProjLen; binZ = upProjZ / upProjLen
                    nrmX = vec3CrossX(binY, binZ, tY, tZ)
                    nrmY = vec3CrossY(binX, binZ, tX, tZ)
                    nrmZ = vec3CrossZ(binX, binY, tX, tY)
                    val nrmLen = vec3Length(nrmX, nrmY, nrmZ)
                    if (nrmLen > 1e-9f) { nrmX /= nrmLen; nrmY /= nrmLen; nrmZ /= nrmLen }
                } else {
                    // up cross tangent
                    nrmX = vec3CrossX(0f, 1f, tY, tZ)
                    nrmY = vec3CrossY(0f, 1f, tX, tZ)
                    nrmZ = vec3CrossZ(0f, 0f, tX, tY)
                    val nrmLen = vec3Length(nrmX, nrmY, nrmZ)
                    if (nrmLen < 1e-8f) { nrmX = 1f; nrmY = 0f; nrmZ = 0f }
                    else { nrmX /= nrmLen; nrmY /= nrmLen; nrmZ /= nrmLen }
                    binX = vec3CrossX(tY, tZ, nrmY, nrmZ)
                    binY = vec3CrossY(tX, tZ, nrmX, nrmZ)
                    binZ = vec3CrossZ(tX, tY, nrmX, nrmY)
                    val binLen = vec3Length(binX, binY, binZ)
                    if (binLen > 1e-9f) { binX /= binLen; binY /= binLen; binZ /= binLen }
                }

                val twist = twistAngle(distanceAlong, twistEvents)
                if (abs(twist) > 1e-6f) {
                    val ct = cos(twist); val st = sin(twist)
                    val n2x = nrmX * ct + binX * st; val n2y = nrmY * ct + binY * st; val n2z = nrmZ * ct + binZ * st
                    val b2x = -nrmX * st + binX * ct; val b2y = -nrmY * st + binY * ct; val b2z = -nrmZ * st + binZ * ct
                    nrmX = n2x; nrmY = n2y; nrmZ = n2z
                    binX = b2x; binY = b2y; binZ = b2z
                }
            }

            val uCoord = distanceAlong / totalLen
            val uParam = pointIndex.toFloat() / max(1, pointCount - 1).toFloat()
            val center = sin(uCoord * PI.toFloat())
            val centerMask = center * center
            val centerMaskStrong = centerMask * centerMask

            val stretchFromRest = max(0f, globalStretchFactor - 1f)
            val stretchEffect = stretchRatio - 1f
            val dragTension = max(0f, stretchEffect)
            val totalTension = stretchFromRest + dragTension * 0.8f
            val latexDeform = totalTension * 0.35f * centerMaskStrong
            val stretchRelax = if (stretchEffect < 0f) abs(stretchEffect) * 0.2f * centerMask else 0f
            val pinch = latexDeform - stretchRelax * 0.3f
            val adjustedTautness = max(0f, tautness - stretchRelax * 0.3f)
            val endFade = smoothstep(0.04f, 0.14f, uCoord) * smoothstep(0.04f, 0.14f, 1f - uCoord)

            // Repulsors
            var repelMagTotal = 0f
            val repulsorCount = repulsors.size / 4
            if (repulsorCount > 0 && endFade > 1e-4f) {
                var repelX = 0f; var repelY = 0f
                for (ri in 0 until repulsorCount) {
                    val ri4 = ri * 4
                    val cx = repulsors[ri4]; val cy = repulsors[ri4 + 1]
                    val repRadius = repulsors[ri4 + 2]; val strength = repulsors[ri4 + 3]
                    val dx = posX - cx; val dy = posY - cy
                    val d2 = dx * dx + dy * dy
                    if (d2 < 1e-12f) continue
                    val dist = sqrt(d2)
                    val falloff = max(1e-4f, repRadius * 0.85f)
                    val w = smoothstep(repRadius + falloff, repRadius, dist)
                    if (w <= 0f) continue
                    repelX += dx / dist * w * strength
                    repelY += dy / dist * w * strength
                    repelMagTotal += w * strength
                }
                val repelScale = endFade * (0.35f + 0.65f * (1f - pinch))
                posX += repelX * repelScale
                posY += repelY * repelScale
                posZ += min(0.02f, repelMagTotal * 0.22f) * endFade
            }

            // Contact deform
            var contactDeform = 0f
            val contactCount = ropeContactPoints.size / 2
            if (contactCount > 0 && ropeContactRadius > 1e-6f) {
                val inner = ropeContactRadius * 0.4f
                val outer = ropeContactRadius * 1.3f
                for (ci in 0 until contactCount) {
                    val ci2 = ci * 2
                    val dx = posX - ropeContactPoints[ci2]; val dy = posY - ropeContactPoints[ci2 + 1]
                    val dist = sqrt(dx * dx + dy * dy)
                    val w = smoothstep(outer, inner, dist)
                    if (w > 0f) contactDeform = max(contactDeform, w * 0.1f)
                }
            }

            val paramsX = adjustedTautness
            val paramsY = pinch
            val paramsZ = min(1f, repelMagTotal / max(1e-4f, radius))
            val paramsW = if (wormMode) 1f else 0f

            val latexThin = 1f / sqrt(max(1f, 1f + totalTension * stretchThinning * 3f * centerMaskStrong))
            val relaxThick = 1f + stretchRelax * 0.15f
            var baseScale = latexThin * relaxThick
            val endTaper = smoothstep(0.08f, 0.02f, uCoord) * smoothstep(0.08f, 0.02f, 1f - uCoord)
            baseScale *= 1f - endTaper * 0.06f
            val flattenAmt = min(0.3f, totalTension * 0.2f * stretchThinning * centerMaskStrong)
            var scaleNrm = baseScale * (1f - flattenAmt) * (1f - contactDeform)
            var scaleBin = baseScale * (1f + flattenAmt * 0.5f) * (1f - contactDeform)

            if (wormMode) {
                val bodyTaper = sin(uCoord * PI.toFloat())
                val headTaper = smoothstep(0f, wormMeshParams.taperLen, uCoord)
                val tailTaper = smoothstep(0f, wormMeshParams.taperLen, 1f - uCoord)
                val taper = headTaper * tailTaper * (0.5f + 0.5f * bodyTaper)
                val segBulge = 1f + wormMeshParams.segBulge * sin(uCoord * wormMeshParams.segFreq * PI.toFloat() * 2f)
                val wormScale = taper * segBulge * wormMeshParams.thickness
                scaleNrm *= wormScale
                scaleBin *= wormScale
            }

            val lightenAmt = totalTension * centerMaskStrong * 0.35f
            val adjR: Float; val adjG: Float; val adjB: Float
            if (segmentStarts.isNotEmpty()) {
                val segIdx = segmentIndex(pointIndex, segmentStarts)
                val ci = (segIdx % 6) * 3
                adjR = debugColors[ci]; adjG = debugColors[ci + 1]; adjB = debugColors[ci + 2]
            } else {
                adjR = colorR * (1f + lightenAmt) + lightenAmt * 0.15f
                adjG = colorG * (1f + lightenAmt) + lightenAmt * 0.15f
                adjB = colorB * (1f + lightenAmt) + lightenAmt * 0.15f
            }

            // Oscillation
            val oscWave = sin(uCoord * PI.toFloat() * 3f + oscillation * 6f)
            val oscAmp = abs(oscillation) * 0.12f
            val oscOffset = oscWave * oscAmp
            val oscDirLen = sqrt(nrmX * nrmX + nrmY * nrmY)
            if (oscDirLen > 1e-6f) {
                posX += nrmX / oscDirLen * oscOffset
                posY += nrmY / oscDirLen * oscOffset
            } else {
                val fbLen = sqrt(binX * binX + binY * binY)
                if (fbLen > 1e-6f) {
                    posX += binX / fbLen * oscOffset
                    posY += binY / fbLen * oscOffset
                }
            }

            // Profile vertices
            for (k in 0 until profileCount) {
                val pk = k * 2
                val localX = profile.positions[pk]
                val localY = profile.positions[pk + 1]
                val localNX = profile.normals[pk]
                val localNY = profile.normals[pk + 1]

                val wpX = posX + nrmX * localX * scaleNrm + binX * localY * scaleBin
                val wpY = posY + nrmY * localX * scaleNrm + binY * localY * scaleBin
                val wpZ = posZ + nrmZ * localX * scaleNrm + binZ * localY * scaleBin

                var wnX = nrmX * localNX / max(0.01f, scaleNrm) + binX * localNY / max(0.01f, scaleBin)
                var wnY = nrmY * localNX / max(0.01f, scaleNrm) + binY * localNY / max(0.01f, scaleBin)
                var wnZ = nrmZ * localNX / max(0.01f, scaleNrm) + binZ * localNY / max(0.01f, scaleBin)
                val wnLen = vec3Length(wnX, wnY, wnZ)
                if (wnLen > 1e-9f) { wnX /= wnLen; wnY /= wnLen; wnZ /= wnLen }

                val vi = vertIdx * 15
                vertices[vi] = wpX; vertices[vi + 1] = wpY; vertices[vi + 2] = wpZ
                vertices[vi + 3] = wnX; vertices[vi + 4] = wnY; vertices[vi + 5] = wnZ
                vertices[vi + 6] = adjR; vertices[vi + 7] = adjG; vertices[vi + 8] = adjB
                vertices[vi + 9] = uParam; vertices[vi + 10] = profile.v[k]
                vertices[vi + 11] = paramsX; vertices[vi + 12] = paramsY
                vertices[vi + 13] = paramsZ; vertices[vi + 14] = paramsW
                vertIdx++
            }

            // Indices
            if (pointIndex < pointCount - 1) {
                val baseA = pointIndex * profileCount
                val baseB = (pointIndex + 1) * profileCount
                for (k in 0 until profileCount) {
                    val k1 = (k + 1) % profileCount
                    indices[idxIdx++] = baseA + k
                    indices[idxIdx++] = baseB + k
                    indices[idxIdx++] = baseB + k1
                    indices[idxIdx++] = baseA + k
                    indices[idxIdx++] = baseB + k1
                    indices[idxIdx++] = baseA + k1
                }
            }
        }

        return RopeMesh(vertices, vertIdx, indices, idxIdx)
    }

    fun buildHemisphere(
        cx: Float, cy: Float, cz: Float,
        radius: Float, facingX: Float, facingY: Float, facingZ: Float,
        colorR: Float, colorG: Float, colorB: Float,
        segments: Int = 12, rings: Int = 6, darken: Float = 0.7f,
        wormMode: Boolean = false
    ): RopeMesh {
        val r = max(0.001f, radius)
        val seg = max(6, segments)
        val rng = max(3, rings)

        // Compute local frame
        var upX = 0f; var upY = 0f; var upZ = 1f
        if (abs(vec3Dot(upX, upY, upZ, facingX, facingY, facingZ)) > 0.95f) {
            upX = 0f; upY = 1f; upZ = 0f
        }
        val rightArr = vec3Normalize(
            vec3CrossX(upY, upZ, facingY, facingZ),
            vec3CrossY(upX, upZ, facingX, facingZ),
            vec3CrossZ(upX, upY, facingX, facingY)
        )
        val rX = rightArr[0]; val rY = rightArr[1]; val rZ = rightArr[2]
        val fwdArr = vec3Normalize(
            vec3CrossX(facingY, facingZ, rY, rZ),
            vec3CrossY(facingX, facingZ, rX, rZ),
            vec3CrossZ(facingX, facingY, rX, rY)
        )
        val fX = fwdArr[0]; val fY = fwdArr[1]; val fZ = fwdArr[2]

        val darkR = colorR * (1f - darken); val darkG = colorG * (1f - darken); val darkB = colorB * (1f - darken)
        val paramsW = if (wormMode) 1f else 0f

        val vertCap = (rng + 1) * seg + 1
        val vertices = FloatArray(vertCap * 15)
        var vi = 0
        val idxCap = rng * seg * 6 + seg * 3
        val indices = IntArray(idxCap)
        var ii = 0

        for (ring in 0..rng) {
            val phi = (ring.toFloat() / rng) * (PI.toFloat() * 0.5f)
            val t = ring.toFloat() / rng
            val blend = t * t
            val ringR = colorR * (1f - blend) + darkR * blend
            val ringG = colorG * (1f - blend) + darkG * blend
            val ringB = colorB * (1f - blend) + darkB * blend
            val ringRadius = r * cos(phi)
            val ringZ = r * sin(phi)

            for (s in 0 until seg) {
                val theta = (s.toFloat() / seg) * PI.toFloat() * 2f
                val localX = cos(theta) * ringRadius
                val localY = sin(theta) * ringRadius
                val px = cx + rX * localX + fX * localY + facingX * ringZ
                val py = cy + rY * localX + fY * localY + facingY * ringZ
                val pz = cz + rZ * localX + fZ * localY + facingZ * ringZ

                val nx = rX * cos(theta) * cos(phi) + fX * sin(theta) * cos(phi) + facingX * sin(phi)
                val ny = rY * cos(theta) * cos(phi) + fY * sin(theta) * cos(phi) + facingY * sin(phi)
                val nz = rZ * cos(theta) * cos(phi) + fZ * sin(theta) * cos(phi) + facingZ * sin(phi)
                val nLen = vec3Length(nx, ny, nz)

                val vo = vi * 15
                vertices[vo] = px; vertices[vo + 1] = py; vertices[vo + 2] = pz
                vertices[vo + 3] = if (nLen > 1e-9f) nx / nLen else 0f
                vertices[vo + 4] = if (nLen > 1e-9f) ny / nLen else 0f
                vertices[vo + 5] = if (nLen > 1e-9f) nz / nLen else 0f
                vertices[vo + 6] = ringR; vertices[vo + 7] = ringG; vertices[vo + 8] = ringB
                vertices[vo + 9] = 0.5f; vertices[vo + 10] = 0.5f
                vertices[vo + 11] = 0f; vertices[vo + 12] = 0f; vertices[vo + 13] = 0f; vertices[vo + 14] = paramsW
                vi++
            }
        }

        // Indices for rings
        for (ring in 0 until rng) {
            for (s in 0 until seg) {
                val curr = ring * seg + s
                val next = ring * seg + (s + 1) % seg
                val currUp = (ring + 1) * seg + s
                val nextUp = (ring + 1) * seg + (s + 1) % seg
                indices[ii++] = curr; indices[ii++] = currUp; indices[ii++] = next
                indices[ii++] = next; indices[ii++] = currUp; indices[ii++] = nextUp
            }
        }

        // Tip vertex
        val tipVi = vi
        val tvo = vi * 15
        vertices[tvo] = cx + facingX * r; vertices[tvo + 1] = cy + facingY * r; vertices[tvo + 2] = cz + facingZ * r
        vertices[tvo + 3] = facingX; vertices[tvo + 4] = facingY; vertices[tvo + 5] = facingZ
        vertices[tvo + 6] = darkR; vertices[tvo + 7] = darkG; vertices[tvo + 8] = darkB
        vertices[tvo + 9] = 0.5f; vertices[tvo + 10] = 0.5f
        vertices[tvo + 11] = 0f; vertices[tvo + 12] = 0f; vertices[tvo + 13] = 0f; vertices[tvo + 14] = paramsW
        vi++

        val topRing = rng * seg
        for (s in 0 until seg) {
            indices[ii++] = topRing + s
            indices[ii++] = tipVi
            indices[ii++] = topRing + (s + 1) % seg
        }

        return RopeMesh(vertices, vi, indices, ii)
    }

    fun buildSwivel(
        cx: Float, cy: Float, cz: Float,
        tanX: Float, tanY: Float, tanZ: Float,
        holeRadius: Float, bandHalf: Float,
        d1x: Float, d1y: Float, d1z: Float,
        d2x: Float, d2y: Float, d2z: Float,
        colorR: Float, colorG: Float, colorB: Float,
        segments: Int = 16
    ): RopeMesh {
        val params = floatArrayOf(0f, 0f, 0f, 0f)
        val uv0 = 0.5f; val uv1 = 0.5f

        val tLen = vec3Length(tanX, tanY, tanZ)
        val tX = if (tLen > 1e-9f) tanX / tLen else tanX
        val tY = if (tLen > 1e-9f) tanY / tLen else tanY
        val tZ = if (tLen > 1e-9f) tanZ / tLen else tanZ

        val baseR = holeRadius * 1.1f
        val baseDepth = holeRadius * 0.2f
        val baseColR = 0.45f; val baseColG = 0.45f; val baseColB = 0.48f

        val seg = max(8, segments)

        val fcX = cx + tX * baseDepth * 0.5f
        val fcY = cy + tY * baseDepth * 0.5f
        val fcZ = cz + tZ * baseDepth * 0.5f
        val bcX = cx - tX * baseDepth * 0.5f
        val bcY = cy - tY * baseDepth * 0.5f
        val bcZ = cz - tZ * baseDepth * 0.5f

        val vertCap = seg * 7
        val idxCap = seg * 9
        val vertices = FloatArray(vertCap * 15)
        var vi = 0
        val indices = IntArray(idxCap)
        var ii = 0

        fun putVert(px: Float, py: Float, pz: Float, nx: Float, ny: Float, nz: Float, cr: Float, cg: Float, cb: Float) {
            val vo = vi * 15
            vertices[vo] = px; vertices[vo+1] = py; vertices[vo+2] = pz
            vertices[vo+3] = nx; vertices[vo+4] = ny; vertices[vo+5] = nz
            vertices[vo+6] = cr; vertices[vo+7] = cg; vertices[vo+8] = cb
            vertices[vo+9] = uv0; vertices[vo+10] = uv1
            vertices[vo+11] = params[0]; vertices[vo+12] = params[1]
            vertices[vo+13] = params[2]; vertices[vo+14] = params[3]
            vi++
        }

        for (s in 0 until seg) {
            val a0 = s.toFloat() / seg * PI.toFloat() * 2f
            val a1 = (s + 1).toFloat() / seg * PI.toFloat() * 2f
            val p0dX = d1x * cos(a0) + d2x * sin(a0)
            val p0dY = d1y * cos(a0) + d2y * sin(a0)
            val p0dZ = d1z * cos(a0) + d2z * sin(a0)
            val p1dX = d1x * cos(a1) + d2x * sin(a1)
            val p1dY = d1y * cos(a1) + d2y * sin(a1)
            val p1dZ = d1z * cos(a1) + d2z * sin(a1)

            val pf0X = fcX + p0dX * baseR; val pf0Y = fcY + p0dY * baseR; val pf0Z = fcZ + p0dZ * baseR
            val pf1X = fcX + p1dX * baseR; val pf1Y = fcY + p1dY * baseR; val pf1Z = fcZ + p1dZ * baseR
            val pb0X = bcX + p0dX * baseR; val pb0Y = bcY + p0dY * baseR; val pb0Z = bcZ + p0dZ * baseR

            val base = vi
            putVert(pf0X, pf0Y, pf0Z, p0dX, p0dY, p0dZ, baseColR, baseColG, baseColB)
            putVert(pb0X, pb0Y, pb0Z, p0dX, p0dY, p0dZ, baseColR, baseColG, baseColB)
            putVert(pf1X, pf1Y, pf1Z, p1dX, p1dY, p1dZ, baseColR, baseColG, baseColB)
            val pb1X = bcX + p1dX * baseR; val pb1Y = bcY + p1dY * baseR; val pb1Z = bcZ + p1dZ * baseR
            putVert(pb1X, pb1Y, pb1Z, p1dX, p1dY, p1dZ, baseColR, baseColG, baseColB)
            indices[ii++] = base; indices[ii++] = base+1; indices[ii++] = base+2
            indices[ii++] = base+2; indices[ii++] = base+1; indices[ii++] = base+3

            val capBase = vi
            putVert(fcX, fcY, fcZ, tX, tY, tZ, baseColR, baseColG, baseColB)
            putVert(pf0X, pf0Y, pf0Z, tX, tY, tZ, baseColR, baseColG, baseColB)
            putVert(pf1X, pf1Y, pf1Z, tX, tY, tZ, baseColR, baseColG, baseColB)
            indices[ii++] = capBase; indices[ii++] = capBase+1; indices[ii++] = capBase+2
        }

        return RopeMesh(vertices, vi, indices, ii)
    }

    private fun segmentIndex(pointIndex: Int, segmentStarts: IntArray): Int {
        var seg = 0
        for (i in segmentStarts.indices) {
            if (pointIndex >= segmentStarts[i]) seg = i else break
        }
        return seg
    }

    private fun twistAngle(distanceAlong: Float, events: List<TwistEvent>): Float {
        if (events.isEmpty()) return 0f
        var accumulated = 0f
        for (e in events) {
            val distance = abs(distanceAlong - e.dist)
            val norm = min(1f, distance / max(1e-6f, e.window))
            val weight = smoothstep(1f, 0f, norm)
            accumulated += e.angle * weight
        }
        return accumulated
    }

    private fun smoothstep(edge0: Float, edge1: Float, value: Float): Float {
        val normalized = max(0f, min(1f, (value - edge0) / (edge1 - edge0)))
        return normalized * normalized * (3f - 2f * normalized)
    }

    private fun circularProfile(radius: Float, segments: Int): Profile2D {
        val r = max(0.0005f, radius)
        val seg = max(3, min(32, segments))
        val positions = FloatArray(seg * 2)
        val normals = FloatArray(seg * 2)
        val v = FloatArray(seg)

        for (i in 0 until seg) {
            val angle = i.toFloat() / seg * PI.toFloat() * 2f
            val ca = cos(angle); val sa = sin(angle)
            positions[i * 2] = ca * r; positions[i * 2 + 1] = sa * r
            normals[i * 2] = ca; normals[i * 2 + 1] = sa
            v[i] = i.toFloat() / seg
        }
        return Profile2D(positions, normals, v, seg)
    }

    private fun rectangularProfile(width: Float, height: Float): Profile2D {
        val hw = max(0.0005f, width * 0.5f)
        val hh = max(0.0005f, height * 0.5f)
        val cornerR = min(hw, hh) * 0.15f
        val cornerSegs = 3

        val corners = arrayOf(
            floatArrayOf(hw - cornerR, hh - cornerR, 0f, PI.toFloat() * 0.5f),
            floatArrayOf(-hw + cornerR, hh - cornerR, PI.toFloat() * 0.5f, PI.toFloat()),
            floatArrayOf(-hw + cornerR, -hh + cornerR, PI.toFloat(), PI.toFloat() * 1.5f),
            floatArrayOf(hw - cornerR, -hh + cornerR, PI.toFloat() * 1.5f, PI.toFloat() * 2f)
        )

        val totalVerts = corners.size * (cornerSegs + 1)
        val positions = FloatArray(totalVerts * 2)
        val normals = FloatArray(totalVerts * 2)
        val v = FloatArray(totalVerts)

        var idx = 0
        for (corner in corners) {
            val cx = corner[0]; val cy = corner[1]
            val startAngle = corner[2]; val endAngle = corner[3]
            for (s in 0..cornerSegs) {
                val t = s.toFloat() / cornerSegs
                val angle = startAngle + (endAngle - startAngle) * t
                val ca = cos(angle); val sa = sin(angle)
                positions[idx * 2] = cx + ca * cornerR
                positions[idx * 2 + 1] = cy + sa * cornerR
                val nLen = sqrt(ca * ca + sa * sa)
                normals[idx * 2] = if (nLen > 0) ca / nLen else ca
                normals[idx * 2 + 1] = if (nLen > 0) sa / nLen else sa
                v[idx] = idx.toFloat() / totalVerts
                idx++
            }
        }
        return Profile2D(positions, normals, v, totalVerts)
    }

    private fun squareProfile(width: Float, height: Float): Profile2D {
        val hw = max(0.0005f, width * 0.5f)
        val hh = max(0.0005f, height * 0.5f)

        val positions = floatArrayOf(
            hw, hh, -hw, hh,
            -hw, hh, -hw, -hh,
            -hw, -hh, hw, -hh,
            hw, -hh, hw, hh
        )
        val normals = floatArrayOf(
            0f, 1f, 0f, 1f,
            -1f, 0f, -1f, 0f,
            0f, -1f, 0f, -1f,
            1f, 0f, 1f, 0f
        )
        val v = FloatArray(8) { it.toFloat() / 8f }
        return Profile2D(positions, normals, v, 8)
    }
}
