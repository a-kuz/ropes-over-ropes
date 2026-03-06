package com.uzls.four.simulation

import android.util.Log
import kotlin.math.*

/**
 * Exact 1:1 port of the iOS VerletSimulator.
 *
 * All position data uses stride-3 FloatArray layout for cache-friendly access:
 *   [x0, y0, z0, x1, y1, z1, ...]
 * Particle index i maps to array offset i * 3.
 *
 * Pin indices use -1 to represent "no pin" (Swift Optional<Int> nil).
 */
class VerletSimulator(
    /** Flat array of hole positions: [x0,y0, x1,y1, ...] stride-2 */
    val holePositions: FloatArray,
    /** Elevation per hole (same count as holePositions/2). Empty → all zeros. */
    holeElevations: FloatArray = FloatArray(0),
    val holeRadius: Float,
    val boards: Array<BoardDef> = emptyArray()
) {
    companion object {
        private const val TAG = "VerletSim"
        private const val ENABLE_AGENT_DEBUG_LOGS = false

        fun computeFrames(
            positions: FloatArray,
            twistAngles: FloatArray
        ): Array<MaterialFrame> {
            val n = positions.size / 3
            if (n < 2) return emptyArray()

            val frames = Array(n) { MaterialFrame() }
            val upX = 0f; val upY = 0f; val upZ = 1f

            // Initial tangent
            var tPrevX: Float; var tPrevY: Float; var tPrevZ: Float
            run {
                val dx = positions[3] - positions[0]
                val dy = positions[4] - positions[1]
                val dz = positions[5] - positions[2]
                val len = sqrt(dx * dx + dy * dy + dz * dz)
                if (len > 1e-9f) { tPrevX = dx / len; tPrevY = dy / len; tPrevZ = dz / len }
                else { tPrevX = 0f; tPrevY = 0f; tPrevZ = 1f }
            }

            // Initial up vector
            var uPrevX: Float; var uPrevY: Float; var uPrevZ: Float
            run {
                var ux = vec3CrossX(upY, upZ, tPrevY, tPrevZ)
                var uy = vec3CrossY(upX, upZ, tPrevX, tPrevZ)
                var uz = vec3CrossZ(upX, upY, tPrevX, tPrevY)
                if (ux * ux + uy * uy + uz * uz < 1e-8f) {
                    ux = 1f; uy = 0f; uz = 0f
                }
                val len = sqrt(ux * ux + uy * uy + uz * uz)
                uPrevX = ux / len; uPrevY = uy / len; uPrevZ = uz / len
            }

            for (i in 0 until n) {
                val tangentX: Float; val tangentY: Float; val tangentZ: Float
                if (i == 0) {
                    val dx = positions[3] - positions[0]
                    val dy = positions[4] - positions[1]
                    val dz = positions[5] - positions[2]
                    val len = sqrt(dx * dx + dy * dy + dz * dz)
                    if (len > 1e-9f) { tangentX = dx / len; tangentY = dy / len; tangentZ = dz / len }
                    else { tangentX = 0f; tangentY = 0f; tangentZ = 1f }
                } else if (i == n - 1) {
                    val o0 = (n - 2) * 3; val o1 = (n - 1) * 3
                    val dx = positions[o1] - positions[o0]
                    val dy = positions[o1 + 1] - positions[o0 + 1]
                    val dz = positions[o1 + 2] - positions[o0 + 2]
                    val len = sqrt(dx * dx + dy * dy + dz * dz)
                    if (len > 1e-9f) { tangentX = dx / len; tangentY = dy / len; tangentZ = dz / len }
                    else { tangentX = 0f; tangentY = 0f; tangentZ = 1f }
                } else {
                    val oPrev = (i - 1) * 3; val oNext = (i + 1) * 3
                    val dx = positions[oNext] - positions[oPrev]
                    val dy = positions[oNext + 1] - positions[oPrev + 1]
                    val dz = positions[oNext + 2] - positions[oPrev + 2]
                    val len = sqrt(dx * dx + dy * dy + dz * dz)
                    if (len > 1e-9f) { tangentX = dx / len; tangentY = dy / len; tangentZ = dz / len }
                    else { tangentX = 0f; tangentY = 0f; tangentZ = 1f }
                }

                var ux = uPrevX; var uy = uPrevY; var uz = uPrevZ
                if (i > 0) {
                    val axisX = vec3CrossX(tPrevY, tPrevZ, tangentY, tangentZ)
                    val axisY = vec3CrossY(tPrevX, tPrevZ, tangentX, tangentZ)
                    val axisZ = vec3CrossZ(tPrevX, tPrevY, tangentX, tangentY)
                    val axisLen = sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ)
                    if (axisLen > 1e-6f) {
                        val axisNx = axisX / axisLen; val axisNy = axisY / axisLen; val axisNz = axisZ / axisLen
                        val dotClamped = (tPrevX * tangentX + tPrevY * tangentY + tPrevZ * tangentZ).coerceIn(-1.0f, 1.0f)
                        val angle = atan2(axisLen, dotClamped)
                        val rotated = rotateVector(ux, uy, uz, axisNx, axisNy, axisNz, angle)
                        ux = rotated[0]; uy = rotated[1]; uz = rotated[2]
                        // Project onto plane perpendicular to tangent
                        val dotUT = ux * tangentX + uy * tangentY + uz * tangentZ
                        val projX = ux - tangentX * dotUT
                        val projY = uy - tangentY * dotUT
                        val projZ = uz - tangentZ * dotUT
                        if (projX * projX + projY * projY + projZ * projZ > 1e-10f) {
                            val projLen = sqrt(projX * projX + projY * projY + projZ * projZ)
                            ux = projX / projLen; uy = projY / projLen; uz = projZ / projLen
                        }
                    }
                }

                var vx = vec3CrossX(tangentY, tangentZ, uy, uz)
                var vy = vec3CrossY(tangentX, tangentZ, ux, uz)
                var vz = vec3CrossZ(tangentX, tangentY, ux, uy)
                val vLen = sqrt(vx * vx + vy * vy + vz * vz)
                if (vLen > 1e-9f) { vx /= vLen; vy /= vLen; vz /= vLen }

                val twist = twistAngles[i]
                val cosT = cos(twist)
                val sinT = sin(twist)
                val d1x = ux * cosT + vx * sinT
                val d1y = uy * cosT + vy * sinT
                val d1z = uz * cosT + vz * sinT
                val d2x = -ux * sinT + vx * cosT
                val d2y = -uy * sinT + vy * cosT
                val d2z = -uz * sinT + vz * cosT

                frames[i] = MaterialFrame(tangentX, tangentY, tangentZ, d1x, d1y, d1z, d2x, d2y, d2z)

                tPrevX = tangentX; tPrevY = tangentY; tPrevZ = tangentZ
                uPrevX = ux; uPrevY = uy; uPrevZ = uz
            }

            return frames
        }

        fun rotateVector(
            vx: Float, vy: Float, vz: Float,
            ax: Float, ay: Float, az: Float,
            angle: Float
        ): FloatArray {
            val cosA = cos(angle)
            val sinA = sin(angle)
            val cx = vec3CrossX(ay, az, vy, vz)
            val cy = vec3CrossY(ax, az, vx, vz)
            val cz = vec3CrossZ(ax, ay, vx, vy)
            val dot = ax * vx + ay * vy + az * vz
            val oneMinusCos = 1f - cosA
            return floatArrayOf(
                vx * cosA + cx * sinA + ax * dot * oneMinusCos,
                vy * cosA + cy * sinA + ay * dot * oneMinusCos,
                vz * cosA + cz * sinA + az * dot * oneMinusCos
            )
        }
    }

    // Derived
    val holeCount: Int = holePositions.size / 2
    val holeElevations: FloatArray =
        if (holeElevations.isEmpty()) FloatArray(holeCount) else holeElevations
    val holeDepth: Float = holeRadius * 1.25f

    // Band storage
    val bands: MutableList<Band> = mutableListOf()

    // Physics parameters (tuneable) — defaults match Swift exactly
    var gravity: Float = -14.298969268798828f
    var damping: Float = 0.92867755889892578f
    var constraintIterations: Int = 2
        set(value) { field = max(value, 2) }
    var settleSteps: Int = 5
    var liftHeight: Float = 0.30000001192092896f
    var ropeTension: Float = 0.98000001907348633f
    var currentTension: Float = 1.0f
    private val tensionSpeed: Float = 0.5f
    var frictionCoefficient: Float = 0.8f
    var particleCount: Int = 6
    var bendCompliance: Float = 0f
    var bendVelocityCoupling: Float = 0.44999998807907104f
    var twistStiffness: Float = 0.15f
    var twistDamping: Float = 0.4f
    var gravityTorqueStrength: Float = 0.8f
    var stretchThinning: Float = 0.5f
    var squareCrossSection: Boolean = false

    // Sticky (adhesion) parameters
    var stickyEnabled: Boolean = false
    var stickyStrength: Float = 0.5f
    var stickyRadius: Float = 1.5f
    var stickyDamping: Float = 0.9f
    var stickyBreakThreshold: Float = 0.8f

    private val stickyBonds = ArrayList<StickyBond>(256)
    private val stickyBondKeys = HashSet<Long>(256)
    private var stickyFormCounter = 0
    private val stickyFormInterval = 10

    var physicsRate: Float = 120f
        set(value) { field = value.coerceIn(30f, 240f); dt = 1.0f / field }
    private var dt: Float = 1.0f / 120.0f
    var maxSubstepsPerFrame: Int = 4
    private var accumulator: Float = 0f
    private var resampleCounter: Int = 0
    private val resampleInterval: Int = 30
    private var debugStepCounter: Int = 0
    private var debugCollisionHits: Int = 0
    private var debugDragCollisionHits: Int = 0
    private var debugMaxOverlap: Float = 0f
    private var debugLastPairCount: Int = 0
    private var debugMaxParticleDisplacement: Float = 0f
    private var debugClampedParticles: Int = 0
    private var debugDroppedSubsteps: Int = 0
    private var perfIntegrateNs: Long = 0L
    private var perfBroadphaseNs: Long = 0L
    private var perfConstraintNs: Long = 0L
    private var perfPostSolveNs: Long = 0L
    private var perfFrictionStickyNs: Long = 0L
    private var perfStepSamples: Int = 0

    private fun debugLog(runId: String, hypothesisId: String, location: String, message: String, data: String) {
        if (!ENABLE_AGENT_DEBUG_LOGS) return
        val ts = System.currentTimeMillis()
        Log.i(
            TAG,
            "[AGENTDBG] {\"sessionId\":\"d31f9b\",\"runId\":\"$runId\",\"hypothesisId\":\"$hypothesisId\",\"location\":\"$location\",\"message\":\"$message\",\"data\":$data,\"timestamp\":$ts}"
        )
    }

    var dragTargetPosX: Float = 0f
    var dragTargetPosY: Float = 0f
    var dragTargetPosZ: Float = 0f
    var hasDragTargetPos: Boolean = false

    var dragStartPosX: Float = 0f
    var dragStartPosY: Float = 0f
    var dragStartPosZ: Float = 0f
    var hasDragStartPos: Boolean = false

    var logTimer: Float = 0f
    var logEvery: Float = 1.0f

    var dragInfo: DragInfo? = null

    // Idle sleep
    private val idleTimeout: Float = 3.0f
    private var idleTimer: Float = 0f
    var isSleeping: Boolean = false
        private set

    fun wakeUp() {
        idleTimer = 0f
        isSleeping = false
    }

    // Friction sound feedback
    var frictionAccumulator: Float = 0f
    var frictionSpeedAccumulator: Float = 0f
    var frictionPositionAccX: Float = 0f
    var frictionPositionAccY: Float = 0f
    var frictionPositionAccZ: Float = 0f
    var frictionSampleCount: Int = 0

    // Lower animation
    var lowerAnimations: MutableMap<LowerAnimationKey, LowerAnimation> = mutableMapOf()
    val hasLowerAnimations: Boolean
        get() = lowerAnimations.isNotEmpty()

    // Cached material frames per band
    var cachedFrames: Array<Array<MaterialFrame>> = emptyArray()

    private val initDt: Float = 1.0f / 60.0f

    // =========================================================================
    // MARK: - Geometry helpers
    // =========================================================================

    fun holePosition3D(holeIndex: Int, out: FloatArray, offset: Int) {
        if (holeIndex < 0 || holeIndex >= holeCount) {
            out[offset] = 0f; out[offset + 1] = 0f; out[offset + 2] = 0f
            return
        }
        val hi = holeIndex * 2
        val elev = holeElevations[holeIndex]
        out[offset] = holePositions[hi]
        out[offset + 1] = holePositions[hi + 1]
        out[offset + 2] = elev - holeDepth
    }

    /** Returns the 3D hole position as a new FloatArray (convenience, allocates). */
    fun holePosition3D(holeIndex: Int): FloatArray {
        val out = FloatArray(3)
        holePosition3D(holeIndex, out, 0)
        return out
    }

    fun holeSurfaceZ(holeIndex: Int): Float {
        if (holeIndex < 0 || holeIndex >= holeCount) return 0f
        return holeElevations[holeIndex]
    }

    fun isInsideAnyHole(x: Float, y: Float): Boolean {
        val r2 = holeRadius * holeRadius
        for (h in 0 until holeCount) {
            val hx = holePositions[h * 2]
            val hy = holePositions[h * 2 + 1]
            val dx = x - hx
            val dy = y - hy
            if (dx * dx + dy * dy < r2) return true
        }
        return false
    }

    fun boardSurfaceZ(x: Float, y: Float): Float {
        var maxZ = 0f
        for (b in boards) {
            val halfW = b.width * 0.5f
            val halfH = b.height * 0.5f
            if (x >= b.centerX - halfW && x <= b.centerX + halfW &&
                y >= b.centerY - halfH && y <= b.centerY + halfH
            ) {
                if (b.elevation > maxZ) maxZ = b.elevation
            }
        }
        return maxZ
    }

    // =========================================================================
    // MARK: - Band management
    // =========================================================================

    fun addBand(
        radius: Float,
        crossSection: CrossSection? = null,
        particleCount: Int? = null
    ): Int {
        val n = particleCount ?: this.particleCount
        val cs = crossSection ?: CrossSection.Circular(radius)
        val band = Band(
            positions = FloatArray(n * 3),
            previousPositions = FloatArray(n * 3),
            twistAngles = FloatArray(n),
            previousTwistAngles = FloatArray(n),
            segmentLength = 0f,
            radius = radius,
            crossSection = cs,
            pinStart = -1,
            pinEnd = -1,
            active = false
        )
        bands.add(band)
        return bands.size - 1
    }

    fun pin(bandIndex: Int, startHole: Int, endHole: Int) {
        if (bandIndex < 0 || bandIndex >= bands.size) return

        val p0 = holePosition3D(startHole)
        val p1 = holePosition3D(endHole)
        val band = bands[bandIndex]
        val n = band.particleCount
        val pos = band.positions
        val prev = band.previousPositions

        for (i in 0 until n) {
            val t = i.toFloat() / max(1, n - 1).toFloat()
            val off = i * 3
            val x = p0[0] * (1f - t) + p1[0] * t
            val y = p0[1] * (1f - t) + p1[1] * t
            val z: Float = if (i == 0 || i == n - 1) p0[2] else liftHeight
            pos[off] = x; pos[off + 1] = y; pos[off + 2] = z
        }
        // Endpoints go to holes
        pos[0] = p0[0]; pos[1] = p0[1]; pos[2] = p0[2]
        val lastOff = (n - 1) * 3
        pos[lastOff] = p1[0]; pos[lastOff + 1] = p1[1]; pos[lastOff + 2] = p1[2]
        System.arraycopy(pos, 0, prev, 0, pos.size)

        val dx = p1[0] - p0[0]; val dy = p1[1] - p0[1]; val dz = p1[2] - p0[2]
        val dist = sqrt(dx * dx + dy * dy + dz * dz)
        band.segmentLength = dist / max(1, n - 1).toFloat()
        band.pinStart = startHole
        band.pinEnd = endHole
        band.active = true

        val totalLen = band.segmentLength * (n - 1).toFloat()
        Log.w(TAG, "[PIN] band=$bandIndex n=$n dist=${"%.4f".format(dist)} segLen=${"%.6f".format(band.segmentLength)} totalLen=${"%.4f".format(totalLen)} tension=${"%.3f".format(currentTension)}")

        val hasOthers = bands.indices.any { it != bandIndex && bands[it].active && bands[it].pinStart >= 0 }
        doSteps(settleSteps, hasOthers)
    }

    // =========================================================================
    // MARK: - Main update
    // =========================================================================

    fun update(deltaTime: Float) {
        val clampedDt = min(deltaTime, 1.0f / 15.0f)

        if (dragInfo != null || hasLowerAnimations) {
            idleTimer = 0f
            isSleeping = false
        } else {
            idleTimer += clampedDt
        }

        if (idleTimer >= idleTimeout && !isSleeping) {
            for (bi in bands.indices) {
                val b = bands[bi]
                if (b.active && b.fadeOut == 0f) {
                    System.arraycopy(b.positions, 0, b.previousPositions, 0, b.positions.size)
                    System.arraycopy(b.twistAngles, 0, b.previousTwistAngles, 0, b.twistAngles.size)
                }
            }
            isSleeping = true
        }

        if (isSleeping) {
            for (i in bands.indices) {
                if (bands[i].fadeOut > 0f && bands[i].active) {
                    isSleeping = false
                    idleTimer = 0f
                    break
                }
            }
            if (isSleeping) {
                accumulator = 0f
                return
            }
        }

        // Advance suck-into-hole animations
        advanceSuckAnimations(clampedDt)

        accumulator += clampedDt
        val stepsAvailable = (accumulator / dt).toInt().coerceAtLeast(0)
        val n = min(stepsAvailable, maxSubstepsPerFrame)
        accumulator -= n.toFloat() * dt
        if (stepsAvailable > maxSubstepsPerFrame) {
            debugDroppedSubsteps += (stepsAvailable - maxSubstepsPerFrame)
            accumulator = min(accumulator, dt * maxSubstepsPerFrame.toFloat())
        }
        if (n <= 0) return

        // Smooth tension transition
        val targetTension = if (dragInfo != null) 1.0f else ropeTension
        if (currentTension != targetTension) {
            val diff = targetTension - currentTension
            val step = tensionSpeed * clampedDt
            if (abs(diff) <= step) {
                currentTension = targetTension
            } else {
                currentTension += if (diff > 0) step else -step
            }
        }

        logTimer += deltaTime
        val shouldLog = logTimer >= logEvery
        if (shouldLog) logTimer = 0f

        if (shouldLog) {
            val activeBandCount = bands.count { it.active }
            val firstPCount = bands.firstOrNull()?.particleCount ?: 0
            Log.i(TAG, "[FRAME] dt=${"%.4f".format(deltaTime)} substeps=$n fixedDt=${"%.5f".format(dt)} drag=${dragInfo != null} particles=$firstPCount bands=$activeBandCount constIter=$constraintIterations")
            if (ENABLE_AGENT_DEBUG_LOGS && perfStepSamples > 0) {
                val sampleCount = perfStepSamples.toDouble()
                debugLog(
                    runId = "baseline",
                    hypothesisId = "H_phase_split",
                    location = "VerletSimulator.kt:update",
                    message = "phase_profile_ms",
                    data = "{\"samples\":$perfStepSamples,\"integrateMs\":${"%.4f".format(perfIntegrateNs / sampleCount / 1_000_000.0)},\"broadphaseMs\":${"%.4f".format(perfBroadphaseNs / sampleCount / 1_000_000.0)},\"constraintMs\":${"%.4f".format(perfConstraintNs / sampleCount / 1_000_000.0)},\"postSolveMs\":${"%.4f".format(perfPostSolveNs / sampleCount / 1_000_000.0)},\"frictionStickyMs\":${"%.4f".format(perfFrictionStickyNs / sampleCount / 1_000_000.0)},\"effectiveIters\":$constraintIterations,\"dragActive\":${dragInfo != null}}"
                )
                perfIntegrateNs = 0L
                perfBroadphaseNs = 0L
                perfConstraintNs = 0L
                perfPostSolveNs = 0L
                perfFrictionStickyNs = 0L
                perfStepSamples = 0
            }
            if (ENABLE_AGENT_DEBUG_LOGS && debugDroppedSubsteps > 0) {
                debugLog(
                    runId = "post-fix",
                    hypothesisId = "H_substep_budget",
                    location = "VerletSimulator.kt:update",
                    message = "substep_backlog_dropped",
                    data = "{\"droppedSubsteps\":$debugDroppedSubsteps,\"maxSubstepsPerFrame\":$maxSubstepsPerFrame,\"accumulator\":$accumulator,\"dragActive\":${dragInfo != null}}"
                )
            }
            if (debugDroppedSubsteps > 0) {
                debugDroppedSubsteps = 0
            }
        }

        val drag = dragInfo
        if (drag != null && hasDragTargetPos) {
            val idx = if (drag.endIndex == 0) 0 else (bands[drag.bandIndex].particleCount - 1) * 3
            val pos = bands[drag.bandIndex].positions
            val prev = bands[drag.bandIndex].previousPositions
            val sx = if (hasDragStartPos) dragStartPosX else pos[idx]
            val sy = if (hasDragStartPos) dragStartPosY else pos[idx + 1]
            val sz = if (hasDragStartPos) dragStartPosZ else pos[idx + 2]

            for (s in 1..n) {
                val t = s.toFloat() / n.toFloat()
                val ix = sx + (dragTargetPosX - sx) * t
                val iy = sy + (dragTargetPosY - sy) * t
                val iz = sz + (dragTargetPosZ - sz) * t
                pos[idx] = ix; pos[idx + 1] = iy; pos[idx + 2] = iz
                prev[idx] = ix; prev[idx + 1] = iy; prev[idx + 2] = iz
                verletStep(collide = true, stepDt = dt)
            }
            dragStartPosX = dragTargetPosX
            dragStartPosY = dragTargetPosY
            dragStartPosZ = dragTargetPosZ
            hasDragStartPos = true
        } else {
            for (s in 0 until n) {
                verletStep(collide = true, stepDt = dt)
            }
        }

        // Post-drag lower animation
        updateLowerAnimation(clampedDt)
    }

    private fun advanceSuckAnimations(clampedDt: Float) {
        for (i in bands.indices) {
            val band = bands[i]
            if (band.fadeOut <= 0f || !band.active) continue
            val hole = band.suckHole
            if (hole < 0) continue

            val n = band.particleCount
            val hi = hole * 2
            val holeX = holePositions[hi]
            val holeY = holePositions[hi + 1]
            val holeElev = holeSurfaceZ(hole)
            val holeBelowX = holeX
            val holeBelowY = holeY
            val holeBelowZ = holeElev - holeDepth

            val pullSpeed = Band.FADE_OUT_SPEED * band.segmentLength
            band.suckConsumed += pullSpeed * clampedDt

            val fromEnd = band.suckFromEnd
            val suckSegs = band.suckSegLengths
            val origPositions = band.suckOrigPositions
            val R = band.radius

            val arcLen = FloatArray(n)
            if (fromEnd == 1) {
                for (k in 1 until n) {
                    val segL = if (k - 1 < suckSegs.size) suckSegs[k - 1] else band.segmentLength
                    arcLen[k] = arcLen[k - 1] + segL
                }
            } else {
                for (k in n - 2 downTo 0) {
                    val segL = if (k < suckSegs.size) suckSegs[k] else band.segmentLength
                    arcLen[k] = arcLen[k + 1] + segL
                }
            }

            val totalArc = if (fromEnd == 1) arcLen[n - 1] else arcLen[0]
            val consumed = band.suckConsumed

            for (k in 0 until n) {
                val shifted = arcLen[k] - consumed
                val po = k * 3

                if (shifted <= 0f) {
                    band.positions[po] = holeBelowX
                    band.positions[po + 1] = holeBelowY
                    band.positions[po + 2] = holeBelowZ + shifted
                } else {
                    if (fromEnd == 1) {
                        var seg = 0
                        var acc = 0f
                        while (seg < n - 1) {
                            val segL = if (seg < suckSegs.size) suckSegs[seg] else band.segmentLength
                            if (acc + segL >= shifted) break
                            acc += segL
                            seg++
                        }
                        val segL = if (seg < suckSegs.size) suckSegs[seg] else band.segmentLength
                        val t = if (segL > 1e-9f) (shifted - acc) / segL else 0f
                        val p0 = seg * 3
                        val p1 = if (seg + 1 < n) (seg + 1) * 3 else p0
                        band.positions[po] = origPositions[p0] + (origPositions[p1] - origPositions[p0]) * min(t, 1f)
                        band.positions[po + 1] = origPositions[p0 + 1] + (origPositions[p1 + 1] - origPositions[p0 + 1]) * min(t, 1f)
                        band.positions[po + 2] = origPositions[p0 + 2] + (origPositions[p1 + 2] - origPositions[p0 + 2]) * min(t, 1f)
                    } else {
                        var seg = n - 1
                        var acc = 0f
                        while (seg > 0) {
                            val segL = if (seg - 1 < suckSegs.size) suckSegs[seg - 1] else band.segmentLength
                            if (acc + segL >= shifted) break
                            acc += segL
                            seg--
                        }
                        val segL = if (seg - 1 >= 0 && seg - 1 < suckSegs.size) suckSegs[seg - 1] else band.segmentLength
                        val t = if (segL > 1e-9f) (shifted - acc) / segL else 0f
                        val p0 = seg * 3
                        val p1 = if (seg - 1 >= 0) (seg - 1) * 3 else p0
                        band.positions[po] = origPositions[p0] + (origPositions[p1] - origPositions[p0]) * min(t, 1f)
                        band.positions[po + 1] = origPositions[p0 + 1] + (origPositions[p1 + 1] - origPositions[p0 + 1]) * min(t, 1f)
                        band.positions[po + 2] = origPositions[p0 + 2] + (origPositions[p1 + 2] - origPositions[p0 + 2]) * min(t, 1f)
                    }

                    val surfZ = boardSurfaceZ(x = band.positions[po], y = band.positions[po + 1])
                    if (band.positions[po + 2] >= surfZ && band.positions[po + 2] < surfZ + R) {
                        band.positions[po + 2] = surfZ + R
                    }
                }

                band.previousPositions[po] = band.positions[po]
                band.previousPositions[po + 1] = band.positions[po + 1]
                band.previousPositions[po + 2] = band.positions[po + 2]
            }

            if (consumed >= totalArc) {
                band.fadeOut = 1f
                band.active = false
                band.pinStart = -1
                band.pinEnd = -1
                band.suckHole = -1
            } else {
                var aboveCount = 0
                for (k in 0 until n) {
                    val ko = k * 3
                    val surfZ = boardSurfaceZ(x = band.positions[ko], y = band.positions[ko + 1])
                    if (band.positions[ko + 2] >= surfZ) aboveCount++
                }
                band.fadeOut = min(1.0f - aboveCount.toFloat() / n.toFloat(), 0.999f)
            }
        }
    }

    private fun updateLowerAnimation(deltaTime: Float) {
        if (lowerAnimations.isEmpty()) return

        val keys = lowerAnimations.keys.toList()
        for (key in keys) {
            val anim = lowerAnimations[key] ?: continue
            anim.timer += deltaTime

            val bi = anim.bandIndex
            val band = bands[bi]
            val idx = if (anim.endIndex == 0) 0 else (band.particleCount - 1) * 3

            if (anim.hasReturnPos) {
                val t = min(anim.timer / anim.returnDuration, 1.0f)
                val eased = 1.0f - (1.0f - t) * (1.0f - t)
                val px = anim.startPosX + (anim.returnPosX - anim.startPosX) * eased
                val py = anim.startPosY + (anim.returnPosY - anim.startPosY) * eased
                val pz = anim.startPosZ + (anim.returnPosZ - anim.startPosZ) * eased
                band.positions[idx] = px; band.positions[idx + 1] = py; band.positions[idx + 2] = pz
                band.previousPositions[idx] = px; band.previousPositions[idx + 1] = py; band.previousPositions[idx + 2] = pz

                if (t >= 1.0f) {
                    anim.startPosX = anim.returnPosX
                    anim.startPosY = anim.returnPosY
                    anim.startPosZ = anim.returnPosZ
                    anim.hasReturnPos = false
                    anim.timer = 0f
                }

                lowerAnimations[key] = anim
                continue
            }

            val holePos = holePosition3D(anim.targetHole)
            val t = min(anim.timer / LowerAnimation.DURATION, 1.0f)
            val eased = 1.0f - (1.0f - t) * (1.0f - t)
            val px = anim.startPosX + (holePos[0] - anim.startPosX) * eased
            val py = anim.startPosY + (holePos[1] - anim.startPosY) * eased
            val pz = anim.startPosZ + (holePos[2] - anim.startPosZ) * eased
            band.positions[idx] = px; band.positions[idx + 1] = py; band.positions[idx + 2] = pz
            band.previousPositions[idx] = px; band.previousPositions[idx + 1] = py; band.previousPositions[idx + 2] = pz

            if (t >= 1.0f) {
                if (anim.endIndex == 0) {
                    band.pinStart = anim.targetHole
                } else {
                    band.pinEnd = anim.targetHole
                }
                band.positions[idx] = holePos[0]; band.positions[idx + 1] = holePos[1]; band.positions[idx + 2] = holePos[2]
                band.previousPositions[idx] = holePos[0]; band.previousPositions[idx + 1] = holePos[1]; band.previousPositions[idx + 2] = holePos[2]
                lowerAnimations.remove(key)
                continue
            }

            lowerAnimations[key] = anim
        }
    }

    // =========================================================================
    // MARK: - doSteps (used during init)
    // =========================================================================

    fun doSteps(n: Int, collide: Boolean) {
        for (s in 0 until n) {
            verletStep(collide = collide, stepDt = initDt)
        }
    }

    // =========================================================================
    // MARK: - Verlet step (core physics)
    // =========================================================================

    private fun verletStep(collide: Boolean, stepDt: Float) {
        val stepStartNs = if (ENABLE_AGENT_DEBUG_LOGS) System.nanoTime() else 0L
        val dt2 = stepDt * stepDt
        debugStepCounter++
        debugCollisionHits = 0
        debugDragCollisionHits = 0
        debugMaxOverlap = 0f
        debugLastPairCount = 0
        debugMaxParticleDisplacement = 0f
        debugClampedParticles = 0

        // 1. Verlet position update + velocity limiting
        val gravZ = gravity * dt2
        for (bi in bands.indices) {
            val band = bands[bi]
            if (!band.active || band.fadeOut != 0f) continue
            val n = band.particleCount
            val pos = band.positions
            val old = band.previousPositions
            val maxMove = band.radius * 2.0f
            for (i in 1 until n - 1) {
                val off = i * 3
                val px = pos[off]; val py = pos[off + 1]; val pz = pos[off + 2]
                val ox = old[off]; val oy = old[off + 1]; val oz = old[off + 2]
                var vx = (px - ox) * damping
                var vy = (py - oy) * damping
                var vz = (pz - oz) * damping
                val velLen = sqrt(vx * vx + vy * vy + vz * vz)
                if (velLen > debugMaxParticleDisplacement) debugMaxParticleDisplacement = velLen
                if (velLen > maxMove) {
                    val s = maxMove / velLen
                    vx *= s; vy *= s; vz *= s
                    debugClampedParticles++
                }
                old[off] = px; old[off + 1] = py; old[off + 2] = pz
                pos[off] = px + vx; pos[off + 1] = py + vy; pos[off + 2] = pz + vz + gravZ
            }

            if (band.crossSection.isRectangular) {
                val maxTwistVel = 0.08f
                val tw = band.twistAngles
                val otw = band.previousTwistAngles
                for (i in 1 until n - 1) {
                    val twist = tw[i]
                    val oldTwist = otw[i]
                    var twistVel = (twist - oldTwist) * twistDamping
                    twistVel = max(-maxTwistVel, min(maxTwistVel, twistVel))
                    otw[i] = twist
                    tw[i] = twist + twistVel
                }
            }
        }

        val afterIntegrateNs = if (ENABLE_AGENT_DEBUG_LOGS) System.nanoTime() else 0L

        // 2. Build active bands list for collision
        val active: IntArray
        if (collide) {
            var count = 0
            for (bi in bands.indices) {
                if (bands[bi].active && bands[bi].fadeOut == 0f) count++
            }
            active = IntArray(count)
            var idx = 0
            for (bi in bands.indices) {
                if (bands[bi].active && bands[bi].fadeOut == 0f) {
                    active[idx++] = bi
                }
            }
        } else {
            active = IntArray(0)
        }

        val effectiveIters = constraintIterations

        // Build collision pair list (broadphase)
        var collisionPairs: LongArray = if (collide) buildCollisionPairs(active) else LongArray(0)
        debugLastPairCount = collisionPairs.size

        // Recompute material frames for rectangular bands
        recomputeFrames()

        // Gravity torque for rectangular bands
        for (bi in bands.indices) {
            val band = bands[bi]
            if (!band.active || band.fadeOut != 0f || !band.crossSection.isRectangular) continue
            val n = band.particleCount
            if (bi >= cachedFrames.size || cachedFrames[bi].size != n) continue
            val frames = cachedFrames[bi]
            for (i in 1 until n - 1) {
                val d1z = frames[i].d1z
                val torque = -d1z * gravityTorqueStrength * dt2
                val maxTorque = 0.01f
                band.twistAngles[i] += max(-maxTorque, min(maxTorque, torque))
            }
        }

        val afterBroadphaseNs = if (ENABLE_AGENT_DEBUG_LOGS) System.nanoTime() else 0L

        // Constraint + collision iterations
        for (iter in 0 until effectiveIters) {
            for (bi in bands.indices) {
                if (bands[bi].active && bands[bi].fadeOut == 0f) {
                    bandConstraints(bi, stepDt)
                }
            }
            if (collide) {
                if (iter > 0 && iter % 3 == 0) {
                    collisionPairs = buildCollisionPairs(active)
                }
                resolveCollisionPairs(collisionPairs, injectVelocity = false)
            }
        }

        val afterConstraintNs = if (ENABLE_AGENT_DEBUG_LOGS) System.nanoTime() else 0L

        // Post-solve: collision-only passes until converged
        var postSolveHadCollision = false
        if (collide) {
            for (pass in 0 until 3) {
                val hadCollision = resolveCollisionPairs(collisionPairs, injectVelocity = true)
                if (hadCollision) postSolveHadCollision = true
                for (biIdx in active.indices) {
                    val bi = active[biIdx]
                    val band = bands[bi]
                    val n = band.particleCount
                    val pos = band.positions
                    val prev = band.previousPositions

                    if (band.pinStart >= 0) {
                        val hp = holePosition3D(band.pinStart)
                        pos[0] = hp[0]; pos[1] = hp[1]; pos[2] = hp[2]
                        prev[0] = hp[0]; prev[1] = hp[1]; prev[2] = hp[2]
                    }
                    if (band.pinEnd >= 0) {
                        val hp = holePosition3D(band.pinEnd)
                        val lo = (n - 1) * 3
                        pos[lo] = hp[0]; pos[lo + 1] = hp[1]; pos[lo + 2] = hp[2]
                        prev[lo] = hp[0]; prev[lo + 1] = hp[1]; prev[lo + 2] = hp[2]
                    }

                    if (band.crossSection.isRectangular && bi < cachedFrames.size && cachedFrames[bi].size == n) {
                        val cs = band.crossSection
                        val frames = cachedFrames[bi]
                        for (i in 1 until n - 1) {
                            val off = i * 3
                            val frame = frames[i]
                            val zExtent = cs.effectiveRadius(
                                0f, 0f, 1f,
                                frame.d1x, frame.d1y, frame.d1z,
                                frame.d2x, frame.d2y, frame.d2z
                            )
                            val floorZ = boardSurfaceZ(x = pos[off], y = pos[off + 1]) + zExtent
                            if (pos[off + 2] < floorZ) pos[off + 2] = floorZ
                        }
                    } else {
                        val R = band.radius
                        for (i in 1 until n - 1) {
                            val off = i * 3
                            val floorZ = boardSurfaceZ(x = pos[off], y = pos[off + 1]) + R
                            if (pos[off + 2] < floorZ) pos[off + 2] = floorZ
                        }
                    }
                }
                if (!hadCollision) break
            }
        }

        if (ENABLE_AGENT_DEBUG_LOGS && debugStepCounter % 120 == 0) {
            debugLog(
                runId = "baseline",
                hypothesisId = "H4",
                location = "VerletSimulator.kt:verletStep",
                message = "solver_profile",
                data = "{\"activeBands\":${active.size},\"pairCount\":$debugLastPairCount,\"effectiveIters\":$effectiveIters,\"currentTension\":$currentTension,\"dragActive\":${dragInfo != null},\"postSolveHadCollision\":$postSolveHadCollision}"
            )
            debugLog(
                runId = "baseline",
                hypothesisId = "H2_H3",
                location = "VerletSimulator.kt:verletStep",
                message = "collision_resolution_profile",
                data = "{\"collisionHits\":$debugCollisionHits,\"dragCollisionHits\":$debugDragCollisionHits,\"maxOverlap\":$debugMaxOverlap,\"maxParticleDisplacement\":$debugMaxParticleDisplacement,\"clampedParticles\":$debugClampedParticles}"
            )
            if (active.size >= 2 && debugLastPairCount == 0) {
                debugLog(
                    runId = "baseline",
                    hypothesisId = "H1",
                    location = "VerletSimulator.kt:verletStep",
                    message = "broadphase_zero_pairs",
                    data = "{\"activeBands\":${active.size},\"effectiveIters\":$effectiveIters,\"currentTension\":$currentTension,\"dragActive\":${dragInfo != null}}"
                )
            }
        }

        val afterPostSolveNs = if (ENABLE_AGENT_DEBUG_LOGS) System.nanoTime() else 0L

        // Board friction
        val boardMu = frictionCoefficient * 0.5f
        if (boardMu > 0f) {
            for (biIdx in active.indices) {
                val bi = active[biIdx]
                val band = bands[bi]
                val n = band.particleCount
                val R = band.radius
                val pos = band.positions
                val prev = band.previousPositions
                for (i in 1 until n - 1) {
                    val off = i * 3
                    val floorZ = boardSurfaceZ(x = pos[off], y = pos[off + 1]) + R
                    if (pos[off + 2] <= floorZ + 1e-4f) {
                        val vx = pos[off] - prev[off]
                        val vy = pos[off + 1] - prev[off + 1]
                        val velLen = sqrt(vx * vx + vy * vy)
                        if (velLen > 1e-8f) {
                            val scale = max(0.0f, 1.0f - boardMu)
                            prev[off] = pos[off] - vx * scale
                            prev[off + 1] = pos[off + 1] - vy * scale
                        }
                    }
                }
            }
        }

        // Sticky adhesion
        if (stickyEnabled && collide && active.size >= 2) {
            updateStickyBonds(active, stepDt)
        }

        if (ENABLE_AGENT_DEBUG_LOGS) {
            val stepEndNs = System.nanoTime()
            perfIntegrateNs += (afterIntegrateNs - stepStartNs)
            perfBroadphaseNs += (afterBroadphaseNs - afterIntegrateNs)
            perfConstraintNs += (afterConstraintNs - afterBroadphaseNs)
            perfPostSolveNs += (afterPostSolveNs - afterConstraintNs)
            perfFrictionStickyNs += (stepEndNs - afterPostSolveNs)
            perfStepSamples++
        }
    }

    // =========================================================================
    // MARK: - Sticky adhesion
    // =========================================================================

    private fun updateStickyBonds(activeBands: IntArray, stepDt: Float) {
        val strength = stickyStrength
        val breakThresh = stickyBreakThreshold

        stickyFormCounter++
        val shouldForm = stickyFormCounter >= stickyFormInterval
        if (shouldForm) {
            stickyFormCounter = 0
            formStickyBonds(activeBands)
        }

        resolveStickyBonds(stepDt, strength, breakThresh)
    }

    private fun formStickyBonds(activeBands: IntArray) {
        val radiusMul = stickyRadius
        val maxBonds = 128

        for (ai in activeBands.indices) {
            val bi = activeBands[ai]
            val bandI = bands[bi]
            if (!bandI.active || bandI.fadeOut != 0f) continue
            val posI = bandI.positions
            val nI = bandI.particleCount
            val rI = bandI.radius

            for (aj in ai + 1 until activeBands.size) {
                if (stickyBonds.size >= maxBonds) return

                val bj = activeBands[aj]
                val bandJ = bands[bj]
                if (!bandJ.active || bandJ.fadeOut != 0f) continue
                val posJ = bandJ.positions
                val nJ = bandJ.particleCount
                val rJ = bandJ.radius

                val captureR = (rI + rJ) * radiusMul
                val captureR2 = captureR * captureR
                val step = max(2, min(nI, nJ) / 6)

                var pi = 1
                while (pi < nI - 1) {
                    val oI = pi * 3
                    val pxI = posI[oI]; val pyI = posI[oI + 1]; val pzI = posI[oI + 2]

                    var pj = 1
                    while (pj < nJ - 1) {
                        val oJ = pj * 3
                        val dx = pxI - posJ[oJ]
                        val dy = pyI - posJ[oJ + 1]
                        val dz = pzI - posJ[oJ + 2]
                        val d2 = dx * dx + dy * dy + dz * dz

                        if (d2 < captureR2) {
                            val key = stickyKey(bi, pi, bj, pj)
                            if (key !in stickyBondKeys) {
                                stickyBondKeys.add(key)
                                stickyBonds.add(StickyBond(bi, pi, bj, pj, sqrt(d2)))
                            }
                        }
                        pj += step
                    }
                    pi += step
                }
            }
        }
    }

    private fun resolveStickyBonds(stepDt: Float, strength: Float, breakThresh: Float) {
        val drag = dragInfo
        var i = 0
        while (i < stickyBonds.size) {
            val bond = stickyBonds[i]
            val bandA = bands.getOrNull(bond.bandA)
            val bandB = bands.getOrNull(bond.bandB)

            if (bandA == null || bandB == null || !bandA.active || !bandB.active ||
                bandA.fadeOut != 0f || bandB.fadeOut != 0f
            ) {
                removeStickyBond(i)
                continue
            }

            val oA = bond.particleA * 3
            val oB = bond.particleB * 3
            if (oA + 2 >= bandA.positions.size || oB + 2 >= bandB.positions.size) {
                removeStickyBond(i)
                continue
            }

            val posA = bandA.positions
            val posB = bandB.positions
            val dx = posA[oA] - posB[oB]
            val dy = posA[oA + 1] - posB[oB + 1]
            val dz = posA[oA + 2] - posB[oB + 2]
            val dist2 = dx * dx + dy * dy + dz * dz

            val maxDist = bond.restDistance + breakThresh * (bandA.radius + bandB.radius)
            if (dist2 > maxDist * maxDist) {
                bond.life -= stepDt * 8f
                if (bond.life <= 0f) {
                    removeStickyBond(i)
                    continue
                }
            } else {
                bond.life = min(1f, bond.life + stepDt * 4f)
            }

            val dist = sqrt(dist2)
            if (dist > 1e-6f && dist > bond.restDistance) {
                val correction = (dist - bond.restDistance) * strength * bond.life * 0.5f
                val invDist = 1f / dist
                val cx = dx * invDist * correction
                val cy = dy * invDist * correction
                val cz = dz * invDist * correction

                val skipA = drag != null && drag.bandIndex == bond.bandA
                val skipB = drag != null && drag.bandIndex == bond.bandB

                if (!skipA) {
                    posA[oA] -= cx; posA[oA + 1] -= cy; posA[oA + 2] -= cz
                }
                if (!skipB) {
                    posB[oB] += cx; posB[oB + 1] += cy; posB[oB + 2] += cz
                }

                val damp = stickyDamping * bond.life
                if (damp > 0f) {
                    val prevA = bandA.previousPositions
                    val prevB = bandB.previousPositions
                    val relVx = (posA[oA] - prevA[oA]) - (posB[oB] - prevB[oB])
                    val relVy = (posA[oA + 1] - prevA[oA + 1]) - (posB[oB + 1] - prevB[oB + 1])
                    val relVz = (posA[oA + 2] - prevA[oA + 2]) - (posB[oB + 2] - prevB[oB + 2])
                    val nX = dx * invDist; val nY = dy * invDist; val nZ = dz * invDist
                    val dampF = (relVx * nX + relVy * nY + relVz * nZ) * damp * 0.5f
                    if (!skipA) {
                        prevA[oA] += nX * dampF; prevA[oA + 1] += nY * dampF; prevA[oA + 2] += nZ * dampF
                    }
                    if (!skipB) {
                        prevB[oB] -= nX * dampF; prevB[oB + 1] -= nY * dampF; prevB[oB + 2] -= nZ * dampF
                    }
                }
            }
            i++
        }
    }

    private fun removeStickyBond(index: Int) {
        val bond = stickyBonds[index]
        stickyBondKeys.remove(stickyKey(bond.bandA, bond.particleA, bond.bandB, bond.particleB))
        val last = stickyBonds.size - 1
        if (index < last) stickyBonds[index] = stickyBonds[last]
        stickyBonds.removeAt(last)
    }

    private fun stickyKey(bandA: Int, partA: Int, bandB: Int, partB: Int): Long {
        val a = min(bandA, bandB)
        val b = max(bandA, bandB)
        val pA = if (bandA <= bandB) partA else partB
        val pB = if (bandA <= bandB) partB else partA
        return (a.toLong() shl 48) or (pA.toLong() shl 32) or (b.toLong() shl 16) or pB.toLong()
    }

    fun clearStickyBonds() {
        stickyBonds.clear()
        stickyBondKeys.clear()
    }

    // =========================================================================
    // MARK: - Resample
    // =========================================================================

    private fun resampleBand(bi: Int) {
        val band = bands[bi]
        val n = band.particleCount
        if (n < 4) return

        val pos = band.positions
        val prev = band.previousPositions
        val twist = band.twistAngles
        val prevTwist = band.previousTwistAngles

        // Compute curvature
        val curvature = FloatArray(n)
        for (i in 1 until n - 1) {
            val o0 = (i - 1) * 3; val o1 = i * 3; val o2 = (i + 1) * 3
            val d0x = pos[o1] - pos[o0]; val d0y = pos[o1 + 1] - pos[o0 + 1]; val d0z = pos[o1 + 2] - pos[o0 + 2]
            val d1x = pos[o2] - pos[o1]; val d1y = pos[o2 + 1] - pos[o1 + 1]; val d1z = pos[o2 + 2] - pos[o1 + 2]
            val len0 = sqrt(d0x * d0x + d0y * d0y + d0z * d0z)
            val len1 = sqrt(d1x * d1x + d1y * d1y + d1z * d1z)
            if (len0 > 1e-9f && len1 > 1e-9f) {
                val cosA = (d0x * d1x + d0y * d1y + d0z * d1z) / (len0 * len1)
                curvature[i] = max(1.0f - cosA, 0f)
            }
        }

        val curvatureScale = 8.0f
        val wArcLen = FloatArray(n)
        for (i in 1 until n) {
            val o0 = (i - 1) * 3; val o1 = i * 3
            val dx = pos[o1] - pos[o0]; val dy = pos[o1 + 1] - pos[o0 + 1]; val dz = pos[o1 + 2] - pos[o0 + 2]
            val segLen = sqrt(dx * dx + dy * dy + dz * dz)
            val avgCurv = (curvature[i - 1] + curvature[i]) * 0.5f
            val weight = 1.0f + curvatureScale * avgCurv
            wArcLen[i] = wArcLen[i - 1] + segLen * weight
        }
        val totalW = wArcLen[n - 1]
        if (totalW <= 1e-6f) return

        val arcLen = FloatArray(n)
        for (i in 1 until n) {
            val o0 = (i - 1) * 3; val o1 = i * 3
            val dx = pos[o1] - pos[o0]; val dy = pos[o1 + 1] - pos[o0 + 1]; val dz = pos[o1 + 2] - pos[o0 + 2]
            arcLen[i] = arcLen[i - 1] + sqrt(dx * dx + dy * dy + dz * dz)
        }

        var maxSeg = 0f; var minSeg = Float.MAX_VALUE
        for (i in 0 until n - 1) {
            val s = arcLen[i + 1] - arcLen[i]
            if (s > maxSeg) maxSeg = s
            if (s > 1e-9f && s < minSeg) minSeg = s
        }
        if (minSeg < 1e-9f || maxSeg / max(minSeg, 1e-9f) <= 1.5f) return

        // Save original arrays for interpolation
        val origPos = pos.copyOf()
        val origPrev = prev.copyOf()
        val origTwist = twist.copyOf()
        val origPrevTwist = prevTwist.copyOf()

        val idealW = totalW / (n - 1).toFloat()
        var seg = 0
        for (i in 1 until n - 1) {
            val targetW = idealW * i.toFloat()
            while (seg < n - 2 && wArcLen[seg + 1] < targetW) {
                seg++
            }
            val wStart = wArcLen[seg]
            val wLen = wArcLen[seg + 1] - wStart
            val t = if (wLen > 1e-9f) (targetW - wStart) / wLen else 0f

            val o0 = seg * 3; val o1 = (seg + 1) * 3; val oi = i * 3
            pos[oi] = origPos[o0] + (origPos[o1] - origPos[o0]) * t
            pos[oi + 1] = origPos[o0 + 1] + (origPos[o1 + 1] - origPos[o0 + 1]) * t
            pos[oi + 2] = origPos[o0 + 2] + (origPos[o1 + 2] - origPos[o0 + 2]) * t

            prev[oi] = origPrev[o0] + (origPrev[o1] - origPrev[o0]) * t
            prev[oi + 1] = origPrev[o0 + 1] + (origPrev[o1 + 1] - origPrev[o0 + 1]) * t
            prev[oi + 2] = origPrev[o0 + 2] + (origPrev[o1 + 2] - origPrev[o0 + 2]) * t

            twist[i] = origTwist[seg] + (origTwist[seg + 1] - origTwist[seg]) * t
            prevTwist[i] = origPrevTwist[seg] + (origPrevTwist[seg + 1] - origPrevTwist[seg]) * t
        }
    }

    // =========================================================================
    // MARK: - Band constraints
    // =========================================================================

    private fun bandConstraints(bi: Int, dt: Float) {
        val band = bands[bi]
        val n = band.particleCount
        val segLen = band.segmentLength * currentTension
        val alpha = max(bendCompliance, 0f) / max(dt * dt, 1e-8f)
        val bendCoupling = max(0f, min(bendVelocityCoupling, 1f))
        val pinS = band.pinStart
        val pinE = band.pinEnd
        val cs = band.crossSection
        val isRect = cs.isRectangular
        val R = band.radius

        val pos = band.positions
        val prev = band.previousPositions

        val hasFrames = isRect && bi < cachedFrames.size && cachedFrames[bi].size == n
        val frames = if (hasFrames) cachedFrames[bi] else emptyArray()

        // Distance constraints (red-black Gauss-Seidel)
        for (offset in 0..1) {
            var idx = offset
            while (idx < n - 1) {
                val o0 = idx * 3; val o1 = (idx + 1) * 3
                val dx = pos[o1] - pos[o0]; val dy = pos[o1 + 1] - pos[o0 + 1]; val dz = pos[o1 + 2] - pos[o0 + 2]
                val dist2 = dx * dx + dy * dy + dz * dz
                if (dist2 > 1e-12f) {
                    val dist = sqrt(dist2)
                    val factor = (dist - segLen) / dist * 0.5f
                    val cx = dx * factor; val cy = dy * factor; val cz = dz * factor
                    if (idx > 0) {
                        pos[o0] += cx; pos[o0 + 1] += cy; pos[o0 + 2] += cz
                    }
                    if (idx + 1 < n - 1) {
                        pos[o1] -= cx; pos[o1 + 1] -= cy; pos[o1 + 2] -= cz
                    }
                }
                idx += 2
            }
        }

        // Bending constraints
        if (n >= 3) {
            for (i in 1 until n - 1) {
                val o0 = (i - 1) * 3; val o1 = i * 3; val o2 = (i + 1) * 3
                val cX = pos[o0] - 2f * pos[o1] + pos[o2]
                val cY = pos[o0 + 1] - 2f * pos[o1 + 1] + pos[o2 + 1]
                val cZ = pos[o0 + 2] - 2f * pos[o1 + 2] + pos[o2 + 2]
                val c2 = cX * cX + cY * cY + cZ * cZ
                if (c2 < 1e-14f) continue

                val denom = 6.0f + alpha
                val lx = cX / denom; val ly = cY / denom; val lz = cZ / denom

                var d0x = -lx; var d0y = -ly; var d0z = -lz
                var d1x = lx * 2f; var d1y = ly * 2f; var d1z = lz * 2f
                var d2x = -lx; var d2y = -ly; var d2z = -lz

                if (i - 1 == 0) {
                    // Pin start: redistribute d0 to d1 and d2
                    d1x += d0x * 0.5f; d1y += d0y * 0.5f; d1z += d0z * 0.5f
                    d2x += d0x * 0.5f; d2y += d0y * 0.5f; d2z += d0z * 0.5f
                    d0x = 0f; d0y = 0f; d0z = 0f
                }
                if (i + 1 == n - 1) {
                    // Pin end: redistribute d2 to d0 and d1
                    d0x += d2x * 0.5f; d0y += d2y * 0.5f; d0z += d2z * 0.5f
                    d1x += d2x * 0.5f; d1y += d2y * 0.5f; d1z += d2z * 0.5f
                    d2x = 0f; d2y = 0f; d2z = 0f
                }

                pos[o0] += d0x; pos[o0 + 1] += d0y; pos[o0 + 2] += d0z
                pos[o1] += d1x; pos[o1 + 1] += d1y; pos[o1 + 2] += d1z
                pos[o2] += d2x; pos[o2 + 1] += d2y; pos[o2 + 2] += d2z

                prev[o0] += d0x * bendCoupling; prev[o0 + 1] += d0y * bendCoupling; prev[o0 + 2] += d0z * bendCoupling
                prev[o1] += d1x * bendCoupling; prev[o1 + 1] += d1y * bendCoupling; prev[o1 + 2] += d1z * bendCoupling
                prev[o2] += d2x * bendCoupling; prev[o2 + 1] += d2y * bendCoupling; prev[o2 + 2] += d2z * bendCoupling
            }
        }

        // Pin constraints
        if (pinS >= 0) {
            val hp = holePosition3D(pinS)
            pos[0] = hp[0]; pos[1] = hp[1]; pos[2] = hp[2]
        }
        if (pinE >= 0) {
            val hp = holePosition3D(pinE)
            val lo = (n - 1) * 3
            pos[lo] = hp[0]; pos[lo + 1] = hp[1]; pos[lo + 2] = hp[2]
        }

        // Board collision
        if (frames.isNotEmpty()) {
            val upNx = 0f; val upNy = 0f; val upNz = 1f
            for (i in 1 until n - 1) {
                val off = i * 3
                val frame = frames[i]
                val zExtent = cs.effectiveRadius(upNx, upNy, upNz, frame.d1x, frame.d1y, frame.d1z, frame.d2x, frame.d2y, frame.d2z)
                val floorZ = boardSurfaceZ(x = pos[off], y = pos[off + 1]) + zExtent
                if (pos[off + 2] < floorZ) pos[off + 2] = floorZ
            }
        } else {
            for (i in 1 until n - 1) {
                val off = i * 3
                val floorZ = boardSurfaceZ(x = pos[off], y = pos[off + 1]) + R
                if (pos[off + 2] < floorZ) pos[off + 2] = floorZ
            }
        }

        // Twist constraints (rectangular only)
        if (isRect) {
            val stiffness = twistStiffness
            val tw = band.twistAngles
            for (i in 0 until n - 1) {
                val diff = tw[i + 1] - tw[i]
                val corr = diff * stiffness * 0.5f
                if (i > 0) tw[i] += corr
                if (i + 1 < n - 1) tw[i + 1] -= corr
            }
            val zeroRestoring = 0.15f
            for (i in 1 until n - 1) {
                tw[i] *= (1.0f - zeroRestoring)
            }
        }
    }

    // =========================================================================
    // MARK: - Collision
    // =========================================================================

    private fun packCollisionPair(bandA: Int, segA: Int, bandB: Int, segB: Int): Long {
        return ((bandA.toLong() and 0xFFFFL) shl 48) or
                ((segA.toLong() and 0xFFFFL) shl 32) or
                ((bandB.toLong() and 0xFFFFL) shl 16) or
                (segB.toLong() and 0xFFFFL)
    }

    fun buildCollisionPairs(activeBands: IntArray): LongArray {
        if (activeBands.isEmpty()) return LongArray(0)
        val pairs = ArrayList<Long>(512)

        for (ai in activeBands.indices) {
            val bi = activeBands[ai]
            val bandI = bands[bi]
            val posI = bandI.positions
            val segsI = bandI.particleCount - 1
            val ri = bandI.crossSection.collisionRadius

            // Self-collision
            if (segsI > 4) {
                val minDist = ri + ri
                val segLen = bandI.segmentLength
                val selfSkip = if (segLen > 1e-6f) max(4, ceil(minDist * 2.5f / segLen).toInt()) else 4
                for (si in 0 until segsI) {
                    val a0 = si * 3; val a1 = (si + 1) * 3
                    val aMinX = min(posI[a0], posI[a1]) - minDist
                    val aMaxX = max(posI[a0], posI[a1]) + minDist
                    val aMinY = min(posI[a0 + 1], posI[a1 + 1]) - minDist
                    val aMaxY = max(posI[a0 + 1], posI[a1 + 1]) + minDist
                    val sjStart = si + selfSkip
                    if (sjStart >= segsI) continue
                    for (sj in sjStart until segsI) {
                        val b0 = sj * 3; val b1 = (sj + 1) * 3
                        if (max(posI[b0], posI[b1]) < aMinX || min(posI[b0], posI[b1]) > aMaxX) continue
                        if (max(posI[b0 + 1], posI[b1 + 1]) < aMinY || min(posI[b0 + 1], posI[b1 + 1]) > aMaxY) continue
                        pairs.add(packCollisionPair(bi, si, bi, sj))
                    }
                }
            }

            // Cross-band collision
            for (aj in ai + 1 until activeBands.size) {
                val bj = activeBands[aj]
                val bandJ = bands[bj]
                val posJ = bandJ.positions
                val segsJ = bandJ.particleCount - 1
                val minDist = ri + bandJ.crossSection.collisionRadius

                for (si in 0 until segsI) {
                    val a0 = si * 3; val a1 = (si + 1) * 3
                    val aMinX = min(posI[a0], posI[a1]) - minDist
                    val aMaxX = max(posI[a0], posI[a1]) + minDist
                    val aMinY = min(posI[a0 + 1], posI[a1 + 1]) - minDist
                    val aMaxY = max(posI[a0 + 1], posI[a1 + 1]) + minDist

                    for (sj in 0 until segsJ) {
                        val b0 = sj * 3; val b1 = (sj + 1) * 3
                        if (max(posJ[b0], posJ[b1]) < aMinX || min(posJ[b0], posJ[b1]) > aMaxX) continue
                        if (max(posJ[b0 + 1], posJ[b1 + 1]) < aMinY || min(posJ[b0 + 1], posJ[b1 + 1]) > aMaxY) continue
                        pairs.add(packCollisionPair(bi, si, bj, sj))
                    }
                }
            }
        }

        return LongArray(pairs.size) { idx -> pairs[idx] }
    }

    private fun resolveCollisionPairs(pairs: LongArray, injectVelocity: Boolean): Boolean {
        var found = false
        for (packed in pairs) {
            val bandA = ((packed ushr 48) and 0xFFFFL).toInt()
            val segA = ((packed ushr 32) and 0xFFFFL).toInt()
            val bandB = ((packed ushr 16) and 0xFFFFL).toInt()
            val segB = (packed and 0xFFFFL).toInt()
            if (collideSegments(bandA, segA, bandB, segB, injectVelocity)) {
                found = true
            }
        }
        return found
    }

    private fun collideSegments(bi: Int, si: Int, bj: Int, sj: Int, injectVelocity: Boolean): Boolean {
        val bandI = bands[bi]
        val bandJ = bands[bj]
        val maxDist = bandI.crossSection.collisionRadius + bandJ.crossSection.collisionRadius
        val maxDist2 = maxDist * maxDist

        val posI = bandI.positions
        val posJ = bandJ.positions
        val o0a = si * 3; val o1a = (si + 1) * 3
        val o0b = sj * 3; val o1b = (sj + 1) * 3

        val a0x = posI[o0a]; val a0y = posI[o0a + 1]; val a0z = posI[o0a + 2]
        val a1x = posI[o1a]; val a1y = posI[o1a + 1]; val a1z = posI[o1a + 2]
        val b0x = posJ[o0b]; val b0y = posJ[o0b + 1]; val b0z = posJ[o0b + 2]
        val b1x = posJ[o1b]; val b1y = posJ[o1b + 1]; val b1z = posJ[o1b + 2]

        val d1x = a1x - a0x; val d1y = a1y - a0y; val d1z = a1z - a0z
        val d2x = b1x - b0x; val d2y = b1y - b0y; val d2z = b1z - b0z
        val rx = a0x - b0x; val ry = a0y - b0y; val rz = a0z - b0z

        val a = d1x * d1x + d1y * d1y + d1z * d1z
        val e = d2x * d2x + d2y * d2y + d2z * d2z
        val f = d2x * rx + d2y * ry + d2z * rz
        val c = d1x * rx + d1y * ry + d1z * rz
        val b = d1x * d2x + d1y * d2y + d1z * d2z

        val denom = a * e - b * b
        var s = 0f
        var t: Float

        if (denom > 1e-12f) {
            s = ((b * f - c * e) / denom).coerceIn(0f, 1f)
        }
        t = (b * s + f) / max(e, 1e-12f)

        if (t < 0f) {
            t = 0f
            s = (-c / max(a, 1e-12f)).coerceIn(0f, 1f)
        } else if (t > 1f) {
            t = 1f
            s = ((b - c) / max(a, 1e-12f)).coerceIn(0f, 1f)
        }

        val closestAx = a0x + d1x * s; val closestAy = a0y + d1y * s; val closestAz = a0z + d1z * s
        val closestBx = b0x + d2x * t; val closestBy = b0y + d2y * t; val closestBz = b0z + d2z * t
        val diffX = closestAx - closestBx; val diffY = closestAy - closestBy; val diffZ = closestAz - closestBz
        val dist2 = diffX * diffX + diffY * diffY + diffZ * diffZ

        if (dist2 >= maxDist2) return false

        val dist: Float
        var normalX: Float; var normalY: Float; var normalZ: Float

        if (dist2 > 1e-12f) {
            dist = sqrt(dist2)
            val invDist = 1f / dist
            normalX = diffX * invDist; normalY = diffY * invDist; normalZ = diffZ * invDist
        } else {
            // Degenerate: use cross product of segment directions
            val axisX = vec3CrossX(d1y, d1z, d2y, d2z)
            val axisY = vec3CrossY(d1x, d1z, d2x, d2z)
            val axisZ = vec3CrossZ(d1x, d1y, d2x, d2y)
            val axisLen2 = axisX * axisX + axisY * axisY + axisZ * axisZ
            if (axisLen2 > 1e-12f) {
                val invLen = 1f / sqrt(axisLen2)
                normalX = axisX * invLen; normalY = axisY * invLen; normalZ = axisZ * invLen
            } else {
                val tLen2 = d1x * d1x + d1y * d1y + d1z * d1z
                val tx: Float; val ty: Float; val tz: Float
                if (tLen2 > 1e-12f) {
                    val invTLen = 1f / sqrt(tLen2)
                    tx = d1x * invTLen; ty = d1y * invTLen; tz = d1z * invTLen
                } else {
                    tx = 1f; ty = 0f; tz = 0f
                }
                val upX: Float; val upY: Float; val upZ: Float
                if (abs(tz) < 0.9f) {
                    upX = 0f; upY = 0f; upZ = 1f
                } else {
                    upX = 0f; upY = 1f; upZ = 0f
                }
                val cx = vec3CrossX(ty, tz, upY, upZ)
                val cy = vec3CrossY(tx, tz, upX, upZ)
                val cz = vec3CrossZ(tx, ty, upX, upY)
                val cLen = sqrt(cx * cx + cy * cy + cz * cz)
                if (cLen > 1e-12f) {
                    normalX = cx / cLen; normalY = cy / cLen; normalZ = cz / cLen
                } else {
                    normalX = 0f; normalY = 1f; normalZ = 0f
                }
            }
            dist = 1e-6f
        }

        val effRadiusA = effectiveCollisionRadius(bi, si, s, normalX, normalY, normalZ)
        val effRadiusB = effectiveCollisionRadius(bj, sj, t, normalX, normalY, normalZ)
        val minDist = effRadiusA + effRadiusB

        if (dist >= minDist) return false

        val overlap = minDist - dist
        debugCollisionHits++
        if (dragInfo != null && (dragInfo!!.bandIndex == bi || dragInfo!!.bandIndex == bj)) {
            debugDragCollisionHits++
        }
        if (overlap > debugMaxOverlap) debugMaxOverlap = overlap
        val corrX = normalX * (overlap * 0.35f)
        val corrY = normalY * (overlap * 0.35f)
        val corrZ = normalZ * (overlap * 0.35f)

        val invS = 1f - s
        posI[o0a] += corrX * invS; posI[o0a + 1] += corrY * invS; posI[o0a + 2] += corrZ * invS
        posI[o1a] += corrX * s; posI[o1a + 1] += corrY * s; posI[o1a + 2] += corrZ * s
        val invT = 1f - t
        posJ[o0b] -= corrX * invT; posJ[o0b + 1] -= corrY * invT; posJ[o0b + 2] -= corrZ * invT
        posJ[o1b] -= corrX * t; posJ[o1b + 1] -= corrY * t; posJ[o1b + 2] -= corrZ * t

        // Coulomb friction
        val mu = frictionCoefficient
        if (mu > 0f) {
            val prevI = bandI.previousPositions
            val prevJ = bandJ.previousPositions

            val velAx = (posI[o0a] - prevI[o0a]) * invS + (posI[o1a] - prevI[o1a]) * s
            val velAy = (posI[o0a + 1] - prevI[o0a + 1]) * invS + (posI[o1a + 1] - prevI[o1a + 1]) * s
            val velAz = (posI[o0a + 2] - prevI[o0a + 2]) * invS + (posI[o1a + 2] - prevI[o1a + 2]) * s
            val velBx = (posJ[o0b] - prevJ[o0b]) * invT + (posJ[o1b] - prevJ[o1b]) * t
            val velBy = (posJ[o0b + 1] - prevJ[o0b + 1]) * invT + (posJ[o1b + 1] - prevJ[o1b + 1]) * t
            val velBz = (posJ[o0b + 2] - prevJ[o0b + 2]) * invT + (posJ[o1b + 2] - prevJ[o1b + 2]) * t

            val relVx = velAx - velBx; val relVy = velAy - velBy; val relVz = velAz - velBz
            val dotRN = relVx * normalX + relVy * normalY + relVz * normalZ
            val tanX = relVx - normalX * dotRN
            val tanY = relVy - normalY * dotRN
            val tanZ = relVz - normalZ * dotRN
            val tangentLen = sqrt(tanX * tanX + tanY * tanY + tanZ * tanZ)
            val minSlide = 0.0002f
            if (tangentLen > minSlide) {
                val maxFriction = mu * overlap * 0.25f
                val frictionMag = min(tangentLen * 0.3f, maxFriction)
                val invTanLen = 1f / tangentLen
                val fdx = tanX * invTanLen * frictionMag
                val fdy = tanY * invTanLen * frictionMag
                val fdz = tanZ * invTanLen * frictionMag

                prevI[o0a] += fdx * invS; prevI[o0a + 1] += fdy * invS; prevI[o0a + 2] += fdz * invS
                prevI[o1a] += fdx * s; prevI[o1a + 1] += fdy * s; prevI[o1a + 2] += fdz * s
                prevJ[o0b] -= fdx * invT; prevJ[o0b + 1] -= fdy * invT; prevJ[o0b + 2] -= fdz * invT
                prevJ[o1b] -= fdx * t; prevJ[o1b + 1] -= fdy * t; prevJ[o1b + 2] -= fdz * t

                val contactPosX = (closestAx + closestBx) * 0.5f
                val contactPosY = (closestAy + closestBy) * 0.5f
                val contactPosZ = (closestAz + closestBz) * 0.5f
                frictionAccumulator += overlap
                frictionSpeedAccumulator += tangentLen
                frictionPositionAccX += contactPosX
                frictionPositionAccY += contactPosY
                frictionPositionAccZ += contactPosZ
                frictionSampleCount++
            }
        }

        if (injectVelocity) {
            val vcX = corrX * 0.15f; val vcY = corrY * 0.15f; val vcZ = corrZ * 0.15f
            val prevI = bandI.previousPositions
            val prevJ = bandJ.previousPositions
            prevI[o0a] -= vcX * invS; prevI[o0a + 1] -= vcY * invS; prevI[o0a + 2] -= vcZ * invS
            prevI[o1a] -= vcX * s; prevI[o1a + 1] -= vcY * s; prevI[o1a + 2] -= vcZ * s
            prevJ[o0b] += vcX * invT; prevJ[o0b + 1] += vcY * invT; prevJ[o0b + 2] += vcZ * invT
            prevJ[o1b] += vcX * t; prevJ[o1b + 1] += vcY * t; prevJ[o1b + 2] += vcZ * t
        }

        return true
    }

    private fun effectiveCollisionRadius(bi: Int, si: Int, param: Float, normalX: Float, normalY: Float, normalZ: Float): Float {
        val cs = bands[bi].crossSection
        if (!cs.isRectangular) return bands[bi].radius
        if (bi >= cachedFrames.size || cachedFrames[bi].size <= si + 1) return bands[bi].radius

        val f0 = cachedFrames[bi][si]
        val f1 = cachedFrames[bi][si + 1]
        val invP = 1f - param
        // Interpolate d1 and d2, then normalize
        var id1x = f0.d1x * invP + f1.d1x * param
        var id1y = f0.d1y * invP + f1.d1y * param
        var id1z = f0.d1z * invP + f1.d1z * param
        val d1Len = sqrt(id1x * id1x + id1y * id1y + id1z * id1z)
        if (d1Len > 1e-9f) { id1x /= d1Len; id1y /= d1Len; id1z /= d1Len }

        var id2x = f0.d2x * invP + f1.d2x * param
        var id2y = f0.d2y * invP + f1.d2y * param
        var id2z = f0.d2z * invP + f1.d2z * param
        val d2Len = sqrt(id2x * id2x + id2y * id2y + id2z * id2z)
        if (d2Len > 1e-9f) { id2x /= d2Len; id2y /= d2Len; id2z /= d2Len }

        return cs.effectiveRadius(normalX, normalY, normalZ, id1x, id1y, id1z, id2x, id2y, id2z)
    }

    fun latexThinningFactor(bi: Int, si: Int, param: Float): Float {
        val band = bands[bi]
        val n = band.particleCount
        if (n < 2) return 1.0f

        val segLen = band.segmentLength
        if (segLen <= 1e-6f) return 1.0f

        val o0 = si * 3; val o1 = (si + 1) * 3
        val dx = band.positions[o1] - band.positions[o0]
        val dy = band.positions[o1 + 1] - band.positions[o0 + 1]
        val dz = band.positions[o1 + 2] - band.positions[o0 + 2]
        val actualLen = sqrt(dx * dx + dy * dy + dz * dz)
        val localStretch = max(0.0f, actualLen / segLen - 1.0f)

        val particleT = (si.toFloat() + param) / (n - 1).toFloat()
        val center = sin(particleT * Math.PI.toFloat())
        val centerMask = center * center * center * center

        val tension = localStretch * centerMask
        return 1.0f / sqrt(max(1.0f, 1.0f + tension * stretchThinning * 3.0f))
    }

    // =========================================================================
    // MARK: - Frames (computeFrames, recomputeFrames, rotateVector)
    // =========================================================================

    fun recomputeFrames() {
        if (cachedFrames.size != bands.size) {
            cachedFrames = Array(bands.size) { emptyArray() }
        }
        for (bi in bands.indices) {
            val band = bands[bi]
            if (!band.active || band.fadeOut != 0f) {
                cachedFrames[bi] = emptyArray()
                continue
            }
            if (band.crossSection.isRectangular) {
                cachedFrames[bi] = computeFrames(band.positions, band.twistAngles)
            } else if (squareCrossSection) {
                val zeroTwist = FloatArray(band.particleCount)
                cachedFrames[bi] = computeFrames(band.positions, zeroTwist)
            } else {
                cachedFrames[bi] = emptyArray()
            }
        }
    }

    // =========================================================================
    // MARK: - Drag
    // =========================================================================

    fun beginDrag(bandIndex: Int, endIndex: Int, worldX: Float, worldY: Float) {
        if (bandIndex < 0 || bandIndex >= bands.size) return
        wakeUp()
        val band = bands[bandIndex]
        val lowerAnimationKey = LowerAnimationKey(bandIndex, endIndex)
        val originalHole: Int
        if (endIndex == 0) {
            originalHole = if (band.pinStart >= 0) band.pinStart else lowerAnimations[lowerAnimationKey]?.targetHole ?: 0
            band.pinStart = -1
        } else {
            originalHole = if (band.pinEnd >= 0) band.pinEnd else lowerAnimations[lowerAnimationKey]?.targetHole ?: 0
            band.pinEnd = -1
        }
        dragInfo = DragInfo(bandIndex, endIndex, if (originalHole >= 0) originalHole else 0)

        lowerAnimations.remove(lowerAnimationKey)

        val elev = holeSurfaceZ(if (originalHole >= 0) originalHole else 0)
        val idx = if (endIndex == 0) 0 else (band.particleCount - 1) * 3
        val liftX = worldX; val liftY = worldY; val liftZ = elev + liftHeight
        band.positions[idx] = liftX; band.positions[idx + 1] = liftY; band.positions[idx + 2] = liftZ
        band.previousPositions[idx] = liftX; band.previousPositions[idx + 1] = liftY; band.previousPositions[idx + 2] = liftZ
        dragStartPosX = liftX; dragStartPosY = liftY; dragStartPosZ = liftZ
        hasDragStartPos = true
        dragTargetPosX = liftX; dragTargetPosY = liftY; dragTargetPosZ = liftZ
        hasDragTargetPos = true
    }

    fun updateDrag(worldX: Float, worldY: Float) {
        val drag = dragInfo ?: return
        val band = bands[drag.bandIndex]
        val idx = if (drag.endIndex == 0) 0 else (band.particleCount - 1) * 3
        dragStartPosX = band.positions[idx]; dragStartPosY = band.positions[idx + 1]; dragStartPosZ = band.positions[idx + 2]
        hasDragStartPos = true
        val surfZ = boardSurfaceZ(x = worldX, y = worldY)
        dragTargetPosX = worldX; dragTargetPosY = worldY; dragTargetPosZ = surfZ + liftHeight
        hasDragTargetPos = true
    }

    fun endDrag(targetHoleIndex: Int) {
        val drag = dragInfo ?: return

        val band = bands[drag.bandIndex]
        val idx = if (drag.endIndex == 0) 0 else (band.particleCount - 1) * 3
        val currentX = band.positions[idx]; val currentY = band.positions[idx + 1]; val currentZ = band.positions[idx + 2]
        val hi = targetHoleIndex * 2
        val holeX = holePositions[hi]; val holeY = holePositions[hi + 1]
        val holeElev = holeSurfaceZ(targetHoleIndex)
        val aboveX = holeX; val aboveY = holeY; val aboveZ = holeElev + liftHeight
        val dx = currentX - aboveX; val dy = currentY - aboveY; val dz = currentZ - aboveZ
        val dist = sqrt(dx * dx + dy * dy + dz * dz)
        val returnDuration = min(max(dist * 0.9f, 0.25f), 0.8f)

        val lowerAnimationKey = LowerAnimationKey(drag.bandIndex, drag.endIndex)
        lowerAnimations[lowerAnimationKey] = LowerAnimation(
            bandIndex = drag.bandIndex,
            endIndex = drag.endIndex,
            targetHole = targetHoleIndex,
            startPosX = currentX,
            startPosY = currentY,
            startPosZ = currentZ,
            returnPosX = aboveX,
            returnPosY = aboveY,
            returnPosZ = aboveZ,
            hasReturnPos = true,
            returnDuration = returnDuration
        )

        dragInfo = null
        hasDragStartPos = false
        hasDragTargetPos = false
    }

    fun endpointZ(bandIndex: Int, endIndex: Int): Float {
        if (bandIndex < 0 || bandIndex >= bands.size) return 0f
        val band = bands[bandIndex]
        val idx = if (endIndex == 0) 0 else (band.particleCount - 1) * 3
        return band.positions[idx + 2]
    }

    fun currentHoleIndex(bandIndex: Int, endIndex: Int): Int {
        if (bandIndex < 0 || bandIndex >= bands.size) return -1
        val band = bands[bandIndex]
        val pinnedHole = if (endIndex == 0) band.pinStart else band.pinEnd
        if (pinnedHole >= 0) return pinnedHole
        return lowerAnimations[LowerAnimationKey(bandIndex, endIndex)]?.targetHole ?: -1
    }

    // =========================================================================
    // MARK: - Level initialization
    // =========================================================================

    fun initializeLevel(ropeConfigs: List<RopeConfig>, actions: List<LevelAction>) {
        bands.clear()
        clearStickyBonds()
        currentTension = ropeTension

        for (config in ropeConfigs) {
            addBand(radius = config.radius, crossSection = config.crossSection, particleCount = particleCount)
        }

        if (actions.isEmpty()) {
            for ((i, config) in ropeConfigs.withIndex()) {
                pin(bandIndex = i, startHole = config.startHole, endHole = config.endHole)
            }
            return
        }

        Log.i(TAG, "Replaying ${actions.size} actions")

        val initStart = System.nanoTime()
        var pinTime = 0L
        var dragTime = 0L
        var pinCount = 0
        var dragCount = 0

        for (action in actions) {
            val t0 = System.nanoTime()
            when (action.type) {
                LevelAction.ActionType.PIN -> {
                    val band = bands[action.ropeIndex]
                    if (action.endIndex == 0) {
                        band.pinStart = action.holeIndex
                    } else {
                        band.pinEnd = action.holeIndex
                    }
                    if (band.pinStart >= 0 && band.pinEnd >= 0) {
                        pinAndSettle(action.ropeIndex)
                        pinCount++
                        pinTime += System.nanoTime() - t0
                    }
                }
                LevelAction.ActionType.DRAG -> {
                    simulateDrag(bandIndex = action.ropeIndex, endIndex = action.endIndex, toHole = action.holeIndex)
                    dragCount++
                    dragTime += System.nanoTime() - t0
                }
            }
        }

        val totalMs = (System.nanoTime() - initStart) / 1_000_000.0
        val pinMs = pinTime / 1_000_000.0
        val dragMs = dragTime / 1_000_000.0
        val avgDragMs = if (dragCount > 0) dragMs / dragCount else 0.0
        Log.w(TAG, "[INIT-PROFILE] pins=$pinCount pinTime=${"%.1f".format(pinMs)}ms drags=$dragCount dragTime=${"%.1f".format(dragMs)}ms avgDrag=${"%.1f".format(avgDragMs)}ms total=${"%.1f".format(totalMs)}ms")
    }

    private fun pinAndSettle(bandIndex: Int) {
        val band = bands[bandIndex]
        val startHole = band.pinStart
        val endHole = band.pinEnd
        if (startHole < 0 || endHole < 0) return

        val p0 = holePosition3D(startHole)
        val p1 = holePosition3D(endHole)
        val n = band.particleCount
        val pos = band.positions
        val prev = band.previousPositions

        for (i in 0 until n) {
            val t = i.toFloat() / max(1, n - 1).toFloat()
            val off = i * 3
            pos[off] = p0[0] * (1f - t) + p1[0] * t
            pos[off + 1] = p0[1] * (1f - t) + p1[1] * t
            pos[off + 2] = liftHeight
        }
        pos[0] = p0[0]; pos[1] = p0[1]; pos[2] = p0[2]
        val lastOff = (n - 1) * 3
        pos[lastOff] = p1[0]; pos[lastOff + 1] = p1[1]; pos[lastOff + 2] = p1[2]
        System.arraycopy(pos, 0, prev, 0, pos.size)

        val dx = p1[0] - p0[0]; val dy = p1[1] - p0[1]; val dz = p1[2] - p0[2]
        val dist = sqrt(dx * dx + dy * dy + dz * dz)
        band.segmentLength = dist / max(1, n - 1).toFloat()
        band.active = true

        val totalLen = band.segmentLength * (n - 1).toFloat()
        Log.w(TAG, "[PIN-SETTLE] band=$bandIndex n=$n dist=${"%.4f".format(dist)} segLen=${"%.6f".format(band.segmentLength)} totalLen=${"%.4f".format(totalLen)}")

        val hasOthers = bands.indices.any { it != bandIndex && bands[it].active && bands[it].pinStart >= 0 }
        doSteps(settleSteps, hasOthers)

        var actualLen = 0f
        for (i in 1 until n) {
            val o0 = (i - 1) * 3; val o1 = i * 3
            val ddx = pos[o1] - pos[o0]; val ddy = pos[o1 + 1] - pos[o0 + 1]; val ddz = pos[o1 + 2] - pos[o0 + 2]
            actualLen += sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
        }
        Log.w(TAG, "[PIN-SETTLE-AFTER] band=$bandIndex actualLen=${"%.4f".format(actualLen)} vs restLen=${"%.4f".format(totalLen)}")
    }

    private fun simulateDrag(bandIndex: Int, endIndex: Int, toHole: Int) {
        val band = bands[bandIndex]
        val n = band.particleCount
        val idx = if (endIndex == 0) 0 else (n - 1) * 3
        val pos = band.positions
        val prev = band.previousPositions

        val fromX = pos[idx]; val fromY = pos[idx + 1]; val fromZ = pos[idx + 2]
        val toPos = holePosition3D(toHole)

        if (endIndex == 0) {
            band.pinStart = -1
        } else {
            band.pinEnd = -1
        }

        val fromElev = boardSurfaceZ(x = fromX, y = fromY)
        val toElev = holeSurfaceZ(toHole)
        val maxElev = max(fromElev, toElev)
        val liftFromX = fromX; val liftFromY = fromY; val liftFromZ = maxElev + liftHeight
        val liftToX = toPos[0]; val liftToY = toPos[1]; val liftToZ = maxElev + liftHeight

        val dragSteps = 4
        // Lift phase
        for (s in 1..3) {
            val t = s.toFloat() / 3.0f
            pos[idx] = fromX + (liftFromX - fromX) * t
            pos[idx + 1] = fromY + (liftFromY - fromY) * t
            pos[idx + 2] = fromZ + (liftFromZ - fromZ) * t
            prev[idx] = pos[idx]; prev[idx + 1] = pos[idx + 1]; prev[idx + 2] = pos[idx + 2]
            doSteps(dragSteps, collide = true)
        }

        // Traverse phase
        val traverseSteps = 12
        for (s in 1..traverseSteps) {
            val t = s.toFloat() / traverseSteps.toFloat()
            pos[idx] = liftFromX + (liftToX - liftFromX) * t
            pos[idx + 1] = liftFromY + (liftToY - liftFromY) * t
            pos[idx + 2] = liftFromZ + (liftToZ - liftFromZ) * t
            prev[idx] = pos[idx]; prev[idx + 1] = pos[idx + 1]; prev[idx + 2] = pos[idx + 2]
            doSteps(dragSteps, collide = true)
        }

        // Lower phase
        for (s in 1..3) {
            val t = s.toFloat() / 3.0f
            pos[idx] = liftToX + (toPos[0] - liftToX) * t
            pos[idx + 1] = liftToY + (toPos[1] - liftToY) * t
            pos[idx + 2] = liftToZ + (toPos[2] - liftToZ) * t
            prev[idx] = pos[idx]; prev[idx + 1] = pos[idx + 1]; prev[idx + 2] = pos[idx + 2]
            doSteps(dragSteps, collide = true)
        }

        if (endIndex == 0) {
            band.pinStart = toHole
        } else {
            band.pinEnd = toHole
        }
        pos[idx] = toPos[0]; pos[idx + 1] = toPos[1]; pos[idx + 2] = toPos[2]
        prev[idx] = toPos[0]; prev[idx + 1] = toPos[1]; prev[idx + 2] = toPos[2]

        doSteps(settleSteps, collide = true)
    }

    // =========================================================================
    // MARK: - Snapshots
    // =========================================================================

    fun consumeAndResetFriction(): FrictionEvent? {
        if (frictionSampleCount <= 0) return null
        val n = frictionSampleCount.toFloat()
        val result = FrictionEvent(
            intensity = frictionAccumulator / n,
            relativeSpeed = frictionSpeedAccumulator / n,
            positionX = frictionPositionAccX / n,
            positionY = frictionPositionAccY / n,
            positionZ = frictionPositionAccZ / n
        )
        frictionAccumulator = 0f
        frictionSpeedAccumulator = 0f
        frictionPositionAccX = 0f
        frictionPositionAccY = 0f
        frictionPositionAccZ = 0f
        frictionSampleCount = 0
        return result
    }

    fun startFadeOut(bandIndex: Int) {
        val band = bands[bandIndex]
        val suckTarget: Int
        val suckFromEnd: Int
        if (band.pinStart >= 0) {
            suckTarget = band.pinStart
            suckFromEnd = 1
        } else if (band.pinEnd >= 0) {
            suckTarget = band.pinEnd
            suckFromEnd = 0
        } else {
            suckTarget = 0
            suckFromEnd = 1
        }
        val tailHole = if (suckFromEnd == 1) band.pinEnd else band.pinStart

        band.suckHole = suckTarget
        band.suckTailHole = tailHole
        band.suckFromEnd = suckFromEnd
        band.suckConsumed = 0f
        band.suckFrame = 0

        val n = band.particleCount
        val segs = FloatArray(maxOf(0, n - 1))
        for (k in 0 until n - 1) {
            val o0 = k * 3; val o1 = (k + 1) * 3
            val dx = band.positions[o1] - band.positions[o0]
            val dy = band.positions[o1 + 1] - band.positions[o0 + 1]
            val dz = band.positions[o1 + 2] - band.positions[o0 + 2]
            segs[k] = sqrt(dx * dx + dy * dy + dz * dz)
        }
        band.suckSegLengths = segs
        band.suckOrigPositions = band.positions.copyOf()

        band.fadeOut = 0.001f
        band.pinStart = -1
        band.pinEnd = -1
    }

    fun takeSnapshot(): Snapshot {
        return Snapshot(
            bands = Array(bands.size) { i ->
                val b = bands[i]
                BandSnapshot(
                    positions = b.positions.copyOf(),
                    previousPositions = b.previousPositions.copyOf(),
                    twistAngles = b.twistAngles.copyOf(),
                    previousTwistAngles = b.previousTwistAngles.copyOf(),
                    segmentLength = b.segmentLength,
                    pinStart = b.pinStart,
                    pinEnd = b.pinEnd,
                    active = b.active,
                    fadeOut = b.fadeOut,
                    suckHole = b.suckHole,
                    suckTailHole = b.suckTailHole,
                    suckFromEnd = b.suckFromEnd,
                    suckConsumed = b.suckConsumed,
                    suckSegLengths = b.suckSegLengths.copyOf(),
                    suckOrigPositions = b.suckOrigPositions.copyOf()
                )
            }
        )
    }

    fun restoreSnapshot(snapshot: Snapshot) {
        for (i in bands.indices) {
            if (i >= snapshot.bands.size) break
            val s = snapshot.bands[i]
            val b = bands[i]
            System.arraycopy(s.positions, 0, b.positions, 0, s.positions.size)
            System.arraycopy(s.previousPositions, 0, b.previousPositions, 0, s.previousPositions.size)
            System.arraycopy(s.twistAngles, 0, b.twistAngles, 0, s.twistAngles.size)
            System.arraycopy(s.previousTwistAngles, 0, b.previousTwistAngles, 0, s.previousTwistAngles.size)
            b.segmentLength = s.segmentLength
            b.pinStart = s.pinStart
            b.pinEnd = s.pinEnd
            b.active = s.active
            b.fadeOut = s.fadeOut
            b.suckHole = s.suckHole
            b.suckTailHole = s.suckTailHole
            b.suckFromEnd = s.suckFromEnd
            b.suckConsumed = s.suckConsumed
            b.suckSegLengths = s.suckSegLengths.copyOf()
            b.suckOrigPositions = s.suckOrigPositions.copyOf()
        }
        dragInfo = null
        hasDragStartPos = false
        hasDragTargetPos = false
        lowerAnimations.clear()
        currentTension = ropeTension
        wakeUp()
    }
}
