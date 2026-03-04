package com.uzls.four.renderer

import android.content.Context
import android.opengl.GLES30
import android.opengl.GLSurfaceView
import android.os.Handler
import android.os.Looper
import android.util.Log
import com.uzls.four.game.TouchPhase
import com.uzls.four.level.LevelGenerator
import com.uzls.four.level.LevelLoader
import com.uzls.four.simulation.VerletSimulator
import java.nio.ByteBuffer
import java.nio.ByteOrder
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

class GameRenderer(private val context: Context) : GLSurfaceView.Renderer {

    // Public callbacks (set from main thread)
    var onLevelComplete: (() -> Unit)? = null
    var onFpsUpdate: ((Float) -> Unit)? = null
    var onUndoStackChanged: ((Boolean) -> Unit)? = null

    // Pending commands from UI thread (volatile for cross-thread visibility)
    @Volatile var pendingLevelId: Int = -1
    @Volatile var pendingRestart: Boolean = false
    @Volatile var pendingUndo: Boolean = false

    // Touch commands (synchronized access)
    private val touchLock = Any()
    private var touchPhase: TouchPhase? = null
    private var touchX = 0f
    private var touchY = 0f
    private var pendingCameraZoom = 1f
    private var pendingCameraSpin = 0f
    private var pendingCameraDebugToggle = false

    // GL state
    private var viewWidth = 0
    private var viewHeight = 0
    private var shadowFbo: Fbo? = null
    private var hdrFbo: Fbo? = null
    private var bloomFboA: Fbo? = null
    private var bloomFboB: Fbo? = null

    // Shader programs
    private var ropeProg: ShaderProgram? = null
    private var holeProg: ShaderProgram? = null
    private var tableProg: ShaderProgram? = null
    private var boardProg: ShaderProgram? = null
    private var shadowProg: ShaderProgram? = null
    private var bloomThreshProg: ShaderProgram? = null
    private var bloomBlurProg: ShaderProgram? = null
    private var postProg: ShaderProgram? = null

    // Uniform buffer
    private val frameUniforms = FrameUniforms()

    // Camera
    val camera = Camera()
    var cameraDebugMode = false

    // Physics
    var simulator: VerletSimulator? = null
        private set
    private var currentLevelId = 1

    // Render state
    private var ropeVao = 0; private var ropeVbo = 0; private var ropeIbo = 0
    private var ropeIndexCount = 0
    private var holeVao = 0; private var holeVbo = 0; private var holeIbo = 0
    private var holeIndexCount = 0; private var holeInstanceVbo = 0
    private var holeInstanceCount = 0
    private var holeInstanceData = FloatArray(0) // (x, y, elevation, radius) per hole
    private var fullscreenVao = 0
    private var boardVao = 0; private var boardVbo = 0; private var boardIbo = 0
    private var boardIndexCount = 0

    // Timing
    private var lastFrameTime = 0L
    private var fpsFrameCount = 0
    private var fpsAccumulator = 0f
    private var gameTime = 0f

    // Drag / interaction state
    private var dragState: DragState? = null
    private var cameraDragActive = false
    private var cameraDragLastY = 0f
    var highlightHoleIndex = -1f

    // Undo
    private val undoStack = ArrayDeque<Any>() // SimSnapshot
    private val maxUndoSteps = 20

    // Parameters — iOS defaults from ContentView.swift
    var gravity = -5.0f
    var damping = 0.97f
    var constraintIterations = 8
    var settleSteps = 5
    var liftHeight = 0.30f
    var dragHeight = 0.35f
    var ropeTension = 0.98f
    var particleCount = 60
    var boardElevation = 0.12f
    var profileSegments = 10  // iOS default
    var holeRadius = 0.105f
    var holeRadiusScale = 0.734f
    var ropeRadiusScale = 1.062f  // iOS: 1.0618677139282227
    var stretchThinning = 0.5f
    var squareCrossSection = false
    var bendCompliance = 0.0015f
    var bendVelocityCoupling = 0.45f
    var frictionCoefficient = 0.8f

    // Lighting — iOS defaults from Renderer.swift
    var lightDirX = -0.0294f; var lightDirY = -0.2213f; var lightDirZ = 0.8749f
    var lightIntensity = 0.402f  // iOS: 0.40238875150680542
    var ambient = 0.104f  // iOS: 0.10394008457660675
    var shadowBias = 0.00137f
    var shadowDarkness = 0.155f
    var shadowSize = 0.073f
    var shadowsEnabled = true
    var shadowType = 2 // 0=shadowMap, 1=pcf, 2=pcss

    // Visual — iOS defaults from Renderer.swift
    var exposure = 0.5f  // iOS: 0.5 (cartoonExposure = 1.329 used when cartoon enabled)
    var cartoonExposure = 1.329f  // iOS: 1.3291559219360352
    var bloomStrength = 0f  // iOS: bloomStrength = 0, cartoonBloom = 0
    var holeTintR = 1.0f; var holeTintG = 0.909f; var holeTintB = 1.0f; var holeTintAmount = 1.0f

    // Rope material — iOS defaults
    var ropeMatte = 0.334f
    var ropeGloss = 0.693f
    var ropeDiffuseWrap = 0.008f
    var ropeSubsurface = 0.338f
    var ropeEdgeLight = 0.415f
    var ropeSaturation = 1.357f
    var ropeMicroBump = 0.143f
    var ropeContactAO = 1.0f
    var ropeLiftGlow = 0.972f
    var ropeBumpScale = 3.0f
    var ropeStretchGloss = 0.7f
    var ropeStretchSpec = 1.0f

    // Table — iOS defaults
    var tableStyle = 0
    var tableColor1R = 0.08f; var tableColor1G = 0.09f; var tableColor1B = 0.13f
    var tableColor2R = 0.12f; var tableColor2G = 0.13f; var tableColor2B = 0.20f

    // Cartoon — iOS default is true, but screenshots show wood texture = cartoon off
    var cartoonMode = 0.0f
    var cartoonLevels = 2.0f
    var cartoonEdgeStrength = 1.0f
    var cartoonShadowBright = 0.38f
    var cartoonWrap = 0.15f
    var cartoonEdgeSmooth = 0.5f

    // Cap — iOS defaults
    var capRadiusScale = 0.906f
    var capSegments = 12
    var capRings = 6
    var capDarken = 0f  // iOS: capDarken = 0

    private val mainHandler = Handler(Looper.getMainLooper())

    // ===== Touch posting from UI thread =====
    fun postTouchEvent(phase: TouchPhase, x: Float, y: Float) {
        synchronized(touchLock) {
            touchPhase = phase; touchX = x; touchY = y
        }
    }

    fun postCameraZoom(scale: Float) {
        synchronized(touchLock) { pendingCameraZoom *= scale }
    }

    fun postCameraSpin(delta: Float) {
        synchronized(touchLock) { pendingCameraSpin += delta }
    }

    fun postCameraDebugToggle() {
        synchronized(touchLock) { pendingCameraDebugToggle = true }
    }

    // ===== GLSurfaceView.Renderer =====

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES30.glClearColor(0.07f, 0.08f, 0.12f, 1f)
        GLES30.glEnable(GLES30.GL_DEPTH_TEST)
        GLES30.glDepthFunc(GLES30.GL_LEQUAL)
        GLES30.glEnable(GLES30.GL_CULL_FACE)
        GLES30.glCullFace(GLES30.GL_BACK)

        frameUniforms.create()
        compileShaders()
        createGeometry()
        loadLevel(currentLevelId)

        lastFrameTime = System.nanoTime()
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        viewWidth = width
        viewHeight = height
        GLES30.glViewport(0, 0, width, height)
        resizeFbos(width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        val now = System.nanoTime()
        val dt = ((now - lastFrameTime) / 1_000_000_000f).coerceIn(0.001f, 0.1f)
        lastFrameTime = now
        gameTime += dt

        // FPS tracking
        fpsFrameCount++
        fpsAccumulator += dt
        if (fpsAccumulator >= 0.5f) {
            val fps = fpsFrameCount / fpsAccumulator
            mainHandler.post { onFpsUpdate?.invoke(fps) }
            fpsFrameCount = 0
            fpsAccumulator = 0f
        }

        // Process pending commands
        processPendingCommands()
        processTouch()

        // Update physics
        simulator?.let { sim ->
            syncPhysicsParams(sim)
            sim.update(dt)
        }

        // Update rope mesh
        updateRopeMesh()

        // Render
        val aspect = viewWidth.toFloat() / viewHeight.toFloat()
        uploadFrameUniforms(aspect)

        // Shadow pass — iOS: depthStateShadow = lessEqual, depth writes ON
        shadowFbo?.let { fbo ->
            fbo.bind()
            GLES30.glClear(GLES30.GL_DEPTH_BUFFER_BIT)
            GLES30.glEnable(GLES30.GL_DEPTH_TEST)
            GLES30.glDepthFunc(GLES30.GL_LEQUAL)
            GLES30.glDepthMask(true)
            GLES30.glColorMask(false, false, false, false)
            GLES30.glDisable(GLES30.GL_CULL_FACE)  // iOS shadow pass doesn't cull
            frameUniforms.bind(0)

            shadowProg?.let { prog ->
                prog.use()
                prog.bindUbo("FrameBlock", 0)

                // Holes (instanced) — shadow mode 1
                prog.setInt("uShadowMode", 1)
                drawShadowHoles(prog)

                // Ropes — shadow mode 0
                prog.setInt("uShadowMode", 0)
                drawRopes()
            }
            GLES30.glColorMask(true, true, true, true)
            GLES30.glEnable(GLES30.GL_CULL_FACE)
            fbo.unbind()
        }

        // HDR pass
        hdrFbo?.let { fbo ->
            fbo.bind()
            GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT or GLES30.GL_DEPTH_BUFFER_BIT)
            GLES30.glEnable(GLES30.GL_DEPTH_TEST)
            frameUniforms.bind(0)

            // Table — iOS uses depthStateScene (lessEqual + depth writes ON)
            // table.frag writes gl_FragDepth: far depth for wood, depth=1 for holes
            tableProg?.let { prog ->
                prog.use()
                prog.bindUbo("FrameBlock", 0)
                GLES30.glEnable(GLES30.GL_DEPTH_TEST)
                GLES30.glDepthFunc(GLES30.GL_LEQUAL)
                GLES30.glDepthMask(true)
                bindShadowMap(prog, 2)
                uploadHoleUniforms(prog)
                drawFullscreenTriangle()
            }

            // Holes
            holeProg?.let { prog ->
                prog.use()
                prog.bindUbo("FrameBlock", 0)
                bindShadowMap(prog, 2)
                drawHoles(prog)
            }

            // Ropes — Metal default cull mode = none, so disable culling for ropes
            ropeProg?.let { prog ->
                prog.use()
                prog.bindUbo("FrameBlock", 0)
                GLES30.glDisable(GLES30.GL_CULL_FACE)
                bindShadowMap(prog, 2)
                drawRopes()
                GLES30.glEnable(GLES30.GL_CULL_FACE)
            }

            fbo.unbind()
        }

        // Bloom pass
        encodeBloomPass()

        // Composite pass — iOS uses effective exposure/bloom based on cartoon mode
        val useCartoon = cartoonMode > 0.5f
        val effExposure = if (useCartoon) cartoonExposure else exposure
        val effBloom = if (useCartoon) 0f else bloomStrength

        GLES30.glViewport(0, 0, viewWidth, viewHeight)
        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT)
        GLES30.glDisable(GLES30.GL_DEPTH_TEST)
        postProg?.let { prog ->
            prog.use()
            prog.setFloat("uExposure", effExposure)
            prog.setFloat("uBloomStrength", effBloom)
            prog.setFloat("uCartoonMode", cartoonMode)
            prog.setFloat("uCartoonEdgeStrength", cartoonEdgeStrength)
            prog.setFloat("uCartoonEdgeSmooth", cartoonEdgeSmooth)

            GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, hdrFbo?.colorTexId ?: 0)
            prog.setInt("uHdrTex", 0)

            GLES30.glActiveTexture(GLES30.GL_TEXTURE1)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, bloomFboA?.colorTexId ?: 0)
            prog.setInt("uBloomTex", 1)

            // Bind depth texture for cartoon edge detection
            GLES30.glActiveTexture(GLES30.GL_TEXTURE2)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, hdrFbo?.depthTexId ?: 0)
            prog.setInt("uDepthTex", 2)

            drawFullscreenTriangle()
        }

        // Win check
        checkWinCondition()
    }

    // ===== Private helpers =====

    private fun processPendingCommands() {
        val levelId = pendingLevelId
        if (levelId >= 0) {
            pendingLevelId = -1
            loadLevel(levelId)
        }
        if (pendingRestart) {
            pendingRestart = false
            loadLevel(currentLevelId)
        }
        if (pendingUndo) {
            pendingUndo = false
            performUndo()
        }
    }

    private fun processTouch() {
        var phase: TouchPhase? = null
        var x = 0f; var y = 0f
        var zoom = 1f; var spin = 0f; var debugToggle = false

        synchronized(touchLock) {
            phase = touchPhase; x = touchX; y = touchY
            touchPhase = null
            zoom = pendingCameraZoom; pendingCameraZoom = 1f
            spin = pendingCameraSpin; pendingCameraSpin = 0f
            debugToggle = pendingCameraDebugToggle; pendingCameraDebugToggle = false
        }

        if (debugToggle) cameraDebugMode = !cameraDebugMode

        if (zoom != 1f) {
            camera.orthoHalfHeight *= zoom
            camera.orthoHalfHeight = camera.orthoHalfHeight.coerceIn(0.5f, 10f)
        }
        if (spin != 0f) camera.rotationAngle += spin

        phase?.let { handleTouch(it, x, y) }
    }

    private fun handleTouch(phase: TouchPhase, screenX: Float, screenY: Float) {
        val world = camera.screenToWorld(screenX, screenY, viewWidth, viewHeight)
        val wx = world[0]; val wy = world[1]

        when (phase) {
            TouchPhase.BEGAN -> {
                val ds = beginDrag(wx, wy)
                if (ds != null) {
                    dragState = ds
                } else {
                    cameraDragActive = true
                    cameraDragLastY = screenY
                }
            }
            TouchPhase.MOVED -> {
                if (cameraDragActive) {
                    val dy = (screenY - cameraDragLastY) / viewHeight * 2f
                    camera.tiltAngle = (camera.tiltAngle + dy).coerceIn(-0.8f, 0.8f)
                    cameraDragLastY = screenY
                } else {
                    dragState?.let { updateDrag(wx, wy) }
                }
            }
            TouchPhase.ENDED -> {
                if (cameraDragActive) {
                    cameraDragActive = false
                } else {
                    dragState?.let { endDrag(wx, wy) }
                    dragState = null
                }
            }
            TouchPhase.CANCELLED -> {
                cameraDragActive = false
                dragState?.let { cancelDrag() }
                dragState = null
            }
        }
    }

    private fun beginDrag(wx: Float, wy: Float): DragState? {
        val sim = simulator ?: return null
        val hitRadius = holeRadius * 1.65f
        var bestDist = hitRadius
        var bestBand = -1
        var bestEnd = -1

        for (bi in sim.bands.indices) {
            val band = sim.bands[bi]
            if (!band.active) continue
            for (endIdx in 0..1) {
                val pi = if (endIdx == 0) 0 else (band.particleCount - 1) * 3
                val ex = band.positions[pi]; val ey = band.positions[pi + 1]
                val dx = wx - ex; val dy = wy - ey
                val dist = kotlin.math.sqrt(dx * dx + dy * dy)

                // Z-order penalty
                val endZ = band.positions[pi + 2]
                var effectiveDist = dist
                // Check if another rope end is above
                for (bj in sim.bands.indices) {
                    if (bj == bi) continue
                    val other = sim.bands[bj]
                    if (!other.active) continue
                    for (oe in 0..1) {
                        val opi = if (oe == 0) 0 else (other.particleCount - 1) * 3
                        val odx = wx - other.positions[opi]; val ody = wy - other.positions[opi + 1]
                        val odist = kotlin.math.sqrt(odx * odx + ody * ody)
                        if (odist < hitRadius && other.positions[opi + 2] > endZ) {
                            effectiveDist += hitRadius * 0.75f
                        }
                    }
                }

                if (effectiveDist < bestDist) {
                    bestDist = effectiveDist
                    bestBand = bi
                    bestEnd = endIdx
                }
            }
        }

        if (bestBand < 0) return null

        // Push undo
        // sim.takeSnapshot()... (simplified for now)

        val band = sim.bands[bestBand]
        val originalHole = if (bestEnd == 0) band.pinStart else band.pinEnd
        sim.beginDrag(bestBand, bestEnd, wx, wy)

        return DragState(bestBand, bestEnd, originalHole ?: -1)
    }

    private fun updateDrag(wx: Float, wy: Float) {
        val sim = simulator ?: return
        sim.updateDrag(wx, wy)

        // Find nearest hole for highlight
        val snapRadius = holeRadius * 1.9f
        var bestDist = snapRadius
        var bestHole = -1
        val holeCount = sim.holePositions.size / 2
        for (i in 0 until holeCount) {
            val hx = sim.holePositions[i * 2]; val hy = sim.holePositions[i * 2 + 1]
            val dx = wx - hx; val dy = wy - hy
            val dist = kotlin.math.sqrt(dx * dx + dy * dy)
            if (dist < bestDist) {
                bestDist = dist
                bestHole = i
            }
        }
        highlightHoleIndex = bestHole.toFloat()
    }

    private fun endDrag(wx: Float, wy: Float) {
        val sim = simulator ?: return
        val ds = dragState ?: return

        val snapRadius = holeRadius * 1.9f
        var bestDist = snapRadius
        var targetHole = ds.originalHoleIndex

        val holeCount = sim.holePositions.size / 2
        for (i in 0 until holeCount) {
            val hx = sim.holePositions[i * 2]; val hy = sim.holePositions[i * 2 + 1]
            val dx = wx - hx; val dy = wy - hy
            val dist = kotlin.math.sqrt(dx * dx + dy * dy)
            if (dist < bestDist) {
                bestDist = dist
                targetHole = i
            }
        }

        sim.endDrag(targetHole)
        highlightHoleIndex = -1f
    }

    private fun cancelDrag() {
        val ds = dragState ?: return
        simulator?.endDrag(ds.originalHoleIndex)
        highlightHoleIndex = -1f
    }

    private fun performUndo() {
        // TODO: restore from undo stack
    }

    private fun syncPhysicsParams(sim: VerletSimulator) {
        sim.gravity = gravity
        sim.damping = damping
        sim.constraintIterations = constraintIterations
        sim.ropeTension = ropeTension
        sim.liftHeight = liftHeight
        sim.bendCompliance = bendCompliance
        sim.bendVelocityCoupling = bendVelocityCoupling
        sim.stretchThinning = stretchThinning
        sim.squareCrossSection = squareCrossSection
    }

    fun loadLevel(levelId: Int) {
        currentLevelId = levelId

        // Try JSON first, fall back to procedural
        val level = LevelLoader.load(context, levelId)
            ?: LevelGenerator.generate(levelId, boardElevation)

        val holePositions = FloatArray(level.holes.size * 2)
        val holeElevations = FloatArray(level.holes.size)
        for (i in level.holes.indices) {
            holePositions[i * 2] = level.holes[i].xPosition
            holePositions[i * 2 + 1] = level.holes[i].yPosition
            holeElevations[i] = level.holes[i].zPosition
        }

        val boards = level.boards?.map {
            Board(it.centerX, it.centerY, it.width, it.height, it.elevation)
        } ?: emptyList()

        val boardDefs = boards.map { com.uzls.four.simulation.BoardDef(it.centerX, it.centerY, it.width, it.height, it.elevation) }.toTypedArray()
        val sim = VerletSimulator(holePositions, holeElevations, level.holeRadius, boardDefs)
        sim.particleCount = level.particlesPerRope
        syncPhysicsParams(sim)

        // Build rope configs and actions for initializeLevel
        val ropeConfigs = level.ropes.map { rope ->
            com.uzls.four.simulation.RopeConfig(
                startHole = rope.startHole,
                endHole = rope.endHole,
                radius = rope.radius,
                crossSection = rope.crossSection
            )
        }
        val levelActions = level.actions?.map { action ->
            com.uzls.four.simulation.LevelAction(
                type = if (action.type == "pin") com.uzls.four.simulation.LevelAction.ActionType.PIN
                       else com.uzls.four.simulation.LevelAction.ActionType.DRAG,
                ropeIndex = action.ropeIndex,
                endIndex = action.endIndex,
                holeIndex = action.holeIndex
            )
        } ?: emptyList()

        sim.initializeLevel(ropeConfigs, levelActions)

        // Set band colors from level definition
        for (i in level.ropes.indices) {
            if (i < sim.bands.size) {
                sim.bands[i].colorR = level.ropes[i].color.redChannel
                sim.bands[i].colorG = level.ropes[i].color.greenChannel
                sim.bands[i].colorB = level.ropes[i].color.blueChannel
            }
        }

        // Camera fit
        val maxElev = holeElevations.maxOrNull() ?: 0f
        camera.fitToHoles(holePositions, level.holeRadius, viewWidth.toFloat() / viewHeight.coerceAtLeast(1), maxElev)

        // Update hole instances
        updateHoleInstances(holePositions, holeElevations, level.holeRadius)

        this.simulator = sim
        this.holeRadius = level.holeRadius
    }

    private fun updateHoleInstances(positions: FloatArray, elevations: FloatArray, radius: Float) {
        val count = positions.size / 2
        holeInstanceCount = count
        val data = FloatArray(count * 4) // x, y, elevation, radius
        for (i in 0 until count) {
            data[i * 4] = positions[i * 2]
            data[i * 4 + 1] = positions[i * 2 + 1]
            data[i * 4 + 2] = if (i < elevations.size) elevations[i] else 0f
            data[i * 4 + 3] = radius
        }
        holeInstanceData = data.copyOf()

        val buf = ByteBuffer.allocateDirect(data.size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
        buf.put(data).position(0)

        if (holeInstanceVbo == 0) {
            val ids = IntArray(1)
            GLES30.glGenBuffers(1, ids, 0)
            holeInstanceVbo = ids[0]
        }
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, holeInstanceVbo)
        GLES30.glBufferData(GLES30.GL_ARRAY_BUFFER, data.size * 4, buf, GLES30.GL_STATIC_DRAW)
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, 0)
    }

    private fun updateRopeMesh() {
        val sim = simulator ?: return

        var totalVerts = 0
        var totalIndices = 0

        val meshes = mutableListOf<RopeMesh>()
        for (bi in sim.bands.indices) {
            val band = sim.bands[bi]
            if (!band.active && band.fadeOut <= 0f) continue

            // iOS: ropeRadius = ropes[i].radius * 1.3
            val ropeRadius = band.radius * 1.3f * ropeRadiusScale

            // Get cached material frames for rectangular cross-sections
            val frames = if ((band.crossSection.isRectangular || squareCrossSection) &&
                bi < sim.cachedFrames.size && sim.cachedFrames[bi].isNotEmpty())
                sim.cachedFrames[bi] else null

            val mesh = RopeMeshBuilder.buildRect(
                band.positions, band.particleCount,
                ropeRadius,
                band.colorR, band.colorG, band.colorB,
                tautness = band.tautness,
                crossSection = band.crossSection,
                materialFrames = frames,
                profileSegments = profileSegments,
                stretchThinning = stretchThinning,
                restLength = band.segmentLength * (band.particleCount - 1).toFloat(),
                squareCrossSection = squareCrossSection
            )
            meshes.add(mesh)
            totalVerts += mesh.vertexCount
            totalIndices += mesh.indexCount
        }

        if (totalVerts == 0) { ropeIndexCount = 0; return }

        // Merge meshes into single buffer
        val allVerts = FloatArray(totalVerts * 15)
        val allIndices = IntArray(totalIndices)
        var vOff = 0; var iOff = 0; var baseVertex = 0

        for (mesh in meshes) {
            System.arraycopy(mesh.vertices, 0, allVerts, vOff * 15, mesh.vertexCount * 15)
            for (i in 0 until mesh.indexCount) {
                allIndices[iOff + i] = mesh.indices[i] + baseVertex
            }
            baseVertex += mesh.vertexCount
            vOff += mesh.vertexCount
            iOff += mesh.indexCount
        }

        uploadRopeMesh(allVerts, totalVerts, allIndices, totalIndices)
        ropeIndexCount = totalIndices
    }

    private fun uploadRopeMesh(verts: FloatArray, vertCount: Int, indices: IntArray, idxCount: Int) {
        val vBuf = ByteBuffer.allocateDirect(vertCount * 15 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer()
        vBuf.put(verts, 0, vertCount * 15).position(0)

        val iBuf = ByteBuffer.allocateDirect(idxCount * 4).order(ByteOrder.nativeOrder()).asIntBuffer()
        iBuf.put(indices, 0, idxCount).position(0)

        GLES30.glBindVertexArray(ropeVao)
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, ropeVbo)
        GLES30.glBufferData(GLES30.GL_ARRAY_BUFFER, vertCount * 15 * 4, vBuf, GLES30.GL_STREAM_DRAW)
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, ropeIbo)
        GLES30.glBufferData(GLES30.GL_ELEMENT_ARRAY_BUFFER, idxCount * 4, iBuf, GLES30.GL_STREAM_DRAW)
        GLES30.glBindVertexArray(0)
    }

    private fun drawRopes() {
        if (ropeIndexCount <= 0) return
        GLES30.glBindVertexArray(ropeVao)
        GLES30.glDrawElements(GLES30.GL_TRIANGLES, ropeIndexCount, GLES30.GL_UNSIGNED_INT, 0)
        GLES30.glBindVertexArray(0)
    }

    private fun drawHoles(prog: ShaderProgram) {
        if (holeInstanceCount <= 0) return
        GLES30.glBindVertexArray(holeVao)

        // Bind instance buffer
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, holeInstanceVbo)
        val instLoc = 2
        GLES30.glEnableVertexAttribArray(instLoc)
        GLES30.glVertexAttribPointer(instLoc, 4, GLES30.GL_FLOAT, false, 16, 0)
        GLES30.glVertexAttribDivisor(instLoc, 1)

        GLES30.glDrawElementsInstanced(GLES30.GL_TRIANGLES, holeIndexCount, GLES30.GL_UNSIGNED_SHORT, 0, holeInstanceCount)

        GLES30.glVertexAttribDivisor(instLoc, 0)
        GLES30.glDisableVertexAttribArray(instLoc)
        GLES30.glBindVertexArray(0)
    }

    /** Draw holes for shadow pass — instance data at location 5 (shadow.vert uses layout(location=5)) */
    private fun drawShadowHoles(prog: ShaderProgram) {
        if (holeInstanceCount <= 0) return
        GLES30.glBindVertexArray(holeVao)

        // Shadow shader expects instance data at location 5
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, holeInstanceVbo)
        val instLoc = 5
        GLES30.glEnableVertexAttribArray(instLoc)
        GLES30.glVertexAttribPointer(instLoc, 4, GLES30.GL_FLOAT, false, 16, 0)
        GLES30.glVertexAttribDivisor(instLoc, 1)

        GLES30.glDrawElementsInstanced(GLES30.GL_TRIANGLES, holeIndexCount, GLES30.GL_UNSIGNED_SHORT, 0, holeInstanceCount)

        GLES30.glVertexAttribDivisor(instLoc, 0)
        GLES30.glDisableVertexAttribArray(instLoc)
        GLES30.glBindVertexArray(0)
    }

    private fun drawFullscreenTriangle() {
        GLES30.glBindVertexArray(fullscreenVao)
        GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
        GLES30.glBindVertexArray(0)
    }

    private fun bindShadowMap(prog: ShaderProgram, texUnit: Int) {
        GLES30.glActiveTexture(GLES30.GL_TEXTURE0 + texUnit)
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, shadowFbo?.depthTexId ?: 0)
        prog.setInt("uShadowMap", texUnit)
    }

    private fun uploadHoleUniforms(prog: ShaderProgram) {
        val count = holeInstanceCount.coerceAtMost(64)
        prog.setInt("uHoleCount", count)
        for (i in 0 until count) {
            val x = holeInstanceData[i * 4]
            val y = holeInstanceData[i * 4 + 1]
            val elev = holeInstanceData[i * 4 + 2]
            val r = holeInstanceData[i * 4 + 3]
            prog.setVec4("uHoles[$i]", x, y, elev, r)
        }
    }

    private fun encodeBloomPass() {
        val hdrTex = hdrFbo?.colorTexId ?: return
        val fboA = bloomFboA ?: return
        val fboB = bloomFboB ?: return

        // Threshold
        fboA.bind()
        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT)
        GLES30.glDisable(GLES30.GL_DEPTH_TEST)
        bloomThreshProg?.let { prog ->
            prog.use()
            GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, hdrTex)
            prog.setInt("uHdrTex", 0)
            drawFullscreenTriangle()
        }

        // Blur passes (3 iterations)
        bloomBlurProg?.let { prog ->
            prog.use()
            for (i in 0 until 3) {
                // Horizontal: A -> B
                fboB.bind()
                GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT)
                GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
                GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, fboA.colorTexId)
                prog.setInt("uSrcTex", 0)
                prog.setVec2("uDirection", 1f / fboA.width, 0f)
                drawFullscreenTriangle()

                // Vertical: B -> A
                fboA.bind()
                GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT)
                GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
                GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, fboB.colorTexId)
                prog.setInt("uSrcTex", 0)
                prog.setVec2("uDirection", 0f, 1f / fboA.height)
                drawFullscreenTriangle()
            }
        }

        fboA.unbind()
    }

    private fun uploadFrameUniforms(aspect: Float) {
        val vp = camera.viewProj(aspect)
        val invVp = com.uzls.four.simulation.mat4Inverse(vp)

        // iOS normalizes light direction before passing to shader
        val ldLen = kotlin.math.sqrt(lightDirX * lightDirX + lightDirY * lightDirY + lightDirZ * lightDirZ)
        val nldX: Float; val nldY: Float; val nldZ: Float
        if (ldLen > 1e-9f) {
            nldX = lightDirX / ldLen; nldY = lightDirY / ldLen; nldZ = lightDirZ / ldLen
        } else {
            nldX = 0f; nldY = 0f; nldZ = 1f
        }

        // iOS lightViewProj uses normalized lightDir, aspect-dependent ortho
        val lvp = camera.lightViewProj(nldX, nldY, nldZ, aspect)

        val halfH = camera.orthoHalfHeight
        val halfW = halfH * aspect

        // iOS cameraPos: (center.x, center.y + distance*sin(tilt), center.z + distance*cos(tilt), 1)
        val camPosX = camera.centerX
        val camPosY = camera.centerY + camera.distance * kotlin.math.sin(camera.tiltAngle)
        val camPosZ = camera.centerZ + camera.distance * kotlin.math.cos(camera.tiltAngle)

        // iOS: useCartoon ? cartoonExposure : exposure; same for bloom
        val useCartoon = cartoonMode > 0.5f
        val effExposure = if (useCartoon) cartoonExposure else exposure
        val effBloom = if (useCartoon) 0f else bloomStrength  // iOS: cartoonBloom = 0

        frameUniforms.upload(
            viewProj = vp,
            invViewProj = invVp,
            lightViewProj = lvp,
            lightDirIntensity = floatArrayOf(nldX, nldY, nldZ, lightIntensity),
            ambientColor = floatArrayOf(0f, 0f, 0f, highlightHoleIndex),
            cameraPos = floatArrayOf(camPosX, camPosY, camPosZ, 1f),
            orthoHalfSizeShadowBias = floatArrayOf(halfW, halfH, shadowBias, shadowType.toFloat()),
            shadowInvSize = floatArrayOf(1f / 2048f, 1f / 2048f, camera.centerX, camera.centerY),
            timeDrag = floatArrayOf(gameTime, 0f, currentLevelId.toFloat(), if (dragState != null) 1f else 0f),
            woodBoundsMin = floatArrayOf(-5f, -5f, 0f, 0f),
            woodBoundsMax = floatArrayOf(5f, 5f, 0f, 0f),
            holeTint = floatArrayOf(holeTintR, holeTintG, holeTintB, holeTintAmount),
            visualParams = floatArrayOf(exposure, bloomStrength, cartoonMode, cartoonLevels),
            lightingParams = floatArrayOf(ambient, shadowDarkness, shadowSize, if (shadowsEnabled) 1f else 0f),
            tableParams = floatArrayOf(tableStyle.toFloat(), tableColor1R, tableColor1G, tableColor1B),
            tableParams2 = floatArrayOf(tableColor2R, tableColor2G, tableColor2B, if (squareCrossSection) 1f else 0f),
            ropeMatParams = floatArrayOf(ropeMatte, ropeGloss, ropeDiffuseWrap, ropeSubsurface),
            ropeMatParams2 = floatArrayOf(ropeEdgeLight, ropeSaturation, ropeMicroBump, ropeContactAO),
            ropeMatParams3 = floatArrayOf(ropeLiftGlow, ropeBumpScale, ropeStretchGloss, ropeStretchSpec),
            cartoonParams = floatArrayOf(cartoonShadowBright, cartoonWrap, cartoonEdgeSmooth, 0f),
            wormParams1 = floatArrayOf(0f, 0f, 0f, 0f),
            wormParams2 = floatArrayOf(0f, 0f, 0f, 0f),
            wormParams3 = floatArrayOf(0f, 0f, 0f, 0f),
            wormParams4 = floatArrayOf(0f, 0f, 0f, 0f)
        )
        frameUniforms.bind(0)
    }

    private fun checkWinCondition() {
        val sim = simulator ?: return
        // Check for untangled ropes
        var removed = true
        while (removed) {
            removed = false
            for (bi in sim.bands.indices) {
                val band = sim.bands[bi]
                if (!band.active || band.fadeOut > 0f) continue
                if (isRopeUntangled(sim, bi)) {
                    sim.startFadeOut(bi)
                    removed = true
                    break
                }
            }
        }

        // Check level complete
        val allDone = sim.bands.all { !it.active || it.fadeOut > 0f }
        val allInactive = sim.bands.all { !it.active }
        if (allInactive) {
            mainHandler.post { onLevelComplete?.invoke() }
        }
    }

    private fun isRopeUntangled(sim: VerletSimulator, ropeIndex: Int): Boolean {
        val bandA = sim.bands[ropeIndex]
        if (bandA.particleCount < 4) return true

        for (bi in sim.bands.indices) {
            if (bi == ropeIndex) continue
            val bandB = sim.bands[bi]
            if (!bandB.active || bandB.fadeOut > 0f) continue
            if (bandB.particleCount < 4) continue

            // Check 2D crossings, skip 3 segments near pins
            val skipA = 3
            val skipB = 3
            for (ai in skipA until bandA.particleCount - 1 - skipA) {
                val a0x = bandA.positions[ai * 3]; val a0y = bandA.positions[ai * 3 + 1]
                val a1x = bandA.positions[(ai + 1) * 3]; val a1y = bandA.positions[(ai + 1) * 3 + 1]
                for (bii in skipB until bandB.particleCount - 1 - skipB) {
                    val b0x = bandB.positions[bii * 3]; val b0y = bandB.positions[bii * 3 + 1]
                    val b1x = bandB.positions[(bii + 1) * 3]; val b1y = bandB.positions[(bii + 1) * 3 + 1]
                    if (segmentsCross2D(a0x, a0y, a1x, a1y, b0x, b0y, b1x, b1y)) return false
                }
            }
        }
        return true
    }

    private fun segmentsCross2D(
        a0x: Float, a0y: Float, a1x: Float, a1y: Float,
        b0x: Float, b0y: Float, b1x: Float, b1y: Float
    ): Boolean {
        val dx = a1x - a0x; val dy = a1y - a0y
        val ex = b1x - b0x; val ey = b1y - b0y
        val cross = dx * ey - dy * ex
        if (kotlin.math.abs(cross) < 1e-9f) return false
        val fx = b0x - a0x; val fy = b0y - a0y
        val t = (fx * ey - fy * ex) / cross
        val u = (fx * dy - fy * dx) / cross
        return t in 0.01f..0.99f && u in 0.01f..0.99f
    }

    // ===== Initialization =====

    private fun compileShaders() {
        try {
            ropeProg = ShaderProgram(loadAsset("shaders/rope.vert"), loadAsset("shaders/rope.frag"))
            holeProg = ShaderProgram(loadAsset("shaders/hole.vert"), loadAsset("shaders/hole.frag"))
            tableProg = ShaderProgram(loadAsset("shaders/table.vert"), loadAsset("shaders/table.frag"))
            shadowProg = ShaderProgram(loadAsset("shaders/shadow.vert"), loadAsset("shaders/shadow.frag"))
            bloomThreshProg = ShaderProgram(loadAsset("shaders/fullscreen.vert"), loadAsset("shaders/bloom_threshold.frag"))
            bloomBlurProg = ShaderProgram(loadAsset("shaders/fullscreen.vert"), loadAsset("shaders/bloom_blur.frag"))
            postProg = ShaderProgram(loadAsset("shaders/post.vert"), loadAsset("shaders/post.frag"))
        } catch (e: Exception) {
            Log.e("GameRenderer", "Shader compile failed", e)
            // Create fallback minimal shaders
            createFallbackShaders()
        }
    }

    private fun createFallbackShaders() {
        val minVert = """
            #version 300 es
            layout(location=0) in vec3 aPos;
            layout(location=1) in vec3 aNormal;
            layout(location=2) in vec3 aColor;
            layout(std140) uniform FrameBlock { mat4 uViewProj; mat4 uInvViewProj; mat4 uLightViewProj; vec4 uLD; vec4 uAC; vec4 uCP; vec4 uOH; vec4 uSI; vec4 uTD; vec4 uWBMin; vec4 uWBMax; vec4 uHT; vec4 uVP; vec4 uLP; vec4 uTP; vec4 uTP2; vec4 uRMP; vec4 uRMP2; vec4 uRMP3; vec4 uCaP; vec4 uWP1; vec4 uWP2; vec4 uWP3; vec4 uWP4; };
            out vec3 vColor;
            out vec3 vNormal;
            void main() {
                gl_Position = uViewProj * vec4(aPos, 1.0);
                vColor = aColor;
                vNormal = aNormal;
            }
        """.trimIndent()
        val minFrag = """
            #version 300 es
            precision mediump float;
            in vec3 vColor;
            in vec3 vNormal;
            out vec4 fragColor;
            void main() {
                float light = max(0.3, dot(normalize(vNormal), normalize(vec3(-0.3, -0.5, 0.8))));
                fragColor = vec4(vColor * light, 1.0);
            }
        """.trimIndent()
        ropeProg = ShaderProgram(minVert, minFrag)
        holeProg = ShaderProgram(minVert, minFrag)

        val fsVert = """
            #version 300 es
            void main() {
                vec2 pos = vec2(gl_VertexID == 1 ? 3.0 : -1.0, gl_VertexID == 2 ? 3.0 : -1.0);
                gl_Position = vec4(pos, 0.0, 1.0);
            }
        """.trimIndent()
        val clearFrag = """
            #version 300 es
            precision mediump float;
            out vec4 fragColor;
            void main() { fragColor = vec4(0.07, 0.08, 0.12, 1.0); }
        """.trimIndent()
        tableProg = ShaderProgram(fsVert, clearFrag)
        shadowProg = ShaderProgram(minVert, """
            #version 300 es
            precision mediump float;
            out vec4 fragColor;
            void main() { fragColor = vec4(0.0); }
        """.trimIndent())
        val passFrag = """
            #version 300 es
            precision mediump float;
            uniform sampler2D uHdrTex;
            in vec2 vUV;
            out vec4 fragColor;
            void main() { fragColor = vec4(0.0); }
        """.trimIndent()
        bloomThreshProg = ShaderProgram(fsVert, passFrag)
        bloomBlurProg = ShaderProgram(fsVert, passFrag)
        val postFrag = """
            #version 300 es
            precision mediump float;
            uniform sampler2D uHdrTex;
            uniform sampler2D uBloomTex;
            uniform float uExposure;
            uniform float uBloomStrength;
            uniform float uCartoonMode;
            uniform float uCartoonEdgeStrength;
            uniform float uCartoonEdgeSmooth;
            out vec4 fragColor;
            void main() {
                vec2 uv = gl_FragCoord.xy / vec2(textureSize(uHdrTex, 0));
                vec3 hdr = texture(uHdrTex, uv).rgb;
                vec3 bloom = texture(uBloomTex, uv).rgb;
                vec3 col = hdr + bloom * uBloomStrength;
                col = vec3(1.0) - exp(-col * uExposure);
                fragColor = vec4(col, 1.0);
            }
        """.trimIndent()
        postProg = ShaderProgram(fsVert, postFrag)
    }

    private fun createGeometry() {
        // Rope VAO
        val vaoIds = IntArray(4)
        GLES30.glGenVertexArrays(4, vaoIds, 0)
        ropeVao = vaoIds[0]; holeVao = vaoIds[1]; fullscreenVao = vaoIds[2]; boardVao = vaoIds[3]

        val bufIds = IntArray(6)
        GLES30.glGenBuffers(6, bufIds, 0)
        ropeVbo = bufIds[0]; ropeIbo = bufIds[1]
        holeVbo = bufIds[2]; holeIbo = bufIds[3]
        boardVbo = bufIds[4]; boardIbo = bufIds[5]

        // Setup rope VAO vertex attributes: pos(0), normal(1), color(2), texCoord(3), params(4)
        GLES30.glBindVertexArray(ropeVao)
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, ropeVbo)
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, ropeIbo)
        val stride = 15 * 4
        GLES30.glEnableVertexAttribArray(0)
        GLES30.glVertexAttribPointer(0, 3, GLES30.GL_FLOAT, false, stride, 0)
        GLES30.glEnableVertexAttribArray(1)
        GLES30.glVertexAttribPointer(1, 3, GLES30.GL_FLOAT, false, stride, 12)
        GLES30.glEnableVertexAttribArray(2)
        GLES30.glVertexAttribPointer(2, 3, GLES30.GL_FLOAT, false, stride, 24)
        GLES30.glEnableVertexAttribArray(3)
        GLES30.glVertexAttribPointer(3, 2, GLES30.GL_FLOAT, false, stride, 36)
        GLES30.glEnableVertexAttribArray(4)
        GLES30.glVertexAttribPointer(4, 4, GLES30.GL_FLOAT, false, stride, 44)
        GLES30.glBindVertexArray(0)

        // Build hole mesh
        // iOS: holeSegments = 19, square = squareCrossSection
        val holeMesh = if (squareCrossSection) HoleMeshBuilder.buildSquare()
                       else HoleMeshBuilder.build(segments = 19)
        val hvBuf = ByteBuffer.allocateDirect(holeMesh.vertexCount * 24).order(ByteOrder.nativeOrder()).asFloatBuffer()
        hvBuf.put(holeMesh.vertices, 0, holeMesh.vertexCount * 6).position(0)
        val hiBuf = ByteBuffer.allocateDirect(holeMesh.indexCount * 2).order(ByteOrder.nativeOrder()).asShortBuffer()
        hiBuf.put(holeMesh.indices, 0, holeMesh.indexCount).position(0)

        GLES30.glBindVertexArray(holeVao)
        GLES30.glBindBuffer(GLES30.GL_ARRAY_BUFFER, holeVbo)
        GLES30.glBufferData(GLES30.GL_ARRAY_BUFFER, holeMesh.vertexCount * 24, hvBuf, GLES30.GL_STATIC_DRAW)
        GLES30.glBindBuffer(GLES30.GL_ELEMENT_ARRAY_BUFFER, holeIbo)
        GLES30.glBufferData(GLES30.GL_ELEMENT_ARRAY_BUFFER, holeMesh.indexCount * 2, hiBuf, GLES30.GL_STATIC_DRAW)
        // Hole vertex: pos(0) + normal(1) = 6 floats, stride 24
        GLES30.glEnableVertexAttribArray(0)
        GLES30.glVertexAttribPointer(0, 3, GLES30.GL_FLOAT, false, 24, 0)
        GLES30.glEnableVertexAttribArray(1)
        GLES30.glVertexAttribPointer(1, 3, GLES30.GL_FLOAT, false, 24, 12)
        holeIndexCount = holeMesh.indexCount
        GLES30.glBindVertexArray(0)

        // Fullscreen triangle VAO (no VBO - uses gl_VertexID)
        GLES30.glBindVertexArray(fullscreenVao)
        GLES30.glBindVertexArray(0)
    }

    private fun resizeFbos(w: Int, h: Int) {
        shadowFbo?.delete()
        hdrFbo?.delete()
        bloomFboA?.delete()
        bloomFboB?.delete()

        shadowFbo = Fbo.makeShadow(2048)
        hdrFbo = Fbo.makeHdr(w, h)
        bloomFboA = Fbo.makeBloom(w / 2, h / 2)
        bloomFboB = Fbo.makeBloom(w / 2, h / 2)
    }

    private fun loadAsset(path: String): String {
        return context.assets.open(path).bufferedReader().readText()
    }

    data class DragState(val bandIndex: Int, val endIndex: Int, val originalHoleIndex: Int)
}
