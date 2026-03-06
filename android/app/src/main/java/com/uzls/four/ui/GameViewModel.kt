package com.uzls.four.ui

import android.app.Application
import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import android.content.SharedPreferences
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.uzls.four.renderer.GameRenderer
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import org.json.JSONObject
import kotlin.math.ceil

class GameViewModel(application: Application) : AndroidViewModel(application) {
    private val prefs: SharedPreferences =
        application.getSharedPreferences("uzls_four", Context.MODE_PRIVATE)

    var renderer: GameRenderer? = null

    // State flows for UI
    private val _currentLevel = MutableStateFlow(prefs.getInt("level", 11))
    val currentLevel = _currentLevel.asStateFlow()

    private val _fps = MutableStateFlow(0f)
    val fps = _fps.asStateFlow()

    private val _showLevelComplete = MutableStateFlow(false)
    val showLevelComplete = _showLevelComplete.asStateFlow()

    private val _moveCount = MutableStateFlow(0)
    val moveCount = _moveCount.asStateFlow()

    private val _completionMoves = MutableStateFlow(0)
    val completionMoves = _completionMoves.asStateFlow()

    private val _completionStars = MutableStateFlow(0)
    val completionStars = _completionStars.asStateFlow()

    private val _completionPercentile = MutableStateFlow(0)
    val completionPercentile = _completionPercentile.asStateFlow()

    private val _isNewRecord = MutableStateFlow(false)
    val isNewRecord = _isNewRecord.asStateFlow()

    private val _previousRecord = MutableStateFlow<Int?>(null)
    val previousRecord = _previousRecord.asStateFlow()

    private val _canUndo = MutableStateFlow(false)
    val canUndo = _canUndo.asStateFlow()

    private val _showSettings = MutableStateFlow(false)
    val showSettings = _showSettings.asStateFlow()

    val leaderboard = LeaderboardAPI(application)
    private var levelStartTime = System.currentTimeMillis()

    init {
        viewModelScope.launch {
            leaderboard.ensureRegistered()
            leaderboard.flushQueue()
        }
    }

    // Rope parameters
    val particleCount = MutableStateFlow(prefs.getFloat("p.ptc", 69f))
    val gravity = MutableStateFlow(prefs.getFloat("p.grav", 0f))
    val damping = MutableStateFlow(prefs.getFloat("p.damp", 0.552f))
    val ropeTension = MutableStateFlow(prefs.getFloat("p.tens", 1f))
    val frictionCoefficient = MutableStateFlow(prefs.getFloat("p.fric", 1.985f))

    // Solver parameters
    val constraintIterations = MutableStateFlow(prefs.getFloat("p.iter", 3f))
    val maxSubsteps = MutableStateFlow(prefs.getFloat("p.maxsub", 4f))
    val physicsRate = MutableStateFlow(prefs.getFloat("p.rate", 120f))
    val settleSteps = MutableStateFlow(prefs.getFloat("p.settle", 7f))
    val bendCompliance = MutableStateFlow(prefs.getFloat("p.bend", 0.00161f))
    val bendVelocityCoupling = MutableStateFlow(prefs.getFloat("p.bvel", 0.175f))

    // Drag parameters
    val dragHeight = MutableStateFlow(prefs.getFloat("p.dragH", 0.771f))
    val liftHeight = MutableStateFlow(prefs.getFloat("p.lift", 1.151f))
    val boardElevation = MutableStateFlow(prefs.getFloat("p.bElev", 0.02f))

    // Visual parameters
    val squareCrossSection = MutableStateFlow(prefs.getFloat("v.sqcs", 1f) > 0.5f)
    val profileSegments = MutableStateFlow(prefs.getFloat("v.prof", 7f))
    val holeRadiusScale = MutableStateFlow(prefs.getFloat("v.hrscale", 1.156f))
    val ropeRadiusScale = MutableStateFlow(prefs.getFloat("v.rscale", 0.987f))
    val stretchThinning = MutableStateFlow(prefs.getFloat("v.stretch", 0.168f))
    val holeTintR = MutableStateFlow(prefs.getFloat("v.htR", 1.0f))
    val holeTintG = MutableStateFlow(prefs.getFloat("v.htG", 0.89f))
    val holeTintB = MutableStateFlow(prefs.getFloat("v.htB", 0.3f))
    val holeTintAmount = MutableStateFlow(prefs.getFloat("v.htAmt", 1.0f))
    val exposure = MutableStateFlow(prefs.getFloat("v.exp", 0.803f))
    val bloomStrength = MutableStateFlow(prefs.getFloat("v.bloom", 0.117f))

    // Matte parameters
    val ropeMatte = MutableStateFlow(prefs.getFloat("m.matte", 0.047f))
    val ropeGloss = MutableStateFlow(prefs.getFloat("m.gloss", 1.961f))
    val ropeDiffuseWrap = MutableStateFlow(prefs.getFloat("m.dwrap", 0.009f))
    val ropeSubsurface = MutableStateFlow(prefs.getFloat("m.ssurf", 0.766f))
    val ropeEdgeLight = MutableStateFlow(prefs.getFloat("m.edge", 0.479f))
    val ropeSaturation = MutableStateFlow(prefs.getFloat("m.sat", 0.798f))
    val ropeMicroBump = MutableStateFlow(prefs.getFloat("m.bump", 1.5f))
    val ropeBumpScale = MutableStateFlow(prefs.getFloat("m.bscale", 3.99f))
    val ropeContactAO = MutableStateFlow(prefs.getFloat("m.cao", 1.0f))
    val ropeLiftGlow = MutableStateFlow(prefs.getFloat("m.glow", 0.195f))
    val ropeStretchGloss = MutableStateFlow(prefs.getFloat("m.sgloss", 0.887f))
    val ropeStretchSpec = MutableStateFlow(prefs.getFloat("m.sspec", 0.927f))
    val ropeEnvReflect = MutableStateFlow(prefs.getFloat("m.envref", 0.15f))

    // Cartoon parameters
    val cartoonMode = MutableStateFlow(prefs.getFloat("c.mode", 0f))
    val cartoonExposure = MutableStateFlow(prefs.getFloat("c.exp", 1.33f))
    val cartoonLevels = MutableStateFlow(prefs.getFloat("c.levels", 2f))
    val cartoonEdgeStrength = MutableStateFlow(prefs.getFloat("c.edge", 1.0f))
    val cartoonShadowBright = MutableStateFlow(prefs.getFloat("c.shadow", 0.38f))
    val cartoonWrap = MutableStateFlow(prefs.getFloat("c.wrap", 0.15f))
    val cartoonEdgeSmooth = MutableStateFlow(prefs.getFloat("c.esmooth", 0.5f))

    // Table parameters
    val tableStyle = MutableStateFlow(prefs.getFloat("v.tst", 0f))
    val tableColor1R = MutableStateFlow(prefs.getFloat("v.tc1r", 0.123f))
    val tableColor1G = MutableStateFlow(prefs.getFloat("v.tc1g", 0.267f))
    val tableColor1B = MutableStateFlow(prefs.getFloat("v.tc1b", 0.121f))
    val tableColor2R = MutableStateFlow(prefs.getFloat("v.tc2r", 0.235f))
    val tableColor2G = MutableStateFlow(prefs.getFloat("v.tc2g", 0.13f))
    val tableColor2B = MutableStateFlow(prefs.getFloat("v.tc2b", 0.20f))
    val woodSeed = MutableStateFlow(prefs.getFloat("v.wsd", 0.159f))
    val woodBrightness = MutableStateFlow(prefs.getFloat("v.wbr", 0.818f))
    val woodPatternScale = MutableStateFlow(prefs.getFloat("v.wps", 1.128f))

    // Light parameters
    val lightDirX = MutableStateFlow(prefs.getFloat("l.dx", -0.078f))
    val lightDirY = MutableStateFlow(prefs.getFloat("l.dy", 0.063f))
    val lightDirZ = MutableStateFlow(prefs.getFloat("l.dz", 0.197f))
    val lightIntensity = MutableStateFlow(prefs.getFloat("l.int", 0.532f))
    val ambient = MutableStateFlow(prefs.getFloat("l.amb", 0.139f))
    val shadowBias = MutableStateFlow(prefs.getFloat("l.sbias", 0.0001f))
    val shadowDarkness = MutableStateFlow(prefs.getFloat("l.sdark", 0.007f))
    val shadowSize = MutableStateFlow(prefs.getFloat("l.ssize", 0.072f))

    // Sticky adhesion parameters
    val stickyEnabled = MutableStateFlow(prefs.getFloat("s.enabled", 0f) > 0.5f)
    val stickyStrength = MutableStateFlow(prefs.getFloat("s.str", 0f))
    val stickyRadius = MutableStateFlow(prefs.getFloat("s.rad", 4f))
    val stickyDamping = MutableStateFlow(prefs.getFloat("s.damp", 0.165f))
    val stickyBreakThreshold = MutableStateFlow(prefs.getFloat("s.break", 0.255f))

    // Cap parameters
    val capRadiusScale = MutableStateFlow(prefs.getFloat("cap.rscale", 0.33f))
    val capSegments = MutableStateFlow(prefs.getFloat("cap.segs", 4f))
    val capRings = MutableStateFlow(prefs.getFloat("cap.rings", 6f))
    val capDarken = MutableStateFlow(prefs.getFloat("cap.dark", 0.988f))

    fun updateFps(fps: Float) { _fps.value = fps }

    fun loadLevel(id: Int) {
        _currentLevel.value = id
        _showLevelComplete.value = false
        _moveCount.value = 0
        levelStartTime = System.currentTimeMillis()
        prefs.edit().putInt("level", id).apply()
        renderer?.pendingLevelId = id
    }

    fun restartLevel() {
        _showLevelComplete.value = false
        _moveCount.value = 0
        levelStartTime = System.currentTimeMillis()
        renderer?.pendingRestart = true
    }

    fun nextLevel() {
        loadLevel((_currentLevel.value + 1).coerceAtMost(200))
    }

    fun prevLevel() {
        loadLevel((_currentLevel.value - 1).coerceAtLeast(1))
    }

    fun undo() {
        renderer?.pendingUndo = true
    }

    fun resetCamera() {
        renderer?.postCameraReset()
    }

    fun updateMoveCount(count: Int) {
        _moveCount.value = count
    }

    fun updateCanUndo(canUndo: Boolean) {
        _canUndo.value = canUndo
    }

    fun onLevelComplete() {
        val moves = _moveCount.value
        val level = _currentLevel.value
        _completionMoves.value = moves
        _completionStars.value = 3
        _completionPercentile.value = 0

        val recordKey = "record.lvl.$level"
        val prev = if (prefs.contains(recordKey)) prefs.getInt(recordKey, 0) else null
        _previousRecord.value = prev
        val isNew = prev == null || moves < prev
        if (isNew) {
            _isNewRecord.value = true
            prefs.edit().putInt(recordKey, moves).apply()
        } else {
            _isNewRecord.value = false
        }

        _showLevelComplete.value = true

        val timeMs = (System.currentTimeMillis() - levelStartTime).toInt()
        viewModelScope.launch {
            if (isNew) {
                leaderboard.submitResult(level, moves, timeMs)
            }
            val stats = leaderboard.fetchStats(level, moves)
            stats?.percentile?.let { _completionPercentile.value = it }
            val best = stats?.best_moves ?: 0
            if (best > 0) {
                _completionStars.value = when {
                    moves <= best || moves <= ceil(best * 1.5).toInt() -> 3
                    moves > best * 5 -> 1
                    else -> 2
                }
            }
        }
    }

    fun toggleSettings() {
        _showSettings.value = !_showSettings.value
    }

    fun syncParamsToRenderer() {
        val r = renderer ?: return
        r.particleCount = particleCount.value.toInt()
        r.gravity = gravity.value
        r.damping = damping.value
        r.ropeTension = ropeTension.value
        r.frictionCoefficient = frictionCoefficient.value
        r.constraintIterations = constraintIterations.value.toInt()
        r.maxSubsteps = maxSubsteps.value.toInt()
        r.physicsRate = physicsRate.value
        r.settleSteps = settleSteps.value.toInt()
        r.bendCompliance = bendCompliance.value
        r.bendVelocityCoupling = bendVelocityCoupling.value
        r.dragHeight = dragHeight.value
        r.liftHeight = liftHeight.value
        r.boardElevation = boardElevation.value
        r.squareCrossSection = squareCrossSection.value
        r.profileSegments = profileSegments.value.toInt()
        r.holeRadiusScale = holeRadiusScale.value
        r.ropeRadiusScale = ropeRadiusScale.value
        r.stretchThinning = stretchThinning.value
        r.holeTintR = holeTintR.value
        r.holeTintG = holeTintG.value
        r.holeTintB = holeTintB.value
        r.holeTintAmount = holeTintAmount.value
        r.exposure = exposure.value
        r.bloomStrength = bloomStrength.value
        r.tableStyle = tableStyle.value.toInt()
        r.tableColor1R = tableColor1R.value
        r.tableColor1G = tableColor1G.value
        r.tableColor1B = tableColor1B.value
        r.tableColor2R = tableColor2R.value
        r.tableColor2G = tableColor2G.value
        r.tableColor2B = tableColor2B.value
        r.woodSeed = woodSeed.value
        r.woodBrightness = woodBrightness.value
        r.woodPatternScale = woodPatternScale.value
        r.lightDirX = lightDirX.value
        r.lightDirY = lightDirY.value
        r.lightDirZ = lightDirZ.value
        r.lightIntensity = lightIntensity.value
        r.ambient = ambient.value
        r.shadowBias = shadowBias.value
        r.shadowDarkness = shadowDarkness.value
        r.shadowSize = shadowSize.value
        r.ropeMatte = ropeMatte.value
        r.ropeGloss = ropeGloss.value
        r.ropeDiffuseWrap = ropeDiffuseWrap.value
        r.ropeSubsurface = ropeSubsurface.value
        r.ropeEdgeLight = ropeEdgeLight.value
        r.ropeSaturation = ropeSaturation.value
        r.ropeMicroBump = ropeMicroBump.value
        r.ropeBumpScale = ropeBumpScale.value
        r.ropeContactAO = ropeContactAO.value
        r.ropeLiftGlow = ropeLiftGlow.value
        r.ropeStretchGloss = ropeStretchGloss.value
        r.ropeStretchSpec = ropeStretchSpec.value
        r.ropeEnvReflect = ropeEnvReflect.value
        r.cartoonMode = cartoonMode.value
        r.cartoonExposure = cartoonExposure.value
        r.cartoonLevels = cartoonLevels.value
        r.cartoonEdgeStrength = cartoonEdgeStrength.value
        r.cartoonShadowBright = cartoonShadowBright.value
        r.cartoonWrap = cartoonWrap.value
        r.cartoonEdgeSmooth = cartoonEdgeSmooth.value
        r.capRadiusScale = capRadiusScale.value
        r.capSegments = capSegments.value.toInt()
        r.capRings = capRings.value.toInt()
        r.capDarken = capDarken.value
        r.stickyEnabled = stickyEnabled.value
        r.stickyStrength = stickyStrength.value
        r.stickyRadius = stickyRadius.value
        r.stickyDamping = stickyDamping.value
        r.stickyBreakThreshold = stickyBreakThreshold.value
    }

    fun saveParam(key: String, value: Float) {
        prefs.edit().putFloat(key, value).apply()
    }

    fun dumpSettingsToClipboard(): Boolean {
        val json = JSONObject().apply {
            put("particleCount", particleCount.value.toInt())
            put("gravity", gravity.value.toDouble())
            put("damping", damping.value.toDouble())
            put("constraintIterations", constraintIterations.value.toInt())
            put("maxSubsteps", maxSubsteps.value.toInt())
            put("physicsRate", physicsRate.value.toDouble())
            put("settleSteps", settleSteps.value.toInt())
            put("bendCompliance", bendCompliance.value.toDouble())
            put("bendVelocityCoupling", bendVelocityCoupling.value.toDouble())
            put("dragHeight", dragHeight.value.toDouble())
            put("liftHeight", liftHeight.value.toDouble())
            put("boardElevation", boardElevation.value.toDouble())
            put("ropeTension", ropeTension.value.toDouble())
            put("frictionCoefficient", frictionCoefficient.value.toDouble())
            put("profileSegments", profileSegments.value.toInt())
            put("holeRadiusScale", holeRadiusScale.value.toDouble())
            put("ropeRadiusScale", ropeRadiusScale.value.toDouble())
            put("stretchThinning", stretchThinning.value.toDouble())
            put("holeTintR", holeTintR.value.toDouble())
            put("holeTintG", holeTintG.value.toDouble())
            put("holeTintB", holeTintB.value.toDouble())
            put("holeTintAmount", holeTintAmount.value.toDouble())
            put("exposure", exposure.value.toDouble())
            put("bloomStrength", bloomStrength.value.toDouble())
            put("lightIntensity", lightIntensity.value.toDouble())
            put("lightDirX", lightDirX.value.toDouble())
            put("lightDirY", lightDirY.value.toDouble())
            put("lightDirZ", lightDirZ.value.toDouble())
            put("ambient", ambient.value.toDouble())
            put("shadowBias", shadowBias.value.toDouble())
            put("shadowDarkness", shadowDarkness.value.toDouble())
            put("lightSize", shadowSize.value.toDouble())
            put("cartoonShaderEnabled", cartoonMode.value > 0.5f)
            put("cartoonExposure", cartoonExposure.value.toDouble())
            put("cartoonEdgeStrength", cartoonEdgeStrength.value.toDouble())
            put("cartoonLevels", cartoonLevels.value.toInt())
            put("cartoonShadowBright", cartoonShadowBright.value.toDouble())
            put("cartoonWrap", cartoonWrap.value.toDouble())
            put("cartoonEdgeSmooth", cartoonEdgeSmooth.value.toDouble())
            put("tableStyle", when (tableStyle.value.toInt()) {
                0 -> "Wood"
                1 -> "Gradient"
                2 -> "Solid"
                else -> "Wood"
            })
            put("tableColor1R", tableColor1R.value.toDouble())
            put("tableColor1G", tableColor1G.value.toDouble())
            put("tableColor1B", tableColor1B.value.toDouble())
            put("tableColor2R", tableColor2R.value.toDouble())
            put("tableColor2G", tableColor2G.value.toDouble())
            put("tableColor2B", tableColor2B.value.toDouble())
            put("woodSeed", woodSeed.value.toDouble())
            put("woodBrightness", woodBrightness.value.toDouble())
            put("woodPatternScale", woodPatternScale.value.toDouble())
            put("capRadiusScale", capRadiusScale.value.toDouble())
            put("capSegments", capSegments.value.toInt())
            put("capRings", capRings.value.toInt())
            put("capDarken", capDarken.value.toDouble())
            put("squareCrossSection", squareCrossSection.value)
            put("currentLevel", currentLevel.value)
            put("ropeMatte", ropeMatte.value.toDouble())
            put("ropeGloss", ropeGloss.value.toDouble())
            put("ropeDiffuseWrap", ropeDiffuseWrap.value.toDouble())
            put("ropeSubsurface", ropeSubsurface.value.toDouble())
            put("ropeEdgeLight", ropeEdgeLight.value.toDouble())
            put("ropeSaturation", ropeSaturation.value.toDouble())
            put("ropeMicroBump", ropeMicroBump.value.toDouble())
            put("ropeBumpScale", ropeBumpScale.value.toDouble())
            put("ropeContactAO", ropeContactAO.value.toDouble())
            put("ropeLiftGlow", ropeLiftGlow.value.toDouble())
            put("ropeStretchGloss", ropeStretchGloss.value.toDouble())
            put("ropeStretchSpec", ropeStretchSpec.value.toDouble())
            put("ropeEnvReflect", ropeEnvReflect.value.toDouble())
            put("stickyEnabled", stickyEnabled.value)
            put("stickyStrength", stickyStrength.value.toDouble())
            put("stickyRadius", stickyRadius.value.toDouble())
            put("stickyDamping", stickyDamping.value.toDouble())
            put("stickyBreakThreshold", stickyBreakThreshold.value.toDouble())
        }

        val cm = getApplication<Application>().getSystemService(Context.CLIPBOARD_SERVICE) as? ClipboardManager
            ?: return false
        cm.setPrimaryClip(ClipData.newPlainText("uzls_settings", json.toString(2)))
        return true
    }

    fun importSettingsFromClipboard(): Boolean {
        val cm = getApplication<Application>().getSystemService(Context.CLIPBOARD_SERVICE) as? ClipboardManager
            ?: return false
        val text = cm.primaryClip?.getItemAt(0)?.text?.toString() ?: return false
        val json = try { JSONObject(text) } catch (_: Exception) { return false }

        fun f(key: String): Float? = if (json.has(key)) json.getDouble(key).toFloat() else null
        fun b(key: String): Boolean? = if (json.has(key)) json.getBoolean(key) else null

        f("particleCount")?.let { particleCount.value = it; saveParam("p.ptc", it) }
        f("gravity")?.let { gravity.value = it; saveParam("p.grav", it) }
        f("damping")?.let { damping.value = it; saveParam("p.damp", it) }
        f("ropeTension")?.let { ropeTension.value = it; saveParam("p.tens", it) }
        f("constraintIterations")?.let { constraintIterations.value = it; saveParam("p.iter", it) }
        f("maxSubsteps")?.let { maxSubsteps.value = it; saveParam("p.maxsub", it) }
        f("physicsRate")?.let { physicsRate.value = it; saveParam("p.rate", it) }
        f("settleSteps")?.let { settleSteps.value = it; saveParam("p.settle", it) }
        f("bendCompliance")?.let { bendCompliance.value = it; saveParam("p.bend", it) }
        f("bendVelocityCoupling")?.let { bendVelocityCoupling.value = it; saveParam("p.bvel", it) }
        f("dragHeight")?.let { dragHeight.value = it; saveParam("p.dragH", it) }
        f("liftHeight")?.let { liftHeight.value = it; saveParam("p.lift", it) }
        f("profileSegments")?.let { profileSegments.value = it; saveParam("v.prof", it) }
        f("holeRadiusScale")?.let { holeRadiusScale.value = it; saveParam("v.hrscale", it) }
        f("ropeRadiusScale")?.let { ropeRadiusScale.value = it; saveParam("v.rscale", it) }
        f("stretchThinning")?.let { stretchThinning.value = it; saveParam("v.stretch", it) }
        f("holeTintR")?.let { holeTintR.value = it; saveParam("v.htR", it) }
        f("holeTintG")?.let { holeTintG.value = it; saveParam("v.htG", it) }
        f("holeTintB")?.let { holeTintB.value = it; saveParam("v.htB", it) }
        f("holeTintAmount")?.let { holeTintAmount.value = it; saveParam("v.htAmt", it) }
        f("exposure")?.let { exposure.value = it; saveParam("v.exp", it) }
        f("bloomStrength")?.let { bloomStrength.value = it; saveParam("v.bloom", it) }
        b("squareCrossSection")?.let { squareCrossSection.value = it; saveParam("v.sqcs", if (it) 1f else 0f) }
        f("ropeMatte")?.let { ropeMatte.value = it; saveParam("m.matte", it) }
        f("ropeGloss")?.let { ropeGloss.value = it; saveParam("m.gloss", it) }
        f("ropeDiffuseWrap")?.let { ropeDiffuseWrap.value = it; saveParam("m.dwrap", it) }
        f("ropeSubsurface")?.let { ropeSubsurface.value = it; saveParam("m.ssurf", it) }
        f("ropeEdgeLight")?.let { ropeEdgeLight.value = it; saveParam("m.edge", it) }
        f("ropeSaturation")?.let { ropeSaturation.value = it; saveParam("m.sat", it) }
        f("ropeMicroBump")?.let { ropeMicroBump.value = it; saveParam("m.bump", it) }
        f("ropeBumpScale")?.let { ropeBumpScale.value = it; saveParam("m.bscale", it) }
        f("ropeContactAO")?.let { ropeContactAO.value = it; saveParam("m.cao", it) }
        f("ropeLiftGlow")?.let { ropeLiftGlow.value = it; saveParam("m.glow", it) }
        f("ropeStretchGloss")?.let { ropeStretchGloss.value = it; saveParam("m.sgloss", it) }
        f("ropeStretchSpec")?.let { ropeStretchSpec.value = it; saveParam("m.sspec", it) }
        f("ropeEnvReflect")?.let { ropeEnvReflect.value = it; saveParam("m.envref", it) }
        f("lightIntensity")?.let { lightIntensity.value = it; saveParam("l.int", it) }
        f("lightDirX")?.let { lightDirX.value = it; saveParam("l.dx", it) }
        f("lightDirY")?.let { lightDirY.value = it; saveParam("l.dy", it) }
        f("lightDirZ")?.let { lightDirZ.value = it; saveParam("l.dz", it) }
        f("ambient")?.let { ambient.value = it; saveParam("l.amb", it) }
        f("shadowBias")?.let { shadowBias.value = it; saveParam("l.sbias", it) }
        f("shadowDarkness")?.let { shadowDarkness.value = it; saveParam("l.sdark", it) }
        f("lightSize")?.let { shadowSize.value = it; saveParam("l.ssize", it) }
        f("cartoonExposure")?.let { cartoonExposure.value = it; saveParam("c.exp", it) }
        f("cartoonEdgeStrength")?.let { cartoonEdgeStrength.value = it; saveParam("c.edge", it) }
        f("cartoonLevels")?.let { cartoonLevels.value = it; saveParam("c.levels", it) }
        f("cartoonShadowBright")?.let { cartoonShadowBright.value = it; saveParam("c.shadow", it) }
        f("cartoonWrap")?.let { cartoonWrap.value = it; saveParam("c.wrap", it) }
        f("cartoonEdgeSmooth")?.let { cartoonEdgeSmooth.value = it; saveParam("c.esmooth", it) }
        if (json.has("tableStyle")) {
            val style = when (json.getString("tableStyle")) {
                "Wood" -> 0f; "Gradient" -> 1f; "Solid" -> 2f; else -> null
            }
            style?.let { tableStyle.value = it; saveParam("v.tst", it) }
        }
        f("tableColor1R")?.let { tableColor1R.value = it; saveParam("v.tc1r", it) }
        f("tableColor1G")?.let { tableColor1G.value = it; saveParam("v.tc1g", it) }
        f("tableColor1B")?.let { tableColor1B.value = it; saveParam("v.tc1b", it) }
        f("tableColor2R")?.let { tableColor2R.value = it; saveParam("v.tc2r", it) }
        f("tableColor2G")?.let { tableColor2G.value = it; saveParam("v.tc2g", it) }
        f("tableColor2B")?.let { tableColor2B.value = it; saveParam("v.tc2b", it) }
        f("woodSeed")?.let { woodSeed.value = it; saveParam("v.wsd", it) }
        f("woodBrightness")?.let { woodBrightness.value = it; saveParam("v.wbr", it) }
        f("woodPatternScale")?.let { woodPatternScale.value = it; saveParam("v.wps", it) }
        f("capRadiusScale")?.let { capRadiusScale.value = it; saveParam("cap.rscale", it) }
        f("capSegments")?.let { capSegments.value = it; saveParam("cap.segs", it) }
        f("capRings")?.let { capRings.value = it; saveParam("cap.rings", it) }
        f("capDarken")?.let { capDarken.value = it; saveParam("cap.dark", it) }
        b("stickyEnabled")?.let { stickyEnabled.value = it; saveParam("s.enabled", if (it) 1f else 0f) }
        f("stickyStrength")?.let { stickyStrength.value = it; saveParam("s.str", it) }
        f("stickyRadius")?.let { stickyRadius.value = it; saveParam("s.rad", it) }
        f("stickyDamping")?.let { stickyDamping.value = it; saveParam("s.damp", it) }
        f("stickyBreakThreshold")?.let { stickyBreakThreshold.value = it; saveParam("s.break", it) }
        if (json.has("cartoonShaderEnabled")) {
            val enabled = json.getBoolean("cartoonShaderEnabled")
            cartoonMode.value = if (enabled) 1f else 0f
            saveParam("c.mode", cartoonMode.value)
        }

        syncParamsToRenderer()

        if (json.has("currentLevel")) {
            val lvl = json.getInt("currentLevel")
            if (lvl >= 1) loadLevel(lvl)
        }

        return true
    }
}
