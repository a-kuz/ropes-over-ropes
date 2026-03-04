package com.uzls.four.ui

import android.app.Application
import android.content.Context
import android.content.SharedPreferences
import androidx.lifecycle.AndroidViewModel
import com.uzls.four.renderer.GameRenderer
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow

class GameViewModel(application: Application) : AndroidViewModel(application) {
    private val prefs: SharedPreferences =
        application.getSharedPreferences("uzls_four", Context.MODE_PRIVATE)

    var renderer: GameRenderer? = null

    // State flows for UI
    private val _currentLevel = MutableStateFlow(prefs.getInt("level", 1))
    val currentLevel = _currentLevel.asStateFlow()

    private val _fps = MutableStateFlow(0f)
    val fps = _fps.asStateFlow()

    private val _showLevelComplete = MutableStateFlow(false)
    val showLevelComplete = _showLevelComplete.asStateFlow()

    private val _canUndo = MutableStateFlow(false)
    val canUndo = _canUndo.asStateFlow()

    private val _showSettings = MutableStateFlow(false)
    val showSettings = _showSettings.asStateFlow()

    // Physics parameters — iOS defaults
    val particleCount = MutableStateFlow(prefs.getFloat("p.ptc", 60f))
    val gravity = MutableStateFlow(prefs.getFloat("p.grav", -5.0f))
    val damping = MutableStateFlow(prefs.getFloat("p.damp", 0.97f))
    val constraintIterations = MutableStateFlow(prefs.getFloat("p.iter", 8f))
    val settleSteps = MutableStateFlow(prefs.getFloat("p.settle", 5f))
    val ropeTension = MutableStateFlow(prefs.getFloat("p.tens", 0.98f))
    val liftHeight = MutableStateFlow(prefs.getFloat("p.lift", 0.30f))
    val boardElevation = MutableStateFlow(prefs.getFloat("p.bElev", 0.12f))
    val bendCompliance = MutableStateFlow(prefs.getFloat("p.bend", 0.0015f))
    val bendVelocityCoupling = MutableStateFlow(prefs.getFloat("p.bvel", 0.45f))

    // Visual parameters — iOS defaults
    val exposure = MutableStateFlow(prefs.getFloat("v.exp", 1.05f))
    val bloomStrength = MutableStateFlow(prefs.getFloat("v.bloom", 0.35f))
    val profileSegments = MutableStateFlow(prefs.getFloat("v.prof", 16f))
    val ropeRadiusScale = MutableStateFlow(prefs.getFloat("v.rscale", 1.0f))
    val stretchThinning = MutableStateFlow(prefs.getFloat("v.stretch", 0.5f))

    // Light parameters — iOS defaults
    val lightDirX = MutableStateFlow(prefs.getFloat("l.dx", -0.65f))
    val lightDirY = MutableStateFlow(prefs.getFloat("l.dy", -0.35f))
    val lightDirZ = MutableStateFlow(prefs.getFloat("l.dz", 0.67f))
    val lightIntensity = MutableStateFlow(prefs.getFloat("l.int", 1.0f))
    val ambient = MutableStateFlow(prefs.getFloat("l.amb", 0.08f))

    fun updateFps(fps: Float) { _fps.value = fps }

    fun loadLevel(id: Int) {
        _currentLevel.value = id
        _showLevelComplete.value = false
        prefs.edit().putInt("level", id).apply()
        renderer?.pendingLevelId = id
    }

    fun restartLevel() {
        _showLevelComplete.value = false
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

    fun onLevelComplete() {
        _showLevelComplete.value = true
    }

    fun toggleSettings() {
        _showSettings.value = !_showSettings.value
    }

    fun syncParamsToRenderer() {
        val r = renderer ?: return
        r.gravity = gravity.value
        r.damping = damping.value
        r.constraintIterations = constraintIterations.value.toInt()
        r.settleSteps = settleSteps.value.toInt()
        r.ropeTension = ropeTension.value
        r.liftHeight = liftHeight.value
        r.boardElevation = boardElevation.value
        r.bendCompliance = bendCompliance.value
        r.bendVelocityCoupling = bendVelocityCoupling.value
        r.exposure = exposure.value
        r.bloomStrength = bloomStrength.value
        r.profileSegments = profileSegments.value.toInt()
        r.ropeRadiusScale = ropeRadiusScale.value
        r.stretchThinning = stretchThinning.value
        r.lightDirX = lightDirX.value
        r.lightDirY = lightDirY.value
        r.lightDirZ = lightDirZ.value
        r.lightIntensity = lightIntensity.value
        r.ambient = ambient.value
        r.particleCount = particleCount.value.toInt()
    }

    fun saveParam(key: String, value: Float) {
        prefs.edit().putFloat(key, value).apply()
    }
}
