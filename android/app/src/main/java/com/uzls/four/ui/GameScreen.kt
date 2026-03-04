package com.uzls.four.ui

import androidx.compose.animation.*
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import com.uzls.four.game.GameGLSurfaceView
import com.uzls.four.game.Haptics

@Composable
fun GameScreen(viewModel: GameViewModel) {
    val currentLevel by viewModel.currentLevel.collectAsState()
    val fps by viewModel.fps.collectAsState()
    val showComplete by viewModel.showLevelComplete.collectAsState()
    val showSettings by viewModel.showSettings.collectAsState()

    Box(modifier = Modifier.fillMaxSize()) {
        // GL Surface
        AndroidView(
            factory = { ctx ->
                Haptics.init(ctx)
                GameGLSurfaceView(ctx).also { view ->
                    viewModel.renderer = view.gameRenderer
                    view.gameRenderer.onLevelComplete = { viewModel.onLevelComplete() }
                    view.gameRenderer.onFpsUpdate = { viewModel.updateFps(it) }
                    viewModel.syncParamsToRenderer()
                    view.gameRenderer.pendingLevelId = currentLevel
                }
            },
            modifier = Modifier.fillMaxSize()
        )

        // Top toolbar
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .statusBarsPadding()
                .padding(horizontal = 12.dp, vertical = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            // Restart
            ToolbarButton(text = "↻") { viewModel.restartLevel() }

            // Previous level
            ToolbarButton(text = "◀", enabled = currentLevel > 1) { viewModel.prevLevel() }

            // Level display + FPS
            Text(
                text = "Level $currentLevel  ${fps.toInt()} fps",
                color = Color.White.copy(alpha = 0.8f),
                fontSize = 14.sp,
                fontWeight = FontWeight.Medium,
                modifier = Modifier.weight(1f)
            )

            // Next level
            ToolbarButton(text = "▶", enabled = currentLevel < 200) { viewModel.nextLevel() }

            // Settings
            ToolbarButton(text = "⚙") { viewModel.toggleSettings() }
        }

        // Settings panel
        AnimatedVisibility(
            visible = showSettings,
            enter = slideInVertically() + fadeIn(),
            exit = slideOutVertically() + fadeOut(),
            modifier = Modifier
                .align(Alignment.TopCenter)
                .statusBarsPadding()
                .padding(top = 52.dp)
        ) {
            SettingsPanel(viewModel)
        }

        // Victory overlay
        if (showComplete) {
            VictoryOverlay(
                level = currentLevel,
                onNextLevel = { viewModel.nextLevel() }
            )
        }
    }
}

@Composable
private fun ToolbarButton(text: String, enabled: Boolean = true, onClick: () -> Unit) {
    Box(
        modifier = Modifier
            .size(36.dp)
            .clip(CircleShape)
            .background(Color.Black.copy(alpha = if (enabled) 0.5f else 0.2f))
            .clickable(enabled = enabled) { onClick() },
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            color = Color.White.copy(alpha = if (enabled) 0.9f else 0.3f),
            fontSize = 16.sp
        )
    }
}
