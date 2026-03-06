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
    val moveCount by viewModel.moveCount.collectAsState()
    val showComplete by viewModel.showLevelComplete.collectAsState()
    val showSettings by viewModel.showSettings.collectAsState()
    val completionStars by viewModel.completionStars.collectAsState()
    val completionPercentile by viewModel.completionPercentile.collectAsState()
    val canUndo by viewModel.canUndo.collectAsState()

    Box(modifier = Modifier.fillMaxSize()) {
        AndroidView(
            factory = { ctx ->
                Haptics.init(ctx)
                GameGLSurfaceView(ctx).also { view ->
                    viewModel.renderer = view.gameRenderer
                    view.gameRenderer.onLevelComplete = { viewModel.onLevelComplete() }
                    view.gameRenderer.onFpsUpdate = { viewModel.updateFps(it) }
                    view.gameRenderer.onMoveCountUpdate = { viewModel.updateMoveCount(it) }
                    view.gameRenderer.onUndoStackChanged = { viewModel.updateCanUndo(it) }
                    viewModel.syncParamsToRenderer()
                    view.gameRenderer.pendingLevelId = currentLevel
                }
            },
            modifier = Modifier.fillMaxSize()
        )

        if (!showComplete) {
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .statusBarsPadding()
                    .padding(horizontal = 12.dp, vertical = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                ToolbarButton(text = "\u21BB") { viewModel.restartLevel() }
                ToolbarButton(text = "\u2299") { viewModel.resetCamera() }
                ToolbarButton(text = "\u25C0", enabled = currentLevel > 1) { viewModel.prevLevel() }

                Text(
                    text = "Level $currentLevel  ${fps.toInt()} fps",
                    color = Color.White.copy(alpha = 0.8f),
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Medium,
                    modifier = Modifier.weight(1f)
                )

                ToolbarButton(text = "\u25B6", enabled = currentLevel < 200) { viewModel.nextLevel() }
                ToolbarButton(text = "\u2699") { viewModel.toggleSettings() }
            }

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

            Box(
                modifier = Modifier
                    .align(Alignment.BottomStart)
                    .padding(start = 20.dp, bottom = 40.dp)
            ) {
                ToolbarButton(
                    text = "\u21A9",
                    enabled = canUndo
                ) { viewModel.undo() }
            }

            Text(
                text = "$moveCount",
                color = Color.White.copy(alpha = 0.35f),
                fontSize = 15.sp,
                fontWeight = FontWeight.Medium,
                modifier = Modifier
                    .align(Alignment.BottomEnd)
                    .padding(end = 20.dp, bottom = 48.dp)
            )
        }

        if (showComplete) {
            VictoryOverlay(
                level = currentLevel,
                stars = completionStars,
                percentile = completionPercentile,
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
