package com.uzls.four.ui

import androidx.compose.animation.*
import androidx.compose.animation.core.*
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.random.Random

@Composable
fun VictoryOverlay(level: Int, stars: Int, percentile: Int, onNextLevel: () -> Unit) {
    var visible by remember { mutableStateOf(false) }
    LaunchedEffect(Unit) { visible = true }

    val scale by animateFloatAsState(
        targetValue = if (visible) 1f else 0.7f,
        animationSpec = spring(dampingRatio = 0.75f, stiffness = Spring.StiffnessMediumLow)
    )
    val alpha by animateFloatAsState(
        targetValue = if (visible) 1f else 0f,
        animationSpec = tween(300)
    )
    val buttonAlpha by animateFloatAsState(
        targetValue = if (visible) 1f else 0f,
        animationSpec = tween(450, delayMillis = 200)
    )
    val buttonOffsetY by animateFloatAsState(
        targetValue = if (visible) 0f else 30f,
        animationSpec = tween(450, delayMillis = 200, easing = EaseOut)
    )

    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black.copy(alpha = 0.4f * alpha)),
        contentAlignment = Alignment.Center
    ) {
        ConfettiBurst()

        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = Modifier
                .graphicsLayer {
                    scaleX = scale; scaleY = scale
                    this.alpha = alpha
                }
        ) {
            Text(
                text = "Level $level",
                color = Color.White,
                fontSize = 42.sp,
                fontWeight = FontWeight.Bold
            )
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = "completed!",
                color = Color.White.copy(alpha = 0.8f),
                fontSize = 24.sp,
                fontWeight = FontWeight.Medium
            )
            Spacer(modifier = Modifier.height(20.dp))

            Text(
                text = buildString {
                    repeat(stars) { append("\u2605") }
                    repeat(3 - stars) { append("\u2606") }
                },
                color = Color(0xFFFFD700),
                fontSize = 40.sp
            )
            Spacer(modifier = Modifier.height(12.dp))

            if (percentile >= 50) {
                Text(
                    text = "Better than $percentile% of players",
                    color = Color.White.copy(alpha = 0.7f),
                    fontSize = 15.sp,
                    fontWeight = FontWeight.Medium
                )
                Spacer(modifier = Modifier.height(12.dp))
            }

            Box(
                modifier = Modifier
                    .graphicsLayer {
                        this.alpha = buttonAlpha
                        translationY = buttonOffsetY
                    }
                    .clip(RoundedCornerShape(24.dp))
                    .background(
                        Brush.horizontalGradient(
                            listOf(Color(0xFF5A99FF), Color(0xFF8059F2))
                        )
                    )
                    .clickable { onNextLevel() }
                    .padding(horizontal = 36.dp, vertical = 14.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "Level ${level + 1}  \u2192",
                    color = Color.White,
                    fontSize = 18.sp,
                    fontWeight = FontWeight.SemiBold
                )
            }
        }
    }
}

private class ConfettoPiece(
    val x: Float,
    val color: Color,
    val width: Float,
    val height: Float,
    val initialRotation: Float,
    val drift: Float,
    val delay: Int,
    val duration: Int
)

@Composable
private fun ConfettiBurst() {
    val pieces = remember {
        val colors = listOf(
            Color(0xFFFF4444), Color(0xFFFF9500), Color(0xFFFFCC00),
            Color(0xFF34C759), Color(0xFF007AFF), Color(0xFFAF52DE),
            Color(0xFFFF2D55), Color(0xFFFF6699), Color(0xFF5AC8FA),
            Color(0xFFFFD426)
        )
        List(40) { i ->
            val sz = Random.nextFloat() * 6f + 4f
            ConfettoPiece(
                x = Random.nextFloat() * 0.9f + 0.05f,
                color = colors[i % colors.size],
                width = sz,
                height = sz * (Random.nextFloat() * 1f + 0.5f),
                initialRotation = Random.nextFloat() * 360f,
                drift = (Random.nextFloat() - 0.5f) * 60f,
                delay = (Random.nextFloat() * 400).toInt(),
                duration = (Random.nextFloat() * 1200 + 1800).toInt()
            )
        }
    }

    var launched by remember { mutableStateOf(false) }
    LaunchedEffect(Unit) {
        kotlinx.coroutines.delay(100)
        launched = true
    }

    val progress = pieces.map { piece ->
        animateFloatAsState(
            targetValue = if (launched) 1f else 0f,
            animationSpec = tween(
                durationMillis = piece.duration,
                delayMillis = piece.delay,
                easing = EaseIn
            )
        )
    }

    Canvas(modifier = Modifier.fillMaxSize()) {
        val w = size.width
        val h = size.height

        pieces.forEachIndexed { i, piece ->
            val t = progress[i].value
            if (t <= 0f) return@forEachIndexed

            val fadeAlpha = if (t > 0.7f) (1f - t) / 0.3f else 1f
            if (fadeAlpha <= 0f) return@forEachIndexed

            val px = piece.x * w + piece.drift * t
            val py = -20f + (h + 60f) * t
            val rotation = piece.initialRotation + 720f * t

            rotate(degrees = rotation, pivot = Offset(px, py)) {
                drawRoundRect(
                    color = piece.color.copy(alpha = 0.9f * fadeAlpha),
                    topLeft = Offset(px - piece.width / 2f, py - piece.height / 2f),
                    size = Size(piece.width, piece.height),
                    cornerRadius = CornerRadius(piece.width * 0.2f)
                )
            }
        }
    }
}
