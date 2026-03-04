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
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.sin
import kotlin.random.Random

@Composable
fun VictoryOverlay(level: Int, onNextLevel: () -> Unit) {
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

    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Color.Black.copy(alpha = 0.4f * alpha)),
        contentAlignment = Alignment.Center
    ) {
        // Confetti
        ConfettiAnimation()

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
            Spacer(modifier = Modifier.height(32.dp))

            // Next level button
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(16.dp))
                    .background(
                        Brush.horizontalGradient(
                            listOf(Color(0xFF4A6CF7), Color(0xFF9B5DE5))
                        )
                    )
                    .clickable { onNextLevel() }
                    .padding(horizontal = 32.dp, vertical = 14.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "Level ${level + 1}",
                    color = Color.White,
                    fontSize = 18.sp,
                    fontWeight = FontWeight.SemiBold
                )
            }
        }
    }
}

@Composable
private fun ConfettiAnimation() {
    val confettiColors = remember {
        listOf(
            Color(0xFFFF6B6B), Color(0xFF4ECDC4), Color(0xFFFFE66D),
            Color(0xFF95E1D3), Color(0xFFF38181), Color(0xFF6C5CE7),
            Color(0xFF00B894), Color(0xFFE17055), Color(0xFF0984E3)
        )
    }

    data class Confetto(
        val x: Float, var y: Float,
        val color: Color, val size: Float,
        val speedX: Float, val speedY: Float,
        val rotation: Float
    )

    val confetti = remember {
        List(40) {
            Confetto(
                x = Random.nextFloat(),
                y = -Random.nextFloat() * 0.5f,
                color = confettiColors.random(),
                size = Random.nextFloat() * 8f + 4f,
                speedX = (Random.nextFloat() - 0.5f) * 0.003f,
                speedY = Random.nextFloat() * 0.005f + 0.003f,
                rotation = Random.nextFloat() * 360f
            )
        }
    }

    val time by rememberInfiniteTransition().animateFloat(
        initialValue = 0f,
        targetValue = 1000f,
        animationSpec = infiniteRepeatable(tween(100000, easing = LinearEasing))
    )

    Canvas(modifier = Modifier.fillMaxSize()) {
        for (c in confetti) {
            val x = (c.x + c.speedX * time + sin(time * 0.02f + c.rotation) * 0.02f) % 1f
            val y = (c.y + c.speedY * time) % 1.5f
            if (y < 0f || y > 1f) continue
            drawCircle(
                color = c.color.copy(alpha = 0.8f),
                radius = c.size,
                center = Offset(x * size.width, y * size.height)
            )
        }
    }
}
