package com.uzls.four.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Slider
import androidx.compose.material3.SliderDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.flow.MutableStateFlow

@Composable
fun SettingsPanel(viewModel: GameViewModel) {
    var selectedTab by remember { mutableIntStateOf(0) }
    val tabs = listOf("Rope", "Solver", "Drag", "Visual", "Light")

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .heightIn(max = 320.dp)
            .clip(RoundedCornerShape(bottomStart = 16.dp, bottomEnd = 16.dp))
            .background(Color.Black.copy(alpha = 0.85f))
            .padding(12.dp)
    ) {
        // Tab bar
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            tabs.forEachIndexed { index, title ->
                val selected = selectedTab == index
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(8.dp))
                        .background(if (selected) Color.White.copy(0.15f) else Color.Transparent)
                        .clickable { selectedTab = index }
                        .padding(horizontal = 12.dp, vertical = 6.dp)
                ) {
                    Text(title, color = if (selected) Color.White else Color.White.copy(0.5f), fontSize = 13.sp)
                }
            }
        }

        Spacer(modifier = Modifier.height(8.dp))

        Column(modifier = Modifier.verticalScroll(rememberScrollState())) {
            when (selectedTab) {
                0 -> { // Rope
                    ParamSlider("Particles", viewModel.particleCount, 6f, 200f, "p.ptc", viewModel)
                    ParamSlider("Gravity", viewModel.gravity, -30f, 0f, "p.grav", viewModel)
                    ParamSlider("Damping", viewModel.damping, 0.5f, 1f, "p.damp", viewModel)
                    ParamSlider("Tension", viewModel.ropeTension, 0f, 1f, "p.tens", viewModel)
                }
                1 -> { // Solver
                    ParamSlider("Iterations", viewModel.constraintIterations, 1f, 20f, "p.iter", viewModel)
                    ParamSlider("Settle Steps", viewModel.settleSteps, 1f, 100f, "p.settle", viewModel)
                    ParamSlider("Bend Compliance", viewModel.bendCompliance, 0f, 0.1f, "p.bend", viewModel)
                    ParamSlider("Bend Velocity", viewModel.bendVelocityCoupling, 0f, 1f, "p.bvel", viewModel)
                }
                2 -> { // Drag
                    ParamSlider("Lift Height", viewModel.liftHeight, 0f, 1f, "p.lift", viewModel)
                    ParamSlider("Board Elevation", viewModel.boardElevation, 0f, 0.5f, "p.bElev", viewModel)
                }
                3 -> { // Visual
                    ParamSlider("Exposure", viewModel.exposure, 0.1f, 2f, "v.exp", viewModel)
                    ParamSlider("Bloom", viewModel.bloomStrength, 0f, 1f, "v.bloom", viewModel)
                    ParamSlider("Profile Segs", viewModel.profileSegments, 4f, 32f, "v.prof", viewModel)
                    ParamSlider("Rope Scale", viewModel.ropeRadiusScale, 0.2f, 3f, "v.rscale", viewModel)
                    ParamSlider("Stretch Thin", viewModel.stretchThinning, 0f, 2f, "v.stretch", viewModel)
                }
                4 -> { // Light
                    ParamSlider("Dir X", viewModel.lightDirX, -1f, 1f, "l.dx", viewModel)
                    ParamSlider("Dir Y", viewModel.lightDirY, -1f, 1f, "l.dy", viewModel)
                    ParamSlider("Dir Z", viewModel.lightDirZ, -1f, 0f, "l.dz", viewModel)
                    ParamSlider("Intensity", viewModel.lightIntensity, 0f, 5f, "l.int", viewModel)
                    ParamSlider("Ambient", viewModel.ambient, 0f, 1f, "l.amb", viewModel)
                }
            }
        }
    }
}

@Composable
private fun ParamSlider(
    label: String,
    flow: MutableStateFlow<Float>,
    min: Float, max: Float,
    prefKey: String,
    viewModel: GameViewModel
) {
    var value by remember { mutableFloatStateOf(flow.value) }

    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 2.dp)
    ) {
        Text(
            label,
            color = Color.White.copy(0.7f),
            fontSize = 12.sp,
            modifier = Modifier.width(70.dp)
        )
        Slider(
            value = value,
            onValueChange = {
                value = it
                flow.value = it
                viewModel.saveParam(prefKey, it)
                viewModel.syncParamsToRenderer()
            },
            valueRange = min..max,
            modifier = Modifier.weight(1f),
            colors = SliderDefaults.colors(
                thumbColor = Color.White,
                activeTrackColor = Color.White.copy(0.5f),
                inactiveTrackColor = Color.White.copy(0.15f)
            )
        )
        Text(
            String.format("%.2f", value),
            color = Color.White.copy(0.5f),
            fontSize = 11.sp,
            fontWeight = FontWeight.Light,
            modifier = Modifier.width(48.dp)
        )
    }
}
