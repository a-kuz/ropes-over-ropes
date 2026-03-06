package com.uzls.four.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Slider
import androidx.compose.material3.SliderDefaults
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.flow.MutableStateFlow

private val TABS = listOf("Physics", "Visual", "Material", "Light", "Table", "Cartoon", "Cap", "Sticky", "Player")

@Composable
fun SettingsPanel(viewModel: GameViewModel) {
    var selectedTab by remember { mutableIntStateOf(0) }

    var copySuccess by remember { mutableStateOf(false) }
    var importResult by remember { mutableStateOf<Boolean?>(null) }

    LaunchedEffect(copySuccess) {
        if (copySuccess) {
            kotlinx.coroutines.delay(1500)
            copySuccess = false
        }
    }

    LaunchedEffect(importResult) {
        if (importResult != null) {
            kotlinx.coroutines.delay(1500)
            importResult = null
        }
    }

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .heightIn(max = 340.dp)
            .clip(RoundedCornerShape(bottomStart = 16.dp, bottomEnd = 16.dp))
            .background(Color.Black.copy(alpha = 0.85f))
            .padding(12.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(
                modifier = Modifier
                    .weight(1f)
                    .horizontalScroll(rememberScrollState()),
                horizontalArrangement = Arrangement.spacedBy(6.dp)
            ) {
                TABS.forEachIndexed { index, title ->
                    val sel = selectedTab == index
                    Box(
                        modifier = Modifier
                            .clip(RoundedCornerShape(8.dp))
                            .background(if (sel) Color.White.copy(0.15f) else Color.Transparent)
                            .clickable { selectedTab = index }
                            .padding(horizontal = 12.dp, vertical = 6.dp)
                    ) {
                        Text(title, color = if (sel) Color.White else Color.White.copy(0.5f), fontSize = 13.sp)
                    }
                }
            }
            Spacer(modifier = Modifier.width(8.dp))
            val copyColor = if (copySuccess) Color(0xFF4CAF50) else Color.White.copy(0.6f)
            Box(
                modifier = Modifier
                    .size(30.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color.White.copy(0.12f))
                    .clickable { copySuccess = viewModel.dumpSettingsToClipboard() },
                contentAlignment = Alignment.Center
            ) {
                Text("⧉", fontSize = 14.sp, color = copyColor)
            }
            Spacer(modifier = Modifier.width(6.dp))
            val pasteColor = when (importResult) {
                true -> Color(0xFF4CAF50)
                false -> Color(0xFFF44336)
                null -> Color.White.copy(0.6f)
            }
            Box(
                modifier = Modifier
                    .size(30.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color.White.copy(0.12f))
                    .clickable { importResult = viewModel.importSettingsFromClipboard() },
                contentAlignment = Alignment.Center
            ) {
                Text("\uD83D\uDCCB", fontSize = 14.sp, color = pasteColor)
            }
        }

        Spacer(modifier = Modifier.height(8.dp))

        Column(modifier = Modifier.verticalScroll(rememberScrollState())) {
            when (selectedTab) {
                0 -> PhysicsTab(viewModel)
                1 -> VisualTab(viewModel)
                2 -> MaterialTab(viewModel)
                3 -> LightTab(viewModel)
                4 -> TableTab(viewModel)
                5 -> CartoonTab(viewModel)
                6 -> CapTab(viewModel)
                7 -> StickyTab(viewModel)
                8 -> PlayerTab(viewModel)
            }
        }
    }
}

@Composable
private fun PhysicsTab(vm: GameViewModel) {
    ParamSlider("Particles", vm.particleCount, 6f, 200f, "p.ptc", vm)
    ParamSlider("Gravity", vm.gravity, -30f, 0f, "p.grav", vm)
    ParamSlider("Damping", vm.damping, 0.5f, 1f, "p.damp", vm)
    ParamSlider("Tension", vm.ropeTension, 0f, 1f, "p.tens", vm)
    ParamSlider("Friction", vm.frictionCoefficient, 0f, 2f, "p.fric", vm)
    ParamSlider("Iterations", vm.constraintIterations, 1f, 60f, "p.iter", vm)
    ParamSlider("Max Substeps", vm.maxSubsteps, 1f, 12f, "p.maxsub", vm)
    ParamSlider("Physics Rate", vm.physicsRate, 30f, 240f, "p.rate", vm)
    ParamSlider("Settle Steps", vm.settleSteps, 1f, 100f, "p.settle", vm)
    ParamSlider("Bend Compl", vm.bendCompliance, 0f, 0.01f, "p.bend", vm)
    ParamSlider("Bend Damp", vm.bendVelocityCoupling, 0f, 1f, "p.bvel", vm)
    ParamSlider("Drag Height", vm.dragHeight, 0.05f, 1.5f, "p.dragH", vm)
    ParamSlider("Lift Height", vm.liftHeight, 0.05f, 1.5f, "p.lift", vm)
    ParamSlider("Board Elev", vm.boardElevation, 0.02f, 0.5f, "p.bElev", vm)
}

@Composable
private fun VisualTab(vm: GameViewModel) {
    BoolToggle("Square Profile", vm.squareCrossSection, "v.sqcs", vm)
    ParamSlider("Profile Segs", vm.profileSegments, 3f, 32f, "v.prof", vm)
    ParamSlider("Hole Scale", vm.holeRadiusScale, 0.5f, 2f, "v.hrscale", vm)
    ParamSlider("Rope Scale", vm.ropeRadiusScale, 0.5f, 2f, "v.rscale", vm)
    ParamSlider("Stretch Thin", vm.stretchThinning, 0f, 1f, "v.stretch", vm)
    ParamSlider("Hole Tint R", vm.holeTintR, 0f, 1f, "v.htR", vm)
    ParamSlider("Hole Tint G", vm.holeTintG, 0f, 1f, "v.htG", vm)
    ParamSlider("Hole Tint B", vm.holeTintB, 0f, 1f, "v.htB", vm)
    ParamSlider("Tint Amount", vm.holeTintAmount, 0f, 1f, "v.htAmt", vm)
    ParamSlider("Exposure", vm.exposure, 0.1f, 2.5f, "v.exp", vm)
    ParamSlider("Bloom", vm.bloomStrength, 0f, 2f, "v.bloom", vm)
}

@Composable
private fun MaterialTab(vm: GameViewModel) {
    ParamSlider("Matte", vm.ropeMatte, 0f, 1f, "m.matte", vm)
    ParamSlider("Gloss", vm.ropeGloss, 0f, 2f, "m.gloss", vm)
    ParamSlider("Diff Wrap", vm.ropeDiffuseWrap, 0f, 1f, "m.dwrap", vm)
    ParamSlider("Subsurface", vm.ropeSubsurface, 0f, 1f, "m.ssurf", vm)
    ParamSlider("Edge Light", vm.ropeEdgeLight, 0f, 0.5f, "m.edge", vm)
    ParamSlider("Saturation", vm.ropeSaturation, 0f, 2f, "m.sat", vm)
    ParamSlider("Micro Bump", vm.ropeMicroBump, 0f, 1.5f, "m.bump", vm)
    ParamSlider("Bump Scale", vm.ropeBumpScale, 0.5f, 20f, "m.bscale", vm)
    ParamSlider("Contact AO", vm.ropeContactAO, 0f, 1f, "m.cao", vm)
    ParamSlider("Lift Glow", vm.ropeLiftGlow, 0f, 1f, "m.glow", vm)
    ParamSlider("Str Gloss", vm.ropeStretchGloss, 0f, 1f, "m.sgloss", vm)
    ParamSlider("Str Spec", vm.ropeStretchSpec, 0f, 2f, "m.sspec", vm)
    ParamSlider("Reflection", vm.ropeEnvReflect, 0f, 3f, "m.envref", vm)
}

@Composable
private fun LightTab(vm: GameViewModel) {
    ParamSlider("Intensity", vm.lightIntensity, 0.1f, 3f, "l.int", vm)
    ParamSlider("Dir X", vm.lightDirX, -1f, 1f, "l.dx", vm)
    ParamSlider("Dir Y", vm.lightDirY, -1f, 1f, "l.dy", vm)
    ParamSlider("Dir Z", vm.lightDirZ, -1f, 1f, "l.dz", vm)
    ParamSlider("Ambient", vm.ambient, 0f, 0.6f, "l.amb", vm)
    ParamSlider("Shadow Bias", vm.shadowBias, 0.0001f, 0.005f, "l.sbias", vm)
    ParamSlider("Shadow Dark", vm.shadowDarkness, 0f, 0.5f, "l.sdark", vm)
    ParamSlider("Light Size", vm.shadowSize, 0.002f, 0.08f, "l.ssize", vm)
}

private val TABLE_STYLES = listOf("Wood", "Gradient", "Solid")

@Composable
private fun TableTab(vm: GameViewModel) {
    val current = vm.tableStyle.collectAsState().value.toInt()
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier.fillMaxWidth().padding(vertical = 2.dp)
    ) {
        Text("Style", color = Color.White.copy(0.7f), fontSize = 12.sp, modifier = Modifier.width(70.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            TABLE_STYLES.forEachIndexed { index, label ->
                val sel = current == index
                Box(
                    modifier = Modifier
                        .clip(RoundedCornerShape(6.dp))
                        .background(if (sel) Color.White.copy(0.25f) else Color.White.copy(0.08f))
                        .clickable {
                            vm.tableStyle.value = index.toFloat()
                            vm.saveParam("v.tst", index.toFloat())
                            vm.syncParamsToRenderer()
                        }
                        .padding(horizontal = 10.dp, vertical = 4.dp)
                ) {
                    Text(label, color = if (sel) Color.White else Color.White.copy(0.5f), fontSize = 12.sp)
                }
            }
        }
    }
    if (current != 0) {
        ParamSlider("Color1 R", vm.tableColor1R, 0f, 1f, "v.tc1r", vm)
        ParamSlider("Color1 G", vm.tableColor1G, 0f, 1f, "v.tc1g", vm)
        ParamSlider("Color1 B", vm.tableColor1B, 0f, 1f, "v.tc1b", vm)
        if (current == 1) {
            ParamSlider("Color2 R", vm.tableColor2R, 0f, 1f, "v.tc2r", vm)
            ParamSlider("Color2 G", vm.tableColor2G, 0f, 1f, "v.tc2g", vm)
            ParamSlider("Color2 B", vm.tableColor2B, 0f, 1f, "v.tc2b", vm)
        }
    }
}

@Composable
private fun CartoonTab(vm: GameViewModel) {
    ParamSlider("Enabled", vm.cartoonMode, 0f, 1f, "c.mode", vm)
    ParamSlider("Exposure", vm.cartoonExposure, 0.5f, 1.5f, "c.exp", vm)
    ParamSlider("Edge", vm.cartoonEdgeStrength, 0f, 1f, "c.edge", vm)
    ParamSlider("Levels", vm.cartoonLevels, 2f, 6f, "c.levels", vm)
    ParamSlider("Shadow", vm.cartoonShadowBright, 0.1f, 0.8f, "c.shadow", vm)
    ParamSlider("Wrap", vm.cartoonWrap, 0f, 0.5f, "c.wrap", vm)
    ParamSlider("Edge Smooth", vm.cartoonEdgeSmooth, 0f, 1f, "c.esmooth", vm)
}

@Composable
private fun CapTab(vm: GameViewModel) {
    ParamSlider("Radius Scale", vm.capRadiusScale, 0.3f, 2.5f, "cap.rscale", vm)
    ParamSlider("Segments", vm.capSegments, 4f, 48f, "cap.segs", vm)
    ParamSlider("Rings", vm.capRings, 2f, 16f, "cap.rings", vm)
    ParamSlider("Darken", vm.capDarken, 0f, 1f, "cap.dark", vm)
}

@Composable
private fun StickyTab(vm: GameViewModel) {
    BoolToggle("Enabled", vm.stickyEnabled, "s.enabled", vm)
    ParamSlider("Strength", vm.stickyStrength, 0f, 2f, "s.str", vm)
    ParamSlider("Radius", vm.stickyRadius, 0.5f, 4f, "s.rad", vm)
    ParamSlider("Damping", vm.stickyDamping, 0f, 1f, "s.damp", vm)
    ParamSlider("Break Thr", vm.stickyBreakThreshold, 0.1f, 3f, "s.break", vm)
}

// ===== Reusable controls =====

@Composable
private fun BoolToggle(
    label: String,
    flow: MutableStateFlow<Boolean>,
    prefKey: String,
    viewModel: GameViewModel
) {
    var checked by remember { mutableStateOf(flow.value) }
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier.fillMaxWidth().padding(vertical = 2.dp)
    ) {
        Text(label, color = Color.White.copy(0.7f), fontSize = 12.sp, modifier = Modifier.weight(1f))
        Switch(
            checked = checked,
            onCheckedChange = {
                checked = it
                flow.value = it
                viewModel.saveParam(prefKey, if (it) 1f else 0f)
                viewModel.syncParamsToRenderer()
            },
            colors = SwitchDefaults.colors(
                checkedThumbColor = Color.White,
                checkedTrackColor = Color.White.copy(0.5f),
                uncheckedThumbColor = Color.White.copy(0.5f),
                uncheckedTrackColor = Color.White.copy(0.15f)
            )
        )
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
        modifier = Modifier.fillMaxWidth().padding(vertical = 2.dp)
    ) {
        Text(label, color = Color.White.copy(0.7f), fontSize = 12.sp, modifier = Modifier.width(70.dp))
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

@Composable
private fun PlayerTab(viewModel: GameViewModel) {
    var nameInput by remember { mutableStateOf(viewModel.leaderboard.username) }
    var loginInput by remember { mutableStateOf("") }

    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Name", color = Color.White.copy(0.8f), fontSize = 12.sp, modifier = Modifier.width(50.dp))
            BasicTextField(
                value = nameInput,
                onValueChange = { nameInput = it },
                textStyle = TextStyle(color = Color.White, fontSize = 13.sp),
                cursorBrush = SolidColor(Color.White),
                modifier = Modifier
                    .weight(1f)
                    .background(Color.White.copy(0.1f), RoundedCornerShape(6.dp))
                    .padding(horizontal = 8.dp, vertical = 6.dp)
            )
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(Color.White.copy(0.2f))
                    .clickable {
                        val name = nameInput.trim()
                        if (name.isNotEmpty()) { viewModel.leaderboard.username = name }
                    }
                    .padding(horizontal = 10.dp, vertical = 5.dp)
            ) {
                Text("Save", color = Color.White, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
            }
        }

        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("Login", color = Color.White.copy(0.8f), fontSize = 12.sp, modifier = Modifier.width(50.dp))
            BasicTextField(
                value = loginInput,
                onValueChange = { loginInput = it },
                textStyle = TextStyle(color = Color.White, fontSize = 13.sp),
                cursorBrush = SolidColor(Color.White),
                modifier = Modifier
                    .weight(1f)
                    .background(Color.White.copy(0.1f), RoundedCornerShape(6.dp))
                    .padding(horizontal = 8.dp, vertical = 6.dp)
            )
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(6.dp))
                    .background(Color.White.copy(0.2f))
                    .clickable {
                        val name = loginInput.trim()
                        if (name.isNotEmpty()) {
                            viewModel.leaderboard.loginAs(name)
                            nameInput = name
                            loginInput = ""
                        }
                    }
                    .padding(horizontal = 10.dp, vertical = 5.dp)
            ) {
                Text("Go", color = Color.White, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
            }
        }

        if (viewModel.leaderboard.username.isNotEmpty()) {
            Text(
                "Playing as: ${viewModel.leaderboard.username}",
                color = Color.White.copy(0.5f),
                fontSize = 12.sp
            )
        }
    }
}
