use glam::{Vec2, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct HolePos {
    #[serde(rename = "x")]
    pub x_position: f32,
    #[serde(rename = "y")]
    pub y_position: f32,
}

impl HolePos {
    pub fn to_vec2(&self) -> Vec2 {
        Vec2::new(self.x_position, self.y_position)
    }
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct Color {
    #[serde(rename = "r")]
    pub red_channel: f32,
    #[serde(rename = "g")]
    pub green_channel: f32,
    #[serde(rename = "b")]
    pub blue_channel: f32,
}

impl Color {
    pub fn to_vec3(&self) -> Vec3 {
        Vec3::new(self.red_channel, self.green_channel, self.blue_channel)
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CrossSectionDef {
    #[serde(rename = "type")]
    pub kind: String,
    pub width: Option<f32>,
    pub height: Option<f32>,
}

impl CrossSectionDef {
    pub fn to_cross_section(&self, fallback_radius: f32) -> CrossSection {
        match self.kind.as_str() {
            "rectangular" => {
                let w = self.width.unwrap_or(fallback_radius * 2.0);
                let h = self.height.unwrap_or(fallback_radius * 0.7);
                CrossSection::Rectangular { width: w, height: h }
            }
            _ => CrossSection::Circular { radius: fallback_radius },
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Rope {
    #[serde(rename = "startHole")]
    pub start_hole: usize,
    #[serde(rename = "endHole")]
    pub end_hole: usize,
    pub color: Color,
    pub radius: f32,
    #[serde(rename = "crossSection")]
    pub cross_section_def: Option<CrossSectionDef>,
}

impl Rope {
    pub fn cross_section(&self) -> CrossSection {
        self.cross_section_def
            .as_ref()
            .map(|def| def.to_cross_section(self.radius))
            .unwrap_or(CrossSection::Circular { radius: self.radius })
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Action {
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(rename = "ropeIndex")]
    pub rope_index: usize,
    #[serde(rename = "endIndex")]
    pub end_index: usize,
    #[serde(rename = "holeIndex")]
    pub hole_index: usize,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct HookRopeRef {
    #[serde(rename = "fromType")]
    pub from_type: String,
    pub index: usize,
    #[serde(rename = "hookIndex")]
    pub hook_index: Option<usize>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Hook {
    #[serde(rename = "ropeA")]
    pub rope_a: HookRopeRef,
    #[serde(rename = "ropeB")]
    pub rope_b: HookRopeRef,
    #[serde(rename = "N")]
    pub n: i32,
    #[serde(rename = "ropeAStartIsOver")]
    pub rope_a_start_is_over: bool,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct LevelDefinition {
    pub id: u32,
    #[serde(rename = "holeRadius")]
    pub hole_radius: f32,
    #[serde(rename = "particlesPerRope")]
    pub particles_per_rope: usize,
    pub holes: Vec<HolePos>,
    pub ropes: Vec<Rope>,
    pub hooks: Option<Vec<Hook>>,
    pub actions: Option<Vec<Action>>,
}

impl LevelDefinition {
    pub fn hole_positions(&self) -> Vec<Vec2> {
        self.holes.iter().map(|h| h.to_vec2()).collect()
    }
}

#[derive(Debug, Clone, Copy)]
pub enum CrossSection {
    Circular { radius: f32 },
    Rectangular { width: f32, height: f32 },
}

impl CrossSection {
    pub fn collision_radius(&self) -> f32 {
        match self {
            CrossSection::Circular { radius } => *radius,
            CrossSection::Rectangular { width, height } => width.max(*height) * 0.5,
        }
    }

    pub fn half_width(&self) -> f32 {
        match self {
            CrossSection::Circular { radius } => *radius,
            CrossSection::Rectangular { width, .. } => *width * 0.5,
        }
    }

    pub fn half_height(&self) -> f32 {
        match self {
            CrossSection::Circular { radius } => *radius,
            CrossSection::Rectangular { height, .. } => *height * 0.5,
        }
    }

    pub fn is_rectangular(&self) -> bool {
        matches!(self, CrossSection::Rectangular { .. })
    }

    pub fn effective_radius(&self, normal: Vec3, d1: Vec3, d2: Vec3) -> f32 {
        match self {
            CrossSection::Circular { radius } => *radius,
            CrossSection::Rectangular { width, height } => {
                let hw = *width * 0.5;
                let hh = *height * 0.5;
                normal.dot(d1).abs() * hw + normal.dot(d2).abs() * hh
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MaterialFrame {
    pub tangent: Vec3,
    pub d1: Vec3,
    pub d2: Vec3,
}
