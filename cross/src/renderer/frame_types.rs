use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct FrameUniforms {
    pub view_proj: [[f32; 4]; 4],
    pub inv_view_proj: [[f32; 4]; 4],
    pub light_view_proj: [[f32; 4]; 4],
    pub light_dir_intensity: [f32; 4],
    pub ambient_color: [f32; 4],
    pub camera_pos: [f32; 4],
    pub ortho_half_size_shadow_bias: [f32; 4],
    pub shadow_inv_size_unused: [f32; 4],
    pub time_drag: [f32; 4],
    pub hole_mask_bounds: [f32; 4],
    pub wood_bounds_min: [f32; 4],
    pub wood_bounds_max: [f32; 4],
    pub hole_tint: [f32; 4],
    pub visual_params: [f32; 4],
    pub lighting_params: [f32; 4],
    pub table_params: [f32; 4],
    pub table_params2: [f32; 4],
    pub rope_mat_params: [f32; 4],
    pub rope_mat_params2: [f32; 4],
    pub rope_mat_params3: [f32; 4],
    pub cartoon_params: [f32; 4],
    pub worm_params1: [f32; 4],
    pub worm_params2: [f32; 4],
    pub worm_params3: [f32; 4],
    pub worm_params4: [f32; 4],
    pub rope_mat_params4: [f32; 4],
    pub ssr_params: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct PostParams {
    pub exposure: f32,
    pub bloom_strength: f32,
    pub cartoon_edge_strength: f32,
    pub cartoon_mode: f32,
    pub cartoon_edge_smooth: f32,
    pub _pad0: f32,
    pub _pad1: f32,
    pub _pad2: f32,
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct RopeMaterialSettings {
    pub matte: f32,
    pub gloss: f32,
    pub diffuse_wrap: f32,
    pub subsurface: f32,
    pub edge_light: f32,
    pub saturation: f32,
    pub micro_bump: f32,
    pub bump_scale: f32,
    pub contact_ao: f32,
    pub lift_glow: f32,
    pub stretch_gloss: f32,
    pub stretch_spec: f32,
    pub env_reflect: f32,
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct SsrSettings {
    pub strength: f32,
    pub max_steps: u32,
    pub step_size: f32,
    pub thickness: f32,
}

impl Default for SsrSettings {
    fn default() -> Self {
        Self {
            strength: 0.0,
            max_steps: 32,
            step_size: 0.015,
            thickness: 0.06,
        }
    }
}

impl Default for RopeMaterialSettings {
    fn default() -> Self {
        Self {
            matte: 1.0,
            gloss: 0.0,
            diffuse_wrap: 0.17898679,
            subsurface: 0.0,
            edge_light: 0.046988227,
            saturation: 0.5346545,
            micro_bump: 1.2239329,
            bump_scale: 3.2977462,
            contact_ao: 0.81001943,
            lift_glow: 0.0,
            stretch_gloss: 0.10844656,
            stretch_spec: 0.9625906,
            env_reflect: 0.0,
        }
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct LightingSettings {
    pub ambient: f32,
    pub shadow_darkness: f32,
    pub light_intensity: f32,
    pub shadow_bias: f32,
    pub shadow_size: f32,
    pub shadows_enabled: bool,
    pub light_dir: [f32; 3],
    pub shadow_type: u8,
    pub rope_radius_scale: f32,
}

impl Default for LightingSettings {
    fn default() -> Self {
        Self {
            ambient: 0.06113583,
            shadow_darkness: 0.0,
            light_intensity: 0.5905325,
            shadow_bias: 0.0001,
            shadow_size: 0.02985908,
            shadows_enabled: true,
            light_dir: [-0.12931377, -0.22752565, 1.0],
            shadow_type: 2,
            rope_radius_scale: 0.9669485,
        }
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct VisualSettings {
    pub profile_segments: usize,
    pub hole_radius_scale: f32,
    pub stretch_thinning: f32,
    pub exposure: f32,
    pub bloom_strength: f32,
    pub hole_tint: [f32; 4],
    pub cartoon_mode: f32,
    pub cartoon_levels: f32,
    pub square_cross_section: bool,
    pub wave_energy: f32,
}

impl Default for VisualSettings {
    fn default() -> Self {
        Self {
            profile_segments: 4,
            hole_radius_scale: 0.84779215,
            stretch_thinning: 0.093161635,
            exposure: 0.61766064,
            bloom_strength: 1.8433682,
            hole_tint: [1.0, 0.89, 1.0, 0.4529673],
            cartoon_mode: 0.0,
            cartoon_levels: 4.0,
            square_cross_section: false,
            wave_energy: 0.0,
        }
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct TableSettings {
    pub style: u32,
    pub color1: [f32; 3],
    pub color2: [f32; 3],
    pub wood_seed: f32,
    pub wood_brightness: f32,
    pub wood_pattern_scale: f32,
}

impl Default for TableSettings {
    fn default() -> Self {
        Self {
            style: 0,
            color1: [0.08, 0.09, 0.13],
            color2: [0.12, 0.13, 0.20],
            wood_seed: 0.0,
            wood_brightness: 0.5,
            wood_pattern_scale: 1.0,
        }
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct CartoonSettings {
    pub exposure: f32,
    pub edge_strength: f32,
    pub shadow_bright: f32,
    pub wrap: f32,
    pub edge_smooth: f32,
}

impl Default for CartoonSettings {
    fn default() -> Self {
        Self {
            exposure: 1.33,
            edge_strength: 1.0,
            shadow_bright: 0.38,
            wrap: 0.15,
            edge_smooth: 0.5,
        }
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct CapSettings {
    pub radius_scale: f32,
    pub segments: usize,
    pub rings: usize,
    pub darken: f32,
    pub smin_k: f32,
}

impl Default for CapSettings {
    fn default() -> Self {
        Self {
            radius_scale: 1.0,
            segments: 25,
            rings: 6,
            darken: 0.0,
            smin_k: 0.3,
        }
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct WormSettings {
    pub params1: [f32; 4],
    pub params2: [f32; 4],
    pub params3: [f32; 4],
    pub params4: [f32; 4],
}

impl Default for WormSettings {
    fn default() -> Self {
        Self {
            params1: [0.0; 4],
            params2: [0.0; 4],
            params3: [0.0; 4],
            params4: [0.0; 4],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct HoleInstance {
    pub position_radius: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct RopeVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 3],
    pub tex_coord: [f32; 2],
    pub params: [f32; 4],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct HoleVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct BoardVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub world_xy: [f32; 2],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct ShadowSegment {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub radius: f32,
    pub bx: f32,
    pub by: f32,
    pub bz: f32,
    pub rope_id: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct ShadowSegmentCount {
    pub count: u32,
    pub _pad: [u32; 3],
}
