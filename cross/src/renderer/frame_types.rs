use bytemuck::{Pod, Zeroable};

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
    pub rope_mat_params: [f32; 4],
    pub rope_mat_params2: [f32; 4],
    pub rope_mat_params3: [f32; 4],
    pub lighting_params: [f32; 4],
}

#[derive(Clone)]
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
}

impl Default for RopeMaterialSettings {
    fn default() -> Self {
        Self {
            matte: 0.69,
            gloss: 1.17,
            diffuse_wrap: 0.045,
            subsurface: 0.007,
            edge_light: 0.009,
            saturation: 1.36,
            micro_bump: 0.6,
            bump_scale: 7.37,
            contact_ao: 0.649,
            lift_glow: 0.0,
            stretch_gloss: 0.879,
            stretch_spec: 0.371,
        }
    }
}

#[derive(Clone)]
pub struct LightingSettings {
    pub ambient: f32,
    pub shadow_darkness: f32,
    pub light_intensity: f32,
    pub exposure: f32,
    pub shadow_bias: f32,
    pub light_dir: [f32; 3],
    pub shadow_type: u8,
    pub rope_radius_scale: f32,
}

impl Default for LightingSettings {
    fn default() -> Self {
        Self {
            ambient: 0.064,
            shadow_darkness: 0.0,
            light_intensity: 1.24,
            exposure: 0.735,
            shadow_bias: 0.0001,
            light_dir: [-0.131, -0.156, 0.125],
            shadow_type: 2,
            rope_radius_scale: 0.871,
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
pub struct ShadowSegment {
    pub ax: f32, pub ay: f32, pub az: f32, pub radius: f32,
    pub bx: f32, pub by: f32, pub bz: f32, pub rope_id: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
pub struct ShadowSegmentCount {
    pub count: u32,
    pub _pad: [u32; 3],
}
