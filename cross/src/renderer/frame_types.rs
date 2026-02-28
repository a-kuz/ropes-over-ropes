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
