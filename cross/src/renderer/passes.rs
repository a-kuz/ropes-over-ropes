use super::camera::{look_at, ortho, Camera};
use super::frame_types::{
    CartoonSettings, FrameUniforms, LightingSettings, RopeMaterialSettings, TableSettings,
    VisualSettings, WormSettings,
};
use glam::{Mat4, Vec3};

fn hash01(x: f32) -> f32 {
    let v = (x.sin() * 43_758.547).fract();
    if v < 0.0 {
        v + 1.0
    } else {
        v
    }
}

pub fn build_frame_uniforms(
    camera: &Camera,
    aspect: f32,
    shadow_map_size: u32,
    time: f32,
    drag_active: bool,
    victory_time: f32,
    level_seed: f32,
    highlight_hole: i32,
    render_mode: u32,
    cel_mode: bool,
    hole_mask_bounds: [f32; 4],
    table_shadow_mode: u8,
    rope_mat: &RopeMaterialSettings,
    lighting: &LightingSettings,
    visual: &VisualSettings,
    table: &TableSettings,
    cartoon: &CartoonSettings,
    worm: &WormSettings,
) -> FrameUniforms {
    let view_proj = camera.view_proj(aspect);
    let inv_view_proj = view_proj.inverse();

    let ld = lighting.light_dir;
    let ld_vec = Vec3::new(ld[0], ld[1], ld[2]);
    let light_dir = if ld_vec.length_squared() < 1e-6 {
        let azimuth = hash01(level_seed * 13.17 + 1.91) * std::f32::consts::TAU;
        let elevation = 0.18 + hash01(level_seed * 7.73 + 9.41) * 0.92;
        Vec3::new(
            azimuth.cos() * elevation.cos(),
            azimuth.sin() * elevation.cos(),
            elevation.sin(),
        )
        .normalize()
    } else {
        ld_vec.normalize()
    };

    let half_h = camera.ortho_half_height;
    let half_w = half_h * aspect;
    let light_view_proj = make_light_view_proj(light_dir, camera, aspect);
    let inv_shadow = 1.0 / shadow_map_size.max(1) as f32;
    let eye = camera.eye_position();
    let use_cartoon = visual.cartoon_mode > 0.5;
    let eff_exposure = if use_cartoon {
        cartoon.exposure
    } else {
        visual.exposure
    };
    let eff_bloom = if use_cartoon {
        0.0
    } else {
        visual.bloom_strength
    };

    FrameUniforms {
        view_proj: view_proj.to_cols_array_2d(),
        inv_view_proj: inv_view_proj.to_cols_array_2d(),
        light_view_proj: light_view_proj.to_cols_array_2d(),
        light_dir_intensity: [
            light_dir.x,
            light_dir.y,
            light_dir.z,
            lighting.light_intensity,
        ],
        ambient_color: [table_shadow_mode as f32, 0.0, 0.0, highlight_hole as f32],
        camera_pos: [eye.x, eye.y, eye.z, 1.0],
        ortho_half_size_shadow_bias: [
            half_w,
            half_h,
            lighting.shadow_bias,
            lighting.shadow_type as f32,
        ],
        shadow_inv_size_unused: [inv_shadow, inv_shadow, camera.center.x, camera.center.y],
        time_drag: [
            time,
            victory_time,
            level_seed,
            if drag_active { 1.0 } else { 0.0 },
        ],
        hole_mask_bounds,
        wood_bounds_min: [-5.0, -5.0, table.wood_brightness, table.wood_pattern_scale],
        wood_bounds_max: [5.0, 5.0, 0.0, 0.0],
        hole_tint: visual.hole_tint,
        visual_params: [
            eff_exposure,
            eff_bloom,
            visual.cartoon_mode,
            visual.cartoon_levels,
        ],
        lighting_params: [
            lighting.ambient,
            lighting.shadow_darkness,
            lighting.shadow_size,
            if lighting.shadows_enabled { 1.0 } else { 0.0 },
        ],
        table_params: [
            table.style as f32,
            table.color1[0],
            table.color1[1],
            table.color1[2],
        ],
        table_params2: [
            table.color2[0],
            table.color2[1],
            table.color2[2],
            if visual.square_cross_section {
                1.0
            } else {
                0.0
            },
        ],
        rope_mat_params: [
            rope_mat.matte,
            rope_mat.gloss,
            rope_mat.diffuse_wrap,
            rope_mat.subsurface,
        ],
        rope_mat_params2: [
            rope_mat.edge_light,
            rope_mat.saturation,
            rope_mat.micro_bump,
            rope_mat.contact_ao,
        ],
        rope_mat_params3: [
            rope_mat.lift_glow,
            rope_mat.bump_scale,
            rope_mat.stretch_gloss,
            rope_mat.stretch_spec,
        ],
        cartoon_params: [
            cartoon.shadow_bright,
            cartoon.wrap,
            cartoon.edge_smooth,
            cartoon.edge_strength,
        ],
        worm_params1: worm.params1,
        worm_params2: worm.params2,
        worm_params3: worm.params3,
        worm_params4: worm.params4,
        rope_mat_params4: [rope_mat.env_reflect, 0.0, 0.0, 0.0],
    }
}

fn make_light_view_proj(light_dir: Vec3, camera: &Camera, aspect: f32) -> Mat4 {
    let eye = camera.center + light_dir * 4.9;
    let mut up = Vec3::new(0.0, 1.0, 0.0);
    if up.dot(light_dir).abs() > 0.95 {
        up = Vec3::new(1.0, 0.0, 0.0);
    }
    let light_view = look_at(eye, camera.center, up);
    let half_h = camera.ortho_half_height;
    let half_w = half_h * aspect;
    let proj = ortho(
        -half_w * 1.08,
        half_w * 1.08,
        -half_h * 1.08,
        half_h * 1.08,
        0.01,
        12.0,
    );
    proj * light_view
}
