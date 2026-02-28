use glam::{Mat4, Vec3};
use super::camera::{Camera, ortho, look_at};
use super::frame_types::FrameUniforms;

fn hash01(x: f32) -> f32 {
    let v = (x.sin() * 43_758.547).fract();
    if v < 0.0 { v + 1.0 } else { v }
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
) -> FrameUniforms {
    let view_proj = camera.view_proj(aspect);
    let inv_view_proj = view_proj.inverse();
    let azimuth = hash01(level_seed * 13.17 + 1.91) * std::f32::consts::TAU;
    let elevation = 0.18 + hash01(level_seed * 7.73 + 9.41) * 0.92;
    let light_dir = Vec3::new(
        azimuth.cos() * elevation.cos(),
        azimuth.sin() * elevation.cos(),
        elevation.sin(),
    )
    .normalize();
    let half_h = camera.ortho_half_height;
    let half_w = half_h * aspect;
    let light_view_proj = make_light_view_proj(light_dir, camera, aspect);
    let inv_shadow = 1.0 / shadow_map_size.max(1) as f32;
    let eye = camera.eye_position();

    FrameUniforms {
        view_proj: view_proj.to_cols_array_2d(),
        inv_view_proj: inv_view_proj.to_cols_array_2d(),
        light_view_proj: light_view_proj.to_cols_array_2d(),
        light_dir_intensity: [light_dir.x, light_dir.y, light_dir.z, 5.2],
        ambient_color: [table_shadow_mode as f32, 0.0, 0.0, highlight_hole as f32],
        camera_pos: [eye.x, eye.y, eye.z, 1.0],
        ortho_half_size_shadow_bias: [half_w, half_h, 0.0012, 0.0],
        shadow_inv_size_unused: [inv_shadow, inv_shadow, if cel_mode { 1.0 } else { 0.0 }, render_mode as f32],
        time_drag: [time, victory_time, level_seed, if drag_active { 1.0 } else { 0.0 }],
        hole_mask_bounds,
    }
}

fn make_light_view_proj(light_dir: Vec3, camera: &Camera, aspect: f32) -> Mat4 {
    let eye = camera.center + light_dir * 6.0;
    let mut up = Vec3::new(0.0, 0.0, 1.0);
    if up.cross(light_dir).length() < 0.1 {
        up = Vec3::new(0.0, 1.0, 0.0);
    }
    let light_view = look_at(eye, camera.center, up);

    let inv_vp = camera.view_proj(aspect).inverse();
    let ndc_corners: [Vec3; 8] = [
        Vec3::new(-1.0, -1.0, 0.0),
        Vec3::new( 1.0, -1.0, 0.0),
        Vec3::new(-1.0,  1.0, 0.0),
        Vec3::new( 1.0,  1.0, 0.0),
        Vec3::new(-1.0, -1.0, 1.0),
        Vec3::new( 1.0, -1.0, 1.0),
        Vec3::new(-1.0,  1.0, 1.0),
        Vec3::new( 1.0,  1.0, 1.0),
    ];

    let mut min_ls = Vec3::splat(f32::MAX);
    let mut max_ls = Vec3::splat(f32::MIN);

    for ndc in &ndc_corners {
        let world4 = inv_vp * ndc.extend(1.0);
        let world = world4.truncate() / world4.w;
        let ls4 = light_view * world.extend(1.0);
        let ls = ls4.truncate() / ls4.w;
        min_ls = min_ls.min(ls);
        max_ls = max_ls.max(ls);
    }

    let margin = 0.5;
    min_ls.x -= margin;
    min_ls.y -= margin;
    max_ls.x += margin;
    max_ls.y += margin;

    min_ls.z -= 4.0;
    max_ls.z += 2.0;

    let proj = ortho(min_ls.x, max_ls.x, min_ls.y, max_ls.y, -max_ls.z, -min_ls.z);
    proj * light_view
}
