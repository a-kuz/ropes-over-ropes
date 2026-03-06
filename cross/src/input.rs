use crate::renderer::camera::Camera;
use glam::Vec2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputPhase {
    Began,
    Moved,
    Ended,
    Cancelled,
}

pub struct DragState {
    pub rope_index: usize,
    pub end_index: usize,
    pub original_hole_index: usize,
}

pub enum DragResult {
    Snapped { hole: usize, moved: bool },
    Cancelled,
}

pub fn screen_to_world(location: (f32, f32), viewport_size: (f32, f32), camera: &Camera) -> Vec2 {
    let (width, height) = viewport_size;
    let width = width.max(1.0);
    let height = height.max(1.0);
    let aspect = width / height;

    let ndc_x = (location.0 / width) * 2.0 - 1.0;
    let ndc_y = (location.1 / height) * 2.0 - 1.0;

    let view_proj = camera.view_proj(aspect);
    let inv = view_proj.inverse();

    let near4 = inv * glam::Vec4::new(ndc_x, ndc_y, 0.0, 1.0);
    let far4 = inv * glam::Vec4::new(ndc_x, ndc_y, 1.0, 1.0);
    let near_pt = glam::Vec3::new(near4.x / near4.w, near4.y / near4.w, near4.z / near4.w);
    let far_pt = glam::Vec3::new(far4.x / far4.w, far4.y / far4.w, far4.z / far4.w);

    let dir = far_pt - near_pt;
    if dir.z.abs() < 1e-8 {
        return Vec2::new(near_pt.x, near_pt.y);
    }
    let t = -near_pt.z / dir.z;
    let hit = near_pt + dir * t;
    Vec2::new(hit.x, hit.y)
}

pub fn find_nearest_endpoint(
    world: Vec2,
    hole_positions: &[Vec2],
    rope_endpoints: &[(usize, usize)],
    endpoint_z: &dyn Fn(usize, usize) -> f32,
    hole_radius: f32,
) -> Option<(usize, usize, usize)> {
    let hit_radius = hole_radius * 3.5;
    let mut best: Option<(usize, usize, usize, f32)> = None;

    for (rope_index, &(start_hole, end_hole)) in rope_endpoints.iter().enumerate() {
        if start_hole >= hole_positions.len() || end_hole >= hole_positions.len() {
            continue;
        }
        let start_pos = hole_positions[start_hole];
        let end_pos = hole_positions[end_hole];
        let start_z = endpoint_z(rope_index, 0);
        let end_z = endpoint_z(rope_index, 1);

        let start_distance = (world - start_pos).length();
        let start_top_allowed = start_z >= end_z;
        let start_score = start_distance
            + if start_top_allowed {
                0.0
            } else {
                hit_radius * 0.75
            };
        if start_distance < hit_radius {
            if best.is_none() || start_score < best.unwrap().3 {
                best = Some((rope_index, 0, start_hole, start_score));
            }
        }

        let end_distance = (world - end_pos).length();
        let end_top_allowed = end_z >= start_z;
        let end_score = end_distance
            + if end_top_allowed {
                0.0
            } else {
                hit_radius * 0.75
            };
        if end_distance < hit_radius {
            if best.is_none() || end_score < best.unwrap().3 {
                best = Some((rope_index, 1, end_hole, end_score));
            }
        }
    }

    best.map(|(rope, end, hole, _)| (rope, end, hole))
}

pub fn find_snap_hole(
    world: Vec2,
    hole_positions: &[Vec2],
    hole_occupied: &[bool],
    hole_radius: f32,
) -> Option<usize> {
    let snap_radius = hole_radius * 3.8;
    let mut best_index = None;
    let mut best_distance = f32::MAX;

    for (i, &pos) in hole_positions.iter().enumerate() {
        if i >= hole_occupied.len() || hole_occupied[i] {
            continue;
        }
        let distance = (pos - world).length();
        if distance < snap_radius && distance < best_distance {
            best_index = Some(i);
            best_distance = distance;
        }
    }
    best_index
}

pub fn begin_drag_action(
    screen_pos: (f32, f32),
    viewport: (f32, f32),
    camera: &Camera,
    hole_positions: &[Vec2],
    rope_endpoints: &[(usize, usize)],
    endpoint_z: &dyn Fn(usize, usize) -> f32,
    hole_radius: f32,
) -> Option<(DragState, usize)> {
    let world = screen_to_world(screen_pos, viewport, camera);
    let (rope_index, end_index, hole_index) = find_nearest_endpoint(
        world,
        hole_positions,
        rope_endpoints,
        endpoint_z,
        hole_radius,
    )?;
    Some((
        DragState {
            rope_index,
            end_index,
            original_hole_index: hole_index,
        },
        hole_index,
    ))
}

pub fn end_drag_action(
    drag: &DragState,
    screen_pos: (f32, f32),
    viewport: (f32, f32),
    camera: &Camera,
    hole_positions: &[Vec2],
    hole_occupied: &[bool],
    hole_radius: f32,
) -> DragResult {
    let world = screen_to_world(screen_pos, viewport, camera);
    match find_snap_hole(world, hole_positions, hole_occupied, hole_radius) {
        Some(snap_hole) => DragResult::Snapped {
            hole: snap_hole,
            moved: snap_hole != drag.original_hole_index,
        },
        None => DragResult::Cancelled,
    }
}

pub fn apply_camera_pan(
    camera: &mut Camera,
    prev: (f32, f32),
    cur: (f32, f32),
    surface_w: u32,
    surface_h: u32,
) {
    let aspect = surface_w as f32 / surface_h.max(1) as f32;
    let half_h = camera.ortho_half_height;
    let half_w = half_h * aspect;
    let dx = (cur.0 - prev.0) / surface_w as f32 * half_w * 2.0;
    let dy = (cur.1 - prev.1) / surface_h as f32 * half_h * 2.0;
    camera.center.x -= dx;
    camera.center.y -= dy;
}
