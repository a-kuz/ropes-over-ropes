use glam::Vec2;
use crate::renderer::camera::Camera;

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

pub fn screen_to_world(
    location: (f32, f32),
    viewport_size: (f32, f32),
    camera: &Camera,
) -> Vec2 {
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
    let hit_radius = hole_radius * 1.65;
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
        let start_score = start_distance + if start_top_allowed { 0.0 } else { hit_radius * 0.75 };
        if start_distance < hit_radius {
            if best.is_none() || start_score < best.unwrap().3 {
                best = Some((rope_index, 0, start_hole, start_score));
            }
        }

        let end_distance = (world - end_pos).length();
        let end_top_allowed = end_z >= start_z;
        let end_score = end_distance + if end_top_allowed { 0.0 } else { hit_radius * 0.75 };
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
    let snap_radius = hole_radius * 1.9;
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
