use glam::{Vec2, Vec3, Vec4};
use crate::level::definition::{CrossSection, MaterialFrame};
use super::frame_types::RopeVertex;

fn rv(position: Vec3, normal: Vec3, color: Vec3, tex_coord: Vec2, params: Vec4) -> RopeVertex {
    RopeVertex {
        position: position.into(),
        normal: normal.into(),
        color: color.into(),
        tex_coord: tex_coord.into(),
        params: params.into(),
    }
}

pub struct RopeMesh {
    pub vertices: Vec<RopeVertex>,
    pub indices: Vec<u32>,
}

pub struct TwistEvent {
    pub dist: f32,
    pub angle: f32,
    pub window: f32,
}

struct Profile2D {
    positions: Vec<Vec2>,
    normals: Vec<Vec2>,
    v: Vec<f32>,
}

const DEBUG_COLORS: [Vec3; 6] = [
    Vec3::new(1.0, 0.3, 0.3),
    Vec3::new(0.3, 1.0, 0.3),
    Vec3::new(0.3, 0.3, 1.0),
    Vec3::new(1.0, 1.0, 0.3),
    Vec3::new(1.0, 0.3, 1.0),
    Vec3::new(0.3, 1.0, 1.0),
];

fn smoothstep(edge0: f32, edge1: f32, value: f32) -> f32 {
    let normalized = ((value - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    normalized * normalized * (3.0 - 2.0 * normalized)
}

fn rotate(vector: Vec3, axis: Vec3, angle: f32) -> Vec3 {
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    vector * cos_a + axis.cross(vector) * sin_a + axis * axis.dot(vector) * (1.0 - cos_a)
}

fn twist_angle(at: f32, events: &[TwistEvent]) -> f32 {
    if events.is_empty() {
        return 0.0;
    }
    let mut accumulated = 0.0f32;
    for event in events {
        let distance = (at - event.dist).abs();
        let norm = (distance / event.window.max(1e-6)).min(1.0);
        let weight = smoothstep(1.0, 0.0, norm);
        accumulated += event.angle * weight;
    }
    accumulated
}

fn circular_profile(radius: f32, segments: usize) -> Profile2D {
    let r = radius.max(0.0005);
    let seg = segments.clamp(4, 32);
    let mut positions = Vec::with_capacity(seg);
    let mut normals = Vec::with_capacity(seg);
    let mut v = Vec::with_capacity(seg);
    for i in 0..seg {
        let angle = (i as f32 / seg as f32) * std::f32::consts::TAU;
        let ca = angle.cos();
        let sa = angle.sin();
        positions.push(Vec2::new(ca * r, sa * r));
        normals.push(Vec2::new(ca, sa).normalize());
        v.push(i as f32 / seg as f32);
    }
    Profile2D { positions, normals, v }
}

fn rectangular_profile(width: f32, height: f32) -> Profile2D {
    let hw = (width * 0.5).max(0.0005);
    let hh = (height * 0.5).max(0.0005);
    let corner_r = hw.min(hh) * 0.15;
    let corner_segs: usize = 3;

    let corners: [(Vec2, f32, f32); 4] = [
        (Vec2::new(hw - corner_r, hh - corner_r), 0.0, std::f32::consts::FRAC_PI_2),
        (Vec2::new(-hw + corner_r, hh - corner_r), std::f32::consts::FRAC_PI_2, std::f32::consts::PI),
        (Vec2::new(-hw + corner_r, -hh + corner_r), std::f32::consts::PI, std::f32::consts::PI * 1.5),
        (Vec2::new(hw - corner_r, -hh + corner_r), std::f32::consts::PI * 1.5, std::f32::consts::TAU),
    ];

    let total_verts = corners.len() * (corner_segs + 1);
    let mut positions = Vec::with_capacity(total_verts);
    let mut normals = Vec::with_capacity(total_verts);
    let mut v = Vec::with_capacity(total_verts);

    let mut idx = 0u32;
    for &(center, start_angle, end_angle) in &corners {
        for s in 0..=corner_segs {
            let t = s as f32 / corner_segs as f32;
            let angle = start_angle + (end_angle - start_angle) * t;
            let ca = angle.cos();
            let sa = angle.sin();
            positions.push(center + Vec2::new(ca * corner_r, sa * corner_r));
            normals.push(Vec2::new(ca, sa).normalize());
            v.push(idx as f32 / total_verts as f32);
            idx += 1;
        }
    }

    Profile2D { positions, normals, v }
}

fn segment_index(point_index: usize, segment_starts: &[usize]) -> usize {
    if segment_starts.is_empty() {
        return 0;
    }
    let mut seg = 0;
    for (idx, &start) in segment_starts.iter().enumerate() {
        if point_index >= start {
            seg = idx;
        } else {
            break;
        }
    }
    seg
}

#[allow(clippy::too_many_arguments)]
pub fn build_rect(
    points: &[Vec3],
    radius: f32,
    color: Vec3,
    twist_events: &[TwistEvent],
    tautness: f32,
    repulsors: &[Vec4],
    stretch_ratio: f32,
    oscillation: f32,
    segment_starts: &[usize],
    rest_length: f32,
    cross_section: CrossSection,
    material_frames: Option<&[MaterialFrame]>,
    fade_out: f32,
    rope_index: usize,
    profile_segments: usize,
) -> RopeMesh {
    let point_count = points.len();
    if point_count < 2 {
        return RopeMesh { vertices: Vec::new(), indices: Vec::new() };
    }

    let use_physics_frames = cross_section.is_rectangular()
        && material_frames.is_some()
        && material_frames.unwrap().len() == point_count;

    let r = radius.max(0.0005);
    let profile = match cross_section {
        CrossSection::Rectangular { width, height } => rectangular_profile(width, height),
        _ => circular_profile(r, profile_segments),
    };
    let profile_count = profile.positions.len();
    let faceted_profile = !cross_section.is_rectangular() && profile_segments <= 4;
    let ring_vert_count = if faceted_profile {
        profile_count * 2
    } else {
        profile_count
    };

    let mut total_len: f32 = 0.0;
    for i in 1..point_count {
        total_len += (points[i] - points[i - 1]).length();
    }
    total_len = total_len.max(1e-6);

    let effective_rest_length = if rest_length > 0.0 { rest_length } else { total_len };
    let global_stretch_factor = total_len / effective_rest_length.max(1e-6);

    let mut vertices = Vec::with_capacity(point_count * ring_vert_count);
    let mut indices = Vec::with_capacity((point_count - 1) * profile_count * 6);
    let mut ring_bases = Vec::with_capacity(point_count);

    let up = Vec3::Z;

    let mut distance_along: f32 = 0.0;
    let mut t_prev = (points[1] - points[0]).normalize();
    let mut nrm_prev = {
        let mut nrm = up.cross(t_prev);
        if nrm.length_squared() < 1e-8 {
            nrm = Vec3::X;
        }
        nrm.normalize()
    };

    for point_index in 0..point_count {
        let mut position = points[point_index];
        if point_index > 0 {
            distance_along += (points[point_index] - points[point_index - 1]).length();
        }
        let tangent_world = if point_index == 0 {
            (points[1] - points[0]).normalize()
        } else if point_index == point_count - 1 {
            (points[point_count - 1] - points[point_count - 2]).normalize()
        } else {
            (points[point_index + 1] - points[point_index - 1]).normalize()
        };

        let nrm: Vec3;
        let bin: Vec3;

        if use_physics_frames {
            let frame = &material_frames.unwrap()[point_index];
            nrm = frame.d1;
            bin = frame.d2;
        } else {
            let mut n = nrm_prev;
            if point_index > 0 {
                let axis = t_prev.cross(tangent_world);
                let axis_len = axis.length();
                if axis_len > 1e-6 {
                    let axis_n = axis / axis_len;
                    let dot_clamped = t_prev.dot(tangent_world).clamp(-1.0, 1.0);
                    let angle = axis_len.atan2(dot_clamped);
                    n = rotate(nrm_prev, axis_n, angle);
                    let proj = n - tangent_world * n.dot(tangent_world);
                    if proj.length_squared() > 1e-10 {
                        n = proj.normalize();
                    }
                }
            }

            let mut b = tangent_world.cross(n);
            if b.length_squared() < 1e-8 {
                let mut fallback = up.cross(tangent_world);
                if fallback.length_squared() < 1e-8 {
                    fallback = Vec3::X;
                }
                n = fallback.normalize();
                b = tangent_world.cross(n);
            }
            b = b.normalize();

            let twist = twist_angle(distance_along, twist_events);
            if twist.abs() > 1e-6 {
                let n2 = n * twist.cos() + b * twist.sin();
                let b2 = -n * twist.sin() + b * twist.cos();
                n = n2;
                b = b2;
            }

            t_prev = tangent_world;
            nrm_prev = n;

            nrm = n;
            bin = b;
        }

        let u_coord = point_index as f32 / (point_count as f32 - 1.0).max(1.0);
        let center = (u_coord * std::f32::consts::PI).sin();
        let center_mask = center * center;
        let center_mask_strong = center_mask * center_mask;

        let stretch_from_rest = (global_stretch_factor - 1.0).max(0.0);

        let stretch_effect = stretch_ratio - 1.0;
        let drag_tension = stretch_effect.max(0.0);

        let total_tension = stretch_from_rest + drag_tension * 0.8;

        let latex_deform = total_tension * 0.35 * center_mask_strong;

        let stretch_relax = if stretch_effect < 0.0 {
            stretch_effect.abs() * 0.2 * center_mask
        } else {
            0.0
        };

        let pinch = latex_deform - stretch_relax * 0.3;

        let adjusted_tautness = (tautness - stretch_relax * 0.3).max(0.0);

        let end_fade = smoothstep(0.04, 0.14, u_coord) * smoothstep(0.04, 0.14, 1.0 - u_coord);

        let mut repel_mag_total: f32 = 0.0;
        if !repulsors.is_empty() && end_fade > 1e-4 {
            let p2 = Vec2::new(position.x, position.y);
            let mut repel = Vec2::ZERO;
            for rep in repulsors {
                let c = Vec2::new(rep.x, rep.y);
                let rep_radius = rep.z;
                let strength = rep.w;
                let d = p2 - c;
                let d2 = d.length_squared();
                if d2 < 1e-12 {
                    continue;
                }
                let dist = d2.sqrt();
                let falloff = (rep_radius * 0.85).max(1e-4);
                let w = smoothstep(rep_radius + falloff, rep_radius, dist);
                if w <= 0.0 {
                    continue;
                }
                let dir = d / dist;
                repel += dir * (w * strength);
                repel_mag_total += w * strength;
            }
            let repel_scale = end_fade * (0.35 + 0.65 * (1.0 - pinch));
            position.x += repel.x * repel_scale;
            position.y += repel.y * repel_scale;
            position.z += (repel_mag_total * 0.22).min(0.02) * end_fade;
        }

        let params = Vec4::new(
            adjusted_tautness,
            pinch,
            rope_index as f32,
            fade_out,
        );

        let latex_thinning = 1.0 / (1.0 + total_tension * 1.5 * center_mask_strong).max(1.0).sqrt();
        let relax_thickening = 1.0 + stretch_relax * 0.15;
        let scale = latex_thinning * relax_thickening;

        let lighten_amount = total_tension * center_mask_strong * 0.35;
        let base_color = if !segment_starts.is_empty() {
            let seg_idx = segment_index(point_index, segment_starts);
            DEBUG_COLORS[seg_idx % DEBUG_COLORS.len()]
        } else {
            color
        };
        let adjusted_color = base_color * (1.0 + lighten_amount)
            + Vec3::splat(lighten_amount * 0.15);

        let osc_wave = (u_coord * std::f32::consts::PI * 3.0 + oscillation * 6.0).sin();
        let osc_amplitude = oscillation.abs() * 0.12;
        let osc_offset = osc_wave * osc_amplitude;
        let osc_dir = Vec2::new(nrm.x, nrm.y);
        let osc_dir_len = osc_dir.length();
        if osc_dir_len > 1e-6 {
            let osc_dir_norm = osc_dir / osc_dir_len;
            position.x += osc_dir_norm.x * osc_offset;
            position.y += osc_dir_norm.y * osc_offset;
        } else {
            let fallback_dir = Vec2::new(bin.x, bin.y);
            let fallback_len = fallback_dir.length();
            if fallback_len > 1e-6 {
                let fallback_norm = fallback_dir / fallback_len;
                position.x += fallback_norm.x * osc_offset;
                position.y += fallback_norm.y * osc_offset;
            }
        }

        ring_bases.push(vertices.len());
        if faceted_profile {
            for k in 0..profile_count {
                let k_next = (k + 1) % profile_count;
                let local_pos0 = profile.positions[k];
                let local_pos1 = profile.positions[k_next];
                let world_pos0 = position + nrm * (local_pos0.x * scale) + bin * (local_pos0.y * scale);
                let world_pos1 = position + nrm * (local_pos1.x * scale) + bin * (local_pos1.y * scale);
                let edge = world_pos1 - world_pos0;
                let mut face_n = edge.cross(tangent_world);
                let radial = (nrm * ((local_pos0.x + local_pos1.x) * 0.5)
                    + bin * ((local_pos0.y + local_pos1.y) * 0.5))
                    .normalize();
                if face_n.length_squared() < 1e-10 {
                    face_n = radial;
                } else {
                    face_n = face_n.normalize();
                    if face_n.dot(radial) < 0.0 {
                        face_n = -face_n;
                    }
                }
                vertices.push(rv(
                    world_pos0,
                    face_n,
                    adjusted_color,
                    Vec2::new(u_coord, profile.v[k]),
                    params,
                ));
                vertices.push(rv(
                    world_pos1,
                    face_n,
                    adjusted_color,
                    Vec2::new(u_coord, profile.v[k_next]),
                    params,
                ));
            }
        } else {
            for k in 0..profile_count {
                let local_pos = profile.positions[k];
                let local_n = profile.normals[k];
                let world_pos = position + nrm * (local_pos.x * scale) + bin * (local_pos.y * scale);
                let world_n = (nrm * local_n.x + bin * local_n.y).normalize();
                vertices.push(rv(world_pos, world_n, adjusted_color, Vec2::new(u_coord, profile.v[k]), params));
            }
        }
    }

    for point_index in 0..(point_count - 1) {
        let base_a = ring_bases[point_index] as u32;
        let base_b = ring_bases[point_index + 1] as u32;
        if faceted_profile {
            for k in 0..profile_count {
                let k0 = (k * 2) as u32;
                let k1 = k0 + 1;
                indices.push(base_a + k0);
                indices.push(base_b + k0);
                indices.push(base_b + k1);
                indices.push(base_a + k0);
                indices.push(base_b + k1);
                indices.push(base_a + k1);
            }
        } else {
            for k in 0..profile_count {
                let k0 = k as u32;
                let k1 = ((k + 1) % profile_count) as u32;
                indices.push(base_a + k0);
                indices.push(base_b + k0);
                indices.push(base_b + k1);
                indices.push(base_a + k0);
                indices.push(base_b + k1);
                indices.push(base_a + k1);
            }
        }
    }

    RopeMesh { vertices, indices }
}

pub fn build_plug(
    center: Vec3,
    hole_radius: f32,
    color: Vec3,
    rope_index: usize,
    fade_out: f32,
    segments: usize,
) -> RopeMesh {
    let seg = segments.clamp(6, 32);
    let dome_rings = if segments < 12 { 3 } else { 6 };

    let cap_r = hole_radius * 1.15;
    let shaft_r = hole_radius * 0.70;
    let shaft_depth = hole_radius * 0.4;
    let dome_height = cap_r * 0.7;
    let lip_z = hole_radius * 0.02;

    let params = Vec4::new(0.0, 0.0, rope_index as f32, fade_out);
    let dark = color * 0.7;

    let mut vertices = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    let ring = |verts: &mut Vec<RopeVertex>, r: f32, z: f32, n_bias: Vec3, c: Vec3| -> usize {
        let base = verts.len();
        for s in 0..seg {
            let theta = (s as f32 / seg as f32) * std::f32::consts::TAU;
            let dx = theta.cos();
            let dy = theta.sin();
            let n = (Vec3::new(dx, dy, 0.0) + n_bias).normalize();
            verts.push(rv(
                Vec3::new(center.x + dx * r, center.y + dy * r, z),
                n, c, Vec2::new(0.5, 0.5), params,
            ));
        }
        base
    };

    let connect = |idx: &mut Vec<u32>, ring_a: usize, ring_b: usize| {
        for s in 0..seg {
            let a0 = (ring_a + s) as u32;
            let a1 = (ring_a + (s + 1) % seg) as u32;
            let b0 = (ring_b + s) as u32;
            let b1 = (ring_b + (s + 1) % seg) as u32;
            idx.extend_from_slice(&[a0, b0, a1, a1, b0, b1]);
        }
    };

    let r0 = ring(&mut vertices, shaft_r, center.z - shaft_depth, Vec3::ZERO, dark);
    let r1 = ring(&mut vertices, shaft_r, center.z, Vec3::ZERO, dark);
    connect(&mut indices, r0, r1);

    let r2 = ring(&mut vertices, cap_r, center.z + lip_z, Vec3::new(0.0, 0.0, 0.4), color);
    connect(&mut indices, r1, r2);

    let dome_base = center.z + lip_z;
    let mut prev_ring = r2;
    for ri in 1..=dome_rings {
        let t = ri as f32 / dome_rings as f32;
        let phi = t * std::f32::consts::FRAC_PI_2;
        let ring_r = cap_r * phi.cos();
        let ring_z = dome_base + dome_height * phi.sin();
        let n_up = Vec3::new(0.0, 0.0, phi.sin() * 0.5);
        let blend = t * t;
        let ring_col = color * (1.0 - blend * 0.15);
        let cur = ring(&mut vertices, ring_r, ring_z, n_up, ring_col);
        connect(&mut indices, prev_ring, cur);
        prev_ring = cur;
    }

    let tip_idx = vertices.len() as u32;
    let tip_z = dome_base + dome_height;
    vertices.push(rv(
        Vec3::new(center.x, center.y, tip_z),
        Vec3::Z, color * 0.85, Vec2::new(0.5, 0.5), params,
    ));
    for s in 0..seg {
        let curr = (prev_ring + s) as u32;
        let next = (prev_ring + (s + 1) % seg) as u32;
        indices.extend_from_slice(&[curr, tip_idx, next]);
    }

    RopeMesh { vertices, indices }
}

pub fn build_hemisphere(
    center: Vec3,
    radius: f32,
    facing: Vec3,
    color: Vec3,
    segments: usize,
    rings: usize,
    darken: f32,
) -> RopeMesh {
    let r = radius.max(0.001);
    let seg = segments.max(6);
    let rng = rings.max(3);

    let mut up = Vec3::Z;
    if up.dot(facing).abs() > 0.95 {
        up = Vec3::Y;
    }
    let right = up.cross(facing).normalize();
    let forward = facing.cross(right).normalize();

    let dark_color = color * (1.0 - darken);

    let mut vertices = Vec::with_capacity((rng + 1) * seg + 1);
    let mut indices = Vec::with_capacity(rng * seg * 6);

    for ring in 0..=rng {
        let phi = (ring as f32 / rng as f32) * std::f32::consts::FRAC_PI_2;
        let t = ring as f32 / rng as f32;
        let blend = t * t;
        let ring_color = color * (1.0 - blend) + dark_color * blend;
        let ring_r = r * phi.cos();
        let ring_z = r * phi.sin();
        for s in 0..seg {
            let theta = (s as f32 / seg as f32) * std::f32::consts::TAU;
            let local_x = theta.cos() * ring_r;
            let local_y = theta.sin() * ring_r;
            let pos = center + right * local_x + forward * local_y + facing * ring_z;
            let n = (right * (theta.cos() * phi.cos())
                + forward * (theta.sin() * phi.cos())
                + facing * phi.sin())
            .normalize();
            vertices.push(rv(pos, n, ring_color, Vec2::new(0.5, 0.5), Vec4::ZERO));
        }
    }

    for ring in 0..rng {
        for s in 0..seg {
            let curr = (ring * seg + s) as u32;
            let next = (ring * seg + (s + 1) % seg) as u32;
            let curr_up = ((ring + 1) * seg + s) as u32;
            let next_up = ((ring + 1) * seg + (s + 1) % seg) as u32;
            indices.extend_from_slice(&[curr, curr_up, next, next, curr_up, next_up]);
        }
    }

    let tip_idx = vertices.len() as u32;
    vertices.push(rv(center + facing * r, facing, dark_color, Vec2::new(0.5, 0.5), Vec4::ZERO));
    let top_ring = rng * seg;
    for s in 0..seg {
        let curr = (top_ring + s) as u32;
        let next = (top_ring + (s + 1) % seg) as u32;
        indices.extend_from_slice(&[curr, tip_idx, next]);
    }

    RopeMesh { vertices, indices }
}
