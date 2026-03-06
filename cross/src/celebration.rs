use crate::renderer::camera::Camera;
use glam::{Vec2, Vec3};

pub struct CelebrationBand {
    pub positions: Vec<Vec3>,
    pub previous_positions: Vec<Vec3>,
    pub hole_pos: Vec2,
    pub color: [f32; 3],
    pub radius: f32,
    pub segment_length: f32,
    pub phase: f32,
    pub speed: f32,
}

pub struct CameraSnapshot {
    pub center: Vec3,
    pub distance: f32,
    pub ortho_half_height: f32,
    pub tilt_angle: f32,
    pub orbit_angle: f32,
    pub perspective_blend: f32,
}

impl CameraSnapshot {
    pub fn capture(camera: &Camera) -> Self {
        Self {
            center: camera.center,
            distance: camera.distance,
            ortho_half_height: camera.ortho_half_height,
            tilt_angle: camera.tilt_angle,
            orbit_angle: camera.orbit_angle,
            perspective_blend: camera.perspective_blend,
        }
    }

    pub fn restore(&self, camera: &mut Camera) {
        camera.center = self.center;
        camera.distance = self.distance;
        camera.ortho_half_height = self.ortho_half_height;
        camera.tilt_angle = self.tilt_angle;
        camera.orbit_angle = self.orbit_angle;
        camera.perspective_blend = self.perspective_blend;
    }
}

pub fn spawn_celebration_bands(
    holes: &[Vec2],
    rope_count: usize,
    rope_colors: &[[f32; 3]],
    rope_radii: &[f32],
) -> Vec<CelebrationBand> {
    if holes.is_empty() {
        return Vec::new();
    }

    let rope_count = rope_count.min(8);
    let particle_count = 10;
    let seg_len = 0.08;
    let mut bands = Vec::with_capacity(rope_count);

    for i in 0..rope_count {
        let hole_idx = i % holes.len();
        let hp = holes[hole_idx];
        let base = if i < rope_colors.len() {
            rope_colors[i]
        } else {
            [1.0, 0.5, 0.3]
        };
        let glow = 1.8;
        let color = [base[0] * glow, base[1] * glow, base[2] * glow];
        let radius = if i < rope_radii.len() {
            rope_radii[i]
        } else {
            0.038
        };

        let mut positions = Vec::with_capacity(particle_count);
        for j in 0..particle_count {
            let z = -0.5 + j as f32 * seg_len;
            positions.push(Vec3::new(hp.x, hp.y, z));
        }
        let previous_positions = positions.clone();

        let phase = i as f32 * 1.3;
        let speed = 2.5 + (i as f32 * 0.7) % 1.5;

        bands.push(CelebrationBand {
            positions,
            previous_positions,
            hole_pos: hp,
            color,
            radius,
            segment_length: seg_len,
            phase,
            speed,
        });
    }

    bands
}

pub fn update_celebration(bands: &mut [CelebrationBand], victory_time: f32) {
    for band in bands {
        let pop_t = (victory_time * 2.5).min(1.0);
        let pop_ease = pop_t * pop_t * (3.0 - 2.0 * pop_t);
        let base_z = -0.5 + pop_ease * 1.7;

        let n = band.positions.len();
        for j in 0..n {
            let u = j as f32 / (n as f32 - 1.0).max(1.0);

            let sway_x = (victory_time * band.speed + band.phase + u * 5.0).sin();
            let sway_y = (victory_time * band.speed * 0.6 + band.phase * 1.7 + u * 4.0).cos();
            let amp = u * u * 0.25 * pop_ease;

            let bounce = if victory_time > 0.4 {
                let bt = victory_time - 0.4;
                ((bt * band.speed * 1.0 + band.phase).sin() * 0.5 + 0.5) * 0.15 * u
            } else {
                0.0
            };

            let z = base_z + u * band.segment_length * (n as f32 - 1.0) + bounce;

            band.positions[j] = Vec3::new(
                band.hole_pos.x + sway_x * amp,
                band.hole_pos.y + sway_y * amp,
                z,
            );
        }

        band.positions[0].z = base_z.min(0.0);
    }
}

pub fn animate_victory_camera(
    camera: &mut Camera,
    snap: &CameraSnapshot,
    victory_time: f32,
    dt: f32,
) {
    let blend_t = (victory_time / 1.0).min(1.0);
    let blend_s = blend_t * blend_t * (3.0 - 2.0 * blend_t);
    camera.perspective_blend = blend_s;

    let move_vt = (victory_time - 0.5).max(0.0);
    let move_t = (move_vt / 2.0).min(1.0);
    let t = move_t * move_t * (3.0 - 2.0 * move_t);

    camera.tilt_angle = snap.tilt_angle + t * 0.55;
    camera.distance = snap.distance * (1.0 + t * 0.2);
    camera.ortho_half_height = snap.ortho_half_height * (1.0 - t * 0.3);

    let orbit_speed = 0.3 + 0.12 * (move_vt * 0.5).sin();
    camera.orbit_angle += dt * orbit_speed * t;
}
