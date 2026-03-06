use super::super::level::definition::{Board, CrossSection, MaterialFrame};
use glam::{Vec2, Vec3};
use std::collections::HashMap;

#[derive(Clone)]
pub struct BoardDef {
    pub center_x: f32,
    pub center_y: f32,
    pub width: f32,
    pub height: f32,
    pub elevation: f32,
}

impl From<&Board> for BoardDef {
    fn from(value: &Board) -> Self {
        Self {
            center_x: value.center_x,
            center_y: value.center_y,
            width: value.width,
            height: value.height,
            elevation: value.elevation,
        }
    }
}

// --- Band ---

pub struct Band {
    pub positions: Vec<Vec3>,
    pub previous_positions: Vec<Vec3>,
    pub twist_angles: Vec<f32>,
    pub previous_twist_angles: Vec<f32>,
    pub segment_length: f32,
    pub radius: f32,
    pub cross_section: CrossSection,
    pub pin_start: Option<usize>,
    pub pin_end: Option<usize>,
    pub active: bool,
    pub fade_out: f32,
    pub suck_hole: Option<usize>,
    pub suck_tail_hole: Option<usize>,
    pub suck_from_end: usize,
    pub suck_consumed: f32,
    pub suck_frame: i32,
    pub suck_seg_lengths: Vec<f32>,
    pub suck_orig_positions: Vec<Vec3>,
}

impl Band {
    pub const FADE_OUT_SPEED: f32 = 45.0;
}

// --- DragInfo ---

pub struct DragInfo {
    pub band_index: usize,
    pub end_index: usize,
    pub original_hole_index: usize,
}

// --- LowerAnimation ---

pub struct LowerAnimation {
    pub band_index: usize,
    pub end_index: usize,
    pub target_hole: usize,
    pub start_pos: Vec3,
    pub timer: f32,
    pub return_pos: Option<Vec3>,
    pub return_duration: f32,
}

impl LowerAnimation {
    pub const DURATION: f32 = 0.55;
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct LowerAnimationKey {
    pub band_index: usize,
    pub end_index: usize,
}

// --- CollisionPair ---

pub struct CollisionPair {
    pub band_a: u16,
    pub seg_a: u16,
    pub band_b: u16,
    pub seg_b: u16,
}

// --- RopeConfig ---

pub struct RopeConfig {
    pub start_hole: usize,
    pub end_hole: usize,
    pub radius: f32,
    pub cross_section: Option<CrossSection>,
}

// --- LevelAction ---

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ActionType {
    Pin,
    Drag,
}

pub struct LevelAction {
    pub action_type: ActionType,
    pub rope_index: usize,
    pub end_index: usize,
    pub hole_index: usize,
}

// --- Undo snapshots ---

#[derive(Clone)]
pub struct BandSnapshot {
    pub positions: Vec<Vec3>,
    pub previous_positions: Vec<Vec3>,
    pub twist_angles: Vec<f32>,
    pub previous_twist_angles: Vec<f32>,
    pub segment_length: f32,
    pub pin_start: Option<usize>,
    pub pin_end: Option<usize>,
    pub active: bool,
    pub fade_out: f32,
    pub suck_hole: Option<usize>,
    pub suck_tail_hole: Option<usize>,
    pub suck_from_end: usize,
    pub suck_consumed: f32,
    pub suck_frame: i32,
    pub suck_seg_lengths: Vec<f32>,
    pub suck_orig_positions: Vec<Vec3>,
}

#[derive(Clone)]
pub struct Snapshot {
    pub bands: Vec<BandSnapshot>,
}

// --- VerletSimulator ---

pub struct VerletSimulator {
    pub bands: Vec<Band>,
    pub hole_positions: Vec<Vec2>,
    pub hole_elevations: Vec<f32>,
    pub hole_radius: f32,
    pub hole_depth: f32,
    pub boards: Vec<BoardDef>,

    pub gravity: f32,
    pub damping: f32,
    pub constraint_iterations: usize,
    pub settle_steps: usize,
    pub lift_height: f32,
    pub rope_tension: f32,
    pub friction_coefficient: f32,
    pub particle_count: usize,
    pub twist_stiffness: f32,
    pub twist_damping: f32,
    pub gravity_torque_strength: f32,
    pub bend_compliance: f32,
    pub bend_velocity_coupling: f32,
    pub stretch_thinning: f32,
    pub square_cross_section: bool,

    current_tension: f32,
    tension_speed: f32,
    dt: f32,
    init_dt: f32,
    accumulator: f32,

    pub drag_info: Option<DragInfo>,
    drag_target_pos: Option<Vec3>,
    drag_start_pos: Option<Vec3>,

    pub lower_animations: HashMap<LowerAnimationKey, LowerAnimation>,

    pub cached_frames: Vec<Vec<MaterialFrame>>,
    idle_timeout: f32,
    idle_timer: f32,
    pub is_sleeping: bool,
    friction_accumulator: f32,
    friction_speed_accumulator: f32,
    friction_position_accumulator: Vec3,
    friction_sample_count: usize,
}

impl VerletSimulator {
    pub fn new(
        hole_positions: Vec<Vec2>,
        hole_elevations: Vec<f32>,
        hole_radius: f32,
        boards: Vec<BoardDef>,
    ) -> Self {
        let hole_elevations = if hole_elevations.len() == hole_positions.len() {
            hole_elevations
        } else {
            vec![0.0; hole_positions.len()]
        };
        Self {
            hole_depth: hole_radius * 1.25,
            hole_positions,
            hole_elevations,
            hole_radius,
            boards,
            bands: Vec::new(),
            gravity: -6.3927,
            damping: 0.9470582,
            constraint_iterations: 11,
            settle_steps: 5,
            lift_height: 0.30,
            rope_tension: 0.88084733,
            friction_coefficient: 0.8,
            particle_count: 65,
            twist_stiffness: 0.15,
            twist_damping: 0.4,
            gravity_torque_strength: 0.8,
            bend_compliance: 0.00015530341,
            bend_velocity_coupling: 0.65,
            stretch_thinning: 0.093161635,
            square_cross_section: false,
            current_tension: 1.0,
            tension_speed: 0.5,
            dt: 1.0 / 120.0,
            init_dt: 1.0 / 60.0,
            accumulator: 0.0,
            drag_info: None,
            drag_target_pos: None,
            drag_start_pos: None,
            lower_animations: HashMap::new(),
            cached_frames: Vec::new(),
            idle_timeout: 3.0,
            idle_timer: 0.0,
            is_sleeping: false,
            friction_accumulator: 0.0,
            friction_speed_accumulator: 0.0,
            friction_position_accumulator: Vec3::ZERO,
            friction_sample_count: 0,
        }
    }

    pub fn take_snapshot(&self) -> Snapshot {
        Snapshot {
            bands: self
                .bands
                .iter()
                .map(|b| BandSnapshot {
                    positions: b.positions.clone(),
                    previous_positions: b.previous_positions.clone(),
                    twist_angles: b.twist_angles.clone(),
                    previous_twist_angles: b.previous_twist_angles.clone(),
                    segment_length: b.segment_length,
                    pin_start: b.pin_start,
                    pin_end: b.pin_end,
                    active: b.active,
                    fade_out: b.fade_out,
                    suck_hole: b.suck_hole,
                    suck_tail_hole: b.suck_tail_hole,
                    suck_from_end: b.suck_from_end,
                    suck_consumed: b.suck_consumed,
                    suck_frame: b.suck_frame,
                    suck_seg_lengths: b.suck_seg_lengths.clone(),
                    suck_orig_positions: b.suck_orig_positions.clone(),
                })
                .collect(),
        }
    }

    pub fn restore_snapshot(&mut self, snapshot: &Snapshot) {
        for (band, snap) in self.bands.iter_mut().zip(snapshot.bands.iter()) {
            band.positions = snap.positions.clone();
            band.previous_positions = snap.previous_positions.clone();
            band.twist_angles = snap.twist_angles.clone();
            band.previous_twist_angles = snap.previous_twist_angles.clone();
            band.segment_length = snap.segment_length;
            band.pin_start = snap.pin_start;
            band.pin_end = snap.pin_end;
            band.active = snap.active;
            band.fade_out = snap.fade_out;
            band.suck_hole = snap.suck_hole;
            band.suck_tail_hole = snap.suck_tail_hole;
            band.suck_from_end = snap.suck_from_end;
            band.suck_consumed = snap.suck_consumed;
            band.suck_frame = snap.suck_frame;
            band.suck_seg_lengths = snap.suck_seg_lengths.clone();
            band.suck_orig_positions = snap.suck_orig_positions.clone();
        }
        self.drag_info = None;
        self.drag_start_pos = None;
        self.drag_target_pos = None;
        self.lower_animations.clear();
        self.current_tension = self.rope_tension;
        self.wake_up();
    }

    pub fn has_lower_animations(&self) -> bool {
        !self.lower_animations.is_empty()
    }

    pub fn set_constraint_iterations(&mut self, val: usize) {
        self.constraint_iterations = val.max(2);
    }

    pub fn wake_up(&mut self) {
        self.idle_timer = 0.0;
        self.is_sleeping = false;
    }

    pub fn consume_and_reset_friction(&mut self) -> Option<(f32, f32, Vec3)> {
        if self.friction_sample_count == 0 {
            return None;
        }
        let n = self.friction_sample_count as f32;
        let result = (
            self.friction_accumulator / n,
            self.friction_speed_accumulator / n,
            self.friction_position_accumulator / n,
        );
        self.friction_accumulator = 0.0;
        self.friction_speed_accumulator = 0.0;
        self.friction_position_accumulator = Vec3::ZERO;
        self.friction_sample_count = 0;
        Some(result)
    }

    // --- Band management ---

    pub fn add_band(
        &mut self,
        radius: f32,
        cross_section: Option<CrossSection>,
        particle_count: Option<usize>,
    ) -> usize {
        let n = particle_count.unwrap_or(self.particle_count);
        let cs = cross_section.unwrap_or(CrossSection::Circular { radius });
        let band = Band {
            positions: vec![Vec3::ZERO; n],
            previous_positions: vec![Vec3::ZERO; n],
            twist_angles: vec![0.0; n],
            previous_twist_angles: vec![0.0; n],
            segment_length: 0.0,
            radius,
            cross_section: cs,
            pin_start: None,
            pin_end: None,
            active: false,
            fade_out: 0.0,
            suck_hole: None,
            suck_tail_hole: None,
            suck_from_end: 1,
            suck_consumed: 0.0,
            suck_frame: 0,
            suck_seg_lengths: Vec::new(),
            suck_orig_positions: Vec::new(),
        };
        self.bands.push(band);
        self.bands.len() - 1
    }

    pub fn pin(&mut self, band_index: usize, start_hole: usize, end_hole: usize) {
        if band_index >= self.bands.len() {
            return;
        }

        let p0 = self.hole_position_3d(start_hole);
        let p1 = self.hole_position_3d(end_hole);
        let n = self.bands[band_index].positions.len();

        for i in 0..n {
            let t = i as f32 / (n - 1).max(1) as f32;
            let x = p0.x * (1.0 - t) + p1.x * t;
            let y = p0.y * (1.0 - t) + p1.y * t;
            let z = if i == 0 || i == n - 1 {
                p0.z
            } else {
                self.lift_height
            };
            self.bands[band_index].positions[i] = Vec3::new(x, y, z);
        }
        self.bands[band_index].positions[0] = p0;
        self.bands[band_index].positions[n - 1] = p1;
        self.bands[band_index].previous_positions = self.bands[band_index].positions.clone();

        let dist = (p1 - p0).length();
        self.bands[band_index].segment_length = dist / (n - 1).max(1) as f32;
        self.bands[band_index].pin_start = Some(start_hole);
        self.bands[band_index].pin_end = Some(end_hole);
        self.bands[band_index].active = true;

        let has_others = self
            .bands
            .iter()
            .enumerate()
            .any(|(idx, b)| idx != band_index && b.active && b.pin_start.is_some());
        self.do_steps(self.settle_steps, has_others);
    }

    // --- Physics step ---

    pub fn update(&mut self, delta_time: f32) {
        let clamped_dt = delta_time.min(1.0 / 15.0);
        if self.drag_info.is_some() || self.has_lower_animations() {
            self.idle_timer = 0.0;
            self.is_sleeping = false;
        } else {
            self.idle_timer += clamped_dt;
        }

        if self.idle_timer >= self.idle_timeout && !self.is_sleeping {
            for band in &mut self.bands {
                if band.active && band.fade_out == 0.0 {
                    band.previous_positions.clone_from(&band.positions);
                    band.previous_twist_angles.clone_from(&band.twist_angles);
                }
            }
            self.is_sleeping = true;
        }

        if self.is_sleeping {
            for band in &self.bands {
                if band.fade_out > 0.0 && band.active {
                    self.is_sleeping = false;
                    self.idle_timer = 0.0;
                    break;
                }
            }
            if self.is_sleeping {
                self.accumulator = 0.0;
                return;
            }
        }

        for i in 0..self.bands.len() {
            if !(self.bands[i].fade_out > 0.0 && self.bands[i].active) {
                continue;
            }
            let hole = match self.bands[i].suck_hole {
                Some(h) => h,
                None => continue,
            };

            let n = self.bands[i].positions.len();
            let hole_xy = self.hole_positions[hole];
            let hole_elev = self.hole_surface_z(hole);
            let hole_below = Vec3::new(hole_xy.x, hole_xy.y, hole_elev - self.hole_depth);

            let pull_speed = Band::FADE_OUT_SPEED * self.bands[i].segment_length;
            self.bands[i].suck_consumed += pull_speed * clamped_dt;

            let from_end = self.bands[i].suck_from_end;
            let suck_segs = self.bands[i].suck_seg_lengths.clone();
            let orig_positions = self.bands[i].suck_orig_positions.clone();
            let r = self.bands[i].radius;
            let mut arc_len = vec![0.0f32; n];
            if from_end == 1 {
                for k in 1..n {
                    let seg_l = suck_segs
                        .get(k - 1)
                        .copied()
                        .unwrap_or(self.bands[i].segment_length);
                    arc_len[k] = arc_len[k - 1] + seg_l;
                }
            } else if n > 1 {
                for k in (0..=(n - 2)).rev() {
                    let seg_l = suck_segs
                        .get(k)
                        .copied()
                        .unwrap_or(self.bands[i].segment_length);
                    arc_len[k] = arc_len[k + 1] + seg_l;
                }
            }
            let total_arc = if from_end == 1 {
                arc_len[n - 1]
            } else {
                arc_len[0]
            };
            let consumed = self.bands[i].suck_consumed;

            for k in 0..n {
                let shifted = arc_len[k] - consumed;
                if shifted <= 0.0 {
                    self.bands[i].positions[k] = hole_below - Vec3::new(0.0, 0.0, -shifted);
                } else {
                    if from_end == 1 {
                        let mut seg = 0usize;
                        let mut acc = 0.0f32;
                        while seg < n - 1 {
                            let seg_l = suck_segs
                                .get(seg)
                                .copied()
                                .unwrap_or(self.bands[i].segment_length);
                            if acc + seg_l >= shifted {
                                break;
                            }
                            acc += seg_l;
                            seg += 1;
                        }
                        let seg_l = suck_segs
                            .get(seg)
                            .copied()
                            .unwrap_or(self.bands[i].segment_length);
                        let t = if seg_l > 1e-9 {
                            ((shifted - acc) / seg_l).min(1.0)
                        } else {
                            0.0
                        };
                        let p0 = orig_positions[seg];
                        let p1 = orig_positions.get(seg + 1).copied().unwrap_or(p0);
                        self.bands[i].positions[k] = p0 + (p1 - p0) * t;
                    } else {
                        let mut seg = n - 1;
                        let mut acc = 0.0f32;
                        while seg > 0 {
                            let seg_l = suck_segs
                                .get(seg - 1)
                                .copied()
                                .unwrap_or(self.bands[i].segment_length);
                            if acc + seg_l >= shifted {
                                break;
                            }
                            acc += seg_l;
                            seg -= 1;
                        }
                        let seg_l = suck_segs
                            .get(seg.saturating_sub(1))
                            .copied()
                            .unwrap_or(self.bands[i].segment_length);
                        let t = if seg_l > 1e-9 {
                            ((shifted - acc) / seg_l).min(1.0)
                        } else {
                            0.0
                        };
                        let p0 = orig_positions[seg];
                        let p1 = orig_positions
                            .get(seg.saturating_sub(1))
                            .copied()
                            .unwrap_or(p0);
                        self.bands[i].positions[k] = p0 + (p1 - p0) * t;
                    }

                    let p = self.bands[i].positions[k];
                    let surf_z = self.board_surface_z(p.x, p.y);
                    if p.z >= surf_z && p.z < surf_z + r {
                        self.bands[i].positions[k].z = surf_z + r;
                    }
                }

                self.bands[i].previous_positions[k] = self.bands[i].positions[k];
            }

            if consumed >= total_arc {
                self.bands[i].fade_out = 1.0;
                self.bands[i].active = false;
                self.bands[i].pin_start = None;
                self.bands[i].pin_end = None;
                self.bands[i].suck_hole = None;
            } else {
                let mut above_count = 0usize;
                for k in 0..n {
                    let surf_z = self.board_surface_z(
                        self.bands[i].positions[k].x,
                        self.bands[i].positions[k].y,
                    );
                    if self.bands[i].positions[k].z >= surf_z {
                        above_count += 1;
                    }
                }
                self.bands[i].fade_out = (1.0 - above_count as f32 / n as f32).min(0.999);
            }
        }

        self.accumulator += clamped_dt;
        let steps_needed = (self.accumulator / self.dt) as i32;
        self.accumulator -= steps_needed as f32 * self.dt;
        let num = steps_needed.max(0) as usize;
        if num == 0 {
            return;
        }

        let target_tension = if self.drag_info.is_some() {
            1.0
        } else {
            self.rope_tension
        };
        if self.current_tension != target_tension {
            let diff = target_tension - self.current_tension;
            let step = self.tension_speed * clamped_dt;
            if diff.abs() <= step {
                self.current_tension = target_tension;
            } else {
                self.current_tension += if diff > 0.0 { step } else { -step };
            }
        }

        if let Some(ref drag) = self.drag_info {
            if let Some(target) = self.drag_target_pos {
                let bi = drag.band_index;
                let idx = if drag.end_index == 0 {
                    0
                } else {
                    self.bands[bi].positions.len() - 1
                };
                let start_pos = self.drag_start_pos.unwrap_or(self.bands[bi].positions[idx]);
                let fixed_dt = self.dt;

                for s in 1..=num {
                    let t = s as f32 / num as f32;
                    let interp_pos = start_pos + (target - start_pos) * t;
                    self.bands[bi].positions[idx] = interp_pos;
                    self.bands[bi].previous_positions[idx] = interp_pos;
                    self.verlet_step(true, fixed_dt);
                }
                self.drag_start_pos = Some(target);
            } else {
                let fixed_dt = self.dt;
                for _ in 0..num {
                    self.verlet_step(true, fixed_dt);
                }
            }
        } else {
            let fixed_dt = self.dt;
            for _ in 0..num {
                self.verlet_step(true, fixed_dt);
            }
        }

        self.update_lower_animation(clamped_dt);
    }

    fn update_lower_animation(&mut self, delta_time: f32) {
        if self.lower_animations.is_empty() {
            return;
        }

        let keys: Vec<LowerAnimationKey> = self.lower_animations.keys().copied().collect();
        for key in keys {
            let mut anim = match self.lower_animations.remove(&key) {
                Some(anim) => anim,
                None => continue,
            };
            anim.timer += delta_time;

            let bi = anim.band_index;
            let idx = if anim.end_index == 0 {
                0
            } else {
                self.bands[bi].positions.len() - 1
            };

            if let Some(return_target) = anim.return_pos {
                let t = (anim.timer / anim.return_duration).min(1.0);
                let eased = 1.0 - (1.0 - t) * (1.0 - t);
                let pos = anim.start_pos + (return_target - anim.start_pos) * eased;
                self.bands[bi].positions[idx] = pos;
                self.bands[bi].previous_positions[idx] = pos;

                if t >= 1.0 {
                    anim.start_pos = return_target;
                    anim.return_pos = None;
                    anim.timer = 0.0;
                }

                self.lower_animations.insert(key, anim);
                continue;
            }

            let hole_pos = self.hole_position_3d(anim.target_hole);

            let t = (anim.timer / LowerAnimation::DURATION).min(1.0);
            let eased = 1.0 - (1.0 - t) * (1.0 - t);
            let pos = anim.start_pos + (hole_pos - anim.start_pos) * eased;
            self.bands[bi].positions[idx] = pos;
            self.bands[bi].previous_positions[idx] = pos;

            if t >= 1.0 {
                if anim.end_index == 0 {
                    self.bands[bi].pin_start = Some(anim.target_hole);
                } else {
                    self.bands[bi].pin_end = Some(anim.target_hole);
                }
                self.bands[bi].positions[idx] = hole_pos;
                self.bands[bi].previous_positions[idx] = hole_pos;
                continue;
            }

            self.lower_animations.insert(key, anim);
        }
    }

    fn do_steps(&mut self, n: usize, collide: bool) {
        let dt = self.init_dt;
        for _ in 0..n {
            self.verlet_step(collide, dt);
        }
    }

    fn verlet_step(&mut self, collide: bool, dt: f32) {
        let dt2 = dt * dt;

        // 1. Verlet integration + velocity limiting
        let grav_vec = Vec3::new(0.0, 0.0, self.gravity * dt2);
        let damping = self.damping;
        for bi in 0..self.bands.len() {
            if !self.bands[bi].active || self.bands[bi].fade_out != 0.0 {
                continue;
            }
            let n = self.bands[bi].positions.len();
            let max_move = self.bands[bi].radius * 2.0;
            for i in 1..(n - 1) {
                let pos = self.bands[bi].positions[i];
                let old = self.bands[bi].previous_positions[i];
                let mut vel = (pos - old) * damping;
                let vel_len = vel.length();
                if vel_len > max_move {
                    vel = vel * (max_move / vel_len);
                }
                self.bands[bi].previous_positions[i] = pos;
                self.bands[bi].positions[i] = pos + vel + grav_vec;
            }

            if self.bands[bi].cross_section.is_rectangular() {
                let max_twist_vel: f32 = 0.08;
                let twist_damp = self.twist_damping;
                for i in 1..(n - 1) {
                    let twist = self.bands[bi].twist_angles[i];
                    let old_twist = self.bands[bi].previous_twist_angles[i];
                    let mut twist_vel = (twist - old_twist) * twist_damp;
                    twist_vel = twist_vel.clamp(-max_twist_vel, max_twist_vel);
                    self.bands[bi].previous_twist_angles[i] = twist;
                    self.bands[bi].twist_angles[i] = twist + twist_vel;
                }
            }
        }

        // 2. Build active list + collision pairs
        let active: Vec<usize> = if collide {
            (0..self.bands.len())
                .filter(|&bi| self.bands[bi].active && self.bands[bi].fade_out == 0.0)
                .collect()
        } else {
            Vec::new()
        };

        let effective_iters = self
            .constraint_iterations
            .max((self.constraint_iterations as f32 / self.current_tension.max(0.3)) as usize);

        let collision_pairs: Vec<CollisionPair> = if collide {
            self.build_collision_pairs(&active)
        } else {
            Vec::new()
        };

        self.recompute_frames();

        // Gravity torque for rectangular bands
        for bi in 0..self.bands.len() {
            if !self.bands[bi].active
                || self.bands[bi].fade_out != 0.0
                || !self.bands[bi].cross_section.is_rectangular()
            {
                continue;
            }
            let n = self.bands[bi].positions.len();
            if bi >= self.cached_frames.len() || self.cached_frames[bi].len() != n {
                continue;
            }
            let max_torque: f32 = 0.01;
            let gts = self.gravity_torque_strength;
            for i in 1..(n - 1) {
                let d1z = self.cached_frames[bi][i].d1.z;
                let torque = (-d1z * gts * dt2).clamp(-max_torque, max_torque);
                self.bands[bi].twist_angles[i] += torque;
            }
        }

        // Constraint + collision iterations (interleaved)
        let current_tension = self.current_tension;
        let mut collision_pairs = collision_pairs;
        for iter in 0..effective_iters {
            for bi in 0..self.bands.len() {
                if !self.bands[bi].active || self.bands[bi].fade_out != 0.0 {
                    continue;
                }
                self.band_constraints(bi, current_tension, dt);
            }
            if collide {
                if iter > 0 && iter % 3 == 0 {
                    collision_pairs = self.build_collision_pairs(&active);
                }
                self.resolve_collision_pairs(&collision_pairs, false);
            }
        }

        // Post-solve: collision-only passes
        if collide {
            for _ in 0..3 {
                let had_collision = self.resolve_collision_pairs(&collision_pairs, true);
                for &bi in &active {
                    let n = self.bands[bi].positions.len();
                    if let Some(start_hole) = self.bands[bi].pin_start {
                        let hp = self.hole_position_3d(start_hole);
                        self.bands[bi].positions[0] = hp;
                        self.bands[bi].previous_positions[0] = hp;
                    }
                    if let Some(end_hole) = self.bands[bi].pin_end {
                        let hp = self.hole_position_3d(end_hole);
                        self.bands[bi].positions[n - 1] = hp;
                        self.bands[bi].previous_positions[n - 1] = hp;
                    }
                    if self.bands[bi].cross_section.is_rectangular()
                        && bi < self.cached_frames.len()
                        && self.cached_frames[bi].len() == n
                    {
                        let cs = self.bands[bi].cross_section;
                        let up_n = Vec3::new(0.0, 0.0, 1.0);
                        for i in 1..(n - 1) {
                            let frame = self.cached_frames[bi][i];
                            let z_extent = cs.effective_radius(up_n, frame.d1, frame.d2);
                            let floor_z = self.board_surface_z(
                                self.bands[bi].positions[i].x,
                                self.bands[bi].positions[i].y,
                            ) + z_extent;
                            if self.bands[bi].positions[i].z < floor_z {
                                self.bands[bi].positions[i].z = floor_z;
                            }
                        }
                    } else {
                        let r = self.bands[bi].radius;
                        for i in 1..(n - 1) {
                            let floor_z = self.board_surface_z(
                                self.bands[bi].positions[i].x,
                                self.bands[bi].positions[i].y,
                            ) + r;
                            if self.bands[bi].positions[i].z < floor_z {
                                self.bands[bi].positions[i].z = floor_z;
                            }
                        }
                    }
                }
                if !had_collision {
                    break;
                }
            }
        }

        // Board friction
        let board_mu = self.friction_coefficient * 0.5;
        if board_mu > 0.0 {
            for &bi in &active {
                let n = self.bands[bi].positions.len();
                let r = self.bands[bi].radius;
                for i in 1..(n - 1) {
                    let floor_z = self.board_surface_z(
                        self.bands[bi].positions[i].x,
                        self.bands[bi].positions[i].y,
                    ) + r;
                    if self.bands[bi].positions[i].z <= floor_z + 1e-4 {
                        let vx =
                            self.bands[bi].positions[i].x - self.bands[bi].previous_positions[i].x;
                        let vy =
                            self.bands[bi].positions[i].y - self.bands[bi].previous_positions[i].y;
                        let vel_len = (vx * vx + vy * vy).sqrt();
                        if vel_len > 1e-8 {
                            let scale = (1.0 - board_mu).max(0.0);
                            self.bands[bi].previous_positions[i].x =
                                self.bands[bi].positions[i].x - vx * scale;
                            self.bands[bi].previous_positions[i].y =
                                self.bands[bi].positions[i].y - vy * scale;
                        }
                    }
                }
            }
        }
    }

    // --- Constraints ---

    fn band_constraints(&mut self, bi: usize, current_tension: f32, dt: f32) {
        let n = self.bands[bi].positions.len();
        let seg_len = self.bands[bi].segment_length * current_tension;
        let alpha = self.bend_compliance.max(0.0) / (dt * dt).max(1e-8);
        let bend_coupling = self.bend_velocity_coupling.clamp(0.0, 1.0);
        let pin_s = self.bands[bi].pin_start;
        let pin_e = self.bands[bi].pin_end;
        let hole_s = pin_s.map(|h| self.hole_position_3d(h));
        let hole_e = pin_e.map(|h| self.hole_position_3d(h));
        let cs = self.bands[bi].cross_section;
        let is_rect = cs.is_rectangular();
        let r = self.bands[bi].radius;
        let frames: Vec<MaterialFrame> =
            if is_rect && bi < self.cached_frames.len() && self.cached_frames[bi].len() == n {
                self.cached_frames[bi].clone()
            } else {
                Vec::new()
            };
        let mut prev = self.bands[bi].previous_positions.clone();

        for offset in 0..=1u32 {
            let mut idx = offset as usize;
            while idx < n - 1 {
                let diff = self.bands[bi].positions[idx + 1] - self.bands[bi].positions[idx];
                let dist2 = diff.dot(diff);
                if dist2 > 1e-12 {
                    let dist = dist2.sqrt();
                    let corr = diff * ((dist - seg_len) / dist * 0.5);
                    if idx > 0 {
                        self.bands[bi].positions[idx] += corr;
                    }
                    if idx + 1 < n - 1 {
                        self.bands[bi].positions[idx + 1] -= corr;
                    }
                }
                idx += 2;
            }
        }

        if n >= 3 {
            let denom = 6.0 + alpha;
            for i in 1..(n - 1) {
                let p0 = self.bands[bi].positions[i - 1];
                let p1 = self.bands[bi].positions[i];
                let p2 = self.bands[bi].positions[i + 1];
                let curvature = p0 - p1 * 2.0 + p2;
                if curvature.dot(curvature) < 1e-14 {
                    continue;
                }
                let lambda = curvature / denom;
                let mut d0 = -lambda;
                let mut d1 = lambda * 2.0;
                let mut d2 = -lambda;

                if i - 1 == 0 {
                    d1 += d0 * 0.5;
                    d2 += d0 * 0.5;
                    d0 = Vec3::ZERO;
                }
                if i + 1 == n - 1 {
                    d0 += d2 * 0.5;
                    d1 += d2 * 0.5;
                    d2 = Vec3::ZERO;
                }

                self.bands[bi].positions[i - 1] += d0;
                self.bands[bi].positions[i] += d1;
                self.bands[bi].positions[i + 1] += d2;

                prev[i - 1] += d0 * bend_coupling;
                prev[i] += d1 * bend_coupling;
                prev[i + 1] += d2 * bend_coupling;
            }
        }

        if let Some(hp) = hole_s {
            self.bands[bi].positions[0] = hp;
        }
        if let Some(hp) = hole_e {
            self.bands[bi].positions[n - 1] = hp;
        }

        // Board collision
        if !frames.is_empty() {
            let up_n = Vec3::new(0.0, 0.0, 1.0);
            for i in 1..(n - 1) {
                let frame = frames[i];
                let z_extent = cs.effective_radius(up_n, frame.d1, frame.d2);
                let floor_z = self
                    .board_surface_z(self.bands[bi].positions[i].x, self.bands[bi].positions[i].y)
                    + z_extent;
                if self.bands[bi].positions[i].z < floor_z {
                    self.bands[bi].positions[i].z = floor_z;
                }
            }
        } else {
            for i in 1..(n - 1) {
                let floor_z = self
                    .board_surface_z(self.bands[bi].positions[i].x, self.bands[bi].positions[i].y)
                    + r;
                if self.bands[bi].positions[i].z < floor_z {
                    self.bands[bi].positions[i].z = floor_z;
                }
            }
        }

        // Twist stiffness for rectangular
        if is_rect {
            let stiffness = self.twist_stiffness;
            for i in 0..(n - 1) {
                let diff = self.bands[bi].twist_angles[i + 1] - self.bands[bi].twist_angles[i];
                let corr = diff * stiffness * 0.5;
                if i > 0 {
                    self.bands[bi].twist_angles[i] += corr;
                }
                if i + 1 < n - 1 {
                    self.bands[bi].twist_angles[i + 1] -= corr;
                }
            }
            let zero_restoring = 0.15;
            for i in 1..(n - 1) {
                self.bands[bi].twist_angles[i] *= 1.0 - zero_restoring;
            }
        }
        self.bands[bi].previous_positions = prev;
    }

    // --- Collision ---

    pub fn build_collision_pairs(&self, active_bands: &[usize]) -> Vec<CollisionPair> {
        if active_bands.is_empty() {
            return Vec::new();
        }
        let mut pairs: Vec<CollisionPair> = Vec::with_capacity(512);

        for ai in 0..active_bands.len() {
            let bi = active_bands[ai];
            let pos_i = &self.bands[bi].positions;
            let segs_i = pos_i.len() - 1;
            let ri = self.bands[bi].cross_section.collision_radius();

            if segs_i > 4 {
                let min_dist = ri + ri;
                let seg_len = self.bands[bi].segment_length;
                let self_skip = if seg_len > 1e-6 {
                    4.max(((min_dist * 2.5) / seg_len).ceil() as usize)
                } else {
                    4
                };
                for si in 0..segs_i {
                    let a0 = pos_i[si];
                    let a1 = pos_i[si + 1];
                    let a_min_x = a0.x.min(a1.x) - min_dist;
                    let a_max_x = a0.x.max(a1.x) + min_dist;
                    let a_min_y = a0.y.min(a1.y) - min_dist;
                    let a_max_y = a0.y.max(a1.y) + min_dist;
                    let sj_start = si + self_skip;
                    if sj_start >= segs_i {
                        continue;
                    }
                    for sj in sj_start..segs_i {
                        let b0 = pos_i[sj];
                        let b1 = pos_i[sj + 1];
                        if b0.x.max(b1.x) < a_min_x || b0.x.min(b1.x) > a_max_x {
                            continue;
                        }
                        if b0.y.max(b1.y) < a_min_y || b0.y.min(b1.y) > a_max_y {
                            continue;
                        }
                        pairs.push(CollisionPair {
                            band_a: bi as u16,
                            seg_a: si as u16,
                            band_b: bi as u16,
                            seg_b: sj as u16,
                        });
                    }
                }
            }

            for aj in (ai + 1)..active_bands.len() {
                let bj = active_bands[aj];
                let pos_j = &self.bands[bj].positions;
                let segs_j = pos_j.len() - 1;
                let min_dist = ri + self.bands[bj].cross_section.collision_radius();

                for si in 0..segs_i {
                    let a0 = pos_i[si];
                    let a1 = pos_i[si + 1];
                    let a_min_x = a0.x.min(a1.x) - min_dist;
                    let a_max_x = a0.x.max(a1.x) + min_dist;
                    let a_min_y = a0.y.min(a1.y) - min_dist;
                    let a_max_y = a0.y.max(a1.y) + min_dist;

                    for sj in 0..segs_j {
                        let b0 = pos_j[sj];
                        let b1 = pos_j[sj + 1];
                        if b0.x.max(b1.x) < a_min_x || b0.x.min(b1.x) > a_max_x {
                            continue;
                        }
                        if b0.y.max(b1.y) < a_min_y || b0.y.min(b1.y) > a_max_y {
                            continue;
                        }
                        pairs.push(CollisionPair {
                            band_a: bi as u16,
                            seg_a: si as u16,
                            band_b: bj as u16,
                            seg_b: sj as u16,
                        });
                    }
                }
            }
        }
        pairs
    }

    fn resolve_collision_pairs(&mut self, pairs: &[CollisionPair], inject_velocity: bool) -> bool {
        let mut found = false;
        for p in pairs {
            if self.collide_segments(
                p.band_a as usize,
                p.seg_a as usize,
                p.band_b as usize,
                p.seg_b as usize,
                inject_velocity,
            ) {
                found = true;
            }
        }
        found
    }

    #[inline(always)]
    fn collide_segments(
        &mut self,
        bi: usize,
        si: usize,
        bj: usize,
        sj: usize,
        inject_velocity: bool,
    ) -> bool {
        let max_dist = self.bands[bi].cross_section.collision_radius()
            + self.bands[bj].cross_section.collision_radius();
        let max_dist2 = max_dist * max_dist;

        let a0 = self.bands[bi].positions[si];
        let a1 = self.bands[bi].positions[si + 1];
        let b0 = self.bands[bj].positions[sj];
        let b1 = self.bands[bj].positions[sj + 1];

        let d1 = a1 - a0;
        let d2 = b1 - b0;
        let r = a0 - b0;
        let a = d1.dot(d1);
        let e = d2.dot(d2);
        let f = d2.dot(r);
        let c = d1.dot(r);
        let b = d1.dot(d2);

        let denom = a * e - b * b;
        let mut s: f32 = 0.0;
        let mut t: f32;

        if denom > 1e-12 {
            s = ((b * f - c * e) / denom).clamp(0.0, 1.0);
        }
        t = (b * s + f) / e.max(1e-12);

        if t < 0.0 {
            t = 0.0;
            s = (-c / a.max(1e-12)).clamp(0.0, 1.0);
        } else if t > 1.0 {
            t = 1.0;
            s = ((b - c) / a.max(1e-12)).clamp(0.0, 1.0);
        }

        let closest_a = a0 + d1 * s;
        let closest_b = b0 + d2 * t;
        let diff = closest_a - closest_b;
        let dist2 = diff.dot(diff);

        if dist2 >= max_dist2 {
            return false;
        }

        let (dist, normal) = if dist2 > 1e-12 {
            let dist = dist2.sqrt();
            (dist, diff / dist)
        } else {
            let axis = d1.cross(d2);
            let axis_len2 = axis.length_squared();
            if axis_len2 > 1e-12 {
                (1e-6, axis.normalize())
            } else {
                let tangent = if d1.length_squared() > 1e-12 {
                    d1.normalize()
                } else {
                    Vec3::new(1.0, 0.0, 0.0)
                };
                let up = if tangent.z.abs() < 0.9 {
                    Vec3::new(0.0, 0.0, 1.0)
                } else {
                    Vec3::new(0.0, 1.0, 0.0)
                };
                (1e-6, tangent.cross(up).normalize())
            }
        };

        let eff_radius_a = self.effective_collision_radius(bi, si, s, normal);
        let eff_radius_b = self.effective_collision_radius(bj, sj, t, normal);
        let min_dist = eff_radius_a + eff_radius_b;

        if dist >= min_dist {
            return false;
        }

        let overlap = min_dist - dist;
        let corr = normal * (overlap * 0.35);

        self.bands[bi].positions[si] += corr * (1.0 - s);
        self.bands[bi].positions[si + 1] += corr * s;
        self.bands[bj].positions[sj] -= corr * (1.0 - t);
        self.bands[bj].positions[sj + 1] -= corr * t;

        // Coulomb friction
        let mu = self.friction_coefficient;
        if mu > 0.0 {
            let vel_a = (self.bands[bi].positions[si] - self.bands[bi].previous_positions[si])
                * (1.0 - s)
                + (self.bands[bi].positions[si + 1] - self.bands[bi].previous_positions[si + 1])
                    * s;
            let vel_b = (self.bands[bj].positions[sj] - self.bands[bj].previous_positions[sj])
                * (1.0 - t)
                + (self.bands[bj].positions[sj + 1] - self.bands[bj].previous_positions[sj + 1])
                    * t;
            let rel_vel = vel_a - vel_b;
            let tangent = rel_vel - normal * rel_vel.dot(normal);
            let tangent_len = tangent.length();
            let min_slide: f32 = 0.0002;
            if tangent_len > min_slide {
                let max_friction = mu * overlap * 0.25;
                let friction_mag = (tangent_len * 0.3).min(max_friction);
                let friction_dir = tangent / tangent_len;
                let vel_corr = friction_dir * friction_mag;

                self.bands[bi].previous_positions[si] += vel_corr * (1.0 - s);
                self.bands[bi].previous_positions[si + 1] += vel_corr * s;
                self.bands[bj].previous_positions[sj] -= vel_corr * (1.0 - t);
                self.bands[bj].previous_positions[sj + 1] -= vel_corr * t;
                let contact_pos = (closest_a + closest_b) * 0.5;
                self.friction_accumulator += overlap;
                self.friction_speed_accumulator += tangent_len;
                self.friction_position_accumulator += contact_pos;
                self.friction_sample_count += 1;
            }
        }

        if inject_velocity {
            let vel_corr = corr * 0.15;
            self.bands[bi].previous_positions[si] -= vel_corr * (1.0 - s);
            self.bands[bi].previous_positions[si + 1] -= vel_corr * s;
            self.bands[bj].previous_positions[sj] += vel_corr * (1.0 - t);
            self.bands[bj].previous_positions[sj + 1] += vel_corr * t;
        }
        true
    }

    #[inline(always)]
    fn effective_collision_radius(&self, bi: usize, si: usize, param_s: f32, normal: Vec3) -> f32 {
        let cs = self.bands[bi].cross_section;
        if !cs.is_rectangular() {
            return self.bands[bi].radius;
        }
        if bi >= self.cached_frames.len() || self.cached_frames[bi].len() <= si + 1 {
            return self.bands[bi].radius;
        }
        let f0 = self.cached_frames[bi][si];
        let f1 = self.cached_frames[bi][si + 1];
        let d1 = (f0.d1 * (1.0 - param_s) + f1.d1 * param_s).normalize();
        let d2 = (f0.d2 * (1.0 - param_s) + f1.d2 * param_s).normalize();
        cs.effective_radius(normal, d1, d2)
    }

    #[inline(always)]
    fn latex_thinning_factor(&self, bi: usize, si: usize, param_s: f32) -> f32 {
        let band = &self.bands[bi];
        let n = band.positions.len();
        if n < 2 {
            return 1.0;
        }

        let seg_len = band.segment_length;
        if seg_len <= 1e-6 {
            return 1.0;
        }

        let actual_len = (band.positions[si + 1] - band.positions[si]).length();
        let local_stretch = (actual_len / seg_len - 1.0).max(0.0);

        let particle_t = (si as f32 + param_s) / (n - 1) as f32;
        let center = (particle_t * std::f32::consts::PI).sin();
        let center_mask = center * center * center * center;

        let tension = local_stretch * center_mask;
        1.0 / (1.0 + tension * 1.5).max(1.0).sqrt()
    }

    // --- Drag ---

    pub fn begin_drag(&mut self, band_index: usize, end_index: usize, world_position: Vec2) {
        if band_index >= self.bands.len() {
            return;
        }
        self.wake_up();
        let lower_animation_key = LowerAnimationKey {
            band_index,
            end_index,
        };
        let original_hole: usize;
        if end_index == 0 {
            original_hole = self.bands[band_index]
                .pin_start
                .or_else(|| {
                    self.lower_animations
                        .get(&lower_animation_key)
                        .map(|anim| anim.target_hole)
                })
                .unwrap_or(0);
            self.bands[band_index].pin_start = None;
        } else {
            original_hole = self.bands[band_index]
                .pin_end
                .or_else(|| {
                    self.lower_animations
                        .get(&lower_animation_key)
                        .map(|anim| anim.target_hole)
                })
                .unwrap_or(0);
            self.bands[band_index].pin_end = None;
        }
        self.drag_info = Some(DragInfo {
            band_index,
            end_index,
            original_hole_index: original_hole,
        });

        self.lower_animations.remove(&lower_animation_key);

        let idx = if end_index == 0 {
            0
        } else {
            self.bands[band_index].positions.len() - 1
        };
        let elev = self.hole_surface_z(original_hole);
        let lift_pos = Vec3::new(world_position.x, world_position.y, elev + self.lift_height);
        self.bands[band_index].positions[idx] = lift_pos;
        self.bands[band_index].previous_positions[idx] = lift_pos;
        self.drag_start_pos = Some(lift_pos);
        self.drag_target_pos = Some(lift_pos);
    }

    pub fn update_drag(&mut self, world_position: Vec2) {
        let (bi, end_index) = match &self.drag_info {
            Some(d) => (d.band_index, d.end_index),
            None => return,
        };
        let n = self.bands[bi].positions.len();
        let idx = if end_index == 0 { 0 } else { n - 1 };
        self.drag_start_pos = Some(self.bands[bi].positions[idx]);
        let surf_z = self.board_surface_z(world_position.x, world_position.y);
        self.drag_target_pos = Some(Vec3::new(
            world_position.x,
            world_position.y,
            surf_z + self.lift_height,
        ));
    }

    pub fn end_drag(&mut self, target_hole_index: usize) {
        let drag = match self.drag_info.take() {
            Some(d) => d,
            None => return,
        };

        let idx = if drag.end_index == 0 {
            0
        } else {
            self.bands[drag.band_index].positions.len() - 1
        };
        let current_pos = self.bands[drag.band_index].positions[idx];
        let hole_xy = self.hole_positions[target_hole_index];
        let hole_elev = self.hole_surface_z(target_hole_index);
        let above_hole = Vec3::new(hole_xy.x, hole_xy.y, hole_elev + self.lift_height);
        let dist = (current_pos - above_hole).length();
        let return_duration = (dist * 0.9).clamp(0.25, 0.8);

        let lower_animation_key = LowerAnimationKey {
            band_index: drag.band_index,
            end_index: drag.end_index,
        };
        self.lower_animations.insert(
            lower_animation_key,
            LowerAnimation {
                band_index: drag.band_index,
                end_index: drag.end_index,
                target_hole: target_hole_index,
                start_pos: current_pos,
                timer: 0.0,
                return_pos: Some(above_hole),
                return_duration,
            },
        );

        self.drag_start_pos = None;
        self.drag_target_pos = None;
    }

    pub fn cancel_drag(&mut self) {
        let drag = match self.drag_info.as_ref() {
            Some(d) => d,
            None => return,
        };
        self.end_drag(drag.original_hole_index);
    }

    pub fn start_fade_out(
        &mut self,
        band_index: usize,
        fallback_start_hole: Option<usize>,
        fallback_end_hole: Option<usize>,
    ) {
        if band_index >= self.bands.len() {
            return;
        }
        let band = &mut self.bands[band_index];
        let hole_s = band.pin_start.or(fallback_start_hole);
        let hole_e = band.pin_end.or(fallback_end_hole);
        let (suck_target, suck_from_end) = if let Some(pin_start) = hole_s {
            (pin_start, 1usize)
        } else if let Some(pin_end) = hole_e {
            (pin_end, 0usize)
        } else {
            (0usize, 1usize)
        };
        let tail_hole = if suck_from_end == 1 { hole_e } else { hole_s };

        band.suck_hole = Some(suck_target);
        band.suck_tail_hole = tail_hole;
        band.suck_from_end = suck_from_end;
        band.suck_consumed = 0.0;
        band.suck_frame = 0;

        let mut segs = Vec::with_capacity(band.positions.len().saturating_sub(1));
        for k in 0..band.positions.len().saturating_sub(1) {
            segs.push((band.positions[k + 1] - band.positions[k]).length());
        }
        band.suck_seg_lengths = segs;
        band.suck_orig_positions = band.positions.clone();
        band.fade_out = 0.001;
        band.pin_start = None;
        band.pin_end = None;
    }

    pub fn endpoint_z(&self, band_index: usize, end_index: usize) -> f32 {
        if band_index >= self.bands.len() {
            return 0.0;
        }
        let idx = if end_index == 0 {
            0
        } else {
            self.bands[band_index].positions.len() - 1
        };
        self.bands[band_index].positions[idx].z
    }

    // --- Level initialization ---

    pub fn initialize_level(&mut self, rope_configs: &[RopeConfig], actions: &[LevelAction]) {
        self.bands.clear();
        self.current_tension = self.rope_tension;

        for config in rope_configs {
            self.add_band(
                config.radius,
                config.cross_section,
                Some(self.particle_count),
            );
        }

        if actions.is_empty() {
            for (i, config) in rope_configs.iter().enumerate() {
                self.pin(i, config.start_hole, config.end_hole);
            }
            return;
        }

        for action in actions {
            match action.action_type {
                ActionType::Pin => {
                    if action.end_index == 0 {
                        self.bands[action.rope_index].pin_start = Some(action.hole_index);
                    } else {
                        self.bands[action.rope_index].pin_end = Some(action.hole_index);
                    }
                    if self.bands[action.rope_index].pin_start.is_some()
                        && self.bands[action.rope_index].pin_end.is_some()
                    {
                        self.pin_and_settle(action.rope_index);
                    }
                }
                ActionType::Drag => {
                    self.simulate_drag(action.rope_index, action.end_index, action.hole_index);
                }
            }
        }
    }

    // --- Simulation helpers ---

    fn pin_and_settle(&mut self, band_index: usize) {
        let start_hole = match self.bands[band_index].pin_start {
            Some(h) => h,
            None => return,
        };
        let end_hole = match self.bands[band_index].pin_end {
            Some(h) => h,
            None => return,
        };

        let p0 = self.hole_position_3d(start_hole);
        let p1 = self.hole_position_3d(end_hole);
        let n = self.bands[band_index].positions.len();
        let lift = self.lift_height;

        for i in 0..n {
            let t = i as f32 / (n - 1).max(1) as f32;
            let x = p0.x * (1.0 - t) + p1.x * t;
            let y = p0.y * (1.0 - t) + p1.y * t;
            self.bands[band_index].positions[i] = Vec3::new(x, y, lift);
        }
        self.bands[band_index].positions[0] = p0;
        self.bands[band_index].positions[n - 1] = p1;
        self.bands[band_index].previous_positions = self.bands[band_index].positions.clone();

        let dist = (p1 - p0).length();
        self.bands[band_index].segment_length = dist / (n - 1).max(1) as f32;
        self.bands[band_index].active = true;

        let has_others = self
            .bands
            .iter()
            .enumerate()
            .any(|(idx, b)| idx != band_index && b.active && b.pin_start.is_some());
        let steps = self.settle_steps;
        self.do_steps(steps, has_others);
    }

    fn simulate_drag(&mut self, band_index: usize, end_index: usize, to_hole: usize) {
        let idx = if end_index == 0 {
            0
        } else {
            self.bands[band_index].positions.len() - 1
        };
        let from_pos = self.bands[band_index].positions[idx];
        let to_pos = self.hole_position_3d(to_hole);

        if end_index == 0 {
            self.bands[band_index].pin_start = None;
        } else {
            self.bands[band_index].pin_end = None;
        }

        let from_elev = self.board_surface_z(from_pos.x, from_pos.y);
        let to_elev = self.hole_surface_z(to_hole);
        let max_elev = from_elev.max(to_elev);
        let lift_from = Vec3::new(from_pos.x, from_pos.y, max_elev + self.lift_height);
        let lift_to = Vec3::new(to_pos.x, to_pos.y, max_elev + self.lift_height);

        let drag_steps = 4;

        // Lift
        for s in 1..=3 {
            let t = s as f32 / 3.0;
            self.bands[band_index].positions[idx] = from_pos + (lift_from - from_pos) * t;
            self.bands[band_index].previous_positions[idx] = self.bands[band_index].positions[idx];
            self.do_steps(drag_steps, true);
        }

        // Traverse
        let traverse_steps = 12;
        for s in 1..=traverse_steps {
            let t = s as f32 / traverse_steps as f32;
            self.bands[band_index].positions[idx] = lift_from + (lift_to - lift_from) * t;
            self.bands[band_index].previous_positions[idx] = self.bands[band_index].positions[idx];
            self.do_steps(drag_steps, true);
        }

        // Lower
        for s in 1..=3 {
            let t = s as f32 / 3.0;
            self.bands[band_index].positions[idx] = lift_to + (to_pos - lift_to) * t;
            self.bands[band_index].previous_positions[idx] = self.bands[band_index].positions[idx];
            self.do_steps(drag_steps, true);
        }

        if end_index == 0 {
            self.bands[band_index].pin_start = Some(to_hole);
        } else {
            self.bands[band_index].pin_end = Some(to_hole);
        }
        self.bands[band_index].positions[idx] = to_pos;
        self.bands[band_index].previous_positions[idx] = to_pos;

        let steps = self.settle_steps;
        self.do_steps(steps, true);
    }

    // --- Utility ---

    pub fn hole_position_3d(&self, hole_index: usize) -> Vec3 {
        if hole_index >= self.hole_positions.len() {
            return Vec3::ZERO;
        }
        let p = self.hole_positions[hole_index];
        let z = self.hole_elevations.get(hole_index).copied().unwrap_or(0.0) - self.hole_depth;
        Vec3::new(p.x, p.y, z)
    }

    pub fn hole_surface_z(&self, hole_index: usize) -> f32 {
        self.hole_elevations.get(hole_index).copied().unwrap_or(0.0)
    }

    pub fn board_surface_z(&self, x: f32, y: f32) -> f32 {
        let mut max_z: f32 = 0.0;
        for board in &self.boards {
            let half_w = board.width * 0.5;
            let half_h = board.height * 0.5;
            if x >= board.center_x - half_w
                && x <= board.center_x + half_w
                && y >= board.center_y - half_h
                && y <= board.center_y + half_h
            {
                max_z = max_z.max(board.elevation);
            }
        }
        max_z
    }

    // --- Material frame computation ---

    pub fn compute_frames(positions: &[Vec3], twist_angles: &[f32]) -> Vec<MaterialFrame> {
        let n = positions.len();
        if n < 2 {
            return Vec::new();
        }

        let zero_frame = MaterialFrame {
            tangent: Vec3::ZERO,
            d1: Vec3::ZERO,
            d2: Vec3::ZERO,
        };
        let mut frames = vec![zero_frame; n];
        let up = Vec3::new(0.0, 0.0, 1.0);

        let mut t_prev = (positions[1] - positions[0]).normalize();
        let mut u_prev = {
            let mut u = up.cross(t_prev);
            if u.length_squared() < 1e-8 {
                u = Vec3::new(1.0, 0.0, 0.0);
            }
            u.normalize()
        };

        for i in 0..n {
            let tangent = if i == 0 {
                (positions[1] - positions[0]).normalize()
            } else if i == n - 1 {
                (positions[n - 1] - positions[n - 2]).normalize()
            } else {
                (positions[i + 1] - positions[i - 1]).normalize()
            };

            let mut u = u_prev;
            if i > 0 {
                let axis = t_prev.cross(tangent);
                let axis_len = axis.length();
                if axis_len > 1e-6 {
                    let axis_n = axis / axis_len;
                    let dot_clamped = t_prev.dot(tangent).clamp(-1.0, 1.0);
                    let angle = axis_len.atan2(dot_clamped);
                    u = Self::rotate_vector(u, axis_n, angle);
                    let proj = u - tangent * u.dot(tangent);
                    if proj.length_squared() > 1e-10 {
                        u = proj.normalize();
                    }
                }
            }

            let mut v = tangent.cross(u);
            if v.length_squared() < 1e-8 {
                let mut fallback = up.cross(tangent);
                if fallback.length_squared() < 1e-8 {
                    fallback = Vec3::new(1.0, 0.0, 0.0);
                }
                u = fallback.normalize();
                v = tangent.cross(u);
            }
            v = v.normalize();

            let twist = twist_angles[i];
            let cos_t = twist.cos();
            let sin_t = twist.sin();
            let d1 = u * cos_t + v * sin_t;
            let d2 = -u * sin_t + v * cos_t;

            frames[i] = MaterialFrame { tangent, d1, d2 };

            t_prev = tangent;
            u_prev = u;
        }

        frames
    }

    pub fn rotate_vector(vector: Vec3, axis: Vec3, angle: f32) -> Vec3 {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        vector * cos_a + axis.cross(vector) * sin_a + axis * axis.dot(vector) * (1.0 - cos_a)
    }

    pub fn recompute_frames(&mut self) {
        if self.cached_frames.len() != self.bands.len() {
            self.cached_frames = vec![Vec::new(); self.bands.len()];
        }
        for bi in 0..self.bands.len() {
            if !self.bands[bi].active || self.bands[bi].fade_out != 0.0 {
                self.cached_frames[bi] = Vec::new();
                continue;
            }
            if self.bands[bi].cross_section.is_rectangular() {
                self.cached_frames[bi] =
                    Self::compute_frames(&self.bands[bi].positions, &self.bands[bi].twist_angles);
            } else if self.square_cross_section {
                self.cached_frames[bi] = Self::compute_frames(
                    &self.bands[bi].positions,
                    &vec![0.0; self.bands[bi].positions.len()],
                );
            } else {
                self.cached_frames[bi] = Vec::new();
            }
        }
    }
}
