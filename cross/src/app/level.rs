use uzls_cross::level::generator;
use uzls_cross::simulation::verlet::{BoardDef, VerletSimulator};
use uzls_cross::storage::save_level_to_storage;

use super::{fit_camera, App, UndoEntry};

impl App {
    pub fn load_level(&mut self, level_id: usize) {
        self.current_level_id = level_id;
        self.drag_state = None;
        self.undo_stack.clear();
        save_level_to_storage(level_id);

        let level = uzls_cross::level::loader::load_embedded(level_id)
            .unwrap_or_else(|| generator::generate(level_id as u32));

        let hole_positions = level.hole_positions();
        let hole_elevations = level.hole_elevations();
        let board_defs: Vec<BoardDef> = level
            .boards
            .as_ref()
            .map(|boards| boards.iter().map(BoardDef::from).collect())
            .unwrap_or_default();
        let mut sim = VerletSimulator::new(
            hole_positions.clone(),
            hole_elevations,
            level.hole_radius,
            board_defs,
        );
        sim.gravity = -6.3927;
        sim.damping = 0.9470582;
        sim.constraint_iterations = 11;
        sim.particle_count = 65;
        sim.settle_steps = 5;
        sim.lift_height = 0.3;
        sim.rope_tension = 0.88084733;
        sim.bend_compliance = 0.00015530341;
        sim.bend_velocity_coupling = 0.65;
        sim.square_cross_section = self.square_cross_section;

        let rope_configs: Vec<_> = level
            .ropes
            .iter()
            .map(|r| uzls_cross::simulation::verlet::RopeConfig {
                start_hole: r.start_hole,
                end_hole: r.end_hole,
                radius: r.radius,
                cross_section: Some(r.cross_section()),
            })
            .collect();

        let actions: Vec<_> = level
            .actions
            .as_ref()
            .map(|acts| {
                acts.iter()
                    .filter_map(|a| {
                        let action_type = match a.kind.as_str() {
                            "pin" => uzls_cross::simulation::verlet::ActionType::Pin,
                            "drag" => uzls_cross::simulation::verlet::ActionType::Drag,
                            _ => return None,
                        };
                        Some(uzls_cross::simulation::verlet::LevelAction {
                            action_type,
                            rope_index: a.rope_index,
                            end_index: a.end_index,
                            hole_index: a.hole_index,
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();

        sim.initialize_level(&rope_configs, &actions);

        self.rope_endpoints = level
            .ropes
            .iter()
            .enumerate()
            .map(|(i, r)| {
                let start = sim
                    .bands
                    .get(i)
                    .and_then(|b| b.pin_start)
                    .unwrap_or(r.start_hole);
                let end = sim
                    .bands
                    .get(i)
                    .and_then(|b| b.pin_end)
                    .unwrap_or(r.end_hole);
                (start, end)
            })
            .collect();

        self.rope_colors = level
            .ropes
            .iter()
            .map(|r| {
                [
                    r.color.red_channel,
                    r.color.green_channel,
                    r.color.blue_channel,
                ]
            })
            .collect();
        self.rope_radii = level.ropes.iter().map(|r| r.radius).collect();

        self.hole_occupied = vec![false; hole_positions.len()];
        for &(s, e) in &self.rope_endpoints {
            if s < self.hole_occupied.len() {
                self.hole_occupied[s] = true;
            }
            if e < self.hole_occupied.len() {
                self.hole_occupied[e] = true;
            }
        }

        if let Some(renderer) = &mut self.renderer {
            let hole_elevations_for_render: Vec<f32> =
                level.holes.iter().map(|h| h.z_position).collect();
            let scaled_hole_radius = level.hole_radius * renderer.visual.hole_radius_scale;
            if self.square_cross_section {
                renderer.update_hole_instances_square(
                    &hole_positions,
                    &hole_elevations_for_render,
                    scaled_hole_radius,
                );
            } else {
                renderer.update_hole_instances(
                    &hole_positions,
                    &hole_elevations_for_render,
                    scaled_hole_radius,
                );
            }
            let boards: Vec<(glam::Vec2, glam::Vec2, f32)> = level
                .boards
                .as_ref()
                .map(|items| {
                    items
                        .iter()
                        .map(|b| {
                            (
                                glam::Vec2::new(b.center_x, b.center_y),
                                glam::Vec2::new(b.width, b.height),
                                b.elevation,
                            )
                        })
                        .collect()
                })
                .unwrap_or_default();
            renderer.update_board_mesh(&boards);

            let (sw, sh) = renderer.surface_size();
            let aspect = sw as f32 / sh.max(1) as f32;
            let max_elevation = level
                .holes
                .iter()
                .map(|h| h.z_position)
                .fold(0.0_f32, f32::max);
            fit_camera(
                &mut renderer.camera,
                &hole_positions,
                level.hole_radius,
                aspect,
                max_elevation,
            );
            renderer.needs_wood_bake = true;
        }

        self.min_moves = uzls_cross::level::solver::min_moves_for_level(level_id);

        self.simulator = Some(sim);
        self.level = Some(level);
        self.settle_check_timer = None;
        self.next_level_timer = None;
        self.victory_time = 0.0;
        self.victory_sound_played = false;
        self.celebration_active = false;
        self.celebration_bands.clear();
        self.move_count = 0;
        self.level_start_ms = web_time::Instant::now().elapsed().as_millis() as u64;
        self.victory_submitted = false;
        self.leaderboard.reset_result();
        if let Some(snap) = self.pre_victory_camera.take() {
            if let Some(r) = &mut self.renderer {
                snap.restore(&mut r.camera);
            }
        }
    }

    pub fn push_undo_state(&mut self) {
        if let Some(sim) = &self.simulator {
            self.undo_stack.push(UndoEntry {
                simulator_snapshot: sim.take_snapshot(),
                rope_endpoints: self.rope_endpoints.clone(),
                hole_occupied: self.hole_occupied.clone(),
            });
        }
    }

    pub fn perform_undo(&mut self) {
        if let Some(entry) = self.undo_stack.pop() {
            if let Some(sim) = &mut self.simulator {
                sim.restore_snapshot(&entry.simulator_snapshot);
            }
            self.rope_endpoints = entry.rope_endpoints;
            self.hole_occupied = entry.hole_occupied;
            self.move_count = self.move_count.saturating_sub(1);
            self.drag_state = None;
            self.highlight_hole = -1;
            self.settle_check_timer = None;
        }
    }
}
