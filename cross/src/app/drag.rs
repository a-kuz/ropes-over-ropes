use uzls_cross::input as game_input;
use uzls_cross::input::DragResult;

use super::App;

impl App {
    pub fn try_begin_drag(&mut self, screen_pos: (f32, f32)) {
        let result = {
            let renderer = match &self.renderer {
                Some(r) => r,
                None => return,
            };
            let level = match &self.level {
                Some(l) => l,
                None => return,
            };
            let sim = match &self.simulator {
                Some(s) => s,
                None => return,
            };

            let vp = (
                renderer.surface_size().0 as f32,
                renderer.surface_size().1 as f32,
            );
            let hole_positions = level.hole_positions();
            let hole_radius = level.hole_radius;
            let endpoint_z = |rope: usize, end: usize| -> f32 { sim.endpoint_z(rope, end) };

            game_input::begin_drag_action(
                screen_pos,
                vp,
                &renderer.camera,
                &hole_positions,
                &self.rope_endpoints,
                &endpoint_z,
                hole_radius,
            )
        };

        if let Some((drag, hole_to_free)) = result {
            self.push_undo_state();
            if hole_to_free < self.hole_occupied.len() {
                self.hole_occupied[hole_to_free] = false;
            }
            if let Some(renderer) = &self.renderer {
                let vp = (
                    renderer.surface_size().0 as f32,
                    renderer.surface_size().1 as f32,
                );
                let world = game_input::screen_to_world(screen_pos, vp, &renderer.camera);
                if let Some(sim) = &mut self.simulator {
                    sim.begin_drag(drag.rope_index, drag.end_index, world);
                }
            }
            self.drag_state = Some(drag);
            self.prof_events.push("drag_start");
        }
    }

    pub fn finish_drag(&mut self, screen_pos: (f32, f32)) {
        self.highlight_hole = -1;
        let drag = match self.drag_state.take() {
            Some(d) => d,
            None => return,
        };
        let renderer = match &self.renderer {
            Some(r) => r,
            None => return,
        };
        let level = match &self.level {
            Some(l) => l,
            None => return,
        };

        let vp = (
            renderer.surface_size().0 as f32,
            renderer.surface_size().1 as f32,
        );
        let hole_positions = level.hole_positions();

        match game_input::end_drag_action(
            &drag,
            screen_pos,
            vp,
            &renderer.camera,
            &hole_positions,
            &self.hole_occupied,
            level.hole_radius,
        ) {
            DragResult::Snapped { hole, moved } => {
                let can_write_back = self.simulator.as_ref().map_or(false, |sim| {
                    sim.bands
                        .get(drag.rope_index)
                        .map_or(false, |band| band.active && band.fade_out == 0.0)
                });
                if !can_write_back {
                    self.settle_check_timer = Some(0.5);
                    return;
                }
                if let Some(sim) = &mut self.simulator {
                    sim.end_drag(hole);
                }
                if drag.end_index == 0 {
                    self.rope_endpoints[drag.rope_index].0 = hole;
                } else {
                    self.rope_endpoints[drag.rope_index].1 = hole;
                }
                if hole < self.hole_occupied.len() {
                    self.hole_occupied[hole] = true;
                }
                if moved {
                    self.move_count += 1;
                    if let Some(audio) = &self.audio {
                        audio.play_snap();
                    }
                }
                self.settle_check_timer = Some(0.5);
                self.prof_events.push("drag_snap");
            }
            DragResult::Cancelled => {
                let can_write_back = self.simulator.as_ref().map_or(false, |sim| {
                    sim.bands
                        .get(drag.rope_index)
                        .map_or(false, |band| band.active && band.fade_out == 0.0)
                });
                if !can_write_back {
                    self.undo_stack.pop();
                    self.settle_check_timer = Some(0.5);
                    return;
                }
                if let Some(sim) = &mut self.simulator {
                    sim.cancel_drag();
                }
                if drag.end_index == 0 {
                    self.rope_endpoints[drag.rope_index].0 = drag.original_hole_index;
                } else {
                    self.rope_endpoints[drag.rope_index].1 = drag.original_hole_index;
                }
                if drag.original_hole_index < self.hole_occupied.len() {
                    self.hole_occupied[drag.original_hole_index] = true;
                }
                self.undo_stack.pop();
                self.settle_check_timer = Some(0.5);
                self.prof_events.push("drag_cancel");
            }
        }
    }

    pub fn cancel_drag(&mut self) {
        let drag = match self.drag_state.take() {
            Some(d) => d,
            None => return,
        };
        self.highlight_hole = -1;
        let can_write_back = self.simulator.as_ref().map_or(false, |sim| {
            sim.bands
                .get(drag.rope_index)
                .map_or(false, |band| band.active && band.fade_out == 0.0)
        });
        if !can_write_back {
            self.undo_stack.pop();
            self.settle_check_timer = Some(0.5);
            return;
        }
        if let Some(sim) = &mut self.simulator {
            sim.cancel_drag();
        }
        if drag.end_index == 0 {
            self.rope_endpoints[drag.rope_index].0 = drag.original_hole_index;
        } else {
            self.rope_endpoints[drag.rope_index].1 = drag.original_hole_index;
        }
        if drag.original_hole_index < self.hole_occupied.len() {
            self.hole_occupied[drag.original_hole_index] = true;
        }
        self.undo_stack.pop();
    }

    pub fn update_drag_highlight(&mut self, screen_pos: (f32, f32)) {
        if self.drag_state.is_none() {
            return;
        }
        let renderer = match &self.renderer {
            Some(r) => r,
            None => return,
        };
        let vp = (
            renderer.surface_size().0 as f32,
            renderer.surface_size().1 as f32,
        );
        let world = game_input::screen_to_world(screen_pos, vp, &renderer.camera);
        if let Some(sim) = &mut self.simulator {
            sim.update_drag(world);
        }
        if let Some(level) = &self.level {
            let hole_positions = level.hole_positions();
            self.highlight_hole = game_input::find_snap_hole(
                world,
                &hole_positions,
                &self.hole_occupied,
                level.hole_radius,
            )
            .map(|i| i as i32)
            .unwrap_or(-1);
        }
    }
}
