use std::sync::Arc;
use web_time::Instant;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::window::{Window, WindowId};

use uzls_cross::level::definition::LevelDefinition;
use uzls_cross::level::generator;
use uzls_cross::renderer::frame_types::RopeVertex;
use uzls_cross::renderer::gpu::GpuRenderer;
use uzls_cross::renderer::rope_mesh;
use uzls_cross::simulation::verlet::{Snapshot, VerletSimulator};
use uzls_cross::input;

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
type RendererCell = std::rc::Rc<std::cell::RefCell<Option<GpuRenderer>>>;

#[cfg(target_arch = "wasm32")]
fn save_level_to_storage(level_id: usize) {
    let storage: Option<web_sys::Storage> = web_sys::window()
        .and_then(|w| w.local_storage().ok().flatten());
    if let Some(s) = storage {
        let _ = s.set_item("uzls_level", &level_id.to_string());
    }
}

#[cfg(target_arch = "wasm32")]
fn load_level_from_storage() -> Option<usize> {
    let storage: Option<web_sys::Storage> = web_sys::window()
        .and_then(|w| w.local_storage().ok().flatten());
    let val: Option<String> = storage
        .and_then(|s| s.get_item("uzls_level").ok().flatten());
    val.and_then(|v| v.parse::<usize>().ok())
        .filter(|&v| v >= 1)
}

#[cfg(not(target_arch = "wasm32"))]
fn save_file_path() -> Option<std::path::PathBuf> {
    dirs::data_local_dir().map(|d| d.join("uzls4").join("save.txt"))
}

#[cfg(not(target_arch = "wasm32"))]
fn save_level_to_storage(level_id: usize) {
    if let Some(path) = save_file_path() {
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let _ = std::fs::write(&path, level_id.to_string());
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn load_level_from_storage() -> Option<usize> {
    save_file_path()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|s| s.trim().parse::<usize>().ok())
        .filter(|&v| v >= 1)
}

struct UndoEntry {
    simulator_snapshot: Snapshot,
    rope_endpoints: Vec<(usize, usize)>,
    hole_occupied: Vec<bool>,
}

struct App {
    window: Option<Arc<Window>>,
    renderer: Option<GpuRenderer>,
    simulator: Option<VerletSimulator>,
    level: Option<LevelDefinition>,
    current_level_id: usize,
    time: f32,
    last_instant: Option<Instant>,
    drag_state: Option<input::DragState>,
    highlight_hole: i32,
    hole_occupied: Vec<bool>,
    rope_endpoints: Vec<(usize, usize)>,
    rope_colors: Vec<[f32; 3]>,
    rope_radii: Vec<f32>,
    settle_check_timer: Option<f32>,
    next_level_timer: Option<f32>,
    victory_time: f32,
    last_cursor_pos: (f32, f32),
    right_mouse_down: bool,
    middle_mouse_down: bool,
    pan_last_pos: Option<(f32, f32)>,
    shift_held: bool,
    fps: f32,
    egui_state: Option<egui_winit::State>,
    egui_renderer: Option<egui_wgpu::Renderer>,
    pending_level: Option<usize>,
    wants_restart: bool,
    wants_undo: bool,
    undo_stack: Vec<UndoEntry>,
    table_mode: u32,
    #[cfg(target_arch = "wasm32")]
    pending_renderer: Option<RendererCell>,
    init_done: bool,
}

impl App {
    fn new() -> Self {
        Self {
            window: None,
            renderer: None,
            simulator: None,
            level: None,
            current_level_id: 1,
            time: 0.0,
            last_instant: None,
            drag_state: None,
            highlight_hole: -1,
            hole_occupied: Vec::new(),
            rope_endpoints: Vec::new(),
            rope_colors: Vec::new(),
            rope_radii: Vec::new(),
            settle_check_timer: None,
            next_level_timer: None,
            victory_time: 0.0,
            last_cursor_pos: (0.0, 0.0),
            right_mouse_down: false,
            middle_mouse_down: false,
            pan_last_pos: None,
            shift_held: false,
            fps: 0.0,
            egui_state: None,
            egui_renderer: None,
            pending_level: None,
            wants_restart: false,
            wants_undo: false,
            undo_stack: Vec::new(),
            table_mode: 1,
            #[cfg(target_arch = "wasm32")]
            pending_renderer: None,
            init_done: false,
        }
    }

    fn load_level(&mut self, level_id: usize) {
        self.current_level_id = level_id;
        self.drag_state = None;
        self.undo_stack.clear();
        save_level_to_storage(level_id);

        let level = uzls_cross::level::loader::load_embedded(level_id)
            .unwrap_or_else(|| generator::generate(level_id as u32));

        let hole_positions: Vec<glam::Vec2> = level.holes.iter().map(|h| h.to_vec2()).collect();
        let mut sim = VerletSimulator::new(hole_positions.clone(), level.hole_radius);
        sim.particle_count = level.particles_per_rope;

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
                let start = sim.bands.get(i).and_then(|b| b.pin_start).unwrap_or(r.start_hole);
                let end = sim.bands.get(i).and_then(|b| b.pin_end).unwrap_or(r.end_hole);
                (start, end)
            })
            .collect();

        self.rope_colors = level
            .ropes
            .iter()
            .map(|r| [r.color.red_channel, r.color.green_channel, r.color.blue_channel])
            .collect();
        self.rope_radii = level.ropes.iter().map(|r| r.radius).collect();

        self.hole_occupied = vec![false; hole_positions.len()];
        for &(s, e) in &self.rope_endpoints {
            if s < self.hole_occupied.len() { self.hole_occupied[s] = true; }
            if e < self.hole_occupied.len() { self.hole_occupied[e] = true; }
        }

        if let Some(renderer) = &mut self.renderer {
            renderer.update_hole_instances(&hole_positions, level.hole_radius);

            let (sw, sh) = renderer.surface_size();
            let aspect = sw as f32 / sh.max(1) as f32;
            fit_camera(&mut renderer.camera, &hole_positions, level.hole_radius, aspect);
        }

        self.simulator = Some(sim);
        self.level = Some(level);
        self.settle_check_timer = None;
        self.next_level_timer = None;
        self.victory_time = 0.0;
    }

    fn push_undo_state(&mut self) {
        if let Some(sim) = &self.simulator {
            self.undo_stack.push(UndoEntry {
                simulator_snapshot: sim.take_snapshot(),
                rope_endpoints: self.rope_endpoints.clone(),
                hole_occupied: self.hole_occupied.clone(),
            });
        }
    }

    fn perform_undo(&mut self) {
        if let Some(entry) = self.undo_stack.pop() {
            if let Some(sim) = &mut self.simulator {
                sim.restore_snapshot(&entry.simulator_snapshot);
            }
            self.rope_endpoints = entry.rope_endpoints;
            self.hole_occupied = entry.hole_occupied;
            self.drag_state = None;
            self.highlight_hole = -1;
            self.settle_check_timer = None;
        }
    }

    fn update_and_render(&mut self) {
        #[cfg(target_arch = "wasm32")]
        if !self.init_done {
            let ready = self.pending_renderer.as_ref().and_then(|c| c.borrow_mut().take());
            if let Some(renderer) = ready {
                self.pending_renderer = None;
                self.finish_init_standalone(renderer);
            }
            if !self.init_done {
                return;
            }
        }

        if let Some(lvl) = self.pending_level.take() {
            self.load_level(lvl);
        }
        if self.wants_restart {
            self.wants_restart = false;
            let lvl = self.current_level_id;
            self.load_level(lvl);
        }
        if self.wants_undo {
            self.wants_undo = false;
            self.perform_undo();
        }

        let now = Instant::now();
        let dt = self
            .last_instant
            .map(|prev| now.duration_since(prev).as_secs_f32())
            .unwrap_or(1.0 / 60.0)
            .min(1.0 / 15.0);
        self.last_instant = Some(now);
        self.time += dt;

        let instant_fps = 1.0 / dt.max(0.0001);
        self.fps = if self.fps == 0.0 { instant_fps } else { self.fps * 0.95 + instant_fps * 0.05 };

        if let Some(sim) = &mut self.simulator {
            sim.update(dt);
        }

        if let Some(ref mut timer) = self.settle_check_timer {
            *timer -= dt;
            if *timer <= 0.0 {
                self.settle_check_timer = None;
                self.check_win();
            }
        }

        if let Some(ref mut timer) = self.next_level_timer {
            *timer -= dt;
            if *timer <= 0.0 {
                self.next_level_timer = None;
                let next = self.current_level_id + 1;
                self.load_level(next);
            }
        }

        self.update_rope_buffers();

        let window = match &self.window {
            Some(w) => w.clone(),
            None => return,
        };

        let level = self.current_level_id;
        let fps = self.fps as u32;
        let active_ropes = self.rope_endpoints.iter().filter(|&&(s, _)| s != usize::MAX).count();
        let total_ropes = self.rope_endpoints.len();

        let any_sucking = self.simulator.as_ref().map_or(false, |sim| {
            sim.bands.iter().any(|b| b.active && b.fade_out > 0.0)
        });
        let is_victory = active_ropes == 0 && total_ropes > 0 && !any_sucking;
        if is_victory {
            self.victory_time += dt;
        } else {
            self.victory_time = 0.0;
        }
        let victory_time = self.victory_time;

        let can_undo = !self.undo_stack.is_empty();
        let egui_output = if let Some(egui_state) = &mut self.egui_state {
            let ctx = egui_state.egui_ctx().clone();
            let raw_input = egui_state.take_egui_input(&window);
            let mut hud_action = HudAction { go_to_level: None, restart: false, undo: false, cycle_table: false };
            let full_output = ctx.run(raw_input, |ctx| {
                hud_action = draw_hud(ctx, level, fps, active_ropes, total_ropes, victory_time, can_undo);
            });
            egui_state.handle_platform_output(&window, full_output.platform_output.clone());
            if let Some(lvl) = hud_action.go_to_level {
                self.pending_level = Some(lvl);
            }
            if hud_action.restart {
                self.wants_restart = true;
            }
            if hud_action.undo {
                self.wants_undo = true;
            }
            if hud_action.cycle_table {
                self.table_mode = (self.table_mode + 1) % 4;
            }
            Some((full_output, ctx))
        } else {
            None
        };

        let renderer = match &mut self.renderer { Some(r) => r, None => return };
        let scale = window.scale_factor() as f32;
        let (w, h) = renderer.surface_size();

        let level_seed = self.current_level_id as f32;
        renderer.highlight_hole = self.highlight_hole;
        let mut frame = match renderer.begin_frame(self.time, self.drag_state.is_some(), victory_time, level_seed, self.table_mode) {
            Some(f) => f,
            None => return,
        };

        {
            if let (Some((egui_output, egui_ctx)), Some(egui_rend)) = (egui_output, self.egui_renderer.as_mut()) {
                let sd = egui_wgpu::ScreenDescriptor { size_in_pixels: [w, h], pixels_per_point: scale };
                let prims = egui_ctx.tessellate(egui_output.shapes, scale);
                let dev = renderer.device();
                let q = renderer.queue();
                for (id, delta) in &egui_output.textures_delta.set {
                    egui_rend.update_texture(dev, q, *id, delta);
                }
                egui_rend.update_buffers(dev, q, &mut frame.encoder, &prims, &sd);
                let mut pass = frame.encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("egui_pass"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &frame.screen_view, resolve_target: None,
                        ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
                    })],
                    depth_stencil_attachment: None, timestamp_writes: None, occlusion_query_set: None,
                })
                .forget_lifetime();
                egui_rend.render(&mut pass, &prims, &sd);
                drop(pass);
                for id in &egui_output.textures_delta.free {
                    egui_rend.free_texture(id);
                }
            }
        }

        renderer.end_frame(frame);
    }

}

fn fit_camera(
    camera: &mut uzls_cross::renderer::camera::Camera,
    holes: &[glam::Vec2],
    hole_radius: f32,
    aspect: f32,
) {
    if holes.is_empty() {
        return;
    }
    let mut min_x = f32::MAX;
    let mut max_x = f32::MIN;
    let mut min_y = f32::MAX;
    let mut max_y = f32::MIN;
    for h in holes {
        min_x = min_x.min(h.x);
        max_x = max_x.max(h.x);
        min_y = min_y.min(h.y);
        max_y = max_y.max(h.y);
    }
    let margin = hole_radius * 2.5;
    let content_w = (max_x - min_x) + margin * 2.0;
    let content_h = (max_y - min_y) + margin * 2.0;
    let center_x = (min_x + max_x) * 0.5;
    let center_y = (min_y + max_y) * 0.5;

    let half_h_from_height = content_h * 0.5;
    let half_h_from_width = (content_w * 0.5) / aspect.max(0.01);
    camera.ortho_half_height = half_h_from_height.max(half_h_from_width) * 1.2;
    camera.center = glam::Vec3::new(center_x, center_y, 0.0);
}

struct HudAction {
    go_to_level: Option<usize>,
    restart: bool,
    undo: bool,
    cycle_table: bool,
}

fn draw_hud(ctx: &egui::Context, level: usize, fps: u32, _active_ropes: usize, _total_ropes: usize, victory_time: f32, can_undo: bool) -> HudAction {
    let mut action = HudAction { go_to_level: None, restart: false, undo: false, cycle_table: false };
    let level_input_id = egui::Id::new("level_input_active");
    let level_text_id = egui::Id::new("level_input_text");

    let btn = |ui: &mut egui::Ui, text: &str| -> egui::Response {
        ui.add(
            egui::Button::new(
                egui::RichText::new(text).color(egui::Color32::WHITE).size(18.0),
            )
            .fill(egui::Color32::from_black_alpha(140))
            .corner_radius(22)
            .min_size(egui::vec2(44.0, 44.0)),
        )
    };

    egui::Area::new(egui::Id::new("level_badge"))
        .fixed_pos(egui::pos2(12.0, 12.0))
        .show(ctx, |ui| {
            egui::Frame::none()
                .fill(egui::Color32::from_black_alpha(140))
                .corner_radius(10)
                .inner_margin(egui::Margin { left: 14, right: 14, top: 8, bottom: 8 })
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if btn(ui, "\u{25C0}").clicked() && level > 1 {
                            action.go_to_level = Some(level - 1);
                        }
                        let mut editing: bool = ctx.data_mut(|d| *d.get_temp_mut_or(level_input_id, false));
                        if editing {
                            let mut text: String = ctx.data_mut(|d| d.get_temp_mut_or(level_text_id, String::new()).clone());
                            let resp = ui.add(
                                egui::TextEdit::singleline(&mut text)
                                    .desired_width(50.0)
                                    .font(egui::TextStyle::Heading)
                            );
                            ctx.data_mut(|d| d.insert_temp(level_text_id, text.clone()));
                            if resp.lost_focus() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                                if let Ok(n) = text.trim().parse::<usize>() {
                                    if n >= 1 { action.go_to_level = Some(n); }
                                }
                                editing = false;
                                ctx.data_mut(|d| d.insert_temp(level_input_id, false));
                            }
                            if !resp.has_focus() {
                                resp.request_focus();
                            }
                        } else {
                            let label_resp = ui.add(
                                egui::Label::new(
                                    egui::RichText::new(format!("Level {}", level))
                                        .color(egui::Color32::WHITE)
                                        .size(18.0)
                                        .strong(),
                                ).sense(egui::Sense::click()),
                            );
                            if label_resp.clicked() {
                                ctx.data_mut(|d| {
                                    d.insert_temp(level_input_id, true);
                                    d.insert_temp(level_text_id, level.to_string());
                                });
                            }
                        }
                        if btn(ui, "\u{25B6}").clicked() {
                            action.go_to_level = Some(level + 1);
                        }
                    });
                    ui.label(
                        egui::RichText::new(format!("{} fps", fps))
                            .color(egui::Color32::from_white_alpha(100))
                            .size(11.0)
                            .monospace(),
                    );
                });
        });

    egui::Area::new(egui::Id::new("top_buttons"))
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-12.0, 12.0))
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                let undo_btn = ui.add_enabled(
                    can_undo,
                    egui::Button::new(
                        egui::RichText::new("\u{21A9}")
                            .color(if can_undo { egui::Color32::WHITE } else { egui::Color32::from_white_alpha(80) })
                            .size(18.0),
                    )
                    .fill(egui::Color32::from_black_alpha(140))
                    .corner_radius(22)
                    .min_size(egui::vec2(44.0, 44.0)),
                );
                if undo_btn.clicked() {
                    action.undo = true;
                }
                if btn(ui, "\u{27F3}").clicked() {
                    action.restart = true;
                }
                if btn(ui, "\u{25A3}").clicked() {
                    action.cycle_table = true;
                }
            });
        });

    if victory_time > 0.0 {
        let appear_t = (victory_time / 0.4).min(1.0);
        let bounce = if appear_t < 1.0 {
            let t = appear_t;
            let overshoot = 1.0 + (1.0 - t).powi(2) * 0.3 * (t * std::f32::consts::PI * 3.0).sin();
            t * overshoot
        } else {
            1.0 + (victory_time * 2.5).sin() * 0.015
        };
        let alpha = (appear_t * 255.0).min(255.0) as u8;
        let bg_alpha = (appear_t * 200.0).min(200.0) as u8;

        let title_size = 32.0 * bounce;
        let sub_size = 18.0 * bounce;
        let btn_alpha = ((appear_t - 0.3).max(0.0) / 0.3).min(1.0);

        egui::Area::new(egui::Id::new("victory"))
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, -20.0 * (1.0 - appear_t)))
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::from_black_alpha(bg_alpha))
                    .corner_radius(16)
                    .inner_margin(egui::Margin { left: 32, right: 32, top: 20, bottom: 20 })
                    .show(ui, |ui| {
                        ui.vertical_centered(|ui| {
                            ui.label(
                                egui::RichText::new(format!("Level {}", level))
                                    .color(egui::Color32::from_rgba_unmultiplied(255, 255, 255, alpha))
                                    .size(title_size)
                                    .strong(),
                            );
                            ui.label(
                                egui::RichText::new("completed!")
                                    .color(egui::Color32::from_rgba_unmultiplied(255, 255, 255, (alpha as f32 * 0.7) as u8))
                                    .size(sub_size),
                            );
                            ui.add_space(8.0);
                            if btn_alpha > 0.01 {
                                let btn_color = egui::Color32::from_rgba_unmultiplied(100, 160, 255, (btn_alpha * 255.0) as u8);
                                if ui.add(
                                    egui::Button::new(
                                        egui::RichText::new(format!("Level {} \u{25B6}", level + 1))
                                            .color(egui::Color32::from_rgba_unmultiplied(255, 255, 255, (btn_alpha * 255.0) as u8))
                                            .size(18.0)
                                            .strong(),
                                    )
                                    .fill(btn_color)
                                    .corner_radius(20)
                                    .min_size(egui::vec2(140.0, 40.0)),
                                ).clicked() {
                                    action.go_to_level = Some(level + 1);
                                }
                            }
                        });
                    });
            });

        ctx.request_repaint();
    }

    action
}

impl App {
    fn update_rope_buffers(&mut self) {
        let sim = match &self.simulator {
            Some(s) => s,
            None => return,
        };

        let mut all_vertices: Vec<RopeVertex> = Vec::new();
        let mut all_indices: Vec<u32> = Vec::new();
        let mut base_vertex: u32 = 0;

        for rope_index in 0..self.rope_endpoints.len() {
            if rope_index >= sim.bands.len() || !sim.bands[rope_index].active {
                continue;
            }
            let band = &sim.bands[rope_index];
            if band.fade_out == 0.0 && self.rope_endpoints[rope_index].0 == usize::MAX {
                continue;
            }

            let color = if rope_index < self.rope_colors.len() {
                glam::Vec3::from_array(self.rope_colors[rope_index])
            } else {
                glam::Vec3::new(1.0, 1.0, 1.0)
            };
            let radius = if rope_index < self.rope_radii.len() {
                self.rope_radii[rope_index]
            } else {
                0.038
            };

            let rest_length = band.segment_length * (band.positions.len() as f32 - 1.0).max(1.0);

            let mesh = rope_mesh::build_rect(
                &band.positions,
                radius,
                color,
                &[],
                1.0,
                &[],
                1.0,
                0.0,
                &[],
                rest_length,
                band.cross_section,
                None,
                band.fade_out,
                rope_index,
            );

            for idx in &mesh.indices {
                all_indices.push(idx + base_vertex);
            }
            all_vertices.extend_from_slice(&mesh.vertices);
            base_vertex += mesh.vertices.len() as u32;
        }

        let mut shadow_segs: Vec<uzls_cross::renderer::frame_types::ShadowSegment> = Vec::new();
        for rope_index in 0..self.rope_endpoints.len() {
            if rope_index >= sim.bands.len() || !sim.bands[rope_index].active {
                continue;
            }
            let band = &sim.bands[rope_index];
            if band.fade_out > 0.5 { continue; }
            if self.rope_endpoints[rope_index].0 == usize::MAX { continue; }

            let radius = if rope_index < self.rope_radii.len() {
                self.rope_radii[rope_index]
            } else {
                0.038
            };
            let pts = &band.positions;
            if pts.len() < 2 { continue; }
            for i in 0..pts.len() - 1 {
                let az = pts[i].z;
                let bz = pts[i+1].z;
                if az <= 0.0 && bz <= 0.0 { continue; }
                let mut a = pts[i];
                let mut b = pts[i+1];
                if a.z < 0.0 {
                    let t = -a.z / (b.z - a.z).max(1e-6);
                    a = a + (b - a) * t;
                    a.z = 0.0;
                } else if b.z < 0.0 {
                    let t = -b.z / (a.z - b.z).max(1e-6);
                    b = b + (a - b) * t;
                    b.z = 0.0;
                }
                shadow_segs.push(uzls_cross::renderer::frame_types::ShadowSegment {
                    ax: a.x, ay: a.y, az: a.z, radius,
                    bx: b.x, by: b.y, bz: b.z, rope_id: rope_index as f32,
                });
            }
        }

        if let Some(renderer) = &mut self.renderer {
            renderer.update_rope_mesh(&all_vertices, &all_indices);
            renderer.update_shadow_segments(&shadow_segs);
        }
    }

    fn check_win(&mut self) {
        let sim = match &self.simulator {
            Some(s) => s,
            None => return,
        };
        let untangled = sim.find_untangled_ropes();
        if !untangled.is_empty() {
            for &rope_index in &untangled {
                if let Some(sim) = &mut self.simulator {
                    if rope_index < sim.bands.len() {
                        let pin_s = sim.bands[rope_index].pin_start;
                        let pin_e = sim.bands[rope_index].pin_end;
                        let suck_target = pin_s.or(pin_e).unwrap_or(0);
                        let suck_from_end: usize = if pin_s.is_some() { 1 } else { 0 };
                        sim.bands[rope_index].suck_hole = Some(suck_target);
                        sim.bands[rope_index].suck_from_end = suck_from_end;
                        sim.bands[rope_index].suck_consumed = 0.0;
                        sim.bands[rope_index].fade_out = 0.001;
                        sim.bands[rope_index].pin_start = None;
                        sim.bands[rope_index].pin_end = None;
                    }
                }
                if rope_index < self.rope_endpoints.len() {
                    let (s, e) = self.rope_endpoints[rope_index];
                    if s < self.hole_occupied.len() { self.hole_occupied[s] = false; }
                    if e < self.hole_occupied.len() { self.hole_occupied[e] = false; }
                    self.rope_endpoints[rope_index] = (usize::MAX, usize::MAX);
                }
            }

            let all_done = self.rope_endpoints.iter().all(|&(s, _)| s == usize::MAX);
            if all_done {
                self.next_level_timer = Some(1.5);
            }
        }
    }
}

impl App {
    fn finish_init_with_event_loop(&mut self, window: Arc<Window>, renderer: GpuRenderer, event_loop: &ActiveEventLoop) {
        let egui_ctx = egui::Context::default();
        let egui_state = egui_winit::State::new(
            egui_ctx,
            egui::ViewportId::ROOT,
            event_loop,
            Some(window.scale_factor() as f32),
            None,
            None,
        );
        let egui_renderer = egui_wgpu::Renderer::new(
            renderer.device(),
            renderer.surface_format(),
            None,
            1,
            false,
        );

        self.egui_state = Some(egui_state);
        self.egui_renderer = Some(egui_renderer);
        self.window = Some(window);
        self.renderer = Some(renderer);
        self.init_done = true;
        self.load_level(load_level_from_storage().unwrap_or(49));
    }

    fn finish_init_standalone(&mut self, renderer: GpuRenderer) {
        let window = self.window.as_ref().unwrap().clone();
        let egui_ctx = egui::Context::default();
        let egui_state = egui_winit::State::new(
            egui_ctx,
            egui::ViewportId::ROOT,
            &window,
            Some(window.scale_factor() as f32),
            None,
            None,
        );
        let egui_renderer = egui_wgpu::Renderer::new(
            renderer.device(),
            renderer.surface_format(),
            None,
            1,
            false,
        );

        self.egui_state = Some(egui_state);
        self.egui_renderer = Some(egui_renderer);
        self.renderer = Some(renderer);
        self.init_done = true;

        let size = window.inner_size();
        if size.width > 0 && size.height > 0 {
            if let Some(r) = &mut self.renderer {
                r.resize(size.width, size.height);
            }
        }

        self.load_level(load_level_from_storage().unwrap_or(49));
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        #[cfg(target_arch = "wasm32")]
        {
            let attrs = Window::default_attributes().with_title("UzlsFour");
            let window = Arc::new(event_loop.create_window(attrs).unwrap());

            use winit::platform::web::WindowExtWebSys;
            let canvas = window.canvas().expect("no canvas");
            canvas.style().set_css_text("width: 100%; height: 100%;");
            let web_window = web_sys::window().unwrap();
            let dpr = web_window.device_pixel_ratio();
            let inner_w = web_window.inner_width().unwrap().as_f64().unwrap();
            let inner_h = web_window.inner_height().unwrap().as_f64().unwrap();
            canvas.set_width((inner_w * dpr) as u32);
            canvas.set_height((inner_h * dpr) as u32);
            web_window.document()
                .and_then(|doc| doc.body())
                .map(|body| body.append_child(&canvas).unwrap());

            self.window = Some(window.clone());

            let cell: RendererCell = std::rc::Rc::new(std::cell::RefCell::new(None));
            self.pending_renderer = Some(cell.clone());

            wasm_bindgen_futures::spawn_local(async move {
                let renderer = GpuRenderer::new(window).await;
                *cell.borrow_mut() = Some(renderer);
            });
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            let attrs = Window::default_attributes()
                .with_title("UzlsFour")
                .with_inner_size(winit::dpi::LogicalSize::new(400, 600));
            let window = Arc::new(event_loop.create_window(attrs).unwrap());
            let renderer = pollster::block_on(GpuRenderer::new(window.clone()));
            self.finish_init_with_event_loop(window, renderer, event_loop);
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        if let Some(egui_state) = &mut self.egui_state {
            let resp = egui_state.on_window_event(&self.window.as_ref().unwrap(), &event);
            if resp.consumed {
                return;
            }
        }

        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
            }
            WindowEvent::Resized(size) => {
                if let Some(renderer) = &mut self.renderer {
                    renderer.resize(size.width, size.height);
                    if let Some(level) = &self.level {
                        let hole_positions: Vec<glam::Vec2> =
                            level.holes.iter().map(|h| h.to_vec2()).collect();
                        let aspect = size.width as f32 / size.height.max(1) as f32;
                        fit_camera(&mut renderer.camera, &hole_positions, level.hole_radius, aspect);
                    }
                }
            }
            WindowEvent::RedrawRequested => {
                self.update_and_render();
                if let Some(window) = &self.window {
                    window.request_redraw();
                }
            }
            WindowEvent::ModifiersChanged(modifiers) => {
                self.shift_held = modifiers.state().shift_key();
            }
            WindowEvent::CursorMoved { position, .. } => {
                let pos = (position.x as f32, position.y as f32);
                if self.right_mouse_down || self.middle_mouse_down {
                    if let (Some(prev), Some(renderer)) = (self.pan_last_pos, &mut self.renderer) {
                        let (sw, sh) = renderer.surface_size();
                        let aspect = sw as f32 / sh.max(1) as f32;
                        let half_h = renderer.camera.ortho_half_height;
                        let half_w = half_h * aspect;
                        let dx = (pos.0 - prev.0) / sw as f32 * half_w * 2.0;
                        let dy = (pos.1 - prev.1) / sh as f32 * half_h * 2.0;
                        renderer.camera.center.x -= dx;
                        renderer.camera.center.y += dy;
                    }
                    self.pan_last_pos = Some(pos);
                }
                self.last_cursor_pos = pos;
                if self.drag_state.is_some() {
                    if let (Some(renderer), Some(sim)) =
                        (&self.renderer, &mut self.simulator)
                    {
                        let world = input::screen_to_world(
                            self.last_cursor_pos,
                            (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32),
                            &renderer.camera,
                        );
                        sim.update_drag(world);
                        if let Some(level) = &self.level {
                            let hole_positions: Vec<glam::Vec2> =
                                level.holes.iter().map(|h| h.to_vec2()).collect();
                            self.highlight_hole = input::find_snap_hole(
                                world, &hole_positions, &self.hole_occupied, level.hole_radius,
                            ).map(|i| i as i32).unwrap_or(-1);
                        }
                    }
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    winit::event::MouseScrollDelta::LineDelta(_, y) => y,
                    winit::event::MouseScrollDelta::PixelDelta(p) => p.y as f32 / 60.0,
                };
                if let Some(renderer) = &mut self.renderer {
                    if self.shift_held {
                        renderer.camera.tilt_angle = (renderer.camera.tilt_angle + scroll * 0.05)
                            .clamp(0.0, 1.2);
                    } else {
                        let factor = 1.0 - scroll * 0.1;
                        renderer.camera.ortho_half_height = (renderer.camera.ortho_half_height * factor)
                            .clamp(0.5, 12.0);
                    }
                }
            }
            WindowEvent::MouseInput {
                state,
                button: winit::event::MouseButton::Right,
                ..
            } => {
                match state {
                    winit::event::ElementState::Pressed => {
                        self.right_mouse_down = true;
                        self.pan_last_pos = Some(self.last_cursor_pos);
                    }
                    winit::event::ElementState::Released => {
                        self.right_mouse_down = false;
                        self.pan_last_pos = None;
                    }
                }
            }
            WindowEvent::MouseInput {
                state,
                button: winit::event::MouseButton::Middle,
                ..
            } => {
                match state {
                    winit::event::ElementState::Pressed => {
                        self.middle_mouse_down = true;
                        self.pan_last_pos = Some(self.last_cursor_pos);
                    }
                    winit::event::ElementState::Released => {
                        self.middle_mouse_down = false;
                        self.pan_last_pos = None;
                    }
                }
            }
            WindowEvent::MouseInput {
                state,
                button: winit::event::MouseButton::Left,
                ..
            } => {
                match state {
                    winit::event::ElementState::Pressed => {
                        if let Some(renderer) = &self.renderer {
                            let vp = (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32);
                            let world = input::screen_to_world(self.last_cursor_pos, vp, &renderer.camera);
                            let level = match &self.level {
                                Some(l) => l,
                                None => return,
                            };
                            let hole_positions: Vec<glam::Vec2> =
                                level.holes.iter().map(|h| h.to_vec2()).collect();
                            let sim = match &self.simulator {
                                Some(s) => s,
                                None => return,
                            };
                            let endpoint_z = |rope: usize, end: usize| -> f32 {
                                sim.endpoint_z(rope, end)
                            };
                            if let Some((rope_index, end_index, hole_index)) =
                                input::find_nearest_endpoint(
                                    world, &hole_positions, &self.rope_endpoints,
                                    &endpoint_z, level.hole_radius,
                                )
                            {
                                self.push_undo_state();
                                if hole_index < self.hole_occupied.len() {
                                    self.hole_occupied[hole_index] = false;
                                }
                                self.drag_state = Some(input::DragState {
                                    rope_index, end_index, original_hole_index: hole_index,
                                });
                                if let Some(sim) = &mut self.simulator {
                                    sim.begin_drag(rope_index, end_index, world);
                                }
                            }
                        }
                    }
                    winit::event::ElementState::Released => {
                        self.highlight_hole = -1;
                        if let Some(drag) = self.drag_state.take() {
                            if let (Some(renderer), Some(sim)) =
                                (&self.renderer, &mut self.simulator)
                            {
                                let world = input::screen_to_world(
                                    self.last_cursor_pos,
                                    (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32),
                                    &renderer.camera,
                                );
                                let level = self.level.as_ref().unwrap();
                                let hole_positions: Vec<glam::Vec2> =
                                    level.holes.iter().map(|h| h.to_vec2()).collect();
                                let snap = input::find_snap_hole(
                                    world, &hole_positions, &self.hole_occupied, level.hole_radius,
                                ).unwrap_or(drag.original_hole_index);
                                sim.end_drag(snap);
                                if drag.end_index == 0 {
                                    self.rope_endpoints[drag.rope_index].0 = snap;
                                } else {
                                    self.rope_endpoints[drag.rope_index].1 = snap;
                                }
                                if snap < self.hole_occupied.len() {
                                    self.hole_occupied[snap] = true;
                                }
                                self.settle_check_timer = Some(0.5);
                            }
                        }
                    }
                }
            }
            WindowEvent::Touch(touch) => {
                let pos = (touch.location.x as f32, touch.location.y as f32);
                match touch.phase {
                    winit::event::TouchPhase::Started => {
                        self.last_cursor_pos = pos;
                        if let Some(renderer) = &self.renderer {
                            let vp = (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32);
                            let world = input::screen_to_world(pos, vp, &renderer.camera);
                            let level = match &self.level { Some(l) => l, None => return };
                            let hole_positions: Vec<glam::Vec2> = level.holes.iter().map(|h| h.to_vec2()).collect();
                            let sim = match &self.simulator { Some(s) => s, None => return };
                            let endpoint_z = |rope: usize, end: usize| -> f32 { sim.endpoint_z(rope, end) };
                            if let Some((rope_index, end_index, hole_index)) =
                                input::find_nearest_endpoint(world, &hole_positions, &self.rope_endpoints, &endpoint_z, level.hole_radius)
                            {
                                self.push_undo_state();
                                if hole_index < self.hole_occupied.len() {
                                    self.hole_occupied[hole_index] = false;
                                }
                                self.drag_state = Some(input::DragState { rope_index, end_index, original_hole_index: hole_index });
                                if let Some(sim) = &mut self.simulator {
                                    sim.begin_drag(rope_index, end_index, world);
                                }
                            }
                        }
                    }
                    winit::event::TouchPhase::Moved => {
                        self.last_cursor_pos = pos;
                        if self.drag_state.is_some() {
                            if let (Some(renderer), Some(sim)) = (&self.renderer, &mut self.simulator) {
                                let world = input::screen_to_world(pos, (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32), &renderer.camera);
                                sim.update_drag(world);
                                if let Some(level) = &self.level {
                                    let hole_positions: Vec<glam::Vec2> = level.holes.iter().map(|h| h.to_vec2()).collect();
                                    self.highlight_hole = input::find_snap_hole(world, &hole_positions, &self.hole_occupied, level.hole_radius)
                                        .map(|i| i as i32).unwrap_or(-1);
                                }
                            }
                        }
                    }
                    winit::event::TouchPhase::Ended | winit::event::TouchPhase::Cancelled => {
                        self.highlight_hole = -1;
                        if let Some(drag) = self.drag_state.take() {
                            if let (Some(renderer), Some(sim)) = (&self.renderer, &mut self.simulator) {
                                let world = input::screen_to_world(pos, (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32), &renderer.camera);
                                let level = self.level.as_ref().unwrap();
                                let hole_positions: Vec<glam::Vec2> = level.holes.iter().map(|h| h.to_vec2()).collect();
                                let snap = input::find_snap_hole(world, &hole_positions, &self.hole_occupied, level.hole_radius)
                                    .unwrap_or(drag.original_hole_index);
                                sim.end_drag(snap);
                                if drag.end_index == 0 {
                                    self.rope_endpoints[drag.rope_index].0 = snap;
                                } else {
                                    self.rope_endpoints[drag.rope_index].1 = snap;
                                }
                                if snap < self.hole_occupied.len() {
                                    self.hole_occupied[snap] = true;
                                }
                                self.settle_check_timer = Some(0.5);
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    {
        std::panic::set_hook(Box::new(console_error_panic_hook::hook));
        console_log::init_with_level(log::Level::Info).expect("console_log init");
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        env_logger::init();
    }

    let event_loop = EventLoop::new().unwrap();
    let app = App::new();

    #[cfg(target_arch = "wasm32")]
    {
        use winit::platform::web::EventLoopExtWebSys;
        event_loop.spawn_app(app);
    }

    #[cfg(not(target_arch = "wasm32"))]
    {
        let mut app = app;
        event_loop.run_app(&mut app).unwrap();
    }
}
