use std::collections::HashMap;
use std::sync::Arc;
use web_time::Instant;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::window::{Window, WindowId};

use uzls_cross::level::definition::LevelDefinition;
use uzls_cross::level::generator;
use uzls_cross::renderer::frame_types::RopeVertex;
use uzls_cross::renderer::gpu::{GpuRenderer, GpuTimings};
use uzls_cross::renderer::rope_mesh;
use uzls_cross::simulation::verlet::{Snapshot, VerletSimulator};
use uzls_cross::input;
use uzls_cross::input::DragResult;
use uzls_cross::audio::AudioPlayer;
use uzls_cross::storage::{save_level_to_storage, load_level_from_storage};
use uzls_cross::hud::{self, HudAction, ProfilingSnapshot, render_mode_scale};
use uzls_cross::celebration::{self, CelebrationBand, CameraSnapshot};

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
type RendererCell = std::rc::Rc<std::cell::RefCell<Option<GpuRenderer>>>;

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
    audio: Option<AudioPlayer>,
    victory_sound_played: bool,
    celebration_active: bool,
    celebration_bands: Vec<CelebrationBand>,
    pre_victory_camera: Option<CameraSnapshot>,
    move_count: u32,
    min_moves: u32,
    render_mode: u8,
    cel_mode: bool,
    downscaled: bool,
    low_fps_frames: u32,
    active_touches: HashMap<u64, (f32, f32)>,
    touch_camera_active: bool,
    #[cfg(target_arch = "wasm32")]
    pending_renderer: Option<RendererCell>,
    init_done: bool,
    prof_physics_ms: f32,
    prof_mesh_ms: f32,
    prof_egui_ms: f32,
    prof_frame_ms: f32,
    prof_check_win_ms: f32,
    prof_shadow_segs_ms: f32,
    prof_submit_ms: f32,
    prof_celebration_ms: f32,
    prof_gpu: GpuTimings,
    prof_show: bool,
    prof_rope_verts: u32,
    prof_rope_tris: u32,
    #[cfg(not(target_arch = "wasm32"))]
    prof_writer: Option<std::io::BufWriter<std::fs::File>>,
    prof_frame_num: u64,
    prof_events: Vec<&'static str>,
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
            audio: AudioPlayer::new(),
            victory_sound_played: false,
            celebration_active: false,
            celebration_bands: Vec::new(),
            pre_victory_camera: None,
            move_count: 0,
            min_moves: 1,
            render_mode: 0,
            cel_mode: false,
            downscaled: false,
            low_fps_frames: 0,
            active_touches: HashMap::new(),
            touch_camera_active: false,
            #[cfg(target_arch = "wasm32")]
            pending_renderer: None,
            init_done: false,
            prof_physics_ms: 0.0,
            prof_mesh_ms: 0.0,
            prof_egui_ms: 0.0,
            prof_frame_ms: 0.0,
            prof_check_win_ms: 0.0,
            prof_shadow_segs_ms: 0.0,
            prof_submit_ms: 0.0,
            prof_celebration_ms: 0.0,
            prof_gpu: GpuTimings::default(),
            prof_show: false,
            prof_rope_verts: 0,
            prof_rope_tris: 0,
            #[cfg(not(target_arch = "wasm32"))]
            prof_writer: None,
            prof_frame_num: 0,
            prof_events: Vec::new(),
        }
    }

    fn load_level(&mut self, level_id: usize) {
        self.current_level_id = level_id;
        self.drag_state = None;
        self.undo_stack.clear();
        save_level_to_storage(level_id);

        let level = uzls_cross::level::loader::load_embedded(level_id)
            .unwrap_or_else(|| generator::generate(level_id as u32));

        let hole_positions = level.hole_positions();
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
        if let Some(snap) = self.pre_victory_camera.take() {
            if let Some(r) = &mut self.renderer {
                snap.restore(&mut r.camera);
            }
        }
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
            self.move_count = self.move_count.saturating_sub(1);
            self.drag_state = None;
            self.highlight_hole = -1;
            self.settle_check_timer = None;
        }
    }

    fn try_begin_drag(&mut self, screen_pos: (f32, f32)) {
        let result = {
            let renderer = match &self.renderer { Some(r) => r, None => return };
            let level = match &self.level { Some(l) => l, None => return };
            let sim = match &self.simulator { Some(s) => s, None => return };

            let vp = (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32);
            let hole_positions = level.hole_positions();
            let hole_radius = level.hole_radius;
            let endpoint_z = |rope: usize, end: usize| -> f32 { sim.endpoint_z(rope, end) };

            input::begin_drag_action(
                screen_pos, vp, &renderer.camera, &hole_positions,
                &self.rope_endpoints, &endpoint_z, hole_radius,
            )
        };

        if let Some((drag, hole_to_free)) = result {
            self.push_undo_state();
            if hole_to_free < self.hole_occupied.len() {
                self.hole_occupied[hole_to_free] = false;
            }
            if let Some(renderer) = &self.renderer {
                let vp = (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32);
                let world = input::screen_to_world(screen_pos, vp, &renderer.camera);
                if let Some(sim) = &mut self.simulator {
                    sim.begin_drag(drag.rope_index, drag.end_index, world);
                }
            }
            self.drag_state = Some(drag);
            self.prof_events.push("drag_start");
        }
    }

    fn finish_drag(&mut self, screen_pos: (f32, f32)) {
        self.highlight_hole = -1;
        let drag = match self.drag_state.take() { Some(d) => d, None => return };
        let renderer = match &self.renderer { Some(r) => r, None => return };
        let level = match &self.level { Some(l) => l, None => return };

        let vp = (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32);
        let hole_positions = level.hole_positions();

        match input::end_drag_action(
            &drag, screen_pos, vp, &renderer.camera,
            &hole_positions, &self.hole_occupied, level.hole_radius,
        ) {
            DragResult::Snapped { hole, moved } => {
                if let Some(sim) = &mut self.simulator { sim.end_drag(hole); }
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
                    if let Some(audio) = &self.audio { audio.play_snap(); }
                }
                self.settle_check_timer = Some(0.5);
                self.prof_events.push("drag_snap");
            }
            DragResult::Cancelled => {
                if let Some(sim) = &mut self.simulator { sim.cancel_drag(); }
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

    fn cancel_drag(&mut self) {
        let drag = match self.drag_state.take() { Some(d) => d, None => return };
        self.highlight_hole = -1;
        if let Some(sim) = &mut self.simulator { sim.cancel_drag(); }
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

    fn update_drag_highlight(&mut self, screen_pos: (f32, f32)) {
        if self.drag_state.is_none() { return; }
        let renderer = match &self.renderer { Some(r) => r, None => return };
        let vp = (renderer.surface_size().0 as f32, renderer.surface_size().1 as f32);
        let world = input::screen_to_world(screen_pos, vp, &renderer.camera);
        if let Some(sim) = &mut self.simulator {
            sim.update_drag(world);
        }
        if let Some(level) = &self.level {
            let hole_positions = level.hole_positions();
            self.highlight_hole = input::find_snap_hole(
                world, &hole_positions, &self.hole_occupied, level.hole_radius,
            ).map(|i| i as i32).unwrap_or(-1);
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
            self.prof_events.push("level_load");
            self.load_level(lvl);
        }
        if self.wants_restart {
            self.wants_restart = false;
            self.prof_events.push("restart");
            let lvl = self.current_level_id;
            self.load_level(lvl);
        }
        if self.wants_undo {
            self.wants_undo = false;
            self.perform_undo();
        }

        let now = Instant::now();
        let raw_dt = self
            .last_instant
            .map(|prev| now.duration_since(prev).as_secs_f32())
            .unwrap_or(1.0 / 60.0);
        let dt = raw_dt.min(1.0 / 15.0);
        self.last_instant = Some(now);
        self.time += dt;

        let instant_fps = 1.0 / raw_dt.max(0.0001);
        self.fps = if self.fps == 0.0 { instant_fps } else { self.fps * 0.95 + instant_fps * 0.05 };

        if self.render_mode >= 1 && self.drag_state.is_some() {
            if self.fps < 30.0 {
                self.low_fps_frames += 1;
                if self.low_fps_frames > 5 && !self.downscaled {
                    self.downscaled = true;
                    if let Some(r) = &mut self.renderer {
                        r.set_render_scale(0.5);
                    }
                    self.prof_events.push("downscale");
                }
            } else {
                self.low_fps_frames = 0;
            }
        }
        if self.downscaled && self.drag_state.is_none() {
            self.downscaled = false;
            self.low_fps_frames = 0;
            if let Some(r) = &mut self.renderer {
                r.set_render_scale(render_mode_scale(self.render_mode));
            }
            self.prof_events.push("upscale");
        }

        let t_physics = Instant::now();
        if let Some(sim) = &mut self.simulator {
            sim.update(dt);
        }
        let physics_elapsed = Instant::now().duration_since(t_physics).as_secs_f32() * 1000.0;

        let t_check_win = Instant::now();
        if let Some(ref mut timer) = self.settle_check_timer {
            *timer -= dt;
            if *timer <= 0.0 {
                self.settle_check_timer = None;
                self.check_win();
            }
        }
        let check_win_elapsed = Instant::now().duration_since(t_check_win).as_secs_f32() * 1000.0;

        let t_mesh = Instant::now();
        let shadow_segs_elapsed = self.update_rope_buffers();
        let mesh_elapsed = Instant::now().duration_since(t_mesh).as_secs_f32() * 1000.0 - shadow_segs_elapsed;

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
        let mut celebration_elapsed = 0.0_f32;
        if is_victory {
            self.victory_time += dt;
            if !self.victory_sound_played {
                self.victory_sound_played = true;
                if let Some(audio) = &self.audio {
                    audio.play_firework(self.time);
                }
            }
            if !self.celebration_active {
                self.celebration_active = true;
                self.prof_events.push("victory");
                if let Some(level) = &self.level {
                    let holes = level.hole_positions();
                    self.celebration_bands = celebration::spawn_celebration_bands(
                        &holes, level.ropes.len(), &self.rope_colors, &self.rope_radii,
                    );
                }
                if let Some(r) = &self.renderer {
                    self.pre_victory_camera = Some(CameraSnapshot::capture(&r.camera));
                }
            }
            let t_celeb = Instant::now();
            celebration::update_celebration(&mut self.celebration_bands, self.victory_time);
            if let (Some(renderer), Some(snap)) = (&mut self.renderer, &self.pre_victory_camera) {
                celebration::animate_victory_camera(&mut renderer.camera, snap, self.victory_time, dt);
            }
            celebration_elapsed = Instant::now().duration_since(t_celeb).as_secs_f32() * 1000.0;
        } else {
            self.victory_time = 0.0;
        }
        let victory_time = self.victory_time;

        let can_undo = !self.undo_stack.is_empty();
        let render_mode = self.render_mode;
        let cel_mode = self.cel_mode;
        let prof_show = self.prof_show;
        let draw_flags = self.renderer.as_ref().map(|r| r.draw_flags).unwrap_or_default();
        let prof_data = ProfilingSnapshot {
            physics_ms: self.prof_physics_ms,
            mesh_ms: self.prof_mesh_ms,
            egui_ms: self.prof_egui_ms,
            frame_ms: self.prof_frame_ms,
            check_win_ms: self.prof_check_win_ms,
            shadow_segs_ms: self.prof_shadow_segs_ms,
            submit_ms: self.prof_submit_ms,
            celebration_ms: self.prof_celebration_ms,
            gpu: self.prof_gpu.clone(),
            rope_verts: self.prof_rope_verts,
            rope_tris: self.prof_rope_tris,
            skip_table: draw_flags.skip_table,
            skip_holes: draw_flags.skip_holes,
            skip_ropes: draw_flags.skip_ropes,
            table_shadow_mode: draw_flags.table_shadow_mode,
        };
        let t_egui = Instant::now();
        let egui_output = if let Some(egui_state) = &mut self.egui_state {
            let ctx = egui_state.egui_ctx().clone();
            let raw_input = egui_state.take_egui_input(&window);
            let mut hud_action = HudAction::default();
            let min_moves = self.min_moves;
            let full_output = ctx.run(raw_input, |ctx| {
                hud_action = hud::draw_hud(ctx, level, fps, active_ropes, total_ropes, victory_time, can_undo, render_mode, self.move_count, min_moves, cel_mode, prof_show, &prof_data);
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
            if hud_action.toggle_hd {
                self.render_mode = (self.render_mode + 1) % 3;
                self.downscaled = false;
                self.low_fps_frames = 0;
                if let Some(r) = &mut self.renderer {
                    r.set_render_scale(render_mode_scale(self.render_mode));
                }
                self.prof_events.push(match self.render_mode {
                    0 => "mode_sd",
                    2 => "mode_uhd",
                    _ => "mode_hd",
                });
            }
            if hud_action.toggle_cel {
                self.cel_mode = !self.cel_mode;
                self.prof_events.push(if self.cel_mode { "cel_on" } else { "cel_off" });
            }
            if hud_action.toggle_prof {
                self.prof_show = !self.prof_show;
                #[cfg(not(target_arch = "wasm32"))]
                self.toggle_prof_file();
            }
            Some((full_output, ctx))
        } else {
            None
        };
        let egui_elapsed = Instant::now().duration_since(t_egui).as_secs_f32() * 1000.0;

        let renderer = match &mut self.renderer { Some(r) => r, None => return };
        let scale = window.scale_factor() as f32;
        let (w, h) = renderer.surface_size();

        let level_seed = self.current_level_id as f32;
        renderer.highlight_hole = self.highlight_hole;
        let mut frame = match renderer.begin_frame(self.time, self.drag_state.is_some(), victory_time, level_seed, self.render_mode as u32, self.cel_mode) {
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

        let t_submit = Instant::now();
        renderer.end_frame(frame);
        let submit_elapsed = Instant::now().duration_since(t_submit).as_secs_f32() * 1000.0;

        let frame_elapsed = Instant::now().duration_since(now).as_secs_f32() * 1000.0;
        let a = 0.9_f32;
        self.prof_physics_ms = self.prof_physics_ms * a + physics_elapsed * (1.0 - a);
        self.prof_mesh_ms = self.prof_mesh_ms * a + mesh_elapsed * (1.0 - a);
        self.prof_egui_ms = self.prof_egui_ms * a + egui_elapsed * (1.0 - a);
        self.prof_frame_ms = self.prof_frame_ms * a + frame_elapsed * (1.0 - a);
        self.prof_check_win_ms = self.prof_check_win_ms * a + check_win_elapsed * (1.0 - a);
        self.prof_shadow_segs_ms = self.prof_shadow_segs_ms * a + shadow_segs_elapsed * (1.0 - a);
        self.prof_submit_ms = self.prof_submit_ms * a + submit_elapsed * (1.0 - a);
        self.prof_celebration_ms = self.prof_celebration_ms * a + celebration_elapsed * (1.0 - a);
        self.prof_gpu = renderer.gpu_timings.clone();
        self.prof_frame_num += 1;

        #[cfg(not(target_arch = "wasm32"))]
        if let Some(w) = &mut self.prof_writer {
            use std::io::Write;
            let gpu = &renderer.gpu_timings;
            let events = if self.prof_events.is_empty() {
                String::new()
            } else {
                self.prof_events.join(";")
            };
            let rscale = renderer.render_scale;
            let rmode = self.render_mode;
            let dragging = if self.drag_state.is_some() { 1 } else { 0 };
            let (sw, sh) = renderer.surface_size();
            let rw = ((sw as f32 * rscale) as u32).max(1);
            let rh = ((sh as f32 * rscale) as u32).max(1);
            let df = &renderer.draw_flags;
            let tsm_letter = match df.table_shadow_mode { 0 => "P", 1 => "L", _ => "N" };
            let skip = format!("{}{}{}{}",
                if df.skip_table { "T" } else { "" },
                if df.skip_holes { "H" } else { "" },
                if df.skip_ropes { "R" } else { "" },
                tsm_letter,
            );
            let _ = writeln!(w,
                "{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{},{},{},{:.2},{},{},{}x{},{},{}",
                self.prof_frame_num,
                frame_elapsed, physics_elapsed, mesh_elapsed, egui_elapsed,
                check_win_elapsed, shadow_segs_elapsed, submit_elapsed, celebration_elapsed,
                gpu.total_ms, gpu.shadow_ms, gpu.hdr_ms, gpu.bloom_ms,
                self.prof_rope_verts, self.prof_rope_tris,
                events,
                rscale, rmode, dragging,
                rw, rh,
                skip,
                df.table_shadow_mode,
            );
        }
        self.prof_events.clear();
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn toggle_prof_file(&mut self) {
        use std::io::Write;
        if self.prof_show {
            let ts = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0);
            let path = format!("profile_{}.csv", ts);
            if let Ok(f) = std::fs::File::create(&path) {
                let mut w = std::io::BufWriter::new(f);
                let _ = writeln!(w, "frame,cpu_frame_ms,cpu_physics_ms,cpu_mesh_ms,cpu_egui_ms,cpu_check_win_ms,cpu_shadow_segs_ms,cpu_submit_ms,cpu_celebration_ms,gpu_total_ms,gpu_shadow_ms,gpu_hdr_ms,gpu_bloom_ms,rope_verts,rope_tris,event,render_scale,render_mode,dragging,render_size,skip,table_shadow_mode");
                self.prof_writer = Some(w);
                log::info!("profiling → {}", path);
            }
        } else {
            if let Some(mut w) = self.prof_writer.take() {
                let _ = w.flush();
                log::info!("profiling file closed");
            }
        }
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
    let required_half_h = half_h_from_height.max(half_h_from_width) * 1.2;
    camera.ortho_half_height = required_half_h;
    camera.center = glam::Vec3::new(center_x, center_y, 0.0);

    let half_fov = (59.0f32 / 2.0).to_radians();
    let dist_from_h = required_half_h / half_fov.tan();
    let dist_from_w = (content_w * 0.5 * 1.2) / (half_fov.tan() * aspect.max(0.01));
    camera.distance = dist_from_h.max(dist_from_w).max(1.0);
}

impl App {
    fn update_rope_buffers(&mut self) -> f32 {
        let sim = match &self.simulator {
            Some(s) => s,
            None => return 0.0,
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
            let visual_radius = radius * 1.3;

            let rest_length = band.segment_length * (band.positions.len() as f32 - 1.0).max(1.0);

            let prof_segs = if self.render_mode == 0 { 6 } else { 8 };
            let mesh = rope_mesh::build_rect(
                &band.positions,
                visual_radius,
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
                prof_segs,
            );

            for idx in &mesh.indices {
                all_indices.push(idx + base_vertex);
            }
            all_vertices.extend_from_slice(&mesh.vertices);
            base_vertex += mesh.vertices.len() as u32;

            if let Some(level) = &self.level {
                let (start_hole, end_hole) = self.rope_endpoints[rope_index];
                let dragging_start = self.drag_state.as_ref()
                    .map_or(false, |d| d.rope_index == rope_index && d.end_index == 0);
                let dragging_end = self.drag_state.as_ref()
                    .map_or(false, |d| d.rope_index == rope_index && d.end_index == 1);
                let hole_r = level.hole_radius;
                let holes = &level.holes;

                let start_pos = if dragging_start {
                    band.positions.first().copied().unwrap_or(glam::Vec3::ZERO)
                } else if start_hole < holes.len() {
                    let hc = holes[start_hole].to_vec2();
                    glam::Vec3::new(hc.x, hc.y, 0.0)
                } else {
                    glam::Vec3::ZERO
                };
                let plug_segs = if self.cel_mode { 8 } else { 20 };
                if start_hole < holes.len() || dragging_start {
                    let plug = rope_mesh::build_plug(
                        start_pos, hole_r, color, rope_index, band.fade_out, plug_segs,
                    );
                    for idx in &plug.indices { all_indices.push(idx + base_vertex); }
                    all_vertices.extend_from_slice(&plug.vertices);
                    base_vertex += plug.vertices.len() as u32;
                }

                let end_pos = if dragging_end {
                    band.positions.last().copied().unwrap_or(glam::Vec3::ZERO)
                } else if end_hole < holes.len() {
                    let hc = holes[end_hole].to_vec2();
                    glam::Vec3::new(hc.x, hc.y, 0.0)
                } else {
                    glam::Vec3::ZERO
                };
                if end_hole < holes.len() || dragging_end {
                    let plug = rope_mesh::build_plug(
                        end_pos, hole_r, color, rope_index, band.fade_out, plug_segs,
                    );
                    for idx in &plug.indices { all_indices.push(idx + base_vertex); }
                    all_vertices.extend_from_slice(&plug.vertices);
                    base_vertex += plug.vertices.len() as u32;
                }
            }
        }

        for (ci, cband) in self.celebration_bands.iter().enumerate() {
            if cband.positions.len() < 2 { continue; }
            let color = glam::Vec3::from_array(cband.color);
            let visual_radius = cband.radius * 1.3;
            let rest_length = cband.segment_length * (cband.positions.len() as f32 - 1.0).max(1.0);
            let cross_section = uzls_cross::level::definition::CrossSection::Circular { radius: cband.radius };

            let prof_segs = if self.render_mode == 0 { 6 } else { 8 };
            let mesh = rope_mesh::build_rect(
                &cband.positions,
                visual_radius,
                color,
                &[],
                1.0,
                &[],
                1.0,
                0.0,
                &[],
                rest_length,
                cross_section,
                None,
                0.0,
                1000 + ci,
                prof_segs,
            );

            for idx in &mesh.indices {
                all_indices.push(idx + base_vertex);
            }
            all_vertices.extend_from_slice(&mesh.vertices);
            base_vertex += mesh.vertices.len() as u32;
        }

        let table_shadow_mode = self
            .renderer
            .as_ref()
            .map(|r| r.draw_flags.table_shadow_mode)
            .unwrap_or(0);
        let need_shadow_segments = !self.cel_mode && (self.render_mode > 0 || table_shadow_mode == 1);

        self.prof_rope_verts = all_vertices.len() as u32;
        self.prof_rope_tris = all_indices.len() as u32 / 3;

        if let Some(renderer) = &mut self.renderer {
            renderer.update_rope_mesh(&all_vertices, &all_indices);
        }

        let t_shadow_segs = Instant::now();
        if need_shadow_segments {
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
                let step = if pts.len() > 12 { 4 } else { 1 };
                let mut i = 0;
                while i < pts.len() - 1 {
                    let next = (i + step).min(pts.len() - 1);
                    let az = pts[i].z;
                    let bz = pts[next].z;
                    if az <= 0.0 && bz <= 0.0 { i = next; continue; }
                    let mut a = pts[i];
                    let mut b = pts[next];
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
                    i = next;
                }
            }

            if let Some(renderer) = &mut self.renderer {
                renderer.update_shadow_segments(&shadow_segs);
            }
        }
        Instant::now().duration_since(t_shadow_segs).as_secs_f32() * 1000.0
    }

    fn check_win(&mut self) {
        let sim = match &self.simulator {
            Some(s) => s,
            None => return,
        };
        let untangled = sim.find_untangled_ropes();
        if !untangled.is_empty() {
            self.prof_events.push("untangle");
            if let Some(audio) = &self.audio {
                audio.play_vanish();
            }
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

            let _all_done = self.rope_endpoints.iter().all(|&(s, _)| s == usize::MAX);
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
        let is_touch = matches!(event, WindowEvent::Touch(_));
        if let Some(egui_state) = &mut self.egui_state {
            let resp = egui_state.on_window_event(&self.window.as_ref().unwrap(), &event);
            if resp.consumed && !is_touch {
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
                        let hole_positions = level.hole_positions();
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
            WindowEvent::KeyboardInput { event, .. } => {
                if event.state == winit::event::ElementState::Pressed {
                    if let winit::keyboard::PhysicalKey::Code(code) = event.physical_key {
                        if code == winit::keyboard::KeyCode::KeyP {
                            self.prof_show = !self.prof_show;
                            #[cfg(not(target_arch = "wasm32"))]
                            self.toggle_prof_file();
                        }
                        if let Some(r) = &mut self.renderer {
                            match code {
                                winit::keyboard::KeyCode::Digit1 => {
                                    r.draw_flags.skip_table = !r.draw_flags.skip_table;
                                    self.prof_events.push(if r.draw_flags.skip_table { "table_off" } else { "table_on" });
                                }
                                winit::keyboard::KeyCode::Digit2 => {
                                    r.draw_flags.skip_holes = !r.draw_flags.skip_holes;
                                    self.prof_events.push(if r.draw_flags.skip_holes { "holes_off" } else { "holes_on" });
                                }
                                winit::keyboard::KeyCode::Digit3 => {
                                    r.draw_flags.skip_ropes = !r.draw_flags.skip_ropes;
                                    self.prof_events.push(if r.draw_flags.skip_ropes { "ropes_off" } else { "ropes_on" });
                                }
                                winit::keyboard::KeyCode::Digit4 => {
                                    r.draw_flags.table_shadow_mode = (r.draw_flags.table_shadow_mode + 1) % 3;
                                    self.prof_events.push(match r.draw_flags.table_shadow_mode {
                                        0 => "tshadow_pcf",
                                        1 => "tshadow_planar",
                                        _ => "tshadow_off",
                                    });
                                }
                                _ => {}
                            }
                        }
                    }
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let pos = (position.x as f32, position.y as f32);
                if self.right_mouse_down || self.middle_mouse_down {
                    if let (Some(prev), Some(renderer)) = (self.pan_last_pos, &mut self.renderer) {
                        let (sw, sh) = renderer.surface_size();
                        input::apply_camera_pan(&mut renderer.camera, prev, pos, sw, sh);
                    }
                    self.pan_last_pos = Some(pos);
                }
                self.last_cursor_pos = pos;
                self.update_drag_highlight(pos);
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
                        if let Some(audio) = &mut self.audio {
                            audio.ensure_context();
                            audio.resume();
                        }
                        self.try_begin_drag(self.last_cursor_pos);
                    }
                    winit::event::ElementState::Released => {
                        self.finish_drag(self.last_cursor_pos);
                    }
                }
            }
            WindowEvent::Touch(touch) => {
                let pos = (touch.location.x as f32, touch.location.y as f32);
                let id = touch.id;
                match touch.phase {
                    winit::event::TouchPhase::Started => {
                        self.active_touches.insert(id, pos);
                        if self.active_touches.len() == 1 && !self.touch_camera_active {
                            self.last_cursor_pos = pos;
                            if let Some(audio) = &mut self.audio {
                                audio.ensure_context();
                                audio.resume();
                            }
                            self.try_begin_drag(pos);
                        }
                        if self.active_touches.len() >= 2 {
                            self.cancel_drag();
                            self.touch_camera_active = true;
                        }
                    }
                    winit::event::TouchPhase::Moved => {
                        let prev = self.active_touches.get(&id).copied();
                        self.active_touches.insert(id, pos);

                        if self.active_touches.len() >= 2 && self.touch_camera_active {
                            let touches: Vec<(u64, (f32, f32))> = self.active_touches.iter().map(|(&k, &v)| (k, v)).collect();
                            if touches.len() >= 2 {
                                let (id_a, cur_a) = touches[0];
                                let (_, cur_b) = touches[1];

                                if let Some(prev_pos) = prev {
                                    let other_pos = if id == id_a { cur_b } else { cur_a };
                                    let my_prev = prev_pos;
                                    let my_cur = pos;

                                    let prev_dx = my_prev.0 - other_pos.0;
                                    let prev_dy = my_prev.1 - other_pos.1;
                                    let cur_dx = my_cur.0 - other_pos.0;
                                    let cur_dy = my_cur.1 - other_pos.1;
                                    let prev_dist = (prev_dx * prev_dx + prev_dy * prev_dy).sqrt();
                                    let cur_dist = (cur_dx * cur_dx + cur_dy * cur_dy).sqrt();

                                    if let Some(renderer) = &mut self.renderer {
                                        let (sw, sh) = renderer.surface_size();

                                        if prev_dist > 1.0 && cur_dist > 1.0 {
                                            let scale = prev_dist / cur_dist;
                                            renderer.camera.ortho_half_height = (renderer.camera.ortho_half_height * scale).clamp(0.5, 12.0);
                                        }

                                        let prev_angle = prev_dy.atan2(prev_dx);
                                        let cur_angle = cur_dy.atan2(cur_dx);
                                        let mut d_angle = cur_angle - prev_angle;
                                        if d_angle > std::f32::consts::PI { d_angle -= 2.0 * std::f32::consts::PI; }
                                        if d_angle < -std::f32::consts::PI { d_angle += 2.0 * std::f32::consts::PI; }
                                        renderer.camera.orbit_angle -= d_angle;

                                        let mid_prev = ((my_prev.0 + other_pos.0) * 0.5, (my_prev.1 + other_pos.1) * 0.5);
                                        let mid_cur = ((my_cur.0 + other_pos.0) * 0.5, (my_cur.1 + other_pos.1) * 0.5);
                                        input::apply_camera_pan(&mut renderer.camera, mid_prev, mid_cur, sw, sh);
                                    }
                                }
                            }
                        } else if self.active_touches.len() == 1 && self.drag_state.is_some() {
                            self.last_cursor_pos = pos;
                            self.update_drag_highlight(pos);
                        }
                    }
                    winit::event::TouchPhase::Ended | winit::event::TouchPhase::Cancelled => {
                        self.active_touches.remove(&id);
                        if self.active_touches.is_empty() {
                            if !self.touch_camera_active {
                                self.finish_drag(pos);
                            }
                            self.touch_camera_active = false;
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
