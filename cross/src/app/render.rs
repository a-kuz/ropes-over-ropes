use web_time::Instant;

use uzls_cross::celebration;
use uzls_cross::hud::{self, render_mode_scale, HudAction, ProfilingSnapshot};
use uzls_cross::renderer::frame_types::{
    CapSettings, CartoonSettings, LightingSettings, RopeMaterialSettings, RopeVertex, TableSettings,
    VisualSettings, WormSettings,
};
use uzls_cross::renderer::rope_mesh;

use super::App;

impl App {
    #[cfg(target_arch = "wasm32")]
    fn update_web_mobile_ui_mode(&mut self) {
        let Some(window) = web_sys::window() else {
            return;
        };
        let width = window
            .inner_width()
            .ok()
            .and_then(|v| v.as_f64())
            .unwrap_or(1024.0);
        self.web_mobile_ui = width <= 900.0;
    }

    #[cfg(target_arch = "wasm32")]
    fn poll_web_ui_actions(&mut self) {
        let Some(window) = web_sys::window() else {
            return;
        };
        let Ok(value) = js_sys::Reflect::get(&window, &wasm_bindgen::JsValue::from_str("__uzlsAction")) else {
            return;
        };
        let Some(action) = value.as_string() else {
            return;
        };
        let _ = js_sys::Reflect::set(
            &window,
            &wasm_bindgen::JsValue::from_str("__uzlsAction"),
            &wasm_bindgen::JsValue::NULL,
        );

        match action.as_str() {
            "prev" if self.current_level_id > 1 => self.pending_level = Some(self.current_level_id - 1),
            "next" => self.pending_level = Some(self.current_level_id + 1),
            "restart" => self.wants_restart = true,
            _ => {}
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn sync_web_top_overlay(&mut self, visible: bool, fps: u32) {
        let Some(window) = web_sys::window() else {
            return;
        };
        let Some(document) = window.document() else {
            return;
        };
        let Some(root) = document.get_element_by_id("top-overlay") else {
            return;
        };
        let display = if visible && self.web_mobile_ui { "flex" } else { "none" };
        let _ = root.set_attribute("style", &format!("display:{};", display));
        if !visible || !self.web_mobile_ui {
            return;
        }

        if let Some(level_el) = document.get_element_by_id("hud-level") {
            level_el.set_inner_html(&format!("Level {}", self.current_level_id));
        }
        if let Some(fps_el) = document.get_element_by_id("hud-fps") {
            fps_el.set_inner_html(&format!("{} fps", fps));
        }
        if let Some(moves_el) = document.get_element_by_id("hud-moves") {
            moves_el.set_inner_html(&format!("Moves: {}", self.move_count));
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn sync_web_victory_overlay(&mut self, visible: bool) {
        let Some(window) = web_sys::window() else {
            return;
        };
        let Some(document) = window.document() else {
            return;
        };
        let Some(root) = document.get_element_by_id("victory-overlay") else {
            return;
        };

        if !visible {
            if self.web_victory_overlay_visible {
                let _ = root.set_attribute("style", "display:none;");
                self.web_victory_overlay_visible = false;
            }
            return;
        }

        let lb_result = self.leaderboard.result();
        let star_count: usize = if lb_result.ready {
            lb_result.stars as usize
        } else if (self.move_count as f32) < self.min_moves as f32 * 1.5 {
            3
        } else if (self.move_count as f32) < self.min_moves as f32 * 3.0 {
            2
        } else {
            1
        };
        let stars = (0..3)
            .map(|i| if i < star_count { "★" } else { "☆" })
            .collect::<Vec<_>>()
            .join("  ");

        if let Some(title) = document.get_element_by_id("victory-title") {
            title.set_inner_html(&format!("Level {}", self.current_level_id));
        }
        if let Some(subtitle) = document.get_element_by_id("victory-subtitle") {
            subtitle.set_inner_html("Complete");
        }
        if let Some(stars_el) = document.get_element_by_id("victory-stars") {
            stars_el.set_inner_html(&stars);
        }
        if let Some(pct_el) = document.get_element_by_id("victory-percentile") {
            if let Some(pct) = lb_result.percentile {
                if pct >= 50 {
                    pct_el.set_inner_html(&format!("Better than {}% of players", pct));
                    let _ = pct_el.set_attribute("style", "display:block;");
                } else {
                    pct_el.set_inner_html("");
                    let _ = pct_el.set_attribute("style", "display:none;");
                }
            } else {
                pct_el.set_inner_html("");
                let _ = pct_el.set_attribute("style", "display:none;");
            }
        }
        if let Some(next_btn) = document.get_element_by_id("victory-next") {
            next_btn.set_inner_html(&format!("Level {} ▶", self.current_level_id + 1));
        }

        let _ = root.set_attribute("style", "display:flex;");
        self.web_victory_overlay_visible = true;
    }

    pub fn update_and_render(&mut self) {
        #[cfg(target_arch = "wasm32")]
        if !self.init_done {
            let ready = self
                .pending_renderer
                .as_ref()
                .and_then(|c| c.borrow_mut().take());
            if let Some(renderer) = ready {
                self.pending_renderer = None;
                self.finish_init_standalone(renderer);
            }
            if !self.init_done {
                return;
            }
        }
        #[cfg(target_arch = "wasm32")]
        self.update_web_mobile_ui_mode();
        #[cfg(target_arch = "wasm32")]
        self.poll_web_ui_actions();

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
        self.fps = if self.fps == 0.0 {
            instant_fps
        } else {
            self.fps * 0.95 + instant_fps * 0.05
        };

        let t_physics = Instant::now();
        if let Some(sim) = &mut self.simulator {
            sim.update(dt);
        }
        let physics_elapsed = Instant::now().duration_since(t_physics).as_secs_f32() * 1000.0;

        let t_check_win = Instant::now();
        if let Some(ref mut timer) = self.settle_check_timer {
            *timer -= dt;
            if *timer <= 0.0 {
                if self.simulator.as_ref().map_or(false, |sim| {
                    self.drag_state.is_some()
                        || sim.drag_info.is_some()
                        || sim.has_lower_animations()
                }) {
                    *timer = 0.5;
                } else {
                    self.settle_check_timer = None;
                    self.check_win();
                }
            }
        }
        let check_win_elapsed = Instant::now().duration_since(t_check_win).as_secs_f32() * 1000.0;

        let t_mesh = Instant::now();
        let shadow_segs_elapsed = self.update_rope_buffers();
        let mesh_elapsed =
            Instant::now().duration_since(t_mesh).as_secs_f32() * 1000.0 - shadow_segs_elapsed;

        let window = match &self.window {
            Some(w) => w.clone(),
            None => return,
        };

        let level = self.current_level_id;
        let fps = self.fps as u32;
        let active_ropes = self
            .rope_endpoints
            .iter()
            .filter(|&&(s, _)| s != usize::MAX)
            .count();
        let total_ropes = self.rope_endpoints.len();

        let any_sucking = self.simulator.as_ref().map_or(false, |sim| {
            sim.bands.iter().any(|b| b.active && b.fade_out > 0.0)
        });
        let is_victory = active_ropes == 0 && total_ropes > 0 && !any_sucking;
        let mut celebration_elapsed = 0.0_f32;
        if is_victory {
            if !self.victory_submitted {
                self.victory_submitted = true;
                let elapsed =
                    web_time::Instant::now().elapsed().as_millis() as u64 - self.level_start_ms;
                let is_new_record = self.move_count < self.min_moves || self.min_moves == 0;
                self.leaderboard.submit_and_fetch(
                    self.current_level_id,
                    self.move_count,
                    elapsed as u32,
                    is_new_record,
                );
            }
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
                        &holes,
                        level.ropes.len(),
                        &self.rope_colors,
                        &self.rope_radii,
                    );
                }
                if let Some(r) = &self.renderer {
                    self.pre_victory_camera =
                        Some(uzls_cross::celebration::CameraSnapshot::capture(&r.camera));
                }
            }
            let t_celeb = Instant::now();
            celebration::update_celebration(&mut self.celebration_bands, self.victory_time);
            if let (Some(renderer), Some(snap)) = (&mut self.renderer, &self.pre_victory_camera) {
                celebration::animate_victory_camera(
                    &mut renderer.camera,
                    snap,
                    self.victory_time,
                    dt,
                );
            }
            celebration_elapsed = Instant::now().duration_since(t_celeb).as_secs_f32() * 1000.0;
        } else {
            self.victory_time = 0.0;
        }
        let victory_time = self.victory_time;
        #[cfg(target_arch = "wasm32")]
        self.sync_web_victory_overlay(victory_time > 0.0);
        #[cfg(target_arch = "wasm32")]
        self.sync_web_top_overlay(victory_time <= 0.0, fps);

        let can_undo = !self.undo_stack.is_empty();
        let render_mode = self.render_mode;
        let cel_mode = self.cel_mode;
        let prof_show = self.prof_show;
        let draw_flags = self
            .renderer
            .as_ref()
            .map(|r| r.draw_flags)
            .unwrap_or_default();
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
        let prev_settings = self.current_persisted_settings();
        let mut settings_changed = false;
        let egui_output = if let Some(egui_state) = &mut self.egui_state {
            #[cfg(target_arch = "wasm32")]
            if self.web_mobile_ui {
                None
            } else {
            let ctx = egui_state.egui_ctx().clone();
            let raw_input = egui_state.take_egui_input(&window);
            let mut hud_action = HudAction::default();
            let min_moves = self.min_moves;
            let settings_open = self.settings_open;
            let (
                mut rope_mat,
                mut lighting_settings,
                mut visual_settings,
                mut table_settings,
                mut cartoon_settings,
                mut cap_settings,
                mut worm_settings,
                mut exact_render_scale,
            ) = self
                .renderer
                .as_ref()
                .map(|r| {
                    (
                        r.rope_material.clone(),
                        r.lighting.clone(),
                        r.visual.clone(),
                        r.table.clone(),
                        r.cartoon.clone(),
                        r.cap.clone(),
                        r.worm.clone(),
                        r.render_scale,
                    )
                })
                .unwrap_or((
                    RopeMaterialSettings::default(),
                    LightingSettings::default(),
                    VisualSettings::default(),
                    TableSettings::default(),
                    CartoonSettings::default(),
                    CapSettings::default(),
                    WormSettings::default(),
                    render_mode_scale(self.render_mode),
                ));
            let mut cel_mode_local = self.cel_mode;
            let mut sq_cross = self.square_cross_section;
            let move_count = self.move_count;
            let full_output = ctx.run(raw_input, |ctx| {
                let lb_result = self.leaderboard.result();
                hud_action = hud::draw_hud(
                    ctx,
                    level,
                    fps,
                    active_ropes,
                    total_ropes,
                    victory_time,
                    can_undo,
                    render_mode,
                    move_count,
                    min_moves,
                    &mut cel_mode_local,
                    prof_show,
                    &prof_data,
                    settings_open,
                    &mut rope_mat,
                    &mut lighting_settings,
                    &mut visual_settings,
                    &mut table_settings,
                    &mut cartoon_settings,
                    &mut cap_settings,
                    &mut worm_settings,
                    &mut exact_render_scale,
                    &mut sq_cross,
                    self.current_level_id,
                    &lb_result,
                );
            });
            if let Some(r) = &mut self.renderer {
                r.rope_material = rope_mat;
                r.lighting = lighting_settings;
                r.visual = visual_settings;
                r.visual.square_cross_section = sq_cross;
                r.table = table_settings;
                r.cartoon = cartoon_settings;
                r.cap = cap_settings;
                r.worm = worm_settings;
                r.set_render_scale(exact_render_scale);
                if let Some(level) = &self.level {
                    let hole_positions = level.hole_positions();
                    let hole_elevations_for_render = level.hole_elevations();
                    let scaled_hole_radius = level.hole_radius * r.visual.hole_radius_scale;
                    if sq_cross {
                        r.update_hole_instances_square(
                            &hole_positions,
                            &hole_elevations_for_render,
                            scaled_hole_radius,
                        );
                    } else {
                        r.update_hole_instances(
                            &hole_positions,
                            &hole_elevations_for_render,
                            scaled_hole_radius,
                        );
                    }
                }
            }
            self.square_cross_section = sq_cross;
            if let Some(sim) = &mut self.simulator {
                sim.square_cross_section = sq_cross;
            }
            self.cel_mode = cel_mode_local;
            egui_state.handle_platform_output(&window, full_output.platform_output.clone());
            settings_changed = self.current_persisted_settings() != prev_settings;
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
                settings_changed = true;
            }
            if hud_action.toggle_cel {
                self.cel_mode = !self.cel_mode;
                self.prof_events
                    .push(if self.cel_mode { "cel_on" } else { "cel_off" });
                settings_changed = true;
            }
            if hud_action.toggle_prof {
                self.prof_show = !self.prof_show;
                #[cfg(not(target_arch = "wasm32"))]
                self.toggle_prof_file();
            }
            if hud_action.toggle_settings {
                self.settings_open = !self.settings_open;
            }
            Some((full_output, ctx))
            }
            #[cfg(not(target_arch = "wasm32"))]
            {
            let ctx = egui_state.egui_ctx().clone();
            let raw_input = egui_state.take_egui_input(&window);
            let mut hud_action = HudAction::default();
            let min_moves = self.min_moves;
            let settings_open = self.settings_open;
            let (
                mut rope_mat,
                mut lighting_settings,
                mut visual_settings,
                mut table_settings,
                mut cartoon_settings,
                mut cap_settings,
                mut worm_settings,
                mut exact_render_scale,
            ) = self
                .renderer
                .as_ref()
                .map(|r| {
                    (
                        r.rope_material.clone(),
                        r.lighting.clone(),
                        r.visual.clone(),
                        r.table.clone(),
                        r.cartoon.clone(),
                        r.cap.clone(),
                        r.worm.clone(),
                        r.render_scale,
                    )
                })
                .unwrap_or((
                    RopeMaterialSettings::default(),
                    LightingSettings::default(),
                    VisualSettings::default(),
                    TableSettings::default(),
                    CartoonSettings::default(),
                    CapSettings::default(),
                    WormSettings::default(),
                    render_mode_scale(self.render_mode),
                ));
            let mut cel_mode_local = self.cel_mode;
            let mut sq_cross = self.square_cross_section;
            let move_count = self.move_count;
            let full_output = ctx.run(raw_input, |ctx| {
                let lb_result = self.leaderboard.result();
                hud_action = hud::draw_hud(
                    ctx,
                    level,
                    fps,
                    active_ropes,
                    total_ropes,
                    victory_time,
                    can_undo,
                    render_mode,
                    move_count,
                    min_moves,
                    &mut cel_mode_local,
                    prof_show,
                    &prof_data,
                    settings_open,
                    &mut rope_mat,
                    &mut lighting_settings,
                    &mut visual_settings,
                    &mut table_settings,
                    &mut cartoon_settings,
                    &mut cap_settings,
                    &mut worm_settings,
                    &mut exact_render_scale,
                    &mut sq_cross,
                    self.current_level_id,
                    &lb_result,
                );
            });
            if let Some(r) = &mut self.renderer {
                r.rope_material = rope_mat;
                r.lighting = lighting_settings;
                r.visual = visual_settings;
                r.visual.square_cross_section = sq_cross;
                r.table = table_settings;
                r.cartoon = cartoon_settings;
                r.cap = cap_settings;
                r.set_render_scale(exact_render_scale);
                if let Some(level) = &self.level {
                    let hole_positions = level.hole_positions();
                    let hole_elevations_for_render = level.hole_elevations();
                    let scaled_hole_radius = level.hole_radius * r.visual.hole_radius_scale;
                    if sq_cross {
                        r.update_hole_instances_square(
                            &hole_positions,
                            &hole_elevations_for_render,
                            scaled_hole_radius,
                        );
                    } else {
                        r.update_hole_instances(
                            &hole_positions,
                            &hole_elevations_for_render,
                            scaled_hole_radius,
                        );
                    }
                }
            }
            self.square_cross_section = sq_cross;
            if let Some(sim) = &mut self.simulator {
                sim.square_cross_section = sq_cross;
            }
            self.cel_mode = cel_mode_local;
            egui_state.handle_platform_output(&window, full_output.platform_output.clone());
            settings_changed = self.current_persisted_settings() != prev_settings;
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
                settings_changed = true;
            }
            if hud_action.toggle_cel {
                self.cel_mode = !self.cel_mode;
                self.prof_events
                    .push(if self.cel_mode { "cel_on" } else { "cel_off" });
                settings_changed = true;
            }
            if hud_action.toggle_prof {
                self.prof_show = !self.prof_show;
                #[cfg(not(target_arch = "wasm32"))]
                self.toggle_prof_file();
            }
            if hud_action.toggle_settings {
                self.settings_open = !self.settings_open;
            }
            Some((full_output, ctx))
            }
        } else {
            None
        };
        if settings_changed {
            self.save_persisted_settings();
        }
        let egui_elapsed = Instant::now().duration_since(t_egui).as_secs_f32() * 1000.0;

        let renderer = match &mut self.renderer {
            Some(r) => r,
            None => return,
        };
        let scale = window.scale_factor() as f32;
        let (w, h) = renderer.surface_size();

        let level_seed = self.current_level_id as f32;
        renderer.highlight_hole = self.highlight_hole;
        let mut frame = match renderer.begin_frame(
            self.time,
            self.drag_state.is_some(),
            victory_time,
            level_seed,
            self.render_mode as u32,
            self.cel_mode,
        ) {
            Some(f) => f,
            None => return,
        };

        {
            if let (Some((egui_output, egui_ctx)), Some(egui_rend)) =
                (egui_output, self.egui_renderer.as_mut())
            {
                let sd = egui_wgpu::ScreenDescriptor {
                    size_in_pixels: [w, h],
                    pixels_per_point: scale,
                };
                let prims = egui_ctx.tessellate(egui_output.shapes, scale);
                let dev = renderer.device();
                let q = renderer.queue();
                for (id, delta) in &egui_output.textures_delta.set {
                    egui_rend.update_texture(dev, q, *id, delta);
                }
                egui_rend.update_buffers(dev, q, &mut frame.encoder, &prims, &sd);
                let mut pass = frame
                    .encoder
                    .begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: Some("egui_pass"),
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &frame.screen_view,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Load,
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: None,
                        timestamp_writes: None,
                        occlusion_query_set: None,
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
            let tsm_letter = match df.table_shadow_mode {
                0 => "P",
                1 => "L",
                _ => "N",
            };
            let skip = format!(
                "{}{}{}{}",
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
    pub fn toggle_prof_file(&mut self) {
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

    pub fn update_rope_buffers(&mut self) -> f32 {
        let sim = match &self.simulator {
            Some(s) => s,
            None => return 0.0,
        };

        let mut all_vertices: Vec<RopeVertex> = Vec::new();
        let mut all_indices: Vec<u32> = Vec::new();
        let mut base_vertex: u32 = 0;

        let rope_scale = self
            .renderer
            .as_ref()
            .map(|r| r.lighting.rope_radius_scale)
            .unwrap_or(1.062);
        let rope_visual_mul = 1.3 * rope_scale;
        let (profile_segments, hole_radius_scale, cap_segments, cap_scale, cap_smin_k) = self
            .renderer
            .as_ref()
            .map(|r| {
                (
                    r.visual.profile_segments,
                    r.visual.hole_radius_scale,
                    r.cap.segments,
                    r.cap.radius_scale,
                    r.cap.smin_k,
                )
            })
            .unwrap_or((10, 0.734, 12, 1.0, 0.18));

        let _ = hole_radius_scale;
        let _ = cap_segments;

        let mut all_rope_points: Vec<&[glam::Vec3]> = Vec::new();
        let mut all_rope_radii_scaled: Vec<f32> = Vec::new();
        for rope_index in 0..self.rope_endpoints.len() {
            if rope_index >= sim.bands.len()
                || !sim.bands[rope_index].active
                || (sim.bands[rope_index].fade_out == 0.0
                    && self.rope_endpoints[rope_index].0 == usize::MAX)
            {
                all_rope_points.push(&[]);
                all_rope_radii_scaled.push(0.0);
                continue;
            }
            all_rope_points.push(&sim.bands[rope_index].positions);
            let r = if rope_index < self.rope_radii.len() {
                self.rope_radii[rope_index]
            } else {
                0.038
            };
            all_rope_radii_scaled.push(r * rope_visual_mul);
        }

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
            let visual_radius = radius * rope_visual_mul;

            let mut contact_points: Vec<glam::Vec2> = Vec::new();
            for other_idx in 0..all_rope_points.len() {
                if other_idx == rope_index || all_rope_points[other_idx].is_empty() {
                    continue;
                }
                let other_pts = all_rope_points[other_idx];
                let rj = all_rope_radii_scaled[other_idx];
                let threshold = (visual_radius + rj) * 1.5;
                let threshold_sq = threshold * threshold;
                for s in 0..other_pts.len().saturating_sub(1) {
                    let mid = (other_pts[s] + other_pts[s + 1]) * 0.5;
                    let mid2 = glam::Vec2::new(mid.x, mid.y);
                    let mut min_d2 = f32::MAX;
                    for p in &band.positions {
                        let d2 = (glam::Vec2::new(p.x, p.y) - mid2).length_squared();
                        if d2 < min_d2 {
                            min_d2 = d2;
                        }
                    }
                    if min_d2 < threshold_sq {
                        contact_points.push(mid2);
                    }
                }
            }

            // Clip positions during suck animation (hide consumed particles below board)
            let visible_positions: Vec<glam::Vec3>;
            let render_positions: &[glam::Vec3] =
                if band.fade_out > 0.0 && band.suck_hole.is_some() {
                    let pts = &band.positions;
                    let n = pts.len();
                    let clip_z = -visual_radius * 1.5;
                    let mut lo = 0;
                    let mut hi = n as i32 - 1;
                    while lo < n && pts[lo].z < clip_z {
                        lo += 1;
                    }
                    while hi >= 0 && pts[hi as usize].z < clip_z {
                        hi -= 1;
                    }
                    if hi < 0 {
                        continue;
                    }
                    let hi = hi as usize;
                    if hi > lo {
                        let mut clipped: Vec<glam::Vec3> = pts[lo..=hi].to_vec();
                        let sink_z = -visual_radius * 2.5;
                        if lo > 0 {
                            if let Some(sh) = band.suck_hole {
                                if let Some(level) = &self.level {
                                    if sh < level.holes.len() {
                                        let h = &level.holes[sh];
                                        clipped.insert(
                                            0,
                                            glam::Vec3::new(h.x_position, h.y_position, sink_z),
                                        );
                                    }
                                }
                            }
                        }
                        if hi < n - 1 {
                            let tail_h = band.suck_tail_hole.or(band.suck_hole);
                            if let Some(th) = tail_h {
                                if let Some(level) = &self.level {
                                    if th < level.holes.len() {
                                        let h = &level.holes[th];
                                        clipped.push(glam::Vec3::new(
                                            h.x_position,
                                            h.y_position,
                                            sink_z,
                                        ));
                                    }
                                }
                            }
                        }
                        visible_positions = clipped;
                        &visible_positions
                    } else {
                        continue;
                    }
                } else {
                    &band.positions
                };

            let rest_length =
                band.segment_length * (render_positions.len() as f32 - 1.0).max(1.0);

            let mesh = rope_mesh::build_rect(
                render_positions,
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
                profile_segments,
                self.square_cross_section,
                &contact_points,
                visual_radius,
                self.renderer
                    .as_ref()
                    .map(|r| r.visual.stretch_thinning)
                    .unwrap_or(0.5),
                cap_scale,
                cap_smin_k,
            );

            for idx in &mesh.indices {
                all_indices.push(idx + base_vertex);
            }
            all_vertices.extend_from_slice(&mesh.vertices);
            base_vertex += mesh.vertices.len() as u32;
        }

        for (ci, cband) in self.celebration_bands.iter().enumerate() {
            if cband.positions.len() < 2 {
                continue;
            }
            let color = glam::Vec3::from_array(cband.color);
            let visual_radius = cband.radius * 1.3;
            let rest_length =
                cband.segment_length * (cband.positions.len() as f32 - 1.0).max(1.0);
            let cross_section = uzls_cross::level::definition::CrossSection::Circular {
                radius: cband.radius,
            };

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
                profile_segments,
                false,
                &[],
                0.0,
                self.renderer
                    .as_ref()
                    .map(|r| r.visual.stretch_thinning)
                    .unwrap_or(0.5),
                cap_scale,
                cap_smin_k,
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
        let need_shadow_segments =
            !self.cel_mode && (self.render_mode > 0 || table_shadow_mode == 1);

        self.prof_rope_verts = all_vertices.len() as u32;
        self.prof_rope_tris = all_indices.len() as u32 / 3;

        if let Some(renderer) = &mut self.renderer {
            renderer.update_rope_mesh(&all_vertices, &all_indices);
        }

        let t_shadow_segs = Instant::now();
        if need_shadow_segments {
            let mut shadow_segs: Vec<uzls_cross::renderer::frame_types::ShadowSegment> =
                Vec::new();
            for rope_index in 0..self.rope_endpoints.len() {
                if rope_index >= sim.bands.len() || !sim.bands[rope_index].active {
                    continue;
                }
                let band = &sim.bands[rope_index];
                if band.fade_out > 0.5 {
                    continue;
                }
                if self.rope_endpoints[rope_index].0 == usize::MAX {
                    continue;
                }

                let radius = if rope_index < self.rope_radii.len() {
                    self.rope_radii[rope_index]
                } else {
                    0.038
                };
                let pts = &band.positions;
                if pts.len() < 2 {
                    continue;
                }
                let step = if pts.len() > 12 { 4 } else { 1 };
                let mut i = 0;
                while i < pts.len() - 1 {
                    let next = (i + step).min(pts.len() - 1);
                    let az = pts[i].z;
                    let bz = pts[next].z;
                    if az <= 0.0 && bz <= 0.0 {
                        i = next;
                        continue;
                    }
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
                        ax: a.x,
                        ay: a.y,
                        az: a.z,
                        radius,
                        bx: b.x,
                        by: b.y,
                        bz: b.z,
                        rope_id: rope_index as f32,
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
}
