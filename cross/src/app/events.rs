use std::sync::Arc;

use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::{Window, WindowId};

use uzls_cross::input as game_input;
use uzls_cross::renderer::gpu::GpuRenderer;

use super::{fit_camera, App};

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }

        #[cfg(target_arch = "wasm32")]
        {
            let attrs = Window::default_attributes().with_title("Strain");
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
            web_window
                .document()
                .and_then(|doc| doc.body())
                .map(|body| body.append_child(&canvas).unwrap());

            self.window = Some(window.clone());

            let cell: super::RendererCell =
                std::rc::Rc::new(std::cell::RefCell::new(None));
            self.pending_renderer = Some(cell.clone());

            wasm_bindgen_futures::spawn_local(async move {
                let renderer = GpuRenderer::new(window).await;
                *cell.borrow_mut() = Some(renderer);
            });
        }

        #[cfg(not(target_arch = "wasm32"))]
        {
            let attrs = Window::default_attributes()
                .with_title("Strain")
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
                                    self.prof_events.push(if r.draw_flags.skip_table {
                                        "table_off"
                                    } else {
                                        "table_on"
                                    });
                                }
                                winit::keyboard::KeyCode::Digit2 => {
                                    r.draw_flags.skip_holes = !r.draw_flags.skip_holes;
                                    self.prof_events.push(if r.draw_flags.skip_holes {
                                        "holes_off"
                                    } else {
                                        "holes_on"
                                    });
                                }
                                winit::keyboard::KeyCode::Digit3 => {
                                    r.draw_flags.skip_ropes = !r.draw_flags.skip_ropes;
                                    self.prof_events.push(if r.draw_flags.skip_ropes {
                                        "ropes_off"
                                    } else {
                                        "ropes_on"
                                    });
                                }
                                winit::keyboard::KeyCode::Digit4 => {
                                    r.draw_flags.table_shadow_mode =
                                        (r.draw_flags.table_shadow_mode + 1) % 3;
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
                        game_input::apply_camera_pan(&mut renderer.camera, prev, pos, sw, sh);
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
                        renderer.camera.tilt_angle =
                            (renderer.camera.tilt_angle + scroll * 0.05).clamp(0.0, 1.2);
                    } else {
                        let factor = 1.0 - scroll * 0.1;
                        renderer.camera.ortho_half_height =
                            (renderer.camera.ortho_half_height * factor).clamp(0.5, 12.0);
                    }
                }
            }
            WindowEvent::MouseInput {
                state,
                button: winit::event::MouseButton::Right,
                ..
            } => match state {
                winit::event::ElementState::Pressed => {
                    self.right_mouse_down = true;
                    self.pan_last_pos = Some(self.last_cursor_pos);
                }
                winit::event::ElementState::Released => {
                    self.right_mouse_down = false;
                    self.pan_last_pos = None;
                }
            },
            WindowEvent::MouseInput {
                state,
                button: winit::event::MouseButton::Middle,
                ..
            } => match state {
                winit::event::ElementState::Pressed => {
                    self.middle_mouse_down = true;
                    self.pan_last_pos = Some(self.last_cursor_pos);
                }
                winit::event::ElementState::Released => {
                    self.middle_mouse_down = false;
                    self.pan_last_pos = None;
                }
            },
            WindowEvent::MouseInput {
                state,
                button: winit::event::MouseButton::Left,
                ..
            } => match state {
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
            },
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
                            let touches: Vec<(u64, (f32, f32))> =
                                self.active_touches.iter().map(|(&k, &v)| (k, v)).collect();
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
                                    let prev_dist =
                                        (prev_dx * prev_dx + prev_dy * prev_dy).sqrt();
                                    let cur_dist = (cur_dx * cur_dx + cur_dy * cur_dy).sqrt();

                                    if let Some(renderer) = &mut self.renderer {
                                        let (sw, sh) = renderer.surface_size();

                                        if prev_dist > 1.0 && cur_dist > 1.0 {
                                            let scale = prev_dist / cur_dist;
                                            renderer.camera.ortho_half_height =
                                                (renderer.camera.ortho_half_height * scale)
                                                    .clamp(0.5, 12.0);
                                        }

                                        let prev_angle = prev_dy.atan2(prev_dx);
                                        let cur_angle = cur_dy.atan2(cur_dx);
                                        let mut d_angle = cur_angle - prev_angle;
                                        if d_angle > std::f32::consts::PI {
                                            d_angle -= 2.0 * std::f32::consts::PI;
                                        }
                                        if d_angle < -std::f32::consts::PI {
                                            d_angle += 2.0 * std::f32::consts::PI;
                                        }
                                        renderer.camera.orbit_angle -= d_angle;

                                        let mid_prev = (
                                            (my_prev.0 + other_pos.0) * 0.5,
                                            (my_prev.1 + other_pos.1) * 0.5,
                                        );
                                        let mid_cur = (
                                            (my_cur.0 + other_pos.0) * 0.5,
                                            (my_cur.1 + other_pos.1) * 0.5,
                                        );
                                        game_input::apply_camera_pan(
                                            &mut renderer.camera,
                                            mid_prev,
                                            mid_cur,
                                            sw,
                                            sh,
                                        );
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
