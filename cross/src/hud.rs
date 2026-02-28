use crate::renderer::gpu::GpuTimings;

pub struct HudAction {
    pub go_to_level: Option<usize>,
    pub restart: bool,
    pub undo: bool,
    pub toggle_hd: bool,
    pub toggle_cel: bool,
    pub toggle_prof: bool,
}

impl Default for HudAction {
    fn default() -> Self {
        Self {
            go_to_level: None,
            restart: false,
            undo: false,
            toggle_hd: false,
            toggle_cel: false,
            toggle_prof: false,
        }
    }
}

pub struct ProfilingSnapshot {
    pub physics_ms: f32,
    pub mesh_ms: f32,
    pub egui_ms: f32,
    pub frame_ms: f32,
    pub check_win_ms: f32,
    pub shadow_segs_ms: f32,
    pub submit_ms: f32,
    pub celebration_ms: f32,
    pub gpu: GpuTimings,
    pub rope_verts: u32,
    pub rope_tris: u32,
    pub skip_table: bool,
    pub skip_holes: bool,
    pub skip_ropes: bool,
    pub table_shadow_mode: u8,
}

pub fn render_mode_scale(mode: u8) -> f32 {
    match mode {
        0 => 0.5,
        2 => 2.0,
        _ => 1.0,
    }
}

pub fn draw_hud(ctx: &egui::Context, level: usize, fps: u32, _active_ropes: usize, _total_ropes: usize, victory_time: f32, can_undo: bool, render_mode: u8, move_count: u32, min_moves: u32, cel_mode: bool, prof_show: bool, prof: &ProfilingSnapshot) -> HudAction {
    let mut action = HudAction::default();
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
                        #[cfg(target_arch = "wasm32")]
                        {
                            let label_resp = ui.add(
                                egui::Label::new(
                                    egui::RichText::new(format!("Level {}", level))
                                        .color(egui::Color32::WHITE)
                                        .size(18.0)
                                        .strong(),
                                ).sense(egui::Sense::click()),
                            );
                            if label_resp.clicked() {
                                if let Some(val) = web_sys::window()
                                    .and_then(|w| w.prompt_with_message("Go to level:").ok().flatten())
                                {
                                    if let Ok(n) = val.trim().parse::<usize>() {
                                        if n >= 1 { action.go_to_level = Some(n); }
                                    }
                                }
                            }
                        }
                        #[cfg(not(target_arch = "wasm32"))]
                        {
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
                        }
                        if btn(ui, "\u{25B6}").clicked() {
                            action.go_to_level = Some(level + 1);
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label(
                            egui::RichText::new(format!("{} fps", fps))
                                .color(egui::Color32::from_white_alpha(100))
                                .size(11.0)
                                .monospace(),
                        );
                        ui.add_space(8.0);
                        ui.label(
                            egui::RichText::new(format!("Moves: {}", move_count))
                                .color(egui::Color32::from_white_alpha(180))
                                .size(12.0),
                        );
                    });
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
                let (hd_label, hd_color) = match render_mode {
                    0 => ("SD", egui::Color32::from_white_alpha(150)),
                    2 => ("UHD", egui::Color32::from_rgb(200, 160, 60)),
                    _ => ("HD", egui::Color32::from_rgb(100, 200, 100)),
                };
                if ui.add(
                    egui::Button::new(
                        egui::RichText::new(hd_label).color(hd_color).size(14.0).strong(),
                    )
                    .fill(egui::Color32::from_black_alpha(140))
                    .corner_radius(22)
                    .min_size(egui::vec2(44.0, 44.0)),
                ).clicked() {
                    action.toggle_hd = true;
                }
                let cel_color = if cel_mode {
                    egui::Color32::from_rgb(255, 180, 60)
                } else {
                    egui::Color32::from_white_alpha(120)
                };
                if ui.add(
                    egui::Button::new(
                        egui::RichText::new("CEL").color(cel_color).size(14.0).strong(),
                    )
                    .fill(egui::Color32::from_black_alpha(140))
                    .corner_radius(22)
                    .min_size(egui::vec2(44.0, 44.0)),
                ).clicked() {
                    action.toggle_cel = true;
                }
                let prof_color = if prof_show {
                    egui::Color32::from_rgb(100, 255, 180)
                } else {
                    egui::Color32::from_white_alpha(120)
                };
                if ui.add(
                    egui::Button::new(
                        egui::RichText::new("\u{23F1}").color(prof_color).size(16.0),
                    )
                    .fill(egui::Color32::from_black_alpha(140))
                    .corner_radius(22)
                    .min_size(egui::vec2(44.0, 44.0)),
                ).clicked() {
                    action.toggle_prof = true;
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
        let btn_alpha = ((appear_t - 0.3).max(0.0) / 0.3).min(1.0);

        let is_perfect = move_count <= min_moves;
        let star_count = if (move_count as f32) < min_moves as f32 * 1.5 {
            3
        } else if (move_count as f32) < min_moves as f32 * 3.0 {
            2
        } else {
            1
        };

        let star_delay = [0.5, 0.9, 1.3];
        let star_anim: Vec<f32> = (0..3).map(|i| {
            ((victory_time - star_delay[i]).max(0.0) / 0.3).min(1.0)
        }).collect();

        let perfect_anim = if is_perfect {
            ((victory_time - 1.6).max(0.0) / 0.4).min(1.0)
        } else {
            0.0
        };

        egui::Area::new(egui::Id::new("victory"))
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, -20.0 * (1.0 - appear_t)))
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::from_black_alpha(bg_alpha))
                    .corner_radius(20)
                    .inner_margin(egui::Margin { left: 40, right: 40, top: 24, bottom: 24 })
                    .show(ui, |ui| {
                        ui.vertical_centered(|ui| {
                            ui.label(
                                egui::RichText::new("YOU WIN!")
                                    .color(egui::Color32::from_rgba_unmultiplied(255, 220, 60, alpha))
                                    .size(48.0 * bounce)
                                    .strong(),
                            );
                            ui.add_space(4.0);
                            ui.label(
                                egui::RichText::new(format!("Level {}", level))
                                    .color(egui::Color32::from_rgba_unmultiplied(255, 255, 255, (alpha as f32 * 0.8) as u8))
                                    .size(20.0)
                                    .strong(),
                            );
                            ui.add_space(8.0);
                            {
                                let mut job = egui::text::LayoutJob::default();
                                for i in 0..3 {
                                    if i > 0 {
                                        job.append("  ", 0.0, egui::TextFormat {
                                            font_id: egui::FontId::proportional(40.0),
                                            color: egui::Color32::TRANSPARENT,
                                            ..Default::default()
                                        });
                                    }
                                    let filled = i < star_count && star_anim[i] > 0.01;
                                    let anim_scale = star_anim[i];
                                    let c = if filled {
                                        let glow = (anim_scale * 255.0).min(255.0) as u8;
                                        egui::Color32::from_rgba_unmultiplied(255, 210, 50, glow)
                                    } else {
                                        let dim = (star_anim[i] * 100.0).min(100.0) as u8;
                                        egui::Color32::from_white_alpha(dim)
                                    };
                                    let size = 40.0 * (0.5 + 0.5 * anim_scale);
                                    job.append(
                                        if filled { "\u{2605}" } else { "\u{2606}" },
                                        0.0,
                                        egui::TextFormat {
                                            font_id: egui::FontId::proportional(size),
                                            color: c,
                                            ..Default::default()
                                        },
                                    );
                                }
                                job.halign = egui::Align::Center;
                                ui.label(job);
                            }
                            ui.add_space(6.0);
                            if is_perfect && perfect_anim > 0.01 {
                                let pa = perfect_anim;
                                let glow = ((victory_time * 4.0).sin() * 0.15 + 0.85).clamp(0.0, 1.0);
                                let perfect_alpha = (pa * 255.0 * glow) as u8;
                                let scale = 1.0 + (1.0 - pa) * 0.5;
                                ui.label(
                                    egui::RichText::new("\u{2728} PERFECT \u{2728}")
                                        .color(egui::Color32::from_rgba_unmultiplied(255, 200, 50, perfect_alpha))
                                        .size(28.0 * scale)
                                        .strong(),
                                );
                                ui.add_space(4.0);
                            }
                            ui.label(
                                egui::RichText::new(format!("Moves: {}", move_count))
                                    .color(egui::Color32::from_rgba_unmultiplied(200, 220, 255, alpha))
                                    .size(18.0),
                            );
                            ui.add_space(12.0);
                            if btn_alpha > 0.01 {
                                let btn_color = egui::Color32::from_rgba_unmultiplied(100, 160, 255, (btn_alpha * 255.0) as u8);
                                if ui.add(
                                    egui::Button::new(
                                        egui::RichText::new(format!("Level {} \u{25B6}", level + 1))
                                            .color(egui::Color32::from_rgba_unmultiplied(255, 255, 255, (btn_alpha * 255.0) as u8))
                                            .size(20.0)
                                            .strong(),
                                    )
                                    .fill(btn_color)
                                    .corner_radius(22)
                                    .min_size(egui::vec2(160.0, 44.0)),
                                ).clicked() {
                                    action.go_to_level = Some(level + 1);
                                }
                            }
                        });
                    });
            });

        ctx.request_repaint();
    }

    egui::Area::new(egui::Id::new("debug_min"))
        .anchor(egui::Align2::RIGHT_BOTTOM, egui::vec2(-8.0, -4.0))
        .show(ctx, |ui| {
            ui.label(
                egui::RichText::new(format!("par {}", min_moves))
                    .color(egui::Color32::from_white_alpha(35))
                    .size(10.0)
                    .monospace(),
            );
        });

    if prof_show {
        egui::Area::new(egui::Id::new("profiling_overlay"))
            .anchor(egui::Align2::LEFT_BOTTOM, egui::vec2(12.0, -12.0))
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::from_black_alpha(200))
                    .corner_radius(8)
                    .inner_margin(egui::Margin::same(10))
                    .show(ui, |ui| {
                        let mono = |s: String, c: egui::Color32| {
                            egui::RichText::new(s).color(c).size(11.0).monospace()
                        };
                        let dim = egui::Color32::from_white_alpha(140);
                        let bright = egui::Color32::from_white_alpha(220);
                        let gpu_c = egui::Color32::from_rgb(120, 200, 255);
                        let cpu_c = egui::Color32::from_rgb(180, 255, 120);

                        ui.label(mono("--- CPU ---".into(), cpu_c));
                        ui.label(mono(format!("frame      {:6.2} ms", prof.frame_ms), bright));
                        ui.label(mono(format!("  physics  {:6.2} ms", prof.physics_ms), dim));
                        ui.label(mono(format!("  mesh     {:6.2} ms", prof.mesh_ms), dim));
                        ui.label(mono(format!("  shadow   {:6.2} ms", prof.shadow_segs_ms), dim));
                        ui.label(mono(format!("  checkwin {:6.2} ms", prof.check_win_ms), dim));
                        ui.label(mono(format!("  celebr   {:6.2} ms", prof.celebration_ms), dim));
                        ui.label(mono(format!("  egui     {:6.2} ms", prof.egui_ms), dim));
                        ui.label(mono(format!("  submit   {:6.2} ms", prof.submit_ms), dim));
                        ui.add_space(4.0);
                        ui.label(mono("--- GPU ---".into(), gpu_c));
                        ui.label(mono(format!("total      {:6.2} ms", prof.gpu.total_ms), bright));
                        ui.label(mono(format!("  shadow   {:6.2} ms", prof.gpu.shadow_ms), dim));
                        ui.label(mono(format!("  hdr      {:6.2} ms", prof.gpu.hdr_ms), dim));
                        ui.label(mono(format!("  bloom    {:6.2} ms", prof.gpu.bloom_ms), dim));
                        ui.add_space(4.0);
                        ui.label(mono("--- Mesh ---".into(), egui::Color32::from_rgb(255, 200, 120)));
                        ui.label(mono(format!("verts      {:>6}", prof.rope_verts), dim));
                        ui.label(mono(format!("tris       {:>6}", prof.rope_tris), dim));
                        ui.add_space(4.0);
                        let on_off = |on: bool| if on { "OFF" } else { " on" };
                        let flag_c = |on: bool| if on { egui::Color32::from_rgb(255, 80, 80) } else { dim };
                        ui.label(mono(format!("1 table  [{}]", on_off(prof.skip_table)), flag_c(prof.skip_table)));
                        ui.label(mono(format!("2 holes  [{}]", on_off(prof.skip_holes)), flag_c(prof.skip_holes)));
                        ui.label(mono(format!("3 ropes  [{}]", on_off(prof.skip_ropes)), flag_c(prof.skip_ropes)));
                        let tshadow_label = match prof.table_shadow_mode { 0 => "pcf", 1 => "planar", _ => "off" };
                        let tshadow_c = if prof.table_shadow_mode == 2 { egui::Color32::from_rgb(255, 80, 80) } else { dim };
                        ui.label(mono(format!("4 tshadow [{}]", tshadow_label), tshadow_c));
                    });
            });
    }

    action
}
