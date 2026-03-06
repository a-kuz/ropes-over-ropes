use crate::leaderboard::LeaderboardResult;
use crate::renderer::frame_types::{
    CapSettings, CartoonSettings, LightingSettings, RopeMaterialSettings, TableSettings,
    VisualSettings,
};
use crate::renderer::gpu::GpuTimings;
use serde_json::Value;

pub struct HudAction {
    pub go_to_level: Option<usize>,
    pub restart: bool,
    pub undo: bool,
    pub toggle_hd: bool,
    pub toggle_cel: bool,
    pub toggle_prof: bool,
    pub toggle_settings: bool,
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
            toggle_settings: false,
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

pub fn draw_hud(
    ctx: &egui::Context,
    level: usize,
    fps: u32,
    _active_ropes: usize,
    _total_ropes: usize,
    victory_time: f32,
    can_undo: bool,
    render_mode: u8,
    move_count: u32,
    min_moves: u32,
    cel_mode: &mut bool,
    prof_show: bool,
    prof: &ProfilingSnapshot,
    settings_open: bool,
    rope_mat: &mut RopeMaterialSettings,
    lighting: &mut LightingSettings,
    visual: &mut VisualSettings,
    table: &mut TableSettings,
    cartoon: &mut CartoonSettings,
    cap: &mut CapSettings,
    render_scale: &mut f32,
    square_cross: &mut bool,
    lb_result: &LeaderboardResult,
) -> HudAction {
    let mut action = HudAction::default();
    let level_input_id = egui::Id::new("level_input_active");
    let level_text_id = egui::Id::new("level_input_text");

    let btn = |ui: &mut egui::Ui, text: &str| -> egui::Response {
        ui.add(
            egui::Button::new(
                egui::RichText::new(text)
                    .color(egui::Color32::WHITE)
                    .size(18.0),
            )
            .fill(egui::Color32::from_black_alpha(140))
            .corner_radius(22)
            .min_size(egui::vec2(44.0, 44.0)),
        )
    };

    if victory_time <= 0.0 {
        egui::Area::new(egui::Id::new("level_badge"))
            .fixed_pos(egui::pos2(12.0, 12.0))
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::from_black_alpha(140))
                    .corner_radius(10)
                    .inner_margin(egui::Margin {
                        left: 14,
                        right: 14,
                        top: 8,
                        bottom: 8,
                    })
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
                                    )
                                    .sense(egui::Sense::click()),
                                );
                                if label_resp.clicked() {
                                    if let Some(val) = web_sys::window().and_then(|w| {
                                        w.prompt_with_message("Go to level:").ok().flatten()
                                    }) {
                                        if let Ok(n) = val.trim().parse::<usize>() {
                                            if n >= 1 {
                                                action.go_to_level = Some(n);
                                            }
                                        }
                                    }
                                }
                            }
                            #[cfg(not(target_arch = "wasm32"))]
                            {
                                let mut editing: bool =
                                    ctx.data_mut(|d| *d.get_temp_mut_or(level_input_id, false));
                                if editing {
                                    let mut text: String = ctx.data_mut(|d| {
                                        d.get_temp_mut_or(level_text_id, String::new()).clone()
                                    });
                                    let resp = ui.add(
                                        egui::TextEdit::singleline(&mut text)
                                            .desired_width(50.0)
                                            .font(egui::TextStyle::Heading),
                                    );
                                    ctx.data_mut(|d| d.insert_temp(level_text_id, text.clone()));
                                    if resp.lost_focus()
                                        || ui.input(|i| i.key_pressed(egui::Key::Enter))
                                    {
                                        if let Ok(n) = text.trim().parse::<usize>() {
                                            if n >= 1 {
                                                action.go_to_level = Some(n);
                                            }
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
                                        )
                                        .sense(egui::Sense::click()),
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
                                .color(if can_undo {
                                    egui::Color32::WHITE
                                } else {
                                    egui::Color32::from_white_alpha(80)
                                })
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
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new(hd_label)
                                    .color(hd_color)
                                    .size(14.0)
                                    .strong(),
                            )
                            .fill(egui::Color32::from_black_alpha(140))
                            .corner_radius(22)
                            .min_size(egui::vec2(44.0, 44.0)),
                        )
                        .clicked()
                    {
                        action.toggle_hd = true;
                    }
                    let cel_color = if *cel_mode {
                        egui::Color32::from_rgb(255, 180, 60)
                    } else {
                        egui::Color32::from_white_alpha(120)
                    };
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("CEL")
                                    .color(cel_color)
                                    .size(14.0)
                                    .strong(),
                            )
                            .fill(egui::Color32::from_black_alpha(140))
                            .corner_radius(22)
                            .min_size(egui::vec2(44.0, 44.0)),
                        )
                        .clicked()
                    {
                        action.toggle_cel = true;
                    }
                    let prof_color = if prof_show {
                        egui::Color32::from_rgb(100, 255, 180)
                    } else {
                        egui::Color32::from_white_alpha(120)
                    };
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("\u{23F1}").color(prof_color).size(16.0),
                            )
                            .fill(egui::Color32::from_black_alpha(140))
                            .corner_radius(22)
                            .min_size(egui::vec2(44.0, 44.0)),
                        )
                        .clicked()
                    {
                        action.toggle_prof = true;
                    }
                    let settings_color = if settings_open {
                        egui::Color32::from_rgb(255, 180, 60)
                    } else {
                        egui::Color32::from_white_alpha(120)
                    };
                    if ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new("\u{2699}")
                                    .color(settings_color)
                                    .size(18.0),
                            )
                            .fill(egui::Color32::from_black_alpha(140))
                            .corner_radius(22)
                            .min_size(egui::vec2(44.0, 44.0)),
                        )
                        .clicked()
                    {
                        action.toggle_settings = true;
                    }
                });
            });
    } // end if victory_time <= 0.0

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

        let star_count: usize = if lb_result.ready {
            lb_result.stars as usize
        } else if (move_count as f32) < min_moves as f32 * 1.5 {
            3
        } else if (move_count as f32) < min_moves as f32 * 3.0 {
            2
        } else {
            1
        };
        let is_perfect = star_count >= 3;

        let star_delay = [0.5, 0.9, 1.3];
        let star_anim: Vec<f32> = (0..3)
            .map(|i| ((victory_time - star_delay[i]).max(0.0) / 0.3).min(1.0))
            .collect();

        let perfect_anim = if is_perfect {
            ((victory_time - 1.6).max(0.0) / 0.4).min(1.0)
        } else {
            0.0
        };

        egui::Area::new(egui::Id::new("victory"))
            .anchor(
                egui::Align2::CENTER_CENTER,
                egui::vec2(0.0, -20.0 * (1.0 - appear_t)),
            )
            .show(ctx, |ui| {
                egui::Frame::none()
                    .fill(egui::Color32::from_black_alpha(bg_alpha))
                    .corner_radius(20)
                    .inner_margin(egui::Margin {
                        left: 40,
                        right: 40,
                        top: 24,
                        bottom: 24,
                    })
                    .show(ui, |ui| {
                        ui.vertical_centered(|ui| {
                            ui.label(
                                egui::RichText::new("YOU WIN!")
                                    .color(egui::Color32::from_rgba_unmultiplied(
                                        255, 220, 60, alpha,
                                    ))
                                    .size(48.0 * bounce)
                                    .strong(),
                            );
                            ui.add_space(4.0);
                            ui.label(
                                egui::RichText::new(format!("Level {}", level))
                                    .color(egui::Color32::from_rgba_unmultiplied(
                                        255,
                                        255,
                                        255,
                                        (alpha as f32 * 0.8) as u8,
                                    ))
                                    .size(20.0)
                                    .strong(),
                            );
                            ui.add_space(8.0);
                            {
                                let mut job = egui::text::LayoutJob::default();
                                for i in 0..3 {
                                    if i > 0 {
                                        job.append(
                                            "  ",
                                            0.0,
                                            egui::TextFormat {
                                                font_id: egui::FontId::proportional(40.0),
                                                color: egui::Color32::TRANSPARENT,
                                                ..Default::default()
                                            },
                                        );
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
                                let glow =
                                    ((victory_time * 4.0).sin() * 0.15 + 0.85).clamp(0.0, 1.0);
                                let perfect_alpha = (pa * 255.0 * glow) as u8;
                                let scale = 1.0 + (1.0 - pa) * 0.5;
                                ui.label(
                                    egui::RichText::new("\u{2728} PERFECT \u{2728}")
                                        .color(egui::Color32::from_rgba_unmultiplied(
                                            255,
                                            200,
                                            50,
                                            perfect_alpha,
                                        ))
                                        .size(28.0 * scale)
                                        .strong(),
                                );
                                ui.add_space(4.0);
                            }
                            if let Some(pct) = lb_result.percentile {
                                if pct >= 50 {
                                    ui.label(
                                        egui::RichText::new(format!(
                                            "Better than {}% of players",
                                            pct
                                        ))
                                        .color(egui::Color32::from_rgba_unmultiplied(
                                            200, 220, 255, alpha,
                                        ))
                                        .size(15.0),
                                    );
                                }
                            }
                            ui.add_space(12.0);
                            if btn_alpha > 0.01 {
                                let btn_color = egui::Color32::from_rgba_unmultiplied(
                                    100,
                                    160,
                                    255,
                                    (btn_alpha * 255.0) as u8,
                                );
                                if ui
                                    .add(
                                        egui::Button::new(
                                            egui::RichText::new(format!(
                                                "Level {} \u{25B6}",
                                                level + 1
                                            ))
                                            .color(egui::Color32::from_rgba_unmultiplied(
                                                255,
                                                255,
                                                255,
                                                (btn_alpha * 255.0) as u8,
                                            ))
                                            .size(20.0)
                                            .strong(),
                                        )
                                        .fill(btn_color)
                                        .corner_radius(22)
                                        .min_size(egui::vec2(160.0, 44.0)),
                                    )
                                    .clicked()
                                {
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
                        ui.label(mono(
                            format!("  shadow   {:6.2} ms", prof.shadow_segs_ms),
                            dim,
                        ));
                        ui.label(mono(
                            format!("  checkwin {:6.2} ms", prof.check_win_ms),
                            dim,
                        ));
                        ui.label(mono(
                            format!("  celebr   {:6.2} ms", prof.celebration_ms),
                            dim,
                        ));
                        ui.label(mono(format!("  egui     {:6.2} ms", prof.egui_ms), dim));
                        ui.label(mono(format!("  submit   {:6.2} ms", prof.submit_ms), dim));
                        ui.add_space(4.0);
                        ui.label(mono("--- GPU ---".into(), gpu_c));
                        ui.label(mono(
                            format!("total      {:6.2} ms", prof.gpu.total_ms),
                            bright,
                        ));
                        ui.label(mono(
                            format!("  shadow   {:6.2} ms", prof.gpu.shadow_ms),
                            dim,
                        ));
                        ui.label(mono(format!("  hdr      {:6.2} ms", prof.gpu.hdr_ms), dim));
                        ui.label(mono(
                            format!("  bloom    {:6.2} ms", prof.gpu.bloom_ms),
                            dim,
                        ));
                        ui.add_space(4.0);
                        ui.label(mono(
                            "--- Mesh ---".into(),
                            egui::Color32::from_rgb(255, 200, 120),
                        ));
                        ui.label(mono(format!("verts      {:>6}", prof.rope_verts), dim));
                        ui.label(mono(format!("tris       {:>6}", prof.rope_tris), dim));
                        ui.add_space(4.0);
                        let on_off = |on: bool| if on { "OFF" } else { " on" };
                        let flag_c = |on: bool| {
                            if on {
                                egui::Color32::from_rgb(255, 80, 80)
                            } else {
                                dim
                            }
                        };
                        ui.label(mono(
                            format!("1 table  [{}]", on_off(prof.skip_table)),
                            flag_c(prof.skip_table),
                        ));
                        ui.label(mono(
                            format!("2 holes  [{}]", on_off(prof.skip_holes)),
                            flag_c(prof.skip_holes),
                        ));
                        ui.label(mono(
                            format!("3 ropes  [{}]", on_off(prof.skip_ropes)),
                            flag_c(prof.skip_ropes),
                        ));
                        let tshadow_label = match prof.table_shadow_mode {
                            0 => "pcf",
                            1 => "planar",
                            _ => "off",
                        };
                        let tshadow_c = if prof.table_shadow_mode == 2 {
                            egui::Color32::from_rgb(255, 80, 80)
                        } else {
                            dim
                        };
                        ui.label(mono(format!("4 tshadow [{}]", tshadow_label), tshadow_c));
                    });
            });
    }

    if settings_open {
        egui::Window::new("Settings")
            .id(egui::Id::new("settings_panel"))
            .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-12.0, 64.0))
            .default_width(280.0)
            .resizable(false)
            .collapsible(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .max_height(600.0)
                    .show(ui, |ui| {
                        ui.checkbox(square_cross, "Square cross-section");
                        ui.add_space(6.0);

                        egui::CollapsingHeader::new("Rope Material")
                            .default_open(true)
                            .show(ui, |ui| {
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.matte, 0.0..=1.0).text("Matte"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.gloss, 0.0..=3.0).text("Gloss"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.diffuse_wrap, 0.0..=0.5)
                                        .text("Diffuse wrap"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.subsurface, 0.0..=1.0)
                                        .text("Subsurface"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.edge_light, 0.0..=1.0)
                                        .text("Edge light"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.saturation, 0.0..=2.0)
                                        .text("Saturation"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.micro_bump, 0.0..=3.0)
                                        .text("Micro bump"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.bump_scale, 0.5..=15.0)
                                        .text("Bump scale"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.contact_ao, 0.0..=2.0)
                                        .text("Contact AO"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.lift_glow, 0.0..=2.0)
                                        .text("Lift glow"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.stretch_gloss, 0.0..=1.0)
                                        .text("Stretch gloss"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.stretch_spec, 0.0..=1.0)
                                        .text("Stretch spec"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut rope_mat.env_reflect, 0.0..=3.0)
                                        .text("Reflection"),
                                );
                            });

                        egui::CollapsingHeader::new("Lighting")
                            .default_open(true)
                            .show(ui, |ui| {
                                ui.add(
                                    egui::Slider::new(&mut lighting.ambient, 0.0..=0.5)
                                        .text("Ambient"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut lighting.shadow_darkness, 0.0..=1.0)
                                        .text("Shadow dark"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut lighting.light_intensity, 0.1..=5.0)
                                        .text("Light intensity"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut lighting.light_dir[0], -1.0..=1.0)
                                        .text("Light X"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut lighting.light_dir[1], -1.0..=1.0)
                                        .text("Light Y"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut lighting.light_dir[2], -1.0..=1.0)
                                        .text("Light Z"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut lighting.shadow_bias, 0.0..=0.01)
                                        .text("Shadow bias"),
                                );
                                ui.add(
                                    egui::Slider::new(&mut lighting.rope_radius_scale, 0.3..=2.0)
                                        .text("Rope scale"),
                                );
                                let shadow_labels = ["ShadowMap", "PCF", "PCSS"];
                                let current = (lighting.shadow_type as usize).min(2);
                                egui::ComboBox::from_label("Shadow type")
                                    .selected_text(shadow_labels[current])
                                    .show_ui(ui, |ui| {
                                        for (i, label) in shadow_labels.iter().enumerate() {
                                            ui.selectable_value(
                                                &mut lighting.shadow_type,
                                                i as u8,
                                                *label,
                                            );
                                        }
                                    });
                            });

                        egui::CollapsingHeader::new("Table")
                            .default_open(false)
                            .show(ui, |ui| {
                                let style_labels = ["Wood", "Gradient", "Solid"];
                                let current = (table.style as usize).min(2);
                                egui::ComboBox::from_label("Style")
                                    .selected_text(style_labels[current])
                                    .show_ui(ui, |ui| {
                                        for (i, label) in style_labels.iter().enumerate() {
                                            ui.selectable_value(&mut table.style, i as u32, *label);
                                        }
                                    });
                                if table.style == 1 || table.style == 2 {
                                    let c1 = &mut table.color1;
                                    let mut rgb1 = [c1[0], c1[1], c1[2]];
                                    ui.horizontal(|ui| {
                                        ui.label("Color 1");
                                        ui.color_edit_button_rgb(&mut rgb1);
                                    });
                                    c1[0] = rgb1[0];
                                    c1[1] = rgb1[1];
                                    c1[2] = rgb1[2];
                                }
                                if table.style == 1 {
                                    let c2 = &mut table.color2;
                                    let mut rgb2 = [c2[0], c2[1], c2[2]];
                                    ui.horizontal(|ui| {
                                        ui.label("Color 2");
                                        ui.color_edit_button_rgb(&mut rgb2);
                                    });
                                    c2[0] = rgb2[0];
                                    c2[1] = rgb2[1];
                                    c2[2] = rgb2[2];
                                }
                            });

                        ui.add_space(8.0);
                        ui.horizontal(|ui| {
                            if ui.button("Reset to defaults").clicked() {
                                *rope_mat = RopeMaterialSettings::default();
                                *lighting = LightingSettings::default();
                                *table = TableSettings::default();
                            }
                            let import_id = egui::Id::new("import_status");
                            let status: Option<(bool, f64)> =
                                ctx.data_mut(|d| d.get_temp(import_id));
                            let now = ctx.input(|i| i.time);
                            let active_status =
                                status.and_then(
                                    |(ok, t)| if now - t < 1.5 { Some(ok) } else { None },
                                );
                            let label = match active_status {
                                Some(true) => "\u{2705} Imported",
                                Some(false) => "\u{274C} Failed",
                                None => "\u{1F4CB} Paste settings",
                            };
                            if ui.button(label).clicked() {
                                let (ok, imported_level) = import_settings_from_clipboard(
                                    rope_mat,
                                    lighting,
                                    visual,
                                    table,
                                    cartoon,
                                    cap,
                                    render_scale,
                                    cel_mode,
                                    square_cross,
                                );
                                if let Some(level) = imported_level {
                                    action.go_to_level = Some(level);
                                }
                                ctx.data_mut(|d| d.insert_temp(import_id, (ok, now)));
                            }
                        });
                    });
            });
    }

    action
}

fn read_clipboard() -> Option<String> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        arboard::Clipboard::new().ok()?.get_text().ok()
    }
    #[cfg(target_arch = "wasm32")]
    {
        None
    }
}

fn import_settings_from_clipboard(
    rope_mat: &mut RopeMaterialSettings,
    lighting: &mut LightingSettings,
    visual: &mut VisualSettings,
    table: &mut TableSettings,
    cartoon: &mut CartoonSettings,
    cap: &mut CapSettings,
    render_scale: &mut f32,
    cel_mode: &mut bool,
    square_cross: &mut bool,
) -> (bool, Option<usize>) {
    let text = match read_clipboard() {
        Some(t) => t,
        None => return (false, None),
    };
    let json: Value = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(_) => return (false, None),
    };

    let f = |key: &str| json.get(key).and_then(|v| v.as_f64()).map(|v| v as f32);

    if let Some(v) = f("ropeMatte") {
        rope_mat.matte = v;
    }
    if let Some(v) = f("ropeGloss") {
        rope_mat.gloss = v;
    }
    if let Some(v) = f("ropeDiffuseWrap") {
        rope_mat.diffuse_wrap = v;
    }
    if let Some(v) = f("ropeSubsurface") {
        rope_mat.subsurface = v;
    }
    if let Some(v) = f("ropeEdgeLight") {
        rope_mat.edge_light = v;
    }
    if let Some(v) = f("ropeSaturation") {
        rope_mat.saturation = v;
    }
    if let Some(v) = f("ropeMicroBump") {
        rope_mat.micro_bump = v;
    }
    if let Some(v) = f("ropeBumpScale") {
        rope_mat.bump_scale = v;
    }
    if let Some(v) = f("ropeContactAO") {
        rope_mat.contact_ao = v;
    }
    if let Some(v) = f("ropeLiftGlow") {
        rope_mat.lift_glow = v;
    }
    if let Some(v) = f("ropeStretchGloss") {
        rope_mat.stretch_gloss = v;
    }
    if let Some(v) = f("ropeStretchSpec") {
        rope_mat.stretch_spec = v;
    }
    if let Some(v) = f("ropeEnvReflect") {
        rope_mat.env_reflect = v;
    }

    if let Some(v) = f("ambient") {
        lighting.ambient = v;
    }
    if let Some(v) = f("shadowDarkness") {
        lighting.shadow_darkness = v;
    }
    if let Some(v) = f("lightIntensity") {
        lighting.light_intensity = v;
    }
    if let Some(v) = f("shadowBias") {
        lighting.shadow_bias = v;
    }
    if let Some(v) = f("lightSize") {
        lighting.shadow_size = v;
    }
    if let Some(v) = f("lightDirX") {
        lighting.light_dir[0] = v;
    }
    if let Some(v) = f("lightDirY") {
        lighting.light_dir[1] = v;
    }
    if let Some(v) = f("lightDirZ") {
        lighting.light_dir[2] = v;
    }
    if let Some(v) = f("ropeRadiusScale") {
        lighting.rope_radius_scale = v;
    }
    if let Some(v) = json.get("shadowsEnabled").and_then(|v| v.as_bool()) {
        lighting.shadows_enabled = v;
    }
    if let Some(s) = json.get("shadowType").and_then(|v| v.as_str()) {
        lighting.shadow_type = match s {
            "ShadowMap" => 0,
            "PCF" => 1,
            "PCSS" => 2,
            _ => lighting.shadow_type,
        };
    }

    if let Some(v) = f("profileSegments") {
        visual.profile_segments = v.round().max(3.0) as usize;
    }
    if let Some(v) = f("holeRadiusScale") {
        visual.hole_radius_scale = v;
    }
    if let Some(v) = f("stretchThinning") {
        visual.stretch_thinning = v;
    }
    if let Some(v) = f("exposure") {
        visual.exposure = v;
    }
    if let Some(v) = f("bloomStrength") {
        visual.bloom_strength = v;
    }
    if let Some(v) = f("holeTintR") {
        visual.hole_tint[0] = v;
    }
    if let Some(v) = f("holeTintG") {
        visual.hole_tint[1] = v;
    }
    if let Some(v) = f("holeTintB") {
        visual.hole_tint[2] = v;
    }
    if let Some(v) = f("holeTintAmount") {
        visual.hole_tint[3] = v;
    }

    if let Some(v) = json.get("tableStyle").and_then(|v| v.as_str()) {
        table.style = match v {
            "Wood" => 0,
            "Gradient" => 1,
            "Solid" => 2,
            _ => table.style,
        };
    } else if let Some(v) = f("tableStyle") {
        table.style = v.round().max(0.0) as u32;
    }
    if let Some(v) = f("tableColor1R") {
        table.color1[0] = v;
    }
    if let Some(v) = f("tableColor1G") {
        table.color1[1] = v;
    }
    if let Some(v) = f("tableColor1B") {
        table.color1[2] = v;
    }
    if let Some(v) = f("tableColor2R") {
        table.color2[0] = v;
    }
    if let Some(v) = f("tableColor2G") {
        table.color2[1] = v;
    }
    if let Some(v) = f("tableColor2B") {
        table.color2[2] = v;
    }
    if let Some(v) = f("woodSeed") {
        table.wood_seed = v;
    }
    if let Some(v) = f("woodBrightness") {
        table.wood_brightness = v;
    }
    if let Some(v) = f("woodPatternScale") {
        table.wood_pattern_scale = v;
    }

    if let Some(v) = f("cartoonExposure") {
        cartoon.exposure = v;
    }
    if let Some(v) = f("cartoonEdgeStrength") {
        cartoon.edge_strength = v;
    }
    if let Some(v) = f("cartoonShadowBright") {
        cartoon.shadow_bright = v;
    }
    if let Some(v) = f("cartoonWrap") {
        cartoon.wrap = v;
    }
    if let Some(v) = f("cartoonEdgeSmooth") {
        cartoon.edge_smooth = v;
    }
    if let Some(v) = f("cartoonLevels") {
        visual.cartoon_levels = v;
    }
    if let Some(v) = json.get("cartoonShaderEnabled").and_then(|v| v.as_bool()) {
        *cel_mode = v;
        visual.cartoon_mode = if v { 1.0 } else { 0.0 };
    }

    if let Some(v) = f("capRadiusScale") {
        cap.radius_scale = v;
    }
    if let Some(v) = f("capSegments") {
        cap.segments = v.round().max(3.0) as usize;
    }
    if let Some(v) = f("capRings") {
        cap.rings = v.round().max(1.0) as usize;
    }
    if let Some(v) = f("capDarken") {
        cap.darken = v;
    }

    if let Some(v) = f("renderScale") {
        *render_scale = v;
    }

    if let Some(v) = json.get("squareCrossSection").and_then(|v| v.as_bool()) {
        *square_cross = v;
        visual.square_cross_section = v;
    }

    let imported_level = json
        .get("currentLevel")
        .and_then(|v| v.as_u64())
        .map(|v| v as usize)
        .filter(|&v| v >= 1);

    (true, imported_level)
}
