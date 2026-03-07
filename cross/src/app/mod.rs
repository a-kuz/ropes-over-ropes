pub mod drag;
pub mod events;
pub mod init;
pub mod level;
pub mod render;
pub mod win;

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use web_time::Instant;
use winit::window::Window;

use uzls_cross::audio::AudioPlayer;
use uzls_cross::celebration::{CameraSnapshot, CelebrationBand};
use uzls_cross::hud::ProfilingSnapshot;
use uzls_cross::input as game_input;
use uzls_cross::leaderboard;
use uzls_cross::level::definition::LevelDefinition;
use uzls_cross::renderer::frame_types::{
    CapSettings, CartoonSettings, LightingSettings, RopeMaterialSettings, TableSettings,
    VisualSettings, WormSettings,
};
use uzls_cross::renderer::gpu::{GpuRenderer, GpuTimings};
use uzls_cross::simulation::verlet::{Snapshot, VerletSimulator};

#[cfg(target_arch = "wasm32")]
pub type RendererCell = std::rc::Rc<std::cell::RefCell<Option<GpuRenderer>>>;

pub struct UndoEntry {
    pub simulator_snapshot: Snapshot,
    pub rope_endpoints: Vec<(usize, usize)>,
    pub hole_occupied: Vec<bool>,
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct PersistedSettings {
    pub rope_material: RopeMaterialSettings,
    pub lighting: LightingSettings,
    pub visual: VisualSettings,
    pub table: TableSettings,
    pub cartoon: CartoonSettings,
    pub cap: CapSettings,
    pub worm: WormSettings,
    pub render_scale: f32,
    pub square_cross_section: bool,
    pub render_mode: u8,
    pub cel_mode: bool,
}

pub struct App {
    pub window: Option<Arc<Window>>,
    pub renderer: Option<GpuRenderer>,
    pub simulator: Option<VerletSimulator>,
    pub level: Option<LevelDefinition>,
    pub current_level_id: usize,
    pub time: f32,
    pub last_instant: Option<Instant>,
    pub drag_state: Option<game_input::DragState>,
    pub highlight_hole: i32,
    pub hole_occupied: Vec<bool>,
    pub rope_endpoints: Vec<(usize, usize)>,
    pub rope_colors: Vec<[f32; 3]>,
    pub rope_radii: Vec<f32>,
    pub settle_check_timer: Option<f32>,
    pub next_level_timer: Option<f32>,
    pub victory_time: f32,
    pub last_cursor_pos: (f32, f32),
    pub right_mouse_down: bool,
    pub middle_mouse_down: bool,
    pub pan_last_pos: Option<(f32, f32)>,
    pub shift_held: bool,
    pub fps: f32,
    pub egui_state: Option<egui_winit::State>,
    pub egui_renderer: Option<egui_wgpu::Renderer>,
    pub pending_level: Option<usize>,
    pub wants_restart: bool,
    pub wants_undo: bool,
    pub undo_stack: Vec<UndoEntry>,
    pub audio: Option<AudioPlayer>,
    pub victory_sound_played: bool,
    pub celebration_active: bool,
    pub celebration_bands: Vec<CelebrationBand>,
    pub pre_victory_camera: Option<CameraSnapshot>,
    pub move_count: u32,
    pub min_moves: u32,
    pub render_mode: u8,
    pub cel_mode: bool,
    pub downscaled: bool,
    pub low_fps_frames: u32,
    pub square_cross_section: bool,
    pub active_touches: HashMap<u64, (f32, f32)>,
    pub touch_camera_active: bool,
    #[cfg(target_arch = "wasm32")]
    pub pending_renderer: Option<RendererCell>,
    #[cfg(target_arch = "wasm32")]
    pub web_victory_overlay_visible: bool,
    #[cfg(target_arch = "wasm32")]
    pub web_mobile_ui: bool,
    pub init_done: bool,
    pub prof_physics_ms: f32,
    pub prof_mesh_ms: f32,
    pub prof_egui_ms: f32,
    pub prof_frame_ms: f32,
    pub prof_check_win_ms: f32,
    pub prof_shadow_segs_ms: f32,
    pub prof_submit_ms: f32,
    pub prof_celebration_ms: f32,
    pub prof_gpu: GpuTimings,
    pub prof_show: bool,
    pub prof_rope_verts: u32,
    pub prof_rope_tris: u32,
    #[cfg(not(target_arch = "wasm32"))]
    pub prof_writer: Option<std::io::BufWriter<std::fs::File>>,
    pub prof_frame_num: u64,
    pub prof_events: Vec<&'static str>,
    pub settings_open: bool,
    pub leaderboard: leaderboard::Leaderboard,
    pub level_start_ms: u64,
    pub victory_submitted: bool,
}

impl App {
    pub fn new() -> Self {
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
            square_cross_section: false,
            active_touches: HashMap::new(),
            touch_camera_active: false,
            #[cfg(target_arch = "wasm32")]
            pending_renderer: None,
            #[cfg(target_arch = "wasm32")]
            web_victory_overlay_visible: false,
            #[cfg(target_arch = "wasm32")]
            web_mobile_ui: false,
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
            settings_open: false,
            leaderboard: leaderboard::Leaderboard::new(),
            level_start_ms: 0,
            victory_submitted: false,
        }
    }

    pub fn current_persisted_settings(&self) -> Option<PersistedSettings> {
        let renderer = self.renderer.as_ref()?;
        Some(PersistedSettings {
            rope_material: renderer.rope_material.clone(),
            lighting: renderer.lighting.clone(),
            visual: renderer.visual.clone(),
            table: renderer.table.clone(),
            cartoon: renderer.cartoon.clone(),
            cap: renderer.cap.clone(),
            worm: renderer.worm.clone(),
            render_scale: renderer.render_scale,
            square_cross_section: self.square_cross_section,
            render_mode: self.render_mode,
            cel_mode: self.cel_mode,
        })
    }

    pub fn apply_persisted_settings(&mut self, settings: PersistedSettings) {
        if let Some(renderer) = &mut self.renderer {
            renderer.rope_material = settings.rope_material;
            renderer.lighting = settings.lighting;
            renderer.visual = settings.visual;
            renderer.table = settings.table;
            renderer.cartoon = settings.cartoon;
            renderer.cap = settings.cap;
            renderer.worm = settings.worm;
            renderer.set_render_scale(settings.render_scale);
        }
        self.square_cross_section = settings.square_cross_section;
        self.render_mode = settings.render_mode;
        self.cel_mode = settings.cel_mode;
    }

    pub fn load_persisted_settings(&mut self) {
        use uzls_cross::storage::load_settings_from_storage;
        let Some(text) = load_settings_from_storage() else {
            return;
        };
        let Ok(settings) = serde_json::from_str::<PersistedSettings>(&text) else {
            return;
        };
        self.apply_persisted_settings(settings);
    }

    pub fn save_persisted_settings(&self) {
        use uzls_cross::storage::save_settings_to_storage;
        let Some(settings) = self.current_persisted_settings() else {
            return;
        };
        if let Ok(text) = serde_json::to_string(&settings) {
            save_settings_to_storage(&text);
        }
    }
}

pub fn fit_camera(
    camera: &mut uzls_cross::renderer::camera::Camera,
    holes: &[glam::Vec2],
    hole_radius: f32,
    aspect: f32,
    max_elevation: f32,
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
    let elevation_padding = if max_elevation > 0.01 {
        max_elevation * 1.5
    } else {
        0.0
    };
    let required_half_h = half_h_from_height.max(half_h_from_width) * 1.2 + elevation_padding;
    camera.ortho_half_height = required_half_h;
    camera.center = glam::Vec3::new(center_x, center_y, 0.0);
    camera.orbit_angle = 0.0;
    if max_elevation > 0.01 && camera.tilt_angle < 0.15 {
        camera.tilt_angle = 0.25;
    } else {
        camera.tilt_angle = 0.0;
    }
    camera.distance = 2.8;
    camera.perspective_blend = 0.0;
}
