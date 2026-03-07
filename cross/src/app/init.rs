use std::sync::Arc;
use winit::event_loop::ActiveEventLoop;
use winit::window::Window;

use uzls_cross::hud::render_mode_scale;
use uzls_cross::renderer::gpu::GpuRenderer;
use uzls_cross::storage::load_level_from_storage;

use super::App;

impl App {
    pub fn finish_init_with_event_loop(
        &mut self,
        window: Arc<Window>,
        renderer: GpuRenderer,
        event_loop: &ActiveEventLoop,
    ) {
        let egui_ctx = egui::Context::default();
        let egui_state = egui_winit::State::new(
            egui_ctx,
            egui::ViewportId::ROOT,
            event_loop,
            Some(window.scale_factor() as f32),
            None,
            None,
        );
        let egui_renderer =
            egui_wgpu::Renderer::new(renderer.device(), renderer.surface_format(), None, 1, false);

        self.egui_state = Some(egui_state);
        self.egui_renderer = Some(egui_renderer);
        self.window = Some(window);
        self.renderer = Some(renderer);
        if let Some(r) = &mut self.renderer {
            r.set_render_scale(render_mode_scale(self.render_mode));
        }
        self.load_persisted_settings();
        if let Some(r) = &mut self.renderer {
            r.visual.square_cross_section = self.square_cross_section;
        }
        self.init_done = true;
        self.load_level(load_level_from_storage().unwrap_or(1));
    }

    pub fn finish_init_standalone(&mut self, renderer: GpuRenderer) {
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
        let egui_renderer =
            egui_wgpu::Renderer::new(renderer.device(), renderer.surface_format(), None, 1, false);

        self.egui_state = Some(egui_state);
        self.egui_renderer = Some(egui_renderer);
        self.renderer = Some(renderer);
        if let Some(r) = &mut self.renderer {
            r.set_render_scale(render_mode_scale(self.render_mode));
        }
        self.load_persisted_settings();
        if let Some(r) = &mut self.renderer {
            r.visual.square_cross_section = self.square_cross_section;
        }
        self.init_done = true;

        let size = window.inner_size();
        if size.width > 0 && size.height > 0 {
            if let Some(r) = &mut self.renderer {
                r.resize(size.width, size.height);
            }
        }

        self.load_level(load_level_from_storage().unwrap_or(1));
    }
}
