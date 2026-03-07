use super::App;

impl App {
    pub fn check_win(&mut self) {
        let mut removed = true;
        while removed {
            removed = false;
            for rope_index in 0..self.rope_endpoints.len() {
                if self.rope_endpoints[rope_index].0 == usize::MAX {
                    continue;
                }
                let should_fade = match &self.simulator {
                    Some(sim) => {
                        if rope_index >= sim.bands.len() {
                            false
                        } else {
                            sim.bands[rope_index].fade_out == 0.0
                                && sim.is_rope_untangled(rope_index)
                        }
                    }
                    None => false,
                };
                if !should_fade {
                    continue;
                }
                if let Some(sim) = &mut self.simulator {
                    if rope_index < sim.bands.len() {
                        let (start_hole, end_hole) = self.rope_endpoints[rope_index];
                        let fallback_start_hole = (start_hole != usize::MAX).then_some(start_hole);
                        let fallback_end_hole = (end_hole != usize::MAX).then_some(end_hole);
                        sim.start_fade_out(rope_index, fallback_start_hole, fallback_end_hole);
                    }
                }
                let (s, e) = self.rope_endpoints[rope_index];
                if s < self.hole_occupied.len() {
                    self.hole_occupied[s] = false;
                }
                if e < self.hole_occupied.len() {
                    self.hole_occupied[e] = false;
                }
                self.rope_endpoints[rope_index] = (usize::MAX, usize::MAX);
                removed = true;
                self.prof_events.push("untangle");
                if let Some(audio) = &self.audio {
                    audio.play_vanish();
                }
                break;
            }
        }
    }
}
