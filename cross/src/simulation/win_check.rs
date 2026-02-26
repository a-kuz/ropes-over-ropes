use glam::Vec2;
use super::verlet::VerletSimulator;

impl VerletSimulator {
    pub fn find_untangled_ropes(&self) -> Vec<usize> {
        let mut untangled = Vec::new();
        for rope_index in 0..self.bands.len() {
            if !self.bands[rope_index].active {
                continue;
            }
            if self.bands[rope_index].fade_out > 0.0 {
                continue;
            }
            if self.is_rope_untangled(rope_index) {
                untangled.push(rope_index);
            }
        }
        untangled
    }

    fn is_rope_untangled(&self, rope_index: usize) -> bool {
        let band_a = &self.bands[rope_index];
        let n_a = band_a.positions.len();
        let skip = 3usize;

        for other_index in 0..self.bands.len() {
            if other_index == rope_index {
                continue;
            }
            let band_b = &self.bands[other_index];
            if !band_b.active || band_b.fade_out > 0.0 {
                continue;
            }
            let n_b = band_b.positions.len();

            let start_a = skip;
            let end_a = if n_a > 1 + skip { n_a - 1 - skip } else { start_a };
            let start_b = skip;
            let end_b = if n_b > 1 + skip { n_b - 1 - skip } else { start_b };

            for i in start_a..end_a {
                let a0 = Vec2::new(band_a.positions[i].x, band_a.positions[i].y);
                let a1 = Vec2::new(band_a.positions[i + 1].x, band_a.positions[i + 1].y);
                for j in start_b..end_b {
                    let b0 = Vec2::new(band_b.positions[j].x, band_b.positions[j].y);
                    let b1 = Vec2::new(band_b.positions[j + 1].x, band_b.positions[j + 1].y);
                    if segments_cross_2d(a0, a1, b0, b1) {
                        return false;
                    }
                }
            }
        }
        true
    }
}

fn segments_cross_2d(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> bool {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let cross = d1.x * d2.y - d1.y * d2.x;
    if cross.abs() < 1e-9 {
        return false;
    }
    let d = b0 - a0;
    let t = (d.x * d2.y - d.y * d2.x) / cross;
    let u = (d.x * d1.y - d.y * d1.x) / cross;
    t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99
}
