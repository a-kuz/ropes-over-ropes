use glam::Vec2;
use std::collections::HashSet;

use super::definition::*;

struct SeededRNG {
    state: u64,
}

impl SeededRNG {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    fn next(&mut self, bound: usize) -> usize {
        if bound == 0 {
            return 0;
        }
        (self.next_u64() % bound as u64) as usize
    }
}

const COLORS: [Color; 10] = [
    Color { red_channel: 0.95, green_channel: 0.90, blue_channel: 0.35 },
    Color { red_channel: 0.95, green_channel: 0.55, blue_channel: 0.65 },
    Color { red_channel: 0.55, green_channel: 0.85, blue_channel: 0.95 },
    Color { red_channel: 0.75, green_channel: 0.60, blue_channel: 0.90 },
    Color { red_channel: 0.55, green_channel: 0.92, blue_channel: 0.60 },
    Color { red_channel: 0.95, green_channel: 0.70, blue_channel: 0.40 },
    Color { red_channel: 0.85, green_channel: 0.45, blue_channel: 0.85 },
    Color { red_channel: 0.45, green_channel: 0.90, blue_channel: 0.85 },
    Color { red_channel: 0.90, green_channel: 0.80, blue_channel: 0.60 },
    Color { red_channel: 0.65, green_channel: 0.75, blue_channel: 0.95 },
];

struct Difficulty {
    rope_count: usize,
    total_drags: usize,
}

fn difficulty(level_id: u32) -> Difficulty {
    match level_id {
        1..=2 => Difficulty { rope_count: 3, total_drags: 3 },
        3..=5 => Difficulty { rope_count: 4, total_drags: 4 },
        6..=9 => Difficulty { rope_count: 5, total_drags: 6 },
        10..=15 => Difficulty { rope_count: 6, total_drags: 8 },
        16..=25 => Difficulty { rope_count: 7, total_drags: 10 },
        26..=50 => Difficulty {
            rope_count: 8,
            total_drags: 12 + ((level_id as usize - 25) / 5),
        },
        _ => Difficulty {
            rope_count: 10usize.min(8 + ((level_id as usize - 50) / 25)),
            total_drags: 22usize.min(16 + ((level_id as usize - 50) / 10)),
        },
    }
}

#[derive(Clone, Copy)]
enum HoleLayout {
    Grid4x5,
    Circle12,
    Hexagon,
    Diamond,
    Cross,
    TwoRings,
    Triangle,
    Star,
    Grid5x6,
    Circle16,
    HexagonLarge,
    DiamondWide,
    CrossLarge,
    ThreeRings,
    TriangleLarge,
    StarLarge,
    Honeycomb,
    Spiral,
    DoubleGrid,
    Scattered,
}

const ALL_LAYOUTS: [HoleLayout; 20] = [
    HoleLayout::Grid4x5,
    HoleLayout::Circle12,
    HoleLayout::Hexagon,
    HoleLayout::Diamond,
    HoleLayout::Cross,
    HoleLayout::TwoRings,
    HoleLayout::Triangle,
    HoleLayout::Star,
    HoleLayout::Grid5x6,
    HoleLayout::Circle16,
    HoleLayout::HexagonLarge,
    HoleLayout::DiamondWide,
    HoleLayout::CrossLarge,
    HoleLayout::ThreeRings,
    HoleLayout::TriangleLarge,
    HoleLayout::StarLarge,
    HoleLayout::Honeycomb,
    HoleLayout::Spiral,
    HoleLayout::DoubleGrid,
    HoleLayout::Scattered,
];

fn grid(cols: usize, rows: usize, spacing_x: f32, spacing_y: f32) -> Vec<HolePos> {
    let ox = -((cols as f32 - 1.0) / 2.0) * spacing_x;
    let oy = -((rows as f32 - 1.0) / 2.0) * spacing_y;
    let mut holes = Vec::new();
    for row in 0..rows {
        for col in 0..cols {
            holes.push(HolePos {
                x_position: ox + col as f32 * spacing_x,
                y_position: oy + row as f32 * spacing_y,
            });
        }
    }
    holes
}

fn circle(count: usize, radius: f32) -> Vec<HolePos> {
    let mut holes = Vec::new();
    for i in 0..count {
        let angle = (i as f32 / count as f32) * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
        holes.push(HolePos {
            x_position: radius * angle.cos(),
            y_position: radius * angle.sin(),
        });
    }
    holes
}

impl HoleLayout {
    fn generate(self) -> Vec<HolePos> {
        match self {
            HoleLayout::Grid4x5 => grid(5, 4, 0.42, 0.45),
            HoleLayout::Circle12 => circle(12, 0.72),
            HoleLayout::Hexagon => {
                let mut holes = vec![HolePos { x_position: 0.0, y_position: 0.0 }];
                let r1: f32 = 0.45;
                for i in 0..6 {
                    let angle = (i as f32 / 6.0) * std::f32::consts::TAU - std::f32::consts::FRAC_PI_6;
                    holes.push(HolePos {
                        x_position: r1 * angle.cos(),
                        y_position: r1 * angle.sin(),
                    });
                }
                let r2: f32 = 0.85;
                for i in 0..12 {
                    let angle = (i as f32 / 12.0) * std::f32::consts::TAU;
                    holes.push(HolePos {
                        x_position: r2 * angle.cos(),
                        y_position: r2 * angle.sin(),
                    });
                }
                holes
            }
            HoleLayout::Diamond => {
                let s: f32 = 0.38;
                let row_counts = [1, 3, 5, 3, 1];
                let mut holes = Vec::new();
                for (row_idx, &count) in row_counts.iter().enumerate() {
                    let y = (row_idx as f32 - 2.0) * s;
                    let ox = -((count as f32 - 1.0) / 2.0) * s;
                    for col in 0..count {
                        holes.push(HolePos {
                            x_position: ox + col as f32 * s,
                            y_position: y,
                        });
                    }
                }
                holes
            }
            HoleLayout::Cross => {
                let s: f32 = 0.40;
                let mut holes = Vec::new();
                for i in -2i32..=2 {
                    holes.push(HolePos {
                        x_position: 0.0,
                        y_position: i as f32 * s,
                    });
                }
                for &i in &[-2i32, -1, 1, 2] {
                    holes.push(HolePos {
                        x_position: i as f32 * s,
                        y_position: 0.0,
                    });
                }
                for &sx in &[-1.0f32, 1.0] {
                    for &sy in &[-1.0f32, 1.0] {
                        holes.push(HolePos {
                            x_position: sx * s,
                            y_position: sy * s,
                        });
                    }
                }
                holes
            }
            HoleLayout::TwoRings => {
                let mut holes = Vec::new();
                let r_inner: f32 = 0.35;
                for i in 0..6 {
                    let angle = (i as f32 / 6.0) * std::f32::consts::TAU;
                    holes.push(HolePos {
                        x_position: r_inner * angle.cos(),
                        y_position: r_inner * angle.sin(),
                    });
                }
                let r_outer: f32 = 0.78;
                for i in 0..10 {
                    let angle =
                        (i as f32 / 10.0) * std::f32::consts::TAU + std::f32::consts::PI / 10.0;
                    holes.push(HolePos {
                        x_position: r_outer * angle.cos(),
                        y_position: r_outer * angle.sin(),
                    });
                }
                holes
            }
            HoleLayout::Triangle => {
                let s: f32 = 0.38;
                let mut holes = Vec::new();
                for row in 0..5usize {
                    let count = row + 1;
                    let ox = -((count as f32 - 1.0) / 2.0) * s;
                    let y = (row as f32 - 2.0) * s * 0.866;
                    for col in 0..count {
                        holes.push(HolePos {
                            x_position: ox + col as f32 * s,
                            y_position: y,
                        });
                    }
                }
                holes
            }
            HoleLayout::Star => {
                let mut holes = vec![HolePos { x_position: 0.0, y_position: 0.0 }];
                for i in 0..5 {
                    let outer_angle =
                        (i as f32 / 5.0) * std::f32::consts::TAU - std::f32::consts::FRAC_PI_2;
                    holes.push(HolePos {
                        x_position: 0.82 * outer_angle.cos(),
                        y_position: 0.82 * outer_angle.sin(),
                    });
                    let inner_angle = outer_angle + std::f32::consts::PI / 5.0;
                    holes.push(HolePos {
                        x_position: 0.38 * inner_angle.cos(),
                        y_position: 0.38 * inner_angle.sin(),
                    });
                }
                for i in 0..5 {
                    let angle = (i as f32 / 5.0) * std::f32::consts::TAU
                        - std::f32::consts::FRAC_PI_2
                        + std::f32::consts::PI / 5.0;
                    holes.push(HolePos {
                        x_position: 0.60 * angle.cos(),
                        y_position: 0.60 * angle.sin(),
                    });
                }
                holes
            }
            HoleLayout::Grid5x6 => grid(6, 5, 0.38, 0.40),
            HoleLayout::Circle16 => circle(16, 0.85),
            HoleLayout::HexagonLarge => {
                let mut holes = vec![HolePos { x_position: 0.0, y_position: 0.0 }];
                for ring in 1..=3u32 {
                    let count = ring * 6;
                    let radius = ring as f32 * 0.35;
                    let offset = ring as f32 * 0.15;
                    for i in 0..count {
                        let angle = (i as f32 / count as f32) * std::f32::consts::TAU + offset;
                        holes.push(HolePos {
                            x_position: radius * angle.cos(),
                            y_position: radius * angle.sin(),
                        });
                    }
                }
                holes
            }
            HoleLayout::DiamondWide => {
                let s: f32 = 0.34;
                let row_counts = [2, 4, 6, 8, 6, 4, 2];
                let mut holes = Vec::new();
                for (row_idx, &count) in row_counts.iter().enumerate() {
                    let y = (row_idx as f32 - 3.0) * s;
                    let ox = -((count as f32 - 1.0) / 2.0) * s;
                    for col in 0..count {
                        holes.push(HolePos {
                            x_position: ox + col as f32 * s,
                            y_position: y,
                        });
                    }
                }
                holes
            }
            HoleLayout::CrossLarge => {
                let s: f32 = 0.32;
                let mut holes = Vec::new();
                for i in -3i32..=3 {
                    holes.push(HolePos {
                        x_position: 0.0,
                        y_position: i as f32 * s,
                    });
                }
                for &i in &[-3i32, -2, -1, 1, 2, 3] {
                    holes.push(HolePos {
                        x_position: i as f32 * s,
                        y_position: 0.0,
                    });
                }
                for &x in &[-2i32, -1, 1, 2] {
                    for &y in &[-1i32, 1] {
                        holes.push(HolePos {
                            x_position: x as f32 * s,
                            y_position: y as f32 * s,
                        });
                    }
                }
                for &x in &[-1i32, 1] {
                    for &y in &[-2i32, 2] {
                        holes.push(HolePos {
                            x_position: x as f32 * s,
                            y_position: y as f32 * s,
                        });
                    }
                }
                holes
            }
            HoleLayout::ThreeRings => {
                let mut holes = vec![HolePos { x_position: 0.0, y_position: 0.0 }];
                for i in 0..5 {
                    let angle = (i as f32 / 5.0) * std::f32::consts::TAU;
                    holes.push(HolePos {
                        x_position: 0.30 * angle.cos(),
                        y_position: 0.30 * angle.sin(),
                    });
                }
                for i in 0..9 {
                    let angle = (i as f32 / 9.0) * std::f32::consts::TAU + std::f32::consts::PI / 9.0;
                    holes.push(HolePos {
                        x_position: 0.60 * angle.cos(),
                        y_position: 0.60 * angle.sin(),
                    });
                }
                for i in 0..13 {
                    let angle = (i as f32 / 13.0) * std::f32::consts::TAU;
                    holes.push(HolePos {
                        x_position: 0.92 * angle.cos(),
                        y_position: 0.92 * angle.sin(),
                    });
                }
                holes
            }
            HoleLayout::TriangleLarge => {
                let s: f32 = 0.30;
                let mut holes = Vec::new();
                for row in 0..7usize {
                    let count = row + 1;
                    let ox = -((count as f32 - 1.0) / 2.0) * s;
                    let y = (row as f32 - 3.0) * s * 0.866;
                    for col in 0..count {
                        holes.push(HolePos {
                            x_position: ox + col as f32 * s,
                            y_position: y,
                        });
                    }
                }
                holes
            }
            HoleLayout::StarLarge => {
                let mut holes = vec![HolePos { x_position: 0.0, y_position: 0.0 }];
                for i in 0..6 {
                    let angle = (i as f32 / 6.0) * std::f32::consts::TAU;
                    holes.push(HolePos {
                        x_position: 0.95 * angle.cos(),
                        y_position: 0.95 * angle.sin(),
                    });
                }
                for i in 0..6 {
                    let angle = (i as f32 / 6.0) * std::f32::consts::TAU + std::f32::consts::PI / 6.0;
                    holes.push(HolePos {
                        x_position: 0.42 * angle.cos(),
                        y_position: 0.42 * angle.sin(),
                    });
                }
                for i in 0..6 {
                    let angle = (i as f32 / 6.0) * std::f32::consts::TAU + std::f32::consts::PI / 6.0;
                    holes.push(HolePos {
                        x_position: 0.68 * angle.cos(),
                        y_position: 0.68 * angle.sin(),
                    });
                }
                for i in 0..6 {
                    let angle = (i as f32 / 6.0) * std::f32::consts::TAU;
                    holes.push(HolePos {
                        x_position: 0.68 * angle.cos(),
                        y_position: 0.68 * angle.sin(),
                    });
                }
                holes
            }
            HoleLayout::Honeycomb => {
                let s: f32 = 0.36;
                let mut holes = Vec::new();
                for row in -2i32..=2 {
                    let (cols, x_off) = if row.abs() % 2 == 0 {
                        (5, 0.0)
                    } else {
                        (4, 0.5 * s)
                    };
                    let ox = -((cols as f32 - 1.0) / 2.0) * s + x_off;
                    let y = row as f32 * s * 0.866;
                    for col in 0..cols {
                        holes.push(HolePos {
                            x_position: ox + col as f32 * s,
                            y_position: y,
                        });
                    }
                }
                holes
            }
            HoleLayout::Spiral => {
                let mut holes = vec![HolePos { x_position: 0.0, y_position: 0.0 }];
                for i in 0..20 {
                    let t = i as f32 / 20.0;
                    let r = 0.15 + t * 0.75;
                    let angle = t * 4.5 * std::f32::consts::PI;
                    holes.push(HolePos {
                        x_position: r * angle.cos(),
                        y_position: r * angle.sin(),
                    });
                }
                holes
            }
            HoleLayout::DoubleGrid => {
                let s: f32 = 0.38;
                let gap: f32 = 0.22;
                let mut holes = Vec::new();
                for grid_idx in 0..2 {
                    let y_base = if grid_idx == 0 {
                        -(1.5 * s + gap / 2.0)
                    } else {
                        gap / 2.0 - 0.5 * s
                    };
                    for row in 0..3usize {
                        for col in 0..4usize {
                            holes.push(HolePos {
                                x_position: (col as f32 - 1.5) * s,
                                y_position: y_base + row as f32 * s,
                            });
                        }
                    }
                }
                holes
            }
            HoleLayout::Scattered => {
                vec![
                    HolePos { x_position: -0.75, y_position: -0.70 },
                    HolePos { x_position: -0.30, y_position: -0.80 },
                    HolePos { x_position:  0.20, y_position: -0.75 },
                    HolePos { x_position:  0.70, y_position: -0.65 },
                    HolePos { x_position: -0.85, y_position: -0.25 },
                    HolePos { x_position: -0.40, y_position: -0.30 },
                    HolePos { x_position:  0.05, y_position: -0.35 },
                    HolePos { x_position:  0.50, y_position: -0.20 },
                    HolePos { x_position:  0.88, y_position: -0.30 },
                    HolePos { x_position: -0.70, y_position:  0.15 },
                    HolePos { x_position: -0.25, y_position:  0.10 },
                    HolePos { x_position:  0.25, y_position:  0.18 },
                    HolePos { x_position:  0.72, y_position:  0.10 },
                    HolePos { x_position: -0.80, y_position:  0.60 },
                    HolePos { x_position: -0.35, y_position:  0.55 },
                    HolePos { x_position:  0.10, y_position:  0.65 },
                    HolePos { x_position:  0.55, y_position:  0.58 },
                    HolePos { x_position:  0.85, y_position:  0.55 },
                    HolePos { x_position: -0.50, y_position:  0.90 },
                    HolePos { x_position:  0.00, y_position:  0.92 },
                    HolePos { x_position:  0.50, y_position:  0.88 },
                ]
            }
        }
    }
}

fn pick_structured_pairs(
    holes: &[HolePos],
    count: usize,
    rng: &mut SeededRNG,
) -> Vec<(usize, usize)> {
    if holes.len() < 4 {
        return vec![];
    }
    let cx: f32 = holes.iter().map(|h| h.x_position).sum::<f32>() / holes.len() as f32;
    let cy: f32 = holes.iter().map(|h| h.y_position).sum::<f32>() / holes.len() as f32;

    let mut indexed: Vec<(usize, f32)> = holes
        .iter()
        .enumerate()
        .map(|(idx, h)| (idx, (h.y_position - cy).atan2(h.x_position - cx)))
        .collect();
    indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

    let offset = rng.next(indexed.len());
    let n = indexed.len();
    let half = n / 2;

    let mut pairs: Vec<(usize, usize)> = Vec::new();
    let mut used: HashSet<usize> = HashSet::new();

    for i in 0..count {
        if pairs.len() >= count {
            break;
        }
        let a_idx = (i * 2 + offset) % n;
        let a = indexed[a_idx].0;
        if used.contains(&a) {
            continue;
        }
        let shift: isize = if i % 2 == 0 { 1 } else { -1 };
        let b_idx = ((a_idx as isize + half as isize + shift + n as isize) as usize) % n;
        let b = indexed[b_idx].0;
        if b != a && !used.contains(&b) {
            pairs.push((a, b));
            used.insert(a);
            used.insert(b);
        }
    }

    if pairs.len() < count {
        for i in 0..n {
            if pairs.len() >= count {
                break;
            }
            let a = indexed[(i + offset) % n].0;
            if used.contains(&a) {
                continue;
            }
            for j in (i + 1)..n {
                let b = indexed[(j + offset) % n].0;
                if used.contains(&b) {
                    continue;
                }
                let dx = holes[a].x_position - holes[b].x_position;
                let dy = holes[a].y_position - holes[b].y_position;
                let dist = (dx * dx + dy * dy).sqrt();
                if dist > 0.5 {
                    pairs.push((a, b));
                    used.insert(a);
                    used.insert(b);
                    break;
                }
            }
        }
    }

    pairs
}

pub fn generate(level_id: u32) -> LevelDefinition {
    let mut rng = SeededRNG::new((level_id as u64).wrapping_mul(2654435761));
    let layout = ALL_LAYOUTS[level_id as usize % ALL_LAYOUTS.len()];
    let holes = layout.generate();
    let diff = difficulty(level_id);
    let max_ropes = (holes.len().saturating_sub(3)) / 2;
    let rope_count = diff.rope_count.min(max_ropes).max(1);

    let rope_pairs = pick_structured_pairs(&holes, rope_count, &mut rng);

    let ropes: Vec<Rope> = rope_pairs
        .iter()
        .enumerate()
        .map(|(i, &(start, end))| Rope {
            start_hole: start,
            end_hole: end,
            color: COLORS[i % COLORS.len()],
            radius: 0.038,
            cross_section_def: None,
        })
        .collect();

    let mut actions: Vec<Action> = Vec::new();
    for (i, rope) in ropes.iter().enumerate() {
        actions.push(Action {
            kind: "pin".to_string(),
            rope_index: i,
            end_index: 0,
            hole_index: rope.start_hole,
        });
        actions.push(Action {
            kind: "pin".to_string(),
            rope_index: i,
            end_index: 1,
            hole_index: rope.end_hole,
        });
    }

    let mut current_endpoints: Vec<(usize, usize)> =
        ropes.iter().map(|r| (r.start_hole, r.end_hole)).collect();
    let hole_vecs: Vec<Vec2> = holes.iter().map(|h| h.to_vec2()).collect();

    let try_drag = |rope_idx: usize, end_idx: usize, target_rope: usize,
                     endpoints: &mut Vec<(usize, usize)>, acts: &mut Vec<Action>,
                     hvecs: &[Vec2], n_ropes: usize| -> bool {
        let current_hole = if end_idx == 0 { endpoints[rope_idx].0 } else { endpoints[rope_idx].1 };
        let anchor_hole = if end_idx == 0 { endpoints[rope_idx].1 } else { endpoints[rope_idx].0 };
        let anchor_pos = hvecs[anchor_hole];
        let ts = hvecs[endpoints[target_rope].0];
        let te = hvecs[endpoints[target_rope].1];

        let other_used: HashSet<usize> = (0..n_ropes)
            .filter(|&ri| ri != rope_idx)
            .flat_map(|ri| [endpoints[ri].0, endpoints[ri].1])
            .collect();

        let mut best: Option<usize> = None;
        let mut best_score = f32::NEG_INFINITY;
        for c in 0..hvecs.len() {
            if c == current_hole || c == anchor_hole { continue; }
            if other_used.contains(&c) { continue; }
            if segments_cross(anchor_pos, hvecs[c], ts, te) {
                let dist = (hvecs[c] - (ts + te) * 0.5).length();
                if -dist > best_score {
                    best_score = -dist;
                    best = Some(c);
                }
            }
        }
        if let Some(target_hole) = best {
            if end_idx == 0 { endpoints[rope_idx].0 = target_hole; }
            else { endpoints[rope_idx].1 = target_hole; }
            acts.push(Action { kind: "drag".to_string(), rope_index: rope_idx, end_index: end_idx, hole_index: target_hole });
            true
        } else { false }
    };

    for d in 0..diff.total_drags {
        let rope_idx = d % ropes.len();
        let target_rope = (rope_idx + 1 + d / ropes.len()) % ropes.len();
        if target_rope == rope_idx { continue; }
        let end_idx = (d / ropes.len()) % 2;
        if !try_drag(rope_idx, end_idx, target_rope, &mut current_endpoints, &mut actions, &hole_vecs, ropes.len()) {
            try_drag(rope_idx, 1 - end_idx, target_rope, &mut current_endpoints, &mut actions, &hole_vecs, ropes.len());
        }
    }

    for _ in 0..ropes.len() * 4 {
        let crossings = rope_crossings(&current_endpoints, &hole_vecs);
        let isolated: Vec<usize> = (0..ropes.len()).filter(|&i| crossings[i] == 0).collect();
        if isolated.is_empty() { break; }
        for &rope_idx in &isolated {
            let mut fixed = false;
            for other in 0..ropes.len() {
                if other == rope_idx { continue; }
                for end_idx in 0..=1 {
                    if try_drag(rope_idx, end_idx, other, &mut current_endpoints, &mut actions, &hole_vecs, ropes.len()) {
                        fixed = true; break;
                    }
                }
                if fixed { break; }
            }
        }
    }

    LevelDefinition {
        id: level_id,
        hole_radius: 0.08,
        particles_per_rope: 40,
        holes,
        ropes,
        hooks: None,
        actions: Some(actions),
    }
}

fn segments_cross(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> bool {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let cross = d1.x * d2.y - d1.y * d2.x;
    if cross.abs() < 1e-9 { return false; }
    let d = b0 - a0;
    let t = (d.x * d2.y - d.y * d2.x) / cross;
    let u = (d.x * d1.y - d.y * d1.x) / cross;
    t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99
}

fn rope_crossings(endpoints: &[(usize, usize)], holes: &[Vec2]) -> Vec<usize> {
    let n = endpoints.len();
    let mut counts = vec![0usize; n];
    for i in 0..n {
        let a0 = holes[endpoints[i].0];
        let a1 = holes[endpoints[i].1];
        for j in (i + 1)..n {
            let b0 = holes[endpoints[j].0];
            let b1 = holes[endpoints[j].1];
            if segments_cross(a0, a1, b0, b1) {
                counts[i] += 1;
                counts[j] += 1;
            }
        }
    }
    counts
}
