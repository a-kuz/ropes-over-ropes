use glam::Vec2;
use std::collections::HashSet;
use std::f32::consts::{FRAC_PI_2, PI, TAU};

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
    Color {
        red_channel: 0.95,
        green_channel: 0.30,
        blue_channel: 0.05,
    },
    Color {
        red_channel: 0.10,
        green_channel: 0.35,
        blue_channel: 0.92,
    },
    Color {
        red_channel: 0.90,
        green_channel: 0.12,
        blue_channel: 0.25,
    },
    Color {
        red_channel: 0.15,
        green_channel: 0.75,
        blue_channel: 0.30,
    },
    Color {
        red_channel: 0.92,
        green_channel: 0.78,
        blue_channel: 0.05,
    },
    Color {
        red_channel: 0.60,
        green_channel: 0.10,
        blue_channel: 0.72,
    },
    Color {
        red_channel: 0.05,
        green_channel: 0.65,
        blue_channel: 0.72,
    },
    Color {
        red_channel: 0.85,
        green_channel: 0.15,
        blue_channel: 0.55,
    },
    Color {
        red_channel: 0.20,
        green_channel: 0.55,
        blue_channel: 0.90,
    },
    Color {
        red_channel: 0.80,
        green_channel: 0.50,
        blue_channel: 0.05,
    },
];

struct Difficulty {
    rope_count: usize,
    total_drags: usize,
    short_rope_count: usize,
}

#[derive(Clone, Copy)]
enum BoardLayout {
    TwoSides,
    Bridge,
    Staircase,
    Platform,
    Valley,
}

struct BoardLayoutResult {
    holes: Vec<HolePos>,
    boards: Vec<Board>,
}

fn difficulty(level_id: u32) -> Difficulty {
    match level_id {
        1..=2 => Difficulty {
            rope_count: 3,
            total_drags: 3,
            short_rope_count: 0,
        },
        3..=5 => Difficulty {
            rope_count: 4,
            total_drags: 4,
            short_rope_count: if level_id >= 4 { 2 } else { 0 },
        },
        6..=9 => Difficulty {
            rope_count: 5,
            total_drags: 6,
            short_rope_count: 3,
        },
        10..=15 => Difficulty {
            rope_count: 6,
            total_drags: 8,
            short_rope_count: 4,
        },
        16..=25 => Difficulty {
            rope_count: 7,
            total_drags: 10,
            short_rope_count: 5,
        },
        26..=50 => Difficulty {
            rope_count: 8,
            total_drags: 12 + ((level_id as usize - 25) / 5),
            short_rope_count: 6,
        },
        _ => Difficulty {
            rope_count: 10usize.min(8 + ((level_id as usize - 50) / 25)),
            total_drags: 22usize.min(16 + ((level_id as usize - 50) / 10)),
            short_rope_count: 8usize.min(6 + ((level_id as usize - 50) / 30)),
        },
    }
}

#[derive(Clone, Copy)]
enum HoleLayout {
    Grid,
    Circle,
    Hexagon,
    Diamond,
    Cross,
    Rings,
    Triangle,
    Star,
    Honeycomb,
    Spiral,
    DoubleGrid,
    Scattered,
    Columns,
    CenterPlusRing,
    RotatedSquare,
    SquareInCircle,
    Arc,
    ConcentricSquares,
    TriangleInCircle,
    Crescent,
    LetterH,
    LetterT,
    Arrow,
    Zigzag,
    Clover,
    Clusters,
    ThreeColumns,
    RingWithSpokes,
    DiamondOutline,
    BowTie,
}

const ALL_LAYOUTS: [HoleLayout; 30] = [
    HoleLayout::Grid,
    HoleLayout::Circle,
    HoleLayout::Hexagon,
    HoleLayout::Diamond,
    HoleLayout::Cross,
    HoleLayout::Rings,
    HoleLayout::Triangle,
    HoleLayout::Star,
    HoleLayout::Honeycomb,
    HoleLayout::Spiral,
    HoleLayout::DoubleGrid,
    HoleLayout::Scattered,
    HoleLayout::Columns,
    HoleLayout::CenterPlusRing,
    HoleLayout::RotatedSquare,
    HoleLayout::SquareInCircle,
    HoleLayout::Arc,
    HoleLayout::ConcentricSquares,
    HoleLayout::TriangleInCircle,
    HoleLayout::Crescent,
    HoleLayout::LetterH,
    HoleLayout::LetterT,
    HoleLayout::Arrow,
    HoleLayout::Zigzag,
    HoleLayout::Clover,
    HoleLayout::Clusters,
    HoleLayout::ThreeColumns,
    HoleLayout::RingWithSpokes,
    HoleLayout::DiamondOutline,
    HoleLayout::BowTie,
];

fn min_holes(level_id: u32, rope_count: usize) -> usize {
    let base = rope_count * 2 + 4;
    match level_id {
        1..=5 => base.max(10),
        6..=15 => base.max(14),
        16..=30 => base.max(18),
        31..=60 => base.max(20),
        61..=100 => base.max(22),
        _ => base.max(24),
    }
}

impl HoleLayout {
    fn generate(self, n: usize) -> Vec<HolePos> {
        match self {
            HoleLayout::Grid => {
                let cols = 3usize.max((n as f32 * 1.25).sqrt().ceil() as usize);
                let rows = 3usize.max((n + cols - 1) / cols);
                let s = 0.42_f32.min(1.6 / (cols.max(rows) as f32 - 1.0).max(1.0));
                let ox = -(cols as f32 - 1.0) / 2.0 * s;
                let oy = -(rows as f32 - 1.0) / 2.0 * s;
                let mut pts = Vec::new();
                for row in 0..rows {
                    for col in 0..cols {
                        pts.push(hp(ox + col as f32 * s, oy + row as f32 * s));
                    }
                }
                pts
            }
            HoleLayout::Circle => {
                let count = 8usize.max(n);
                let r = 0.90_f32.min(0.40 + count as f32 * 0.035);
                (0..count)
                    .map(|i| {
                        let a = i as f32 / count as f32 * TAU - FRAC_PI_2;
                        hp(r * a.cos(), r * a.sin())
                    })
                    .collect()
            }
            HoleLayout::Hexagon => {
                let mut pts = vec![hp(0.0, 0.0)];
                let mut ring = 1u32;
                while pts.len() < n {
                    let count = ring * 6;
                    let r = ring as f32 * 0.95_f32.min(0.35).min(0.95 / (ring as f32 + 1.0));
                    for i in 0..count {
                        let a = i as f32 / count as f32 * TAU + ring as f32 * 0.15;
                        pts.push(hp(r * a.cos(), r * a.sin()));
                    }
                    ring += 1;
                }
                pts
            }
            HoleLayout::Diamond => {
                let mut half = 2usize.max((n as f32).sqrt().ceil() as usize);
                if half % 2 == 0 {
                    half += 1;
                }
                let s = 0.38_f32.min(1.6 / half as f32);
                let mut pts = Vec::new();
                for row_idx in 0..(half * 2 - 1) {
                    let dist = (row_idx as isize - (half as isize - 1)).unsigned_abs();
                    let count = half - dist;
                    let y = (row_idx as f32 - (half as f32 - 1.0)) * s;
                    let ox = -(count as f32 - 1.0) / 2.0 * s;
                    for col in 0..count {
                        pts.push(hp(ox + col as f32 * s, y));
                    }
                }
                pts
            }
            HoleLayout::Cross => {
                let arm = 2usize.max(n / 5);
                let s = 0.40_f32.min(1.6 / (arm as f32 * 2.0));
                let mut pts = Vec::new();
                for i in -(arm as i32)..=(arm as i32) {
                    pts.push(hp(0.0, i as f32 * s));
                }
                for i in -(arm as i32)..=(arm as i32) {
                    if i != 0 {
                        pts.push(hp(i as f32 * s, 0.0));
                    }
                }
                let fill = 1usize.max(arm - 1);
                for dx in 1..=fill {
                    for dy in 1..=fill {
                        for &sx in &[-1.0_f32, 1.0] {
                            for &sy in &[-1.0_f32, 1.0] {
                                pts.push(hp(sx * dx as f32 * s, sy * dy as f32 * s));
                            }
                        }
                    }
                }
                pts
            }
            HoleLayout::Rings => {
                let ring_count = if n <= 14 {
                    2
                } else if n <= 22 {
                    3
                } else {
                    4
                };
                let mut pts = Vec::new();
                let per_ring = 4usize.max((n - if ring_count > 2 { 1 } else { 0 }) / ring_count);
                if ring_count > 2 {
                    pts.push(hp(0.0, 0.0));
                }
                for ring in 0..ring_count {
                    let r = 0.25 + ring as f32 * (0.65 / (ring_count as f32 - 1.0).max(1.0));
                    let count = 4usize.max(per_ring + ring * 2);
                    let offset = ring as f32 * PI / count as f32;
                    for i in 0..count {
                        let a = i as f32 / count as f32 * TAU + offset;
                        pts.push(hp(r * a.cos(), r * a.sin()));
                    }
                }
                pts
            }
            HoleLayout::Triangle => {
                let mut rows = 4usize;
                while rows * (rows + 1) / 2 < n {
                    rows += 1;
                }
                let s = 0.38_f32.min(1.6 / rows as f32);
                let cy = (rows as f32 - 1.0) / 2.0;
                let mut pts = Vec::new();
                for row in 0..rows {
                    let count = row + 1;
                    let ox = -(count as f32 - 1.0) / 2.0 * s;
                    let y = (row as f32 - cy) * s * 0.866;
                    for col in 0..count {
                        pts.push(hp(ox + col as f32 * s, y));
                    }
                }
                pts
            }
            HoleLayout::Star => {
                let points = if n <= 14 {
                    5
                } else if n <= 22 {
                    6
                } else {
                    8
                };
                let mut pts = vec![hp(0.0, 0.0)];
                for i in 0..points {
                    let outer_a = i as f32 / points as f32 * TAU - FRAC_PI_2;
                    pts.push(hp(0.85 * outer_a.cos(), 0.85 * outer_a.sin()));
                    let inner_a = outer_a + PI / points as f32;
                    pts.push(hp(0.40 * inner_a.cos(), 0.40 * inner_a.sin()));
                }
                if pts.len() < n {
                    let extra = n - pts.len();
                    for i in 0..extra {
                        let a = i as f32 / extra as f32 * TAU - FRAC_PI_2 + 0.3;
                        pts.push(hp(0.62 * a.cos(), 0.62 * a.sin()));
                    }
                }
                pts
            }
            HoleLayout::Honeycomb => {
                let mut half_rows = 2usize;
                while (2 * half_rows + 1) * (half_rows + 2) < n {
                    half_rows += 1;
                }
                let s = 0.36_f32.min(1.6 / (2 * half_rows + 1) as f32);
                let h = s * 0.866;
                let mut pts = Vec::new();
                for row in -(half_rows as i32)..=(half_rows as i32) {
                    let (cols, offset) = if row.abs() % 2 == 0 {
                        (half_rows + 2, 0.0)
                    } else {
                        (half_rows + 1, s * 0.5)
                    };
                    let ox = -(cols as f32 - 1.0) / 2.0 * s + offset;
                    for col in 0..cols {
                        pts.push(hp(ox + col as f32 * s, row as f32 * h));
                    }
                }
                pts
            }
            HoleLayout::Spiral => {
                let count = 10usize.max(n - 1);
                let mut pts = vec![hp(0.0, 0.0)];
                let turns = 2.0 + count as f32 * 0.12;
                for i in 1..=count {
                    let t = i as f32 / count as f32;
                    let r = 0.12 + t * 0.78;
                    let a = t * turns * PI;
                    pts.push(hp(r * a.cos(), r * a.sin()));
                }
                pts
            }
            HoleLayout::DoubleGrid => {
                let half = 6usize.max(n / 2);
                let cols = 3usize.max((half as f32 * 1.5).sqrt().ceil() as usize);
                let rows = 2usize.max((half + cols - 1) / cols);
                let s = 0.38_f32.min(1.5 / (cols.max(rows * 2 + 1) as f32 - 1.0).max(1.0));
                let gap = s * 0.6;
                let mut pts = Vec::new();
                for &gy in &[-1.0_f32, 1.0] {
                    let cy = gy * ((rows as f32 - 1.0) / 2.0 * s + gap);
                    for row in 0..rows {
                        for col in 0..cols {
                            let x = -(cols as f32 - 1.0) / 2.0 * s + col as f32 * s;
                            let y = cy + (row as f32 - (rows as f32 - 1.0) / 2.0) * s;
                            pts.push(hp(x, y));
                        }
                    }
                }
                pts
            }
            HoleLayout::Scattered => {
                let cols = 3usize.max((n as f32 * 1.3).sqrt().ceil() as usize);
                let rows = 3usize.max((n + cols - 1) / cols);
                let sx = 1.6 / cols as f32;
                let sy = 1.6 / rows as f32;
                let mut pts = Vec::new();
                let mut idx = 0usize;
                for row in 0..rows {
                    for col in 0..cols {
                        if idx >= n {
                            break;
                        }
                        let jx = ((idx * 7 + 13) % 17) as f32 / 17.0 - 0.5;
                        let jy = ((idx * 11 + 3) % 13) as f32 / 13.0 - 0.5;
                        let x = -0.8 + col as f32 * sx + jx * sx * 0.35;
                        let y = -0.8 + row as f32 * sy + jy * sy * 0.35;
                        pts.push(hp(x, y));
                        idx += 1;
                    }
                }
                pts
            }
            HoleLayout::Columns => {
                let col_count = if n <= 12 {
                    2
                } else if n <= 20 {
                    3
                } else {
                    4
                };
                let row_count = 3usize.max((n + col_count - 1) / col_count);
                let s = 0.35_f32.min(1.5 / (row_count as f32 - 1.0).max(1.0));
                let x_spread = 0.55_f32.min(0.30 + col_count as f32 * 0.12);
                let mut pts = Vec::new();
                for col in 0..col_count {
                    let x = -x_spread
                        + col as f32 * (2.0 * x_spread / (col_count as f32 - 1.0).max(1.0));
                    for row in 0..row_count {
                        let y = (row as f32 - (row_count as f32 - 1.0) / 2.0) * s;
                        pts.push(hp(x, y));
                    }
                }
                pts
            }
            HoleLayout::CenterPlusRing => {
                let center_count = if n <= 14 {
                    1
                } else if n <= 20 {
                    2
                } else {
                    3
                };
                let mut pts = Vec::new();
                if center_count == 1 {
                    pts.push(hp(0.0, 0.0));
                } else {
                    let r = 0.12_f32;
                    for i in 0..center_count {
                        let a = i as f32 / center_count as f32 * TAU - FRAC_PI_2;
                        pts.push(hp(r * a.cos(), r * a.sin()));
                    }
                }
                let ring_count = n - center_count;
                let r = 0.85_f32.min(0.55 + ring_count as f32 * 0.02);
                for i in 0..ring_count {
                    let a = i as f32 / ring_count as f32 * TAU - FRAC_PI_2;
                    pts.push(hp(r * a.cos(), r * a.sin()));
                }
                pts
            }
            HoleLayout::RotatedSquare => {
                let rings = 2usize.max((n + 3) / 4);
                let mut pts = Vec::new();
                for ring in 1..=rings {
                    let r = ring as f32 / rings as f32 * 0.85;
                    let per_side = 1usize.max(ring);
                    let count = per_side * 4;
                    for i in 0..count {
                        let a = i as f32 / count as f32 * TAU + PI / 4.0;
                        pts.push(hp(r * a.cos(), r * a.sin()));
                    }
                }
                pts
            }
            HoleLayout::SquareInCircle => {
                let sq_side = 2usize.max(n / 4);
                let ring_count = 8usize.max(n - sq_side * 4);
                let sq = 0.50_f32.min(0.30 + sq_side as f32 * 0.05);
                let mut pts = Vec::new();
                let s = 2.0 * sq / (sq_side as f32 - 1.0).max(1.0);
                for row in 0..sq_side {
                    for col in 0..sq_side {
                        if row == 0 || row == sq_side - 1 || col == 0 || col == sq_side - 1 {
                            pts.push(hp(-sq + col as f32 * s, -sq + row as f32 * s));
                        }
                    }
                }
                let r = 0.90_f32.min(sq + 0.35);
                for i in 0..ring_count {
                    let a = i as f32 / ring_count as f32 * TAU - FRAC_PI_2;
                    pts.push(hp(r * a.cos(), r * a.sin()));
                }
                pts
            }
            HoleLayout::Arc => {
                let count = 8usize.max(n);
                let r = 0.85_f32.min(0.55 + count as f32 * 0.02);
                (0..count)
                    .map(|i| {
                        let a = i as f32 / (count as f32 - 1.0).max(1.0) * PI - FRAC_PI_2;
                        hp(r * a.cos(), r * a.sin())
                    })
                    .collect()
            }
            HoleLayout::ConcentricSquares => {
                let rings = 2usize.max(n / 6);
                let mut pts = Vec::new();
                for ring in 1..=rings {
                    let r = ring as f32 / rings as f32 * 0.85;
                    let count = 4usize.max(4 + (ring - 1) * 4);
                    for i in 0..count {
                        let a = i as f32 / count as f32 * TAU + PI / 4.0;
                        pts.push(hp(r * a.cos(), r * a.sin()));
                    }
                }
                pts
            }
            HoleLayout::TriangleInCircle => {
                let inner_count = 3usize.max(n / 4);
                let outer_count = 6usize.max(n - inner_count);
                let r_inner = 0.40_f32.min(0.20 + inner_count as f32 * 0.04);
                let mut pts = Vec::new();
                for i in 0..inner_count {
                    let a = i as f32 / inner_count as f32 * TAU - FRAC_PI_2;
                    pts.push(hp(r_inner * a.cos(), r_inner * a.sin()));
                }
                let r_outer = 0.85_f32.min(r_inner + 0.35);
                for i in 0..outer_count {
                    let a = i as f32 / outer_count as f32 * TAU - FRAC_PI_2;
                    pts.push(hp(r_outer * a.cos(), r_outer * a.sin()));
                }
                pts
            }
            HoleLayout::Crescent => {
                let outer_count = 5usize.max((n * 3 + 2) / 5);
                let inner_count = 4usize.max(n - outer_count);
                let r_out = 0.80_f32;
                let r_in = 0.50_f32;
                let mut pts = Vec::new();
                for i in 0..outer_count {
                    let a = i as f32 / (outer_count as f32 - 1.0).max(1.0) * PI * 0.9
                        - FRAC_PI_2
                        - PI / 8.0;
                    pts.push(hp(r_out * a.cos(), r_out * a.sin()));
                }
                for i in 0..inner_count {
                    let a = i as f32 / (inner_count as f32 - 1.0).max(1.0) * PI * 0.75 - FRAC_PI_2
                        + PI / 10.0;
                    pts.push(hp(r_in * a.cos() + 0.22, r_in * a.sin()));
                }
                pts
            }
            HoleLayout::LetterH => {
                let col_rows = 3usize.max((n - 1) / 2);
                let s = 0.32_f32.min(1.5 / (col_rows as f32 - 1.0).max(1.0));
                let xl = -0.45_f32;
                let xr = 0.45_f32;
                let mut pts = Vec::new();
                for row in 0..col_rows {
                    let y = (row as f32 - (col_rows as f32 - 1.0) / 2.0) * s;
                    pts.push(hp(xl, y));
                    pts.push(hp(xr, y));
                }
                let bar_count = 1usize.max(n - col_rows * 2);
                let bar_s = 0.9 / (bar_count as f32 + 1.0);
                for i in 1..=bar_count {
                    pts.push(hp(xl + i as f32 * bar_s, 0.0));
                }
                pts
            }
            HoleLayout::LetterT => {
                let top_count = 3usize.max(n / 3);
                let stem_count = 3usize.max(n - top_count);
                let top_s = 0.30_f32.min(1.5 / (top_count as f32 - 1.0).max(1.0));
                let stem_s = 0.30_f32.min(1.3 / stem_count as f32);
                let mut pts = Vec::new();
                for col in 0..top_count {
                    let x = (col as f32 - (top_count as f32 - 1.0) / 2.0) * top_s;
                    pts.push(hp(x, 0.65));
                }
                for row in 0..stem_count {
                    pts.push(hp(0.0, 0.65 - (row + 1) as f32 * stem_s));
                }
                pts
            }
            HoleLayout::Arrow => {
                let per_side = 3usize.max(n / 2);
                let mut pts = Vec::new();
                for i in 0..per_side {
                    let t = i as f32 / (per_side as f32 - 1.0).max(1.0);
                    pts.push(hp(-0.60 * (1.0 - t), -0.60 + t * 1.2));
                    pts.push(hp(0.60 * (1.0 - t), -0.60 + t * 1.2));
                }
                pts
            }
            HoleLayout::Zigzag => {
                let count = 8usize.max(n);
                let s = 0.28_f32.min(1.5 / (count as f32 - 1.0).max(1.0));
                (0..count)
                    .map(|i| {
                        let x = ((i % 2) as f32 - 0.5) * 0.65;
                        let y = (i as f32 - (count as f32 - 1.0) / 2.0) * s;
                        hp(x, y)
                    })
                    .collect()
            }
            HoleLayout::Clover => {
                let petals = if n <= 14 {
                    4
                } else if n <= 22 {
                    5
                } else {
                    6
                };
                let per_petal = 3usize.max(n / petals);
                let r = 0.22_f32;
                let big_r = 0.55_f32;
                let mut pts = Vec::new();
                for petal in 0..petals {
                    let ca = petal as f32 / petals as f32 * TAU;
                    let cx = big_r * ca.cos();
                    let cy = big_r * ca.sin();
                    for i in 0..per_petal {
                        let a = i as f32 / per_petal as f32 * TAU + ca;
                        pts.push(hp(cx + r * a.cos(), cy + r * a.sin()));
                    }
                }
                pts
            }
            HoleLayout::Clusters => {
                let cluster_count = if n <= 14 {
                    3
                } else if n <= 22 {
                    4
                } else {
                    5
                };
                let per_cluster = 3usize.max(n / cluster_count);
                let big_r = 0.65_f32;
                let r = 0.18_f32.min(0.10 + per_cluster as f32 * 0.02);
                let mut pts = Vec::new();
                for c in 0..cluster_count {
                    let ca = c as f32 / cluster_count as f32 * TAU - FRAC_PI_2;
                    let cx = big_r * ca.cos();
                    let cy = big_r * ca.sin();
                    for i in 0..per_cluster {
                        let a = i as f32 / per_cluster as f32 * TAU;
                        pts.push(hp(cx + r * a.cos(), cy + r * a.sin()));
                    }
                }
                pts
            }
            HoleLayout::ThreeColumns => {
                let row_count = 3usize.max((n + 2) / 3);
                let s = 0.35_f32.min(1.5 / (row_count as f32 - 1.0).max(1.0));
                let x_pos = [-0.55_f32, 0.0, 0.55];
                let mut pts = Vec::new();
                for &x in &x_pos {
                    for row in 0..row_count {
                        let y = (row as f32 - (row_count as f32 - 1.0) / 2.0) * s;
                        pts.push(hp(x, y));
                    }
                }
                pts
            }
            HoleLayout::RingWithSpokes => {
                let spokes = 3usize.max(n / 4);
                let ring_count = 6usize.max(n - spokes - 1);
                let mut pts = vec![hp(0.0, 0.0)];
                for i in 0..spokes {
                    let a = i as f32 / spokes as f32 * TAU;
                    pts.push(hp(0.35 * a.cos(), 0.35 * a.sin()));
                }
                for i in 0..ring_count {
                    let a = i as f32 / ring_count as f32 * TAU - FRAC_PI_2;
                    pts.push(hp(0.78 * a.cos(), 0.78 * a.sin()));
                }
                pts
            }
            HoleLayout::DiamondOutline => {
                let rings = 2usize.max(n / 4);
                let mut pts = Vec::new();
                for ring in 1..=rings {
                    let r = ring as f32 / rings as f32 * 0.78;
                    let count = 4usize.max(ring * 4);
                    for i in 0..count {
                        let a = i as f32 / count as f32 * TAU + PI / 4.0;
                        pts.push(hp(r * a.cos(), r * a.sin()));
                    }
                }
                pts
            }
            HoleLayout::BowTie => {
                let per_side = 3usize.max((n - 1) / 2);
                let mut pts = vec![hp(0.0, 0.0)];
                let r = 0.70_f32;
                for i in 0..per_side {
                    let a = i as f32 / per_side as f32 * PI - FRAC_PI_2;
                    pts.push(hp(r * a.cos(), r * a.sin()));
                }
                for i in 0..per_side {
                    let a = i as f32 / per_side as f32 * PI + FRAC_PI_2;
                    pts.push(hp(r * a.cos(), r * a.sin()));
                }
                pts
            }
        }
    }
}

fn hp(x: f32, y: f32) -> HolePos {
    HolePos {
        x_position: x,
        y_position: y,
        z_position: 0.0,
    }
}

fn pick_structured_pairs(
    holes: &[HolePos],
    count: usize,
    short_count: usize,
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
    let quarter = 1usize.max(n / 4);

    let mut pairs: Vec<(usize, usize)> = Vec::new();
    let mut used: HashSet<usize> = HashSet::new();

    if short_count > 0 {
        let sector_start = offset % n;
        let sector_len = (quarter + short_count).min(n / 2);

        let sector_holes: Vec<usize> = (0..sector_len)
            .map(|k| indexed[(sector_start + k) % n].0)
            .collect();

        let min_short_dist: f32 = 0.3;
        for i in 0..sector_holes.len() {
            if pairs.len() >= short_count {
                break;
            }
            let a = sector_holes[i];
            if used.contains(&a) {
                continue;
            }
            let shift = 2usize.max((quarter / 2).min(sector_holes.len() / 3));
            for delta in shift..sector_holes.len() {
                let b = sector_holes[(i + delta) % sector_holes.len()];
                if b == a || used.contains(&b) {
                    continue;
                }
                let dx = holes[a].x_position - holes[b].x_position;
                let dy = holes[a].y_position - holes[b].y_position;
                if (dx * dx + dy * dy).sqrt() < min_short_dist {
                    continue;
                }
                pairs.push((a, b));
                used.insert(a);
                used.insert(b);
                break;
            }
        }
    }

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
    let diff = difficulty(level_id);
    let use_3d = level_id >= 10 && level_id % 3 == 0;
    let n = min_holes(level_id, diff.rope_count);
    let (holes, boards) = if use_3d {
        let result = generate_board_layout(pick_board_layout(level_id), n, 0.12);
        (result.holes, Some(result.boards))
    } else {
        let layout = ALL_LAYOUTS[level_id as usize % ALL_LAYOUTS.len()];
        (layout.generate(n), None)
    };
    let max_ropes = (holes.len().saturating_sub(3)) / 2;
    let rope_count = diff.rope_count.min(max_ropes).max(1);

    let short_count = diff.short_rope_count.min(rope_count.saturating_sub(1));
    let rope_pairs = pick_structured_pairs(&holes, rope_count, short_count, &mut rng);

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

    let (short_center, short_radius) = if short_count > 0 {
        let mut sx = 0.0f32;
        let mut sy = 0.0f32;
        let mut cnt = 0.0f32;
        for i in 0..short_count {
            sx += hole_vecs[ropes[i].start_hole].x + hole_vecs[ropes[i].end_hole].x;
            sy += hole_vecs[ropes[i].start_hole].y + hole_vecs[ropes[i].end_hole].y;
            cnt += 2.0;
        }
        let center = Vec2::new(sx / cnt, sy / cnt);
        let mut max_d = 0.0f32;
        for i in 0..short_count {
            max_d = max_d.max((hole_vecs[ropes[i].start_hole] - center).length());
            max_d = max_d.max((hole_vecs[ropes[i].end_hole] - center).length());
        }
        (center, max_d + 0.35)
    } else {
        (Vec2::ZERO, 0.0)
    };

    let try_drag = |rope_idx: usize,
                    end_idx: usize,
                    target_rope: usize,
                    endpoints: &mut Vec<(usize, usize)>,
                    acts: &mut Vec<Action>,
                    hvecs: &[Vec2],
                    n_ropes: usize|
     -> bool {
        let current_hole = if end_idx == 0 {
            endpoints[rope_idx].0
        } else {
            endpoints[rope_idx].1
        };
        let anchor_hole = if end_idx == 0 {
            endpoints[rope_idx].1
        } else {
            endpoints[rope_idx].0
        };
        let anchor_pos = hvecs[anchor_hole];
        let ts = hvecs[endpoints[target_rope].0];
        let te = hvecs[endpoints[target_rope].1];
        let is_short = rope_idx < short_count;

        let other_used: HashSet<usize> = (0..n_ropes)
            .filter(|&ri| ri != rope_idx)
            .flat_map(|ri| [endpoints[ri].0, endpoints[ri].1])
            .collect();

        let mut best: Option<usize> = None;
        let mut best_score = f32::NEG_INFINITY;
        for c in 0..hvecs.len() {
            if c == current_hole || c == anchor_hole {
                continue;
            }
            if other_used.contains(&c) {
                continue;
            }
            if is_short && (hvecs[c] - short_center).length() > short_radius {
                continue;
            }
            if segments_cross(anchor_pos, hvecs[c], ts, te) {
                let dist = (hvecs[c] - (ts + te) * 0.5).length();
                if -dist > best_score {
                    best_score = -dist;
                    best = Some(c);
                }
            }
        }
        if let Some(target_hole) = best {
            if end_idx == 0 {
                endpoints[rope_idx].0 = target_hole;
            } else {
                endpoints[rope_idx].1 = target_hole;
            }
            acts.push(Action {
                kind: "drag".to_string(),
                rope_index: rope_idx,
                end_index: end_idx,
                hole_index: target_hole,
            });
            true
        } else {
            false
        }
    };

    if short_count >= 2 {
        for d in 0..short_count {
            let rope_idx = d;
            let mut done = false;
            for target_idx in 0..short_count {
                if target_idx == rope_idx || done {
                    continue;
                }
                let end_idx = d % 2;
                if try_drag(
                    rope_idx,
                    end_idx,
                    target_idx,
                    &mut current_endpoints,
                    &mut actions,
                    &hole_vecs,
                    ropes.len(),
                ) {
                    done = true;
                } else if try_drag(
                    rope_idx,
                    1 - end_idx,
                    target_idx,
                    &mut current_endpoints,
                    &mut actions,
                    &hole_vecs,
                    ropes.len(),
                ) {
                    done = true;
                }
            }
        }
    }

    for d in 0..diff.total_drags {
        let rope_idx = d % ropes.len();
        let target_rope = (rope_idx + 1 + d / ropes.len()) % ropes.len();
        if target_rope == rope_idx {
            continue;
        }
        let end_idx = (d / ropes.len()) % 2;
        if !try_drag(
            rope_idx,
            end_idx,
            target_rope,
            &mut current_endpoints,
            &mut actions,
            &hole_vecs,
            ropes.len(),
        ) {
            try_drag(
                rope_idx,
                1 - end_idx,
                target_rope,
                &mut current_endpoints,
                &mut actions,
                &hole_vecs,
                ropes.len(),
            );
        }
    }

    for _ in 0..ropes.len() * 8 {
        let crossings = rope_crossings(&current_endpoints, &hole_vecs);
        let isolated: Vec<usize> = (0..ropes.len()).filter(|&i| crossings[i] == 0).collect();
        if isolated.is_empty() {
            break;
        }
        for &rope_idx in &isolated {
            let mut fixed = false;
            for other in 0..ropes.len() {
                if other == rope_idx || fixed {
                    continue;
                }
                for end_idx in 0..=1 {
                    if fixed {
                        continue;
                    }
                    if try_drag(
                        rope_idx,
                        end_idx,
                        other,
                        &mut current_endpoints,
                        &mut actions,
                        &hole_vecs,
                        ropes.len(),
                    ) {
                        fixed = true;
                    }
                }
            }
            if !fixed {
                for other in 0..ropes.len() {
                    if other == rope_idx || fixed {
                        continue;
                    }
                    for end_idx in 0..=1 {
                        if fixed {
                            continue;
                        }
                        if try_drag(
                            other,
                            end_idx,
                            rope_idx,
                            &mut current_endpoints,
                            &mut actions,
                            &hole_vecs,
                            ropes.len(),
                        ) {
                            fixed = true;
                        }
                    }
                }
            }
            if !fixed {
                let all_used: HashSet<usize> = current_endpoints
                    .iter()
                    .flat_map(|ep| [ep.0, ep.1])
                    .collect();
                let free_holes: Vec<usize> = (0..hole_vecs.len())
                    .filter(|h| !all_used.contains(h))
                    .collect();
                'outer: for other in 0..ropes.len() {
                    if other == rope_idx {
                        continue;
                    }
                    let os = hole_vecs[current_endpoints[other].0];
                    let oe = hole_vecs[current_endpoints[other].1];
                    for &h0 in &free_holes {
                        for &h1 in &free_holes {
                            if h1 == h0 {
                                continue;
                            }
                            if segments_cross(hole_vecs[h0], hole_vecs[h1], os, oe) {
                                current_endpoints[rope_idx].0 = h0;
                                current_endpoints[rope_idx].1 = h1;
                                actions.push(Action {
                                    kind: "drag".to_string(),
                                    rope_index: rope_idx,
                                    end_index: 0,
                                    hole_index: h0,
                                });
                                actions.push(Action {
                                    kind: "drag".to_string(),
                                    rope_index: rope_idx,
                                    end_index: 1,
                                    hole_index: h1,
                                });
                                fixed = true;
                                break 'outer;
                            }
                        }
                    }
                }
            }
        }
    }

    LevelDefinition {
        id: level_id,
        hole_radius: 0.08,
        particles_per_rope: 32,
        holes,
        ropes,
        hooks: None,
        actions: Some(actions),
        boards,
    }
}

fn segments_cross(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> bool {
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

fn pick_board_layout(level_id: u32) -> BoardLayout {
    let layouts = [
        BoardLayout::TwoSides,
        BoardLayout::Bridge,
        BoardLayout::Staircase,
        BoardLayout::Platform,
        BoardLayout::Valley,
    ];
    layouts[((level_id / 3) as usize) % layouts.len()]
}

fn generate_board_layout(layout: BoardLayout, n: usize, elevation: f32) -> BoardLayoutResult {
    match layout {
        BoardLayout::TwoSides => two_sides_layout(n, elevation),
        BoardLayout::Bridge => bridge_layout(n, elevation),
        BoardLayout::Staircase => staircase_layout(n, elevation),
        BoardLayout::Platform => platform_layout(n, elevation),
        BoardLayout::Valley => valley_layout(n, elevation),
    }
}

fn two_sides_layout(n: usize, elevation: f32) -> BoardLayoutResult {
    let board_w: f32 = 0.55;
    let board_h: f32 = 1.4;
    let gap: f32 = 0.35;
    let left_board = Board {
        center_x: -(gap + board_w * 0.5),
        center_y: 0.0,
        width: board_w,
        height: board_h,
        elevation,
    };
    let right_board = Board {
        center_x: gap + board_w * 0.5,
        center_y: 0.0,
        width: board_w,
        height: board_h,
        elevation,
    };

    let center_holes = 4usize.max(n / 3);
    let side_holes = 3usize.max((n.saturating_sub(center_holes)) / 2);
    let s: f32 = 0.35;
    let mut holes = Vec::new();

    let center_cols = 2usize.max((center_holes as f32 * 1.5).sqrt().ceil() as usize);
    let center_rows = 2usize.max(center_holes.div_ceil(center_cols));
    let cs = s.min(gap * 1.5 / center_cols as f32);
    for row in 0..center_rows {
        for col in 0..center_cols {
            let x = (col as f32 - (center_cols - 1) as f32 / 2.0) * cs;
            let y = (row as f32 - (center_rows - 1) as f32 / 2.0) * cs;
            holes.push(HolePos {
                x_position: x,
                y_position: y,
                z_position: 0.0,
            });
        }
    }

    let side_rows = 2usize.max(side_holes);
    let ss = s.min(board_h * 0.8 / (side_rows - 1) as f32);
    for i in 0..side_rows {
        let y = (i as f32 - (side_rows - 1) as f32 / 2.0) * ss;
        holes.push(HolePos {
            x_position: left_board.center_x,
            y_position: y,
            z_position: elevation,
        });
    }
    for i in 0..side_rows {
        let y = (i as f32 - (side_rows - 1) as f32 / 2.0) * ss;
        holes.push(HolePos {
            x_position: right_board.center_x,
            y_position: y,
            z_position: elevation,
        });
    }

    BoardLayoutResult {
        holes,
        boards: vec![left_board, right_board],
    }
}

fn bridge_layout(n: usize, elevation: f32) -> BoardLayoutResult {
    let board_w: f32 = 1.8;
    let board_h: f32 = 0.45;
    let bridge = Board {
        center_x: 0.0,
        center_y: 0.0,
        width: board_w,
        height: board_h,
        elevation,
    };

    let top_holes = 3usize.max(n / 3);
    let bottom_holes = 3usize.max(n / 3);
    let bridge_holes = 3usize.max(n.saturating_sub(top_holes + bottom_holes));
    let s: f32 = 0.35;
    let mut holes = Vec::new();

    let cols = 2usize.max((top_holes as f32 * 2.0).sqrt().ceil() as usize);
    let rows = 2usize.max(top_holes.div_ceil(cols));
    let ts = s.min(1.4 / cols as f32);
    for row in 0..rows {
        for col in 0..cols {
            let x = (col as f32 - (cols - 1) as f32 / 2.0) * ts;
            let y = board_h * 0.5 + 0.25 + row as f32 * ts;
            holes.push(HolePos {
                x_position: x,
                y_position: y,
                z_position: 0.0,
            });
        }
    }
    for row in 0..rows {
        for col in 0..cols {
            if holes.len() >= top_holes + bottom_holes {
                break;
            }
            let x = (col as f32 - (cols - 1) as f32 / 2.0) * ts;
            let y = -(board_h * 0.5 + 0.25 + row as f32 * ts);
            holes.push(HolePos {
                x_position: x,
                y_position: y,
                z_position: 0.0,
            });
        }
    }

    let b_cols = 2usize.max(bridge_holes);
    let bs = s.min(board_w * 0.7 / (b_cols - 1) as f32);
    for i in 0..b_cols {
        let x = (i as f32 - (b_cols - 1) as f32 / 2.0) * bs;
        holes.push(HolePos {
            x_position: x,
            y_position: 0.0,
            z_position: elevation,
        });
    }

    BoardLayoutResult {
        holes,
        boards: vec![bridge],
    }
}

fn staircase_layout(n: usize, elevation: f32) -> BoardLayoutResult {
    let steps = 3usize;
    let step_w: f32 = 0.55;
    let step_h: f32 = 1.2;
    let step_gap: f32 = 0.08;
    let holes_per_step = 3usize.max(n / (steps + 1));
    let s: f32 = 0.32;

    let mut boards = Vec::new();
    let mut holes = Vec::new();
    let floor_holes = 3usize.max(n.saturating_sub(holes_per_step * steps));
    let floor_x = -(steps as f32 * (step_w + step_gap)) * 0.5 - 0.3;
    let floor_rows = 2usize.max(floor_holes);
    let fs = s.min(step_h * 0.8 / (floor_rows - 1) as f32);
    for i in 0..floor_rows {
        let y = (i as f32 - (floor_rows - 1) as f32 / 2.0) * fs;
        holes.push(HolePos {
            x_position: floor_x,
            y_position: y,
            z_position: 0.0,
        });
    }

    for step in 0..steps {
        let elev = (step + 1) as f32 * elevation;
        let cx = step as f32 * (step_w + step_gap) - (steps - 1) as f32 * (step_w + step_gap) * 0.5
            + step_w * 0.5;
        boards.push(Board {
            center_x: cx,
            center_y: 0.0,
            width: step_w,
            height: step_h,
            elevation: elev,
        });
        let step_rows = 2usize.max(holes_per_step);
        let ss = s.min(step_h * 0.7 / (step_rows - 1) as f32);
        for i in 0..step_rows {
            let y = (i as f32 - (step_rows - 1) as f32 / 2.0) * ss;
            holes.push(HolePos {
                x_position: cx,
                y_position: y,
                z_position: elev,
            });
        }
    }

    BoardLayoutResult { holes, boards }
}

fn platform_layout(n: usize, elevation: f32) -> BoardLayoutResult {
    let board_w: f32 = 0.8;
    let board_h: f32 = 0.8;
    let platform = Board {
        center_x: 0.0,
        center_y: 0.0,
        width: board_w,
        height: board_h,
        elevation,
    };
    let platform_holes = 4usize.max(n / 3);
    let ground_holes = n.saturating_sub(platform_holes);
    let s: f32 = 0.35;
    let mut holes = Vec::new();

    let p_cols = 2usize.max((platform_holes as f32).sqrt().ceil() as usize);
    let p_rows = 2usize.max(platform_holes.div_ceil(p_cols));
    let ps = s.min(board_w * 0.7 / (p_cols.max(p_rows) - 1) as f32);
    for row in 0..p_rows {
        for col in 0..p_cols {
            let x = (col as f32 - (p_cols - 1) as f32 / 2.0) * ps;
            let y = (row as f32 - (p_rows - 1) as f32 / 2.0) * ps;
            holes.push(HolePos {
                x_position: x,
                y_position: y,
                z_position: elevation,
            });
        }
    }

    let ring_r = board_w * 0.5 + 0.35;
    let ring_count = 8usize.max(ground_holes);
    for i in 0..ring_count {
        let a = i as f32 / ring_count as f32 * TAU - FRAC_PI_2;
        holes.push(HolePos {
            x_position: ring_r * a.cos(),
            y_position: ring_r * a.sin(),
            z_position: 0.0,
        });
    }

    BoardLayoutResult {
        holes,
        boards: vec![platform],
    }
}

fn valley_layout(n: usize, elevation: f32) -> BoardLayoutResult {
    let board_w: f32 = 0.50;
    let board_h: f32 = 1.6;
    let left_board = Board {
        center_x: -0.65,
        center_y: 0.0,
        width: board_w,
        height: board_h,
        elevation,
    };
    let right_board = Board {
        center_x: 0.65,
        center_y: 0.0,
        width: board_w,
        height: board_h,
        elevation,
    };

    let side_holes = 3usize.max(n / 3);
    let valley_holes = 4usize.max(n.saturating_sub(side_holes * 2));
    let s: f32 = 0.32;
    let mut holes = Vec::new();

    let s_rows = 2usize.max(side_holes);
    let ss = s.min(board_h * 0.7 / (s_rows - 1) as f32);
    for i in 0..s_rows {
        let y = (i as f32 - (s_rows - 1) as f32 / 2.0) * ss;
        holes.push(HolePos {
            x_position: left_board.center_x,
            y_position: y,
            z_position: elevation,
        });
    }
    for i in 0..s_rows {
        let y = (i as f32 - (s_rows - 1) as f32 / 2.0) * ss;
        holes.push(HolePos {
            x_position: right_board.center_x,
            y_position: y,
            z_position: elevation,
        });
    }

    let v_cols = 2usize.max((valley_holes as f32 * 2.0).sqrt().ceil() as usize);
    let v_rows = 2usize.max(valley_holes.div_ceil(v_cols));
    let vs = s.min(0.5 / (v_cols.max(v_rows) - 1) as f32);
    for row in 0..v_rows {
        for col in 0..v_cols {
            let x = (col as f32 - (v_cols - 1) as f32 / 2.0) * vs;
            let y = (row as f32 - (v_rows - 1) as f32 / 2.0) * vs;
            holes.push(HolePos {
                x_position: x,
                y_position: y,
                z_position: 0.0,
            });
        }
    }

    BoardLayoutResult {
        holes,
        boards: vec![left_board, right_board],
    }
}
