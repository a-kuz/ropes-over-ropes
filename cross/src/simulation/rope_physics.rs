use glam::Vec3;

pub fn linking_number(pts_a: &[Vec3], pts_b: &[Vec3]) -> i32 {
    let mut sum: i32 = 0;
    if pts_a.len() < 2 || pts_b.len() < 2 {
        return 0;
    }
    for i in 0..pts_a.len() - 1 {
        let a0x = pts_a[i].x;
        let a0y = pts_a[i].y;
        let a1x = pts_a[i + 1].x;
        let a1y = pts_a[i + 1].y;
        let d1x = a1x - a0x;
        let d1y = a1y - a0y;
        for j in 0..pts_b.len() - 1 {
            let b0x = pts_b[j].x;
            let b0y = pts_b[j].y;
            let b1x = pts_b[j + 1].x;
            let b1y = pts_b[j + 1].y;
            let d2x = b1x - b0x;
            let d2y = b1y - b0y;
            let cross = d1x * d2y - d1y * d2x;
            if cross.abs() < 1e-9 {
                continue;
            }
            let dx = b0x - a0x;
            let dy = b0y - a0y;
            let t = (dx * d2y - dy * d2x) / cross;
            let u = (dx * d1y - dy * d1x) / cross;
            if t > 0.01 && t < 0.99 && u > 0.01 && u < 0.99 {
                let z_a = pts_a[i].z * (1.0 - t) + pts_a[i + 1].z * t;
                let z_b = pts_b[j].z * (1.0 - u) + pts_b[j + 1].z * u;
                let orientation = if cross > 0.0 { 1 } else { -1 };
                let over = if z_a > z_b { 1 } else { -1 };
                sum += orientation * over;
            }
        }
    }
    sum
}
