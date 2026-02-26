use glam::Vec3;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct HoleVertex {
    pub position: Vec3,
    pub normal: Vec3,
}

pub struct HoleMesh {
    pub vertices: Vec<HoleVertex>,
    pub indices: Vec<u16>,
}

pub fn build(segments: usize, inner_radius: f32, outer_radius: f32, depth: f32) -> HoleMesh {
    let seg_count = segments.clamp(12, 128);
    let inner = inner_radius.clamp(0.05, 0.98);
    let outer = outer_radius.clamp(inner + 0.01, 1.5);
    let hole_depth = depth.max(0.1);

    let mut vertices = Vec::with_capacity(seg_count * 6);
    let mut indices = Vec::with_capacity(seg_count * 18);

    let ring_point = |radius: f32, angle: f32, z: f32| -> Vec3 {
        Vec3::new(angle.cos() * radius, angle.sin() * radius, z)
    };

    let up = Vec3::Z;

    let top_z: f32 = 0.005;

    for seg_index in 0..seg_count {
        let a0 = (seg_index as f32 / seg_count as f32) * std::f32::consts::TAU;
        let a1 = ((seg_index + 1) as f32 / seg_count as f32) * std::f32::consts::TAU;

        let o0 = ring_point(outer, a0, top_z);
        let o1 = ring_point(outer, a1, top_z);
        let i0 = ring_point(inner, a0, top_z);
        let i1 = ring_point(inner, a1, top_z);

        let base = vertices.len() as u16;
        vertices.push(HoleVertex { position: o0, normal: up });
        vertices.push(HoleVertex { position: o1, normal: up });
        vertices.push(HoleVertex { position: i0, normal: up });
        vertices.push(HoleVertex { position: i1, normal: up });

        indices.extend_from_slice(&[
            base, base + 2, base + 1,
            base + 1, base + 2, base + 3,
        ]);
    }

    for seg_index in 0..seg_count {
        let a0 = (seg_index as f32 / seg_count as f32) * std::f32::consts::TAU;
        let a1 = ((seg_index + 1) as f32 / seg_count as f32) * std::f32::consts::TAU;

        let top0 = ring_point(inner, a0, top_z);
        let top1 = ring_point(inner, a1, top_z);
        let bot0 = ring_point(inner, a0, -hole_depth);
        let bot1 = ring_point(inner, a1, -hole_depth);

        let n0 = Vec3::new(-a0.cos(), -a0.sin(), 0.0).normalize();
        let n1 = Vec3::new(-a1.cos(), -a1.sin(), 0.0).normalize();

        let base = vertices.len() as u16;
        vertices.push(HoleVertex { position: top0, normal: n0 });
        vertices.push(HoleVertex { position: bot0, normal: n0 });
        vertices.push(HoleVertex { position: top1, normal: n1 });
        vertices.push(HoleVertex { position: bot1, normal: n1 });

        indices.extend_from_slice(&[
            base, base + 1, base + 2,
            base + 2, base + 1, base + 3,
        ]);
    }

    HoleMesh { vertices, indices }
}
