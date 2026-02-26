use glam::{Mat4, Vec3};

pub struct Camera {
    pub center: Vec3,
    pub distance: f32,
    pub ortho_half_height: f32,
    pub tilt_angle: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            center: Vec3::ZERO,
            distance: 2.8,
            ortho_half_height: 2.05,
            tilt_angle: 0.0,
        }
    }
}

impl Camera {
    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let y_offset = self.distance * self.tilt_angle.sin();
        let z_offset = self.distance * self.tilt_angle.cos();
        let eye = self.center + Vec3::new(0.0, y_offset, z_offset);
        let view = look_at(eye, self.center, Vec3::new(0.0, 1.0, 0.0));
        let half_h = self.ortho_half_height;
        let half_w = self.ortho_half_height * aspect;
        let proj = ortho(-half_w, half_w, -half_h, half_h, 0.01, 10.0);
        proj * view
    }

    pub fn eye_position(&self) -> Vec3 {
        self.center
            + Vec3::new(
                0.0,
                self.distance * self.tilt_angle.sin(),
                self.distance * self.tilt_angle.cos(),
            )
    }
}

pub fn ortho(left: f32, right: f32, bottom: f32, top: f32, near: f32, far: f32) -> Mat4 {
    let rl = right - left;
    let tb = top - bottom;
    let f_n = far - near;
    Mat4::from_cols_array_2d(&[
        [2.0 / rl, 0.0, 0.0, 0.0],
        [0.0, 2.0 / tb, 0.0, 0.0],
        [0.0, 0.0, -1.0 / f_n, 0.0],
        [-(right + left) / rl, -(top + bottom) / tb, -near / f_n, 1.0],
    ])
}

pub fn look_at(eye: Vec3, center: Vec3, up: Vec3) -> Mat4 {
    let f = (center - eye).normalize();
    let s = f.cross(up).normalize();
    let u = s.cross(f);
    Mat4::from_cols_array_2d(&[
        [s.x, u.x, -f.x, 0.0],
        [s.y, u.y, -f.y, 0.0],
        [s.z, u.z, -f.z, 0.0],
        [-s.dot(eye), -u.dot(eye), f.dot(eye), 1.0],
    ])
}
