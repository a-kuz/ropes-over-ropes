use glam::{Mat4, Vec3};

pub struct Camera {
    pub center: Vec3,
    pub distance: f32,
    pub ortho_half_height: f32,
    pub tilt_angle: f32,
    pub orbit_angle: f32,
    pub perspective_blend: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            center: Vec3::ZERO,
            distance: 2.8,
            ortho_half_height: 2.05,
            tilt_angle: 0.0,
            orbit_angle: 0.0,
            perspective_blend: 0.0,
        }
    }
}

impl Camera {
    pub fn view_proj(&self, aspect: f32) -> Mat4 {
        let eye = self.eye_position();
        let view = look_at(eye, self.center, Vec3::Y);
        let rot = rotation_z(self.orbit_angle);

        let half_h = self.ortho_half_height;
        let half_w = half_h * aspect;
        let ortho_proj = ortho(-half_w, half_w, -half_h, half_h, 0.01, 20.0);

        if self.perspective_blend < 0.001 {
            return ortho_proj * rot * view;
        }

        let fov = 59.0f32.to_radians();
        let persp_proj = Mat4::perspective_rh(fov, aspect, 0.01, 20.0);

        if self.perspective_blend > 0.999 {
            return persp_proj * rot * view;
        }

        let t = self.perspective_blend;
        let a = (ortho_proj * rot * view).to_cols_array();
        let b = (persp_proj * rot * view).to_cols_array();
        let mut out = [0.0f32; 16];
        for i in 0..16 {
            out[i] = a[i] * (1.0 - t) + b[i] * t;
        }
        Mat4::from_cols_array(&out)
    }

    pub fn eye_position(&self) -> Vec3 {
        let y_offset = self.distance * self.tilt_angle.sin();
        let z_offset = self.distance * self.tilt_angle.cos();
        self.center + Vec3::new(0.0, y_offset, z_offset)
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

pub fn rotation_z(angle: f32) -> Mat4 {
    let c = angle.cos();
    let s = angle.sin();
    Mat4::from_cols_array_2d(&[
        [c, s, 0.0, 0.0],
        [-s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
}
