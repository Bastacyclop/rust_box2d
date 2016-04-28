use wrapped2d::b2;

pub struct Camera {
    pub position: [f64; 2],
    pub size: [f64; 2],
}

impl Camera {
    pub fn new() -> Camera {
        Camera {
            position: [0., 0.],
            size: [1., 1.],
        }
    }

    pub fn transform_world_to_gl(&self) -> [[f64; 3]; 2] {
        let translate_x = -self.position[0];
        let translate_y = -self.position[1];
        let scale_x = 2. / self.size[0];
        let scale_y = 2. / self.size[1];
        [[scale_x, 0., translate_x * scale_x], [0., scale_y, translate_y * scale_y]]
    }

    pub fn gl_to_world(&self, x: f64, y: f64) -> b2::Vec2 {
        let scale_x = self.size[0] / 2.;
        let scale_y = self.size[1] / 2.;
        let translate_x = self.position[0];
        let translate_y = self.position[1];
        b2::Vec2 {
            x: (x * scale_x + translate_x) as f32,
            y: (y * scale_y + translate_y) as f32,
        }
    }
}
