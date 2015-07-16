pub struct Camera {
    pub position: [f64; 2],
    pub scale: [f64; 2]
}

impl Camera {
    pub fn new() -> Camera {
        Camera {
            position: [0., 0.],
            scale: [1., 1.]
        }
    }

    pub fn transform(&self) -> [[f64; 3]; 2] {
        let s = self.scale;
        let p = self.position;
        [
            [ s[0],  0. , p[0]*s[0] ],
            [  0. , s[1], p[1]*s[1] ],
        ]
    }
}
