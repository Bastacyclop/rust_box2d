#[deriving(Clone, PartialEq, Show)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

#[deriving(Clone, PartialEq, Show)]
pub struct Rot {
    pub sin: f32,
    pub cos: f32,
}

#[deriving(Clone, PartialEq, Show)]
pub struct Transform {
    pub p: Vec2,
    pub q: Rot,
}
