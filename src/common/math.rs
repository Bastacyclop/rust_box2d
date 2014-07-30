use std::num::Zero;

#[repr(C)]
#[deriving(Clone, PartialEq, Show)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub fn sqr_norm(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }
    pub fn norm(&self) -> f32 {
        self.sqr_norm().sqrt()
    }
    pub fn sqew(&self) -> Vec2 {
        Vec2 { x: -self.y, y: self.x }
    }
}

impl Add<Vec2, Vec2> for Vec2 {
    fn add(&self, rhs: &Vec2) -> Vec2 {
        Vec2 { x: self.x+rhs.x, y: self.y+rhs.y }
    }
}

impl Sub<Vec2, Vec2> for Vec2 {
    fn sub(&self, rhs: &Vec2) -> Vec2 {
        Vec2 { x: self.x-rhs.x, y: self.y-rhs.y }
    }
}

impl Zero for Vec2 {
    fn zero() -> Vec2 {
        Vec2 { x: 0., y: 0. }
    }
    fn is_zero(&self) -> bool {
        if self.x == 0. &&
           self.y == 0. {
            true  
        } else {
            false
        }
    }
}

impl Mul<f32, Vec2> for Vec2 {
    fn mul(&self, rhs: &f32) -> Vec2 {
        Vec2 { x: self.x*(*rhs), y: self.y*(*rhs) }
    }
}

impl Div<f32, Vec2> for Vec2 {
    fn div(&self, rhs: &f32) -> Vec2 {
        Vec2 { x: self.x/(*rhs), y: self.y/(*rhs) }
    }
}

impl Neg<Vec2> for Vec2 {
    fn neg(&self) -> Vec2 {
        Vec2 { x: -self.x, y: -self.y }
    }
}

#[repr(C)]
#[deriving(Clone, PartialEq, Show)]
pub struct Rot {
    pub sin: f32,
    pub cos: f32,
}

impl Rot {
    pub fn identity() -> Rot {
        Rot { sin: 0., cos: 1. }
    }
    pub fn x_axis(&self) -> Vec2 {
        Vec2 { x: self.cos, y: self.sin }
    }
    pub fn y_axis(&self) -> Vec2 {
        Vec2 { x: -self.sin, y: self.cos }
    }
}

#[repr(C)]
#[deriving(Clone, PartialEq, Show)]
pub struct Transform {
    pub pos: Vec2,
    pub rot: Rot,
}

impl Transform {
    pub fn identity() -> Transform {
        Transform {
            pos: Zero::zero(),
            rot: Rot::identity()
        }
    }
}
