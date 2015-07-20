use std::ops::{ Add, Sub, Mul, Div, Neg };

macro_rules! forward_ref_binop {
    (impl $imp:ident, $method:ident for $t:ty, $u:ty) => {
        impl<'a> $imp<$u> for &'a $t {
            type Output = <$t as $imp<$u>>::Output;

            #[inline]
            fn $method(self, other: $u) -> <$t as $imp<$u>>::Output {
                $imp::$method(*self, other)
            }
        }

        impl<'a> $imp<&'a $u> for $t {
            type Output = <$t as $imp<$u>>::Output;

            #[inline]
            fn $method(self, other: &'a $u) -> <$t as $imp<$u>>::Output {
                $imp::$method(self, *other)
            }
        }

        impl<'a, 'b> $imp<&'a $u> for &'b $t {
            type Output = <$t as $imp<$u>>::Output;

            #[inline]
            fn $method(self, other: &'a $u) -> <$t as $imp<$u>>::Output {
                $imp::$method(*self, *other)
            }
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub fn sqr_norm(&self) -> f32 {
        self.x*self.x + self.y*self.y
    }

    pub fn norm(&self) -> f32 {
        self.sqr_norm().sqrt()
    }

    pub fn sqew(&self) -> Vec2 {
        Vec2 { x: -self.y, y: self.x }
    }
}

impl Add for Vec2 {
    type Output = Vec2;

    fn add(self, other: Vec2) -> Vec2 {
        Vec2 { x: self.x + other.x, y: self.y + other.y }
    }
}

forward_ref_binop! { impl Add, add for Vec2, Vec2 }

impl Sub for Vec2 {
    type Output = Vec2;

    fn sub(self, other: Vec2) -> Vec2 {
        Vec2 { x: self.x - other.x, y: self.y - other.y }
    }
}

forward_ref_binop! { impl Sub, sub for Vec2, Vec2 }

impl Mul<f32> for Vec2 {
    type Output = Vec2;

    fn mul(self, factor: f32) -> Vec2 {
        Vec2 { x: self.x*factor, y: self.y*factor }
    }
}

forward_ref_binop! { impl Mul, mul for Vec2, f32 }

impl Div<f32> for Vec2 {
    type Output = Vec2;

    fn div(self, factor: f32) -> Vec2 {
        Vec2 { x: self.x/factor, y: self.y/factor }
    }
}

forward_ref_binop! { impl Div, div for Vec2, f32 }

impl Neg for Vec2 {
    type Output = Vec2;

    fn neg(self) -> Vec2 {
        Vec2 { x: -self.x, y: -self.y }
    }
}

impl<'a> Neg for &'a Vec2 {
    type Output = Vec2;

    fn neg(self) -> Vec2 {
        Vec2 { x: -self.x, y: -self.y }
    }
}

#[repr(C)]
#[derive(Copy, Clone, PartialEq, Debug)]
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
#[derive(Clone, PartialEq, Debug)]
pub struct Transform {
    pub pos: Vec2,
    pub rot: Rot,
}

impl Transform {
    pub fn identity() -> Transform {
        Transform {
            pos: Vec2 { x: 0., y: 0. },
            rot: Rot::identity()
        }
    }
}

#[repr(C)]
#[derive(Clone)]
pub struct Sweep {
    pub local_center: Vec2,
    pub c0: Vec2,
    pub c: Vec2,
    pub a0: f32,
    pub a: f32,
    pub alpha0: f32
}
