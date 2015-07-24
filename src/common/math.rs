use std::ops::{ Add, Sub, Mul, Div, Neg };
#[cfg(feature = "nalgebra")] use nalgebra;
#[cfg(feature = "cgmath")] use cgmath;

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
#[derive(Clone, Copy, PartialEq, Debug)]
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

#[cfg(feature = "nalgebra")]
impl From<Vec2> for nalgebra::Vec2<f32> {
    fn from(v: Vec2) -> nalgebra::Vec2<f32> {
        nalgebra::Vec2 { x: v.x, y: v.y }
    }
}

#[cfg(feature = "nalgebra")]
impl From<nalgebra::Vec2<f32>> for Vec2 {
    fn from(v: nalgebra::Vec2<f32>) -> Vec2 {
        Vec2 { x: v.x, y: v.y }
    }
}

#[cfg(feature = "nalgebra")]
impl From<Vec2> for nalgebra::Pnt2<f32> {
    fn from(v: Vec2) -> nalgebra::Pnt2<f32> {
        nalgebra::Pnt2 { x: v.x, y: v.y }
    }
}

#[cfg(feature = "nalgebra")]
impl From<nalgebra::Pnt2<f32>> for Vec2 {
    fn from(v: nalgebra::Pnt2<f32>) -> Vec2 {
        Vec2 { x: v.x, y: v.y }
    }
}

#[cfg(feature = "cgmath")]
impl From<Vec2> for cgmath::Vector2<f32> {
    fn from(v: Vec2) -> cgmath::Vector2<f32> {
        cgmath::Vector2 { x: v.x, y: v.y }
    }
}

#[cfg(feature = "cgmath")]
impl From<cgmath::Vector2<f32>> for Vec2 {
    fn from(v: cgmath::Vector2<f32>) -> Vec2 {
        Vec2 { x: v.x, y: v.y }
    }
}

#[cfg(feature = "cgmath")]
impl From<Vec2> for cgmath::Point2<f32> {
    fn from(v: Vec2) -> cgmath::Point2<f32> {
        cgmath::Point2 { x: v.x, y: v.y }
    }
}

#[cfg(feature = "cgmath")]
impl From<cgmath::Point2<f32>> for Vec2 {
    fn from(v: cgmath::Point2<f32>) -> Vec2 {
        Vec2 { x: v.x, y: v.y }
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

pub fn cross_vv(a: Vec2, b: Vec2) -> f32 {
    a.x*b.y - a.y*b.x
}

pub fn cross_vs(v: Vec2, s: f32) -> Vec2 {
    Vec2 { x: s*v.y, y: -s*v.x }
}

pub fn cross_sv(s: f32, v: Vec2) -> Vec2 {
    Vec2 { x: -s*v.y, y: s*v.x }
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Rot {
    pub sin: f32,
    pub cos: f32,
}

impl Rot {
    pub fn from_angle(angle: f32) -> Rot {
        Rot {
            sin: angle.sin(),
            cos: angle.cos()
        }
    }

    pub fn identity() -> Rot {
        Rot { sin: 0., cos: 1. }
    }

    pub fn x_axis(&self) -> Vec2 {
        Vec2 { x: self.cos, y: self.sin }
    }

    pub fn y_axis(&self) -> Vec2 {
        Vec2 { x: -self.sin, y: self.cos }
    }

    pub fn angle(&self) -> f32 {
        self.sin.atan2(self.cos)
    }
}

#[cfg(feature = "nalgebra")]
impl From<Rot> for nalgebra::Rot2<f32> {
    fn from(r: Rot) -> nalgebra::Rot2<f32> {
        nalgebra::Rot2::new(nalgebra::Vec1::new(r.angle()))
    }
}

#[cfg(feature = "nalgebra")]
impl<'a> From<&'a nalgebra::Rot2<f32>> for Rot {
    fn from(r: &'a nalgebra::Rot2<f32>) -> Rot {
        use nalgebra::Rotation;
        Rot::from_angle(r.rotation().x)
    }
}

#[cfg(feature = "cgmath")]
impl From<Rot> for cgmath::Basis2<f32> {
    fn from(r: Rot) -> cgmath::Basis2<f32> {
        use cgmath::Rotation2;
        cgmath::Basis2::from_angle(cgmath::Rad { s: r.angle() })
    }
}

#[cfg(feature = "cgmath")]
impl<'a> From<&'a cgmath::Basis2<f32>> for Rot {
    fn from(r: &'a cgmath::Basis2<f32>) -> Rot {
        let col = r.as_ref().y;
        Rot { sin: col.x, cos: col.y }
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

#[cfg(feature = "nalgebra")]
impl<'a> From<&'a Transform> for nalgebra::Iso2<f32> {
    fn from(t: &'a Transform) -> nalgebra::Iso2<f32> {
        nalgebra::Iso2 {
            rotation: t.rot.into(),
            translation: t.pos.into()
        }
    }
}

#[cfg(feature = "nalgebra")]
impl<'a> From<&'a nalgebra::Iso2<f32>> for Transform {
    fn from(i: &'a nalgebra::Iso2<f32>) -> Transform {
        Transform {
            pos: i.translation.into(),
            rot: (&i.rotation).into()
        }
    }
}

impl<'a> Mul<Vec2> for &'a Transform {
    type Output = Vec2;

    fn mul(self, v: Vec2) -> Vec2 {
        let x = (self.rot.cos*v.x - self.rot.sin*v.y) + self.pos.x;
        let y = (self.rot.sin*v.x + self.rot.cos*v.y) + self.pos.y;
        Vec2 { x: x, y: y }
    }
}

#[repr(C)]
#[derive(Clone, Debug)]
pub struct Sweep {
    pub local_center: Vec2,
    pub c0: Vec2,
    pub c: Vec2,
    pub a0: f32,
    pub a: f32,
    pub alpha0: f32
}
