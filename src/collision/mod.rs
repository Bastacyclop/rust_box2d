pub mod shapes;
pub mod distance;
pub mod time_of_impact;

use std::mem;
use wrap::*;
use common::settings::MAX_MANIFOLD_POINTS;
use common::math::{Vec2, Transform};
use collision::shapes::Shape;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ContactFeatureType {
    Vertex,
    Face,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ContactFeature {
    pub index_a: u8,
    pub index_b: u8,
    pub type_a: ContactFeatureType,
    pub type_b: ContactFeatureType,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ContactId(u32);

impl ContactId {
    pub fn feature(&self) -> ContactFeature {
        unsafe { mem::transmute(self.0) }
    }

    pub fn key(&self) -> u32 {
        self.0
    }
}

#[repr(C)]
#[derive(Clone, Debug)]
pub struct ManifoldPoint {
    pub local_point: Vec2,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub id: ContactId,
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ManifoldType {
    Circles,
    FaceA,
    FaceB,
}

#[repr(C)]
#[derive(Debug)]
pub struct Manifold {
    pub points: [ManifoldPoint; MAX_MANIFOLD_POINTS],
    pub local_normal: Vec2,
    pub local_point: Vec2,
    pub manifold_type: ManifoldType,
    pub count: i32,
}

impl Manifold {
    pub fn world_manifold(&self,
                          xf_a: &Transform,
                          radius_a: f32,
                          xf_b: &Transform,
                          radius_b: f32)
                          -> WorldManifold {
        unsafe {
            let mut w = mem::zeroed();
            ffi::WorldManifold_Initialize(&mut w, self, xf_a, radius_a, xf_b, radius_b);
            w
        }
    }
}

#[repr(C)]
#[derive(Clone, Debug)]
pub struct WorldManifold {
    pub normal: Vec2,
    pub points: [Vec2; MAX_MANIFOLD_POINTS],
    pub separations: [f32; MAX_MANIFOLD_POINTS],
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum PointState {
    Null,
    Add,
    Persist,
    Remove,
}

pub fn get_point_states(m1: &Manifold,
                        m2: &Manifold)
                        -> ([PointState; MAX_MANIFOLD_POINTS],
                            [PointState; MAX_MANIFOLD_POINTS]) {
    unsafe {
        let (mut s1, mut s2) = mem::zeroed();
        ffi::get_point_states(&mut s1, &mut s2, m1, m2);
        (s1, s2)
    }
}

#[repr(C)]
#[derive(Clone, Debug)]
pub struct RayCastInput {
    pub p1: Vec2,
    pub p2: Vec2,
    pub max_fraction: f32,
}

#[repr(C)]
#[derive(Clone, Debug)]
pub struct RayCastOutput {
    pub normal: Vec2,
    pub fraction: f32,
}

#[repr(C)]
#[derive(Clone, Debug)]
pub struct AABB {
    pub lower: Vec2,
    pub upper: Vec2,
}

impl AABB {
    pub fn new() -> AABB {
        AABB {
            lower: Vec2 { x: 0., y: 0. },
            upper: Vec2 { x: 0., y: 0. },
        }
    }
}

pub fn test_overlap<A, B>(shape_a: &A,
                          index_a: i32,
                          xf_a: &Transform,
                          shape_b: &B,
                          index_b: i32,
                          xf_b: &Transform)
                          -> bool
    where A: Shape,
          B: Shape
{
    unsafe {
        ffi::test_overlap(shape_a.base_ptr(),
                          index_a,
                          shape_b.base_ptr(),
                          index_b,
                          xf_a,
                          xf_b)
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use collision::shapes::ffi::Shape;
    use common::math::Transform;
    use common::settings::MAX_MANIFOLD_POINTS;
    use super::{Manifold, WorldManifold, PointState};

    extern "C" {
        pub fn WorldManifold_Initialize(slf: *mut WorldManifold,
                                        manifold: *const Manifold,
                                        xf_a: *const Transform,
                                        radius_a: f32,
                                        xf_b: *const Transform,
                                        radius_b: f32);
        pub fn get_point_states(s1: &mut [PointState; MAX_MANIFOLD_POINTS],
                                s2: &mut [PointState; MAX_MANIFOLD_POINTS],
                                m1: *const Manifold,
                                m2: *const Manifold);
        pub fn test_overlap(shape_a: *const Shape,
                            index_a: i32,
                            shape_b: *const Shape,
                            index_b: i32,
                            xf_a: *const Transform,
                            xf_b: *const Transform)
                            -> bool;
    }
}
