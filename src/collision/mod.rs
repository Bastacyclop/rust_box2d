pub mod shapes;
pub use self::shapes::{
    Shape, ShapeType, UnknownShape,
    PolygonShape, EdgeShape, CircleShape, ChainShape,
    MassData
};

use std::mem;
use std::ptr;
use std::marker::PhantomData;
use ffi;
use settings;
use wrap::*;
use math::{ Vec2, Transform, Sweep };

#[repr(u8)]
pub enum ContactFeatureType {
    Vertex,
    Face
}

#[repr(C)]
pub struct ContactFeature {
    pub index_a: u8,
    pub index_b: u8,
    pub type_a: ContactFeatureType,
    pub type_b: ContactFeatureType
}

#[repr(C)]
pub struct ContactId(u32);

impl ContactId {
    pub fn feature(&self) -> ContactFeature {
        unsafe {
            mem::transmute(self.0)
        }
    }

    pub fn key(&self) -> u32 { self.0 }
}

#[repr(C)]
pub struct ManifoldPoint {
    pub local_point: Vec2,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub id: ContactId
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ManifoldType {
    Circles,
    FaceA,
    FaceB
}

#[repr(C)]
pub struct Manifold {
    pub points: [ManifoldPoint; settings::MAX_MANIFOLD_POINTS],
    pub local_normal: Vec2,
    pub local_point: Vec2,
    pub manifold_type: ManifoldType,
    pub count: i32
}

impl Manifold {
    pub fn world_manifold(&self, xf_a: &Transform, radius_a: f32,
                                 xf_b: &Transform, radius_b: f32) -> WorldManifold {
        let mut w = WorldManifold {
            normal: Vec2 { x: 0., y: 0. },
            points: [Vec2 { x: 0., y: 0. }; settings::MAX_MANIFOLD_POINTS],
            separations: [0.; settings::MAX_MANIFOLD_POINTS]
        };
        unsafe {
            ffi::WorldManifold_Initialize(&mut w, self,
                                          xf_a, radius_a,
                                          xf_b, radius_b);
        }
        w
    }
}

#[repr(C)]
pub struct WorldManifold {
    pub normal: Vec2,
    pub points: [Vec2; settings::MAX_MANIFOLD_POINTS],
    pub separations: [f32; settings::MAX_MANIFOLD_POINTS]
}

#[repr(C)]
#[derive(Clone, Copy)]
pub enum PointState {
    Null,
    Add,
    Persist,
    Remove
}

pub fn get_point_states(m1: &Manifold, m2: &Manifold
                        ) -> ([PointState; settings::MAX_MANIFOLD_POINTS],
                              [PointState; settings::MAX_MANIFOLD_POINTS]) {
    let mut s1 = [PointState::Null; settings::MAX_MANIFOLD_POINTS];
    let mut s2 = s1;
    unsafe {
        ffi::get_point_states(&mut s1, &mut s2, m1, m2);
    }
    (s1, s2)
}

#[repr(C)]
#[derive(Clone)]
pub struct RayCastInput {
    pub p1: Vec2,
    pub p2: Vec2,
    pub max_fraction: f32
}

#[repr(C)]
#[derive(Clone)]
pub struct RayCastOutput {
    pub normal: Vec2,
    pub fraction: f32
}

impl RayCastOutput {
    pub fn new() -> RayCastOutput {
        RayCastOutput {
            normal: Vec2 { x: 0., y: 0. },
            fraction: 0.
        }
    }
}

#[repr(C)]
#[derive(Clone)]
pub struct AABB {
    pub lower_bound: Vec2,
    pub upper_bound: Vec2
}

impl AABB {
    pub fn new() -> AABB {
        AABB {
            lower_bound: Vec2 { x: 0., y: 0. },
            upper_bound: Vec2 { x: 0., y: 0. }
        }
    }
}

pub fn test_overlap<A, B>(shape_a: &A, index_a: i32, xf_a: &Transform,
                          shape_b: &B, index_b: i32, xf_b: &Transform) -> bool
    where A: Shape, B: Shape {
    unsafe {
        ffi::test_overlap(shape_a.base_ptr(), index_a,
                          shape_b.base_ptr(), index_b,
                          xf_a, xf_b)
    }
}

#[repr(C)]
#[allow(raw_pointer_derive)]
#[derive(Clone)]
#[doc(hidden)]
pub struct RawDistanceProxy {
    buffer: [Vec2; 2],
    vertices: *const Vec2,
    count: i32,
    radius: f32
}

impl RawDistanceProxy {
    unsafe fn new(shape: *const ffi::Shape, index: i32) -> RawDistanceProxy {
        let mut proxy = RawDistanceProxy {
            buffer: [Vec2 { x: 0., y: 0.}; 2],
            vertices: ptr::null_mut(),
            count: 0,
            radius: 0.
        };
        ffi::DistanceProxy_set(&mut proxy, shape, index);
        proxy
    }
}

#[repr(C)]
pub struct DistanceProxy<'a> {
    raw: RawDistanceProxy,
    phantom: PhantomData<&'a ()>
}

impl<'a> DistanceProxy<'a> {
    pub fn new<S: Shape>(shape: &'a S, index: i32) -> DistanceProxy<'a> {
        DistanceProxy {
            raw: unsafe { RawDistanceProxy::new(shape.base_ptr(), index) },
            phantom: PhantomData
        }
    }
}

#[repr(C)]
pub struct SimplexCache {
    pub metric: f32,
    pub count: u16,
    pub index_a: [u8; 3],
    pub index_b: [u8; 3]
}

#[repr(C)]
#[doc(hidden)]
pub struct RawDistanceInput {
    proxy_a: RawDistanceProxy,
    proxy_b: RawDistanceProxy,
    transform_a: Transform,
    transform_b: Transform,
    use_radii: bool
}

pub struct DistanceInput<'a> {
    raw: RawDistanceInput,
    phantom: PhantomData<&'a ()>
}

impl<'a> DistanceInput<'a> {
    pub fn new(proxy_a: DistanceProxy<'a>,
               proxy_b: DistanceProxy<'a>,
               transform_a: Transform,
               transform_b: Transform,
               use_radii: bool) -> DistanceInput<'a> {
        DistanceInput {
            raw: RawDistanceInput {
                proxy_a: proxy_a.raw,
                proxy_b: proxy_b.raw,
                transform_a: transform_a,
                transform_b: transform_b,
                use_radii: use_radii
            },
            phantom: PhantomData
        }
    }
}

#[repr(C)]
pub struct DistanceOutput {
    pub point_a: Vec2,
    pub point_b: Vec2,
    pub distance: f32,
    pub iterations: i32
}

pub fn distance(cache: &mut SimplexCache, input: &DistanceInput) -> DistanceOutput {
    let mut out = DistanceOutput {
        point_a: Vec2 { x: 0., y: 0. },
        point_b: Vec2 { x: 0., y: 0. },
        distance: 0.,
        iterations: 0
    };
    unsafe {
        ffi::distance(&mut out, cache, &input.raw);
    }
    out
}

#[repr(C)]
#[doc(hidden)]
pub struct RawTOIInput {
    proxy_a: RawDistanceProxy,
    proxy_b: RawDistanceProxy,
    sweep_a: Sweep,
    sweep_b: Sweep,
    t_max: f32
}

pub struct TOIInput<'a> {
    raw: RawTOIInput,
    phantom: PhantomData<&'a ()>
}

impl<'a> TOIInput<'a> {
    pub fn new(proxy_a: DistanceProxy<'a>,
               proxy_b: DistanceProxy<'a>,
               sweep_a: Sweep,
               sweep_b: Sweep,
               t_max: f32) -> TOIInput<'a> {
        TOIInput {
            raw: RawTOIInput {
                proxy_a: proxy_a.raw,
                proxy_b: proxy_b.raw,
                sweep_a: sweep_a,
                sweep_b: sweep_b,
                t_max: t_max
            },
            phantom: PhantomData
        }
    }
}

#[repr(C)]
pub enum TOIState {
    Unknown,
    Failed,
    Overlapped,
    Touching,
    Separated
}

#[repr(C)]
pub struct TOIOutput {
    state: TOIState,
    t: f32
}

pub fn time_of_impact(input: &TOIInput) -> TOIOutput {
    let mut out = TOIOutput {
        state: TOIState::Unknown,
        t: 0.
    };
    unsafe {
        ffi::time_of_impact(&mut out, &input.raw);
    }
    out
}
