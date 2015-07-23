use std::mem;
use std::marker::PhantomData;
use common::math::{ Vec2, Transform };
use collision::shapes::Shape;

#[repr(C)]
#[doc(hidden)]
pub struct RawProxy {
    buffer: [Vec2; 2],
    vertices: *const Vec2,
    count: i32,
    radius: f32
}

impl RawProxy {
    unsafe fn new(shape: *const ffi::Shape, index: i32) -> RawProxy {
        let mut proxy = mem::zeroed();
        ffi::DistanceProxy_set(&mut proxy, shape, index);
        proxy
    }
}

pub struct Proxy<'a> {
    #[doc(hidden)] pub raw: RawProxy,
    #[doc(hidden)] pub phantom: PhantomData<&'a ()>
}

impl<'a> Proxy<'a> {
    pub fn new<S: Shape>(shape: &'a S, index: i32) -> Proxy<'a> {
        Proxy {
            raw: unsafe { RawProxy::new(shape.base_ptr(), index) },
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

#[repr(C)] #[doc(hidden)]
pub struct RawInput {
    proxy_a: RawProxy,
    proxy_b: RawProxy,
    transform_a: Transform,
    transform_b: Transform,
    use_radii: bool
}

pub struct Input<'a> {
    raw: RawInput,
    phantom: PhantomData<&'a ()>
}

impl<'a> Input<'a> {
    pub fn new(proxy_a: Proxy<'a>,
               proxy_b: Proxy<'a>,
               transform_a: Transform,
               transform_b: Transform,
               use_radii: bool) -> Input<'a> {
        Input {
            raw: RawInput {
                proxy_a: proxy_a.raw,
                proxy_b: proxy_b.raw,
                transform_a: transform_a,
                transform_b: transform_b,
                use_radii: use_radii
            },
            phantom: PhantomData
        }
    }

    pub fn query(&self, cache: &mut SimplexCache) -> Output {
        unsafe {
            let mut out = mem::zeroed();
            ffi::distance(&mut out, cache, &self.raw);
            out
        }
    }
}

#[repr(C)]
pub struct Output {
    pub point_a: Vec2,
    pub point_b: Vec2,
    pub distance: f32,
    pub iterations: i32
}

#[doc(hidden)]
pub mod ffi {
    pub use collision::shapes::ffi::Shape;
    use super::{ RawProxy, RawInput, SimplexCache, Output };

    extern {
        pub fn DistanceProxy_set(slf: *mut RawProxy,
                                 shape: *const Shape,
                                 index: i32);
        pub fn distance(output: *mut Output,
                        cache: *mut SimplexCache,
                        input: *const RawInput);
    }
}
