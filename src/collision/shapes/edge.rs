use std::mem;
use std::ptr;

use wrap::*;
use common::math::Vec2;
use super::Shape;

wrap_shape! {
    ffi::EdgeShape => EdgeShape
    < ffi::EdgeShape_as_shape
    > ffi::Shape_as_edge_shape
}

impl EdgeShape {
    pub fn new() -> Self {
        unsafe { EdgeShape::from_ffi(ffi::EdgeShape_new()) }
    }

    pub fn new_with(v1: &Vec2, v2: &Vec2) -> Self {
        let mut s = Self::new();
        s.set(v1, v2);
        s
    }

    pub fn set(&mut self, v1: &Vec2, v2: &Vec2) {
        unsafe { ffi::EdgeShape_set(self.mut_ptr(), v1, v2) }
    }

    pub fn v1(&self) -> Vec2 {
        unsafe { ffi::EdgeShape_get_v1(self.ptr()) }
    }

    pub fn set_v1(&mut self, v1: Vec2) {
        unsafe { ffi::EdgeShape_set_v1(self.mut_ptr(), v1) }
    }

    pub fn v2(&self) -> Vec2 {
        unsafe { ffi::EdgeShape_get_v2(self.ptr()) }
    }

    pub fn set_v2(&mut self, v2: Vec2) {
        unsafe { ffi::EdgeShape_set_v2(self.mut_ptr(), v2) }
    }

    pub fn v0(&self) -> Option<Vec2> {
        unsafe {
            let mut v0 = mem::uninitialized();
            if ffi::EdgeShape_get_v0(self.ptr(), &mut v0) {
                Some(v0)
            } else {
                None
            }
        }
    }

    pub fn set_v0(&mut self, v0: Option<Vec2>) {
        let ptr = v0.as_ref().map(|v0| v0 as *const _).unwrap_or(ptr::null());
        unsafe { ffi::EdgeShape_set_v0(self.mut_ptr(), ptr) }
    }

    pub fn v3(&self) -> Option<Vec2> {
        unsafe {
            let mut v3 = mem::uninitialized();
            if ffi::EdgeShape_get_v3(self.ptr(), &mut v3) {
                Some(v3)
            } else {
                None
            }
        }
    }

    pub fn set_v3(&mut self, v3: Option<Vec2>) {
        let ptr = v3.as_ref().map(|v3| v3 as *const _).unwrap_or(ptr::null());
        unsafe { ffi::EdgeShape_set_v0(self.mut_ptr(), ptr) }
    }
}

impl Drop for EdgeShape {
    fn drop(&mut self) {
        unsafe { ffi::EdgeShape_drop(self.mut_ptr()) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use collision::shapes::ffi::Shape;
    use common::math::Vec2;

    pub enum EdgeShape {}

    extern "C" {
        pub fn EdgeShape_new() -> *mut EdgeShape;
        pub fn EdgeShape_drop(slf: *mut EdgeShape);
        pub fn EdgeShape_as_shape(slf: *mut EdgeShape) -> *mut Shape;
        pub fn Shape_as_edge_shape(slf: *mut Shape) -> *mut EdgeShape;
        pub fn EdgeShape_set(slf: *mut EdgeShape, v1: *const Vec2, v2: *const Vec2);
        pub fn EdgeShape_get_v1(slf: *const EdgeShape) -> Vec2;
        pub fn EdgeShape_set_v1(slf: *mut EdgeShape, v1: Vec2);
        pub fn EdgeShape_get_v2(slf: *const EdgeShape) -> Vec2;
        pub fn EdgeShape_set_v2(slf: *mut EdgeShape, v2: Vec2);
        pub fn EdgeShape_get_v0(slf: *const EdgeShape, v0: &mut Vec2) -> bool;
        pub fn EdgeShape_set_v0(slf: *mut EdgeShape, v0: *const Vec2);
        pub fn EdgeShape_get_v3(slf: *const EdgeShape, v3: &mut Vec2) -> bool;
        pub fn EdgeShape_set_v3(slf: *mut EdgeShape, v3: *const Vec2);
    }
}
