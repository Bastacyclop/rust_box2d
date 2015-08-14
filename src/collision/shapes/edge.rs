use wrap::*;
use common::math::Vec2;
use super::Shape;

wrap_shape! {
    ffi::EdgeShape => EdgeShape
    < ffi::EdgeShape_as_shape
    > ffi::Shape_as_edge_shape
}

impl EdgeShape {
    pub fn new() -> EdgeShape {
        unsafe {
            EdgeShape::from_ffi(ffi::EdgeShape_new())
        }
    }

    pub fn set(&mut self, v1: &Vec2, v2: &Vec2) {
        unsafe {
            ffi::EdgeShape_set(self.mut_ptr(), v1, v2)
        }
    }

    pub fn set_v0(&mut self, v0: Vec2) {
        unsafe {
            ffi::EdgeShape_set_v0(self.mut_ptr(), v0)
        }
    }

    pub fn set_v3(&mut self, v3: Vec2) {
        unsafe {
            ffi::EdgeShape_set_v3(self.mut_ptr(), v3)
        }
    }
}

impl Drop for EdgeShape {
    fn drop(&mut self) {
        unsafe {
            ffi::EdgeShape_drop(self.mut_ptr())
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use collision::shapes::ffi::Shape;
    use common::math::Vec2;

    pub enum EdgeShape {}

    extern {
        pub fn EdgeShape_new() -> *mut EdgeShape;
        pub fn EdgeShape_drop(slf: *mut EdgeShape);
        pub fn EdgeShape_as_shape(slf: *mut EdgeShape) -> *mut Shape;
        pub fn Shape_as_edge_shape(slf: *mut Shape) -> *mut EdgeShape;
        pub fn EdgeShape_set(slf: *mut EdgeShape, v1: *const Vec2, v2: *const Vec2);
        pub fn EdgeShape_set_v0(slf: *mut EdgeShape, v0: Vec2);
        pub fn EdgeShape_set_v3(slf: *mut EdgeShape, v3: Vec2);
    }
}
