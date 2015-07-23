use wrap::*;
use common::math::Vec2;
use super::Shape;

wrap_shape! {
    ffi::PolygonShape => PolygonShape
    < ffi::PolygonShape_as_shape
    > ffi::Shape_as_polygon_shape
}

impl PolygonShape {
    pub fn new() -> PolygonShape {
        unsafe {
            PolygonShape::from_ffi(ffi::PolygonShape_new())
        }
    }

    pub fn set(&mut self, points: &[Vec2]) {
        unsafe {
            ffi::PolygonShape_set(self.mut_ptr(),
                                  points.as_ptr(),
                                  points.len() as i32)
        }
    }

    pub fn set_as_box(&mut self, hw: f32, hh: f32) {
        unsafe {
            ffi::PolygonShape_set_as_box(self.mut_ptr(), hw, hh)
        }
    }

    pub fn set_as_oriented_box(&mut self, hw: f32, hh: f32,
                               center: &Vec2, angle: f32) {
        unsafe {
            ffi::PolygonShape_set_as_oriented_box(self.mut_ptr(),
                                                  hw, hh,
                                                  center, angle)
        }
    }

    pub fn vertex_count(&self) -> i32 {
        unsafe {
            ffi::PolygonShape_get_vertex_count(self.ptr())
        }
    }

    pub fn vertex<'a>(&'a self, index: i32) -> &'a Vec2 {
        unsafe {
            &*ffi::PolygonShape_get_vertex(self.ptr(), index) // Comes from a C++ &
        }
    }

    pub fn validate(&self) -> bool {
        unsafe {
            ffi::PolygonShape_validate(self.ptr())
        }
    }
}

impl Drop for PolygonShape {
    fn drop(&mut self) {
        unsafe {
            ffi::PolygonShape_drop(self.mut_ptr())
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use collision::shapes::ffi::Shape;
    use common::math::Vec2;

    #[repr(C)] pub struct PolygonShape;

    extern {
        pub fn PolygonShape_new() -> *mut PolygonShape;
        pub fn PolygonShape_drop(slf: *mut PolygonShape);
        pub fn PolygonShape_as_shape(slf: *mut PolygonShape) -> *mut Shape;
        pub fn Shape_as_polygon_shape(slf: *mut Shape) -> *mut PolygonShape;
        pub fn PolygonShape_set(slf: *mut PolygonShape, points: *const Vec2, count: i32);
        pub fn PolygonShape_set_as_box(slf: *mut PolygonShape, hw: f32, hh: f32);
        pub fn PolygonShape_set_as_oriented_box(slf: *mut PolygonShape,
                                                hw: f32, hh: f32,
                                                center: *const Vec2,
                                                angle: f32);
        pub fn PolygonShape_get_vertex_count(slf: *const PolygonShape) -> i32;
        pub fn PolygonShape_get_vertex(slf: *const PolygonShape, index: i32) -> *const Vec2;
        pub fn PolygonShape_validate(slf: *const PolygonShape) -> bool;
    }
}
