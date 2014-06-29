use ffi;
use math::Vec2;
use super::{WrappedShape, Shape};
use Wrapped;

wrap!(ffi::PolygonShape into PolygonShape)

impl PolygonShape {
    pub fn new() -> PolygonShape {
        unsafe {
            Wrapped::from_ptr(ffi::PolygonShape_new())
        }
    }
    pub fn set(&mut self, points: Vec<Vec2>) {
        unsafe {
            ffi::PolygonShape_set(self.get_mut_ptr(),
                                  points.as_ptr(),
                                  points.len() as i32)
        }
    }
    pub fn set_as_box(&mut self, hw: f32, hh: f32) {
        unsafe {
            ffi::PolygonShape_set_as_box(self.get_mut_ptr(), hw, hh)
        }
    }
    pub fn set_as_oriented_box(&mut self, hw: f32, hh: f32,
                               center: &Vec2, angle: f32) {
        unsafe {
            ffi::PolygonShape_set_as_oriented_box(self.get_mut_ptr(),
                                                  hw, hh,
                                                  center, angle)
        }
    }
    pub fn get_vertex_count(&self) -> uint {
        unsafe {
            ffi::PolygonShape_get_vertex_count(self.get_ptr()) as uint
        }
    }
    pub fn get_vertex(&self, index: uint) -> &Vec2 {
        unsafe {
            let vertex =
                ffi::PolygonShape_get_vertex(self.get_ptr(), index as i32);
            assert!(!vertex.is_null());
            &*vertex
        }
    }
    pub fn validate(&self) -> bool {
        unsafe {
            ffi::PolygonShape_validate(self.get_ptr())
        }
    }
}

impl_shape!(for PolygonShape
    << ffi::Shape_as_polygon_shape
    >> ffi::PolygonShape_as_shape
    )

impl Drop for PolygonShape {
    fn drop(&mut self) {
        unsafe {
            ffi::PolygonShape_drop(self.get_mut_ptr())
        }
    }
}
