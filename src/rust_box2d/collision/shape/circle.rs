use ffi;
use math::Vec2;
use super::{WrappedShape, Shape};
use Wrapped;

wrap!(ffi::CircleShape into CircleShape)

impl CircleShape {
    pub fn new() -> CircleShape {
        unsafe {
            Wrapped::from_ptr(ffi::CircleShape_new())
        }
    }
    pub fn get_support(&self, dir: &Vec2) -> uint {
        unsafe {
            ffi::CircleShape_get_support(self.get_ptr(), dir) as uint
        }
    }
    pub fn get_support_vertex(&self, dir: &Vec2) -> &Vec2 {
        unsafe {
            let support =
                ffi::CircleShape_get_support_vertex(self.get_ptr(), dir);
            assert!(!support.is_null());
            &*support
        }
    }
    pub fn get_vertex_count(&self) -> uint {
        unsafe {
            ffi::CircleShape_get_vertex_count(self.get_ptr()) as uint
        }
    }
    pub fn get_vertex(&self, index: uint) -> &Vec2 {
        unsafe {
            let vertex =
                ffi::CircleShape_get_vertex(self.get_ptr(), index as i32);
            assert!(!vertex.is_null());
            &*vertex
        }
    }
}

impl_shape!(for CircleShape
    << ffi::Shape_as_circle_shape
    >> ffi::CircleShape_as_shape
    )

impl Drop for CircleShape {
    fn drop(&mut self) {
        unsafe {
            ffi::CircleShape_drop(self.get_mut_ptr())
        }
    }
}
