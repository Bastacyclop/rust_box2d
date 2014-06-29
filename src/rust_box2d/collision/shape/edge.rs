use ffi;
use math::Vec2;
use super::{WrappedShape, Shape};
use Wrapped;

wrap!(ffi::EdgeShape into EdgeShape)

impl EdgeShape {
    pub fn new() -> EdgeShape {
        unsafe {
            Wrapped::from_ptr(ffi::EdgeShape_new())
        }
    }
    pub fn set(&mut self, v1: &Vec2, v2: &Vec2) {
        unsafe {
            ffi::EdgeShape_set(self.get_mut_ptr(), v1, v2)
        }
    }
}

impl_shape!(for EdgeShape
    << ffi::Shape_as_edge_shape
    >> ffi::EdgeShape_as_shape
    )

impl Drop for EdgeShape {
    fn drop(&mut self) {
        unsafe {
            ffi::EdgeShape_drop(self.get_mut_ptr())
        }
    }
}
