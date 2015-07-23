use std::mem;
use wrap::*;
use common::math::Vec2;
use super::{ Shape, EdgeShape };

wrap_shape! {
    ffi::ChainShape => ChainShape
    < ffi::ChainShape_as_shape
    > ffi::Shape_as_chain_shape
}

impl ChainShape {
    pub fn new() -> ChainShape {
        unsafe {
            ChainShape::from_ffi(ffi::ChainShape_new())
        }
    }

    pub fn clear(&mut self) {
        unsafe {
            ffi::ChainShape_clear(self.mut_ptr())
        }
    }

    pub fn create_loop(&mut self, vertices: &[Vec2]) {
        unsafe {
            ffi::ChainShape_create_loop(self.mut_ptr(),
                                        vertices.as_ptr(),
                                        vertices.len() as i32)
        }
    }

    pub fn create_chain(&mut self, vertices: &[Vec2]) {
        unsafe {
            ffi::ChainShape_create_chain(self.mut_ptr(),
                                         vertices.as_ptr(),
                                         vertices.len() as i32)
        }
    }

    pub fn set_prev_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_prev_vertex(self.mut_ptr(), vertex)
        }
    }

    pub fn set_next_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_next_vertex(self.mut_ptr(), vertex)
        }
    }

    pub fn child_edge(&self, index: i32) -> EdgeShape {
        unsafe {
            let edge = mem::zeroed();
            ffi::ChainShape_get_child_edge(self.ptr(), edge, index);
            EdgeShape::from_ffi(edge)
        }
    }
}

impl Drop for ChainShape {
    fn drop(&mut self) {
        unsafe {
            ffi::ChainShape_drop(self.mut_ptr())
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use collision::shapes::ffi::Shape;
    pub use collision::shapes::edge::ffi::EdgeShape;
    use common::math::Vec2;

    #[repr(C)] pub struct ChainShape;

    extern {
        pub fn ChainShape_new() -> *mut ChainShape;
        pub fn ChainShape_drop(slf: *mut ChainShape);
        pub fn ChainShape_as_shape(slf: *mut ChainShape) -> *mut Shape;
        pub fn Shape_as_chain_shape(slf: *mut Shape) -> *mut ChainShape;
        pub fn ChainShape_clear(slf: *mut ChainShape);
        pub fn ChainShape_create_loop(slf: *mut ChainShape,
                                      vertices: *const Vec2,
                                      count: i32);
        pub fn ChainShape_create_chain(slf: *mut ChainShape,
                                       vertices: *const Vec2,
                                       count: i32);
        pub fn ChainShape_set_prev_vertex(slf: *mut ChainShape, vertex: *const Vec2);
        pub fn ChainShape_set_next_vertex(slf: *mut ChainShape, vertex: *const Vec2);
        pub fn ChainShape_get_child_edge(slf: *const ChainShape,
                                         edge: *mut EdgeShape,
                                         index: i32);
    }
}
