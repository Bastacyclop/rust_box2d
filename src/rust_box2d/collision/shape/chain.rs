use ffi;
use math::Vec2;
use super::{EdgeShape, WrappedShape, Shape};
use Wrapped;

wrap!(ffi::ChainShape into ChainShape)

impl ChainShape {
    pub fn new() -> ChainShape {
        unsafe {
            Wrapped::from_ptr(ffi::ChainShape_new())
        }
    }
    pub fn clear(&mut self) {
        unsafe {
            ffi::ChainShape_clear(self.get_mut_ptr())
        }
    }
    pub fn create_loop(&mut self, vertices: Vec<Vec2>) {
        unsafe {
            ffi::ChainShape_create_loop(self.get_mut_ptr(),
                                        vertices.as_ptr(),
                                        vertices.len() as i32                                   
                                        )
        }
    }
    pub fn create_chain(&mut self, vertices: Vec<Vec2>) {
        unsafe {
            ffi::ChainShape_create_chain(self.get_mut_ptr(),
                                         vertices.as_ptr(),
                                         vertices.len() as i32
                                         )
        }
    }
    pub fn set_prev_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_prev_vertex(self.get_mut_ptr(), vertex)
        }
    }
    pub fn set_next_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_next_vertex(self.get_mut_ptr(), vertex)
        }
    }
    pub fn get_child_edge(&self, index: i32) -> EdgeShape {
        unsafe {
            let edge = ffi::EdgeShape_new();
            ffi::ChainShape_get_child_edge(self.get_ptr(), edge, index);
            Wrapped::from_ptr(edge)
        }
    }
}

impl_shape!(for ChainShape
    << ffi::Shape_as_chain_shape
    >> ffi::ChainShape_as_shape
    )

impl Drop for ChainShape {
    fn drop(&mut self) {
        unsafe {
            ffi::ChainShape_drop(self.get_mut_ptr())
        }
    }
}
