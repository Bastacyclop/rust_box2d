use std::mem;
use std::ptr;
use std::slice;

use wrap::*;
use common::math::Vec2;
use super::{Shape, EdgeShape};

wrap_shape! {
    ffi::ChainShape => ChainShape
    < ffi::ChainShape_as_shape
    > ffi::Shape_as_chain_shape
}

impl ChainShape {
    pub fn new() -> Self {
        unsafe { ChainShape::from_ffi(ffi::ChainShape_new()) }
    }

    pub fn new_loop(vertices: &[Vec2]) -> Self {
        let mut s = Self::new();
        s.create_loop(vertices);
        s
    }

    pub fn new_chain(vertices: &[Vec2]) -> Self {
        let mut s = Self::new();
        s.create_chain(vertices);
        s
    }

    pub fn clear(&mut self) {
        unsafe { ffi::ChainShape_clear(self.mut_ptr()) }
    }

    pub fn create_loop(&mut self, vertices: &[Vec2]) {
        unsafe {
            ffi::ChainShape_create_loop(self.mut_ptr(), vertices.as_ptr(), vertices.len() as i32)
        }
    }

    pub fn create_chain(&mut self, vertices: &[Vec2]) {
        unsafe {
            ffi::ChainShape_create_chain(self.mut_ptr(), vertices.as_ptr(), vertices.len() as i32)
        }
    }

    pub fn vertices(&self) -> &[Vec2] {
        unsafe {
            let vertices = ffi::ChainShape_get_vertices_const(self.ptr());
            let count = ffi::ChainShape_get_vertex_count(self.ptr());
            slice::from_raw_parts(vertices, count as usize)
        }
    }

    pub fn prev_vertex(&self) -> Option<Vec2> {
        unsafe {
            let mut v = mem::uninitialized();
            if ffi::ChainShape_get_prev_vertex(self.ptr(), &mut v) {
                Some(v)
            } else {
                None
            }
        }
    }

    pub fn set_prev_vertex(&mut self, v: Option<Vec2>) {
        let ptr = v.as_ref().map(|v0| v0 as *const _).unwrap_or(ptr::null());
        unsafe { ffi::ChainShape_set_prev_vertex(self.mut_ptr(), ptr) }
    }

    pub fn next_vertex(&self) -> Option<Vec2> {
        unsafe {
            let mut v = mem::uninitialized();
            if ffi::ChainShape_get_next_vertex(self.ptr(), &mut v) {
                Some(v)
            } else {
                None
            }
        }
    }

    pub fn set_next_vertex(&mut self, v: Option<Vec2>) {
        let ptr = v.as_ref().map(|v0| v0 as *const _).unwrap_or(ptr::null());
        unsafe { ffi::ChainShape_set_next_vertex(self.mut_ptr(), ptr) }
    }

    pub fn child_edge(&self, index: i32) -> EdgeShape {
        unsafe {
            let mut edge = EdgeShape::new();
            ffi::ChainShape_get_child_edge(self.ptr(), edge.mut_ptr(), index);
            edge
        }
    }
}

impl Drop for ChainShape {
    fn drop(&mut self) {
        unsafe { ffi::ChainShape_drop(self.mut_ptr()) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use collision::shapes::ffi::Shape;
    pub use collision::shapes::edge::ffi::EdgeShape;
    use common::math::Vec2;

    pub enum ChainShape {}

    extern "C" {
        pub fn ChainShape_new() -> *mut ChainShape;
        pub fn ChainShape_drop(slf: *mut ChainShape);
        pub fn ChainShape_as_shape(slf: *mut ChainShape) -> *mut Shape;
        pub fn Shape_as_chain_shape(slf: *mut Shape) -> *mut ChainShape;
        pub fn ChainShape_clear(slf: *mut ChainShape);
        pub fn ChainShape_create_loop(slf: *mut ChainShape, vertices: *const Vec2, count: i32);
        pub fn ChainShape_create_chain(slf: *mut ChainShape, vertices: *const Vec2, count: i32);
        pub fn ChainShape_get_vertices_const(slf: *const ChainShape) -> *const Vec2;
        pub fn ChainShape_get_vertex_count(slf: *const ChainShape) -> i32;
        pub fn ChainShape_get_prev_vertex(slf: *const ChainShape, prev: &mut Vec2) -> bool;
        pub fn ChainShape_set_prev_vertex(slf: *mut ChainShape, vertex: *const Vec2);
        pub fn ChainShape_get_next_vertex(slf: *const ChainShape, next: &mut Vec2) -> bool;
        pub fn ChainShape_set_next_vertex(slf: *mut ChainShape, vertex: *const Vec2);
        pub fn ChainShape_get_child_edge(slf: *const ChainShape,
                                         edge: *mut EdgeShape,
                                         index: i32);
    }
}
