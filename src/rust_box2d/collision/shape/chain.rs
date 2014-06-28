use ffi;
use math::Vec2;
use super::{EdgeShape, ShapeWrapper, Shape};
use WrapStruct;
use Wrapper;

pub type ChainShape = WrapStruct<ffi::ChainShape>;

pub trait ChainShapeT: Wrapper<ffi::ChainShape> {
    fn new() -> Self {
        unsafe {
            Wrapper::from_ptr(ffi::ChainShape_new())
        }
    }
    fn clear(&mut self) {
        unsafe {
            ffi::ChainShape_clear(self.get_mut_ptr())
        }
    }
    fn create_loop(&mut self, vertices: Vec<Vec2>) {
        unsafe {
            ffi::ChainShape_create_loop(self.get_mut_ptr(),
                                        vertices.as_ptr(),
                                        vertices.len() as i32                                   
                                        )
        }
    }
    fn create_chain(&mut self, vertices: Vec<Vec2>) {
        unsafe {
            ffi::ChainShape_create_chain(self.get_mut_ptr(),
                                         vertices.as_ptr(),
                                         vertices.len() as i32
                                         )
        }
    }
    fn set_prev_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_prev_vertex(self.get_mut_ptr(), vertex)
        }
    }
    fn set_next_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_next_vertex(self.get_mut_ptr(), vertex)
        }
    }
    #[unstable]
    fn get_child_edge(&self, index: i32) -> EdgeShape {
        unsafe {
            let edge = ffi::EdgeShape_new();
            ffi::ChainShape_get_child_edge(self.get_ptr(), edge, index);
            Wrapper::from_ptr(edge)
        }
    }
}

impl ChainShapeT for ChainShape {}

impl ShapeWrapper for ChainShape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> ChainShape {
        Wrapper::from_ptr(ffi::Shape_as_chain_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::ChainShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::ChainShape_as_shape(self.ptr)
    }
}

impl Shape for ChainShape {}

#[unsafe_destructor]
impl Drop for ChainShape {
    fn drop(&mut self) {
        unsafe {
            ffi::ChainShape_drop(self.get_mut_ptr())
        }
    }
}
