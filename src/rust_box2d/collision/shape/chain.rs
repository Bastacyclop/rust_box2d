use ffi;
use math::Vec2;
use collision::Shape;
use collision::shape;
use WrapStruct;
use Wrapper;

pub type ChainStruct = WrapStruct<ffi::ChainShape>;

pub trait Chain: Wrapper<ffi::ChainShape> {
    //pub unsafe fn from_ptr(ptr: *mut ffi::ChainShape) -> Self;
    //pub unsafe fn get_ptr(&self) -> *ffi::ChainShape;
    //pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::ChainShape;
    
    fn new() -> ChainStruct {
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
    fn get_child_edge(&self, index: i32) -> shape::Edge {
        unsafe {
            let edge = ffi::EdgeShape_new();
            ffi::ChainShape_get_child_edge(self.get_ptr(), edge, index);
            shape::Edge::from_ptr(edge)
        }
    }
}

impl Chain for ChainStruct {}

impl Shape for ChainStruct {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> ChainStruct {
        Wrapper::from_ptr(ffi::Shape_as_chain_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::ChainShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::ChainShape_as_shape(self.ptr)
    }
}

#[unsafe_destructor]
impl Drop for ChainStruct {
    fn drop(&mut self) {
        unsafe {
            ffi::ChainShape_drop(self.get_mut_ptr())
        }
    }
}
