use ffi;
use math::Vec2;
use collision::Shape;
use collision::shape;

pub struct Chain {
    ptr: *mut ffi::ChainShape
}

impl Chain {
    pub unsafe fn from_ptr(ptr: *mut ffi::ChainShape) -> Chain {
        assert!(!ptr.is_null());
        Chain {
            ptr: ptr
        }
    }
    pub unsafe fn get_ptr(&self) -> *ffi::ChainShape {
        self.ptr as *ffi::ChainShape
    }
    pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::ChainShape {
        self.ptr
    }
    
    pub fn new() -> Chain {
        unsafe {
            Chain::from_ptr(ffi::ChainShape_new())
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
    #[unstable]
    pub fn get_child_edge(&self, index: i32) -> shape::Edge {
        unsafe {
            let edge = ffi::EdgeShape_new();
            ffi::ChainShape_get_child_edge(self.get_ptr(), edge, index);
            shape::Edge::from_ptr(edge)
        }
    }
}

impl Shape for Chain {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Chain {
        Chain::from_ptr(ffi::Shape_as_chain_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::ChainShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::ChainShape_as_shape(self.ptr)
    }
}

impl Drop for Chain {
    fn drop(&mut self) {
        unsafe {
            ffi::ChainShape_drop(self.get_mut_ptr())
        }
    }
}
