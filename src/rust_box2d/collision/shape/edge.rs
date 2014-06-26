use ffi;
use math::Vec2;
use collision::Shape;

pub struct Edge {
    ptr: *mut ffi::EdgeShape
}

impl Edge {
    pub unsafe fn from_ptr(ptr: *mut ffi::EdgeShape) -> Edge {
        assert!(!ptr.is_null());
        Edge {
            ptr: ptr
        }
    }
    pub unsafe fn get_ptr(&self) -> *ffi::EdgeShape {
        self.ptr as *ffi::EdgeShape
    }
    pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::EdgeShape {
        self.ptr
    }
    
    pub fn new() -> Edge {
        unsafe {
            Edge::from_ptr(ffi::EdgeShape_new())
        }
    }
    pub fn set(&mut self, v1: &Vec2, v2: &Vec2) {
        unsafe {
            ffi::EdgeShape_set(self.get_mut_ptr(), v1, v2)
        }
    }
}

impl Shape for Edge {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Edge {
        Edge::from_ptr(ffi::Shape_as_edge_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::EdgeShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::EdgeShape_as_shape(self.ptr)
    }
}

impl Drop for Edge {
    fn drop(&mut self) {
        unsafe {
            ffi::EdgeShape_drop(self.get_mut_ptr())
        }
    }
}
