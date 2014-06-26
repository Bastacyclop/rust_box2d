use ffi;
use math::Vec2;
use collision::Shape;

pub struct Circle {
    ptr: *mut ffi::CircleShape
}

impl Circle {
    pub unsafe fn from_ptr(ptr: *mut ffi::CircleShape) -> Circle {
        assert!(!ptr.is_null());
        Circle {
            ptr: ptr
        }
    }
    pub unsafe fn get_ptr(&self) -> *ffi::CircleShape {
        self.ptr as *ffi::CircleShape
    }
    pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::CircleShape {
        self.ptr
    }
    
    pub fn new() -> Circle {
        unsafe {
            Circle::from_ptr(ffi::CircleShape_new())
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

impl Shape for Circle {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Circle {
        Circle::from_ptr(ffi::Shape_as_circle_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::CircleShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::CircleShape_as_shape(self.ptr)
    }
}

impl Drop for Circle {
    fn drop(&mut self) {
        unsafe {
            ffi::CircleShape_drop(self.get_mut_ptr())
        }
    }
}
