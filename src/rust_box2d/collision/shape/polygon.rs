use ffi;
use math::Vec2;
use collision::Shape;
use collision::shape;

pub struct Polygon {
    ptr: *mut ffi::PolygonShape
}

impl Polygon {
    pub unsafe fn from_ptr(ptr: *mut ffi::PolygonShape) -> Polygon {
        assert!(!ptr.is_null());
        Polygon {
            ptr: ptr
        }
    }
    pub unsafe fn get_ptr(&self) -> *ffi::PolygonShape {
        self.ptr as *ffi::PolygonShape
    }
    pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::PolygonShape {
        self.ptr
    }
    
    pub fn new() -> Polygon {
        unsafe {
            Polygon::from_ptr(ffi::PolygonShape_new())
        }
    }
    pub fn set(&mut self, points: Vec<Vec2>) {
        unsafe {
            ffi::PolygonShape_set(self.get_mut_ptr(),
                                  points.as_ptr(),
                                  points.len() as i32)
        }
    }
    pub fn set_as_box(&mut self, hw: f32, hh: f32) {
        unsafe {
            ffi::PolygonShape_set_as_box(self.get_mut_ptr(), hw, hh)
        }
    }
    pub fn set_as_oriented_box(&mut self, hw: f32, hh: f32,
                               center: Vec2, angle: f32) {
        unsafe {
            ffi::PolygonShape_set_as_oriented_box(self.get_mut_ptr(),
                                                  hw, hh,
                                                  &center, angle)
        }
    }
    pub fn get_vertex_count(&self) -> uint {
        unsafe {
            ffi::PolygonShape_get_vertex_count(self.get_ptr()) as uint
        }
    }
    pub fn get_vertex(&self, index: uint) -> &Vec2 {
        unsafe {
            &*ffi::PolygonShape_get_vertex(self.get_ptr(), index as i32)
        }
    }
    pub fn validate(&self) -> bool {
        unsafe {
            ffi::PolygonShape_validate(self.get_ptr())
        }
    }
}

impl Shape for Polygon {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Polygon {
        Polygon::from_ptr(ffi::Shape_as_polygon_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::PolygonShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::PolygonShape_as_shape(self.ptr)
    }
}

impl Drop for Polygon {
    fn drop(&mut self) {
        unsafe {
            ffi::PolygonShape_drop(self.get_mut_ptr())
        }
    }
}
