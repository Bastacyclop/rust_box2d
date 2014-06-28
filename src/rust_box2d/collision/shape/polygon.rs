use ffi;
use math::Vec2;
use super::{ShapeWrapper, Shape};
use WrapStruct;
use Wrapper;

pub type PolygonShape = WrapStruct<ffi::PolygonShape>;

pub trait PolygonShapeT: Wrapper<ffi::PolygonShape> {
    fn new() -> Self {
        unsafe {
            Wrapper::from_ptr(ffi::PolygonShape_new())
        }
    }
    fn set(&mut self, points: Vec<Vec2>) {
        unsafe {
            ffi::PolygonShape_set(self.get_mut_ptr(),
                                  points.as_ptr(),
                                  points.len() as i32)
        }
    }
    fn set_as_box(&mut self, hw: f32, hh: f32) {
        unsafe {
            ffi::PolygonShape_set_as_box(self.get_mut_ptr(), hw, hh)
        }
    }
    fn set_as_oriented_box(&mut self, hw: f32, hh: f32,
                               center: &Vec2, angle: f32) {
        unsafe {
            ffi::PolygonShape_set_as_oriented_box(self.get_mut_ptr(),
                                                  hw, hh,
                                                  center, angle)
        }
    }
    fn get_vertex_count(&self) -> uint {
        unsafe {
            ffi::PolygonShape_get_vertex_count(self.get_ptr()) as uint
        }
    }
    fn get_vertex(&self, index: uint) -> &Vec2 {
        unsafe {
            let vertex =
                ffi::PolygonShape_get_vertex(self.get_ptr(), index as i32);
            assert!(!vertex.is_null());
            &*vertex
        }
    }
    fn validate(&self) -> bool {
        unsafe {
            ffi::PolygonShape_validate(self.get_ptr())
        }
    }
}

impl PolygonShapeT for PolygonShape {}

impl ShapeWrapper for PolygonShape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> PolygonShape {
        Wrapper::from_ptr(ffi::Shape_as_polygon_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::PolygonShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::PolygonShape_as_shape(self.ptr)
    }
}

impl Shape for PolygonShape {}

#[unsafe_destructor]
impl Drop for PolygonShape {
    fn drop(&mut self) {
        unsafe {
            ffi::PolygonShape_drop(self.get_mut_ptr())
        }
    }
}
