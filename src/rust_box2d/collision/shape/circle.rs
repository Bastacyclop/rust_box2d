use ffi;
use math::Vec2;
use super::{ShapeWrapper, Shape};
use WrapStruct;
use Wrapper;

pub type CircleShape = WrapStruct<ffi::CircleShape>;

pub trait CircleShapeT: Wrapper<ffi::CircleShape> {
    fn new() -> Self {
        unsafe {
            Wrapper::from_ptr(ffi::CircleShape_new())
        }
    }
    fn get_support(&self, dir: &Vec2) -> uint {
        unsafe {
            ffi::CircleShape_get_support(self.get_ptr(), dir) as uint
        }
    }
    fn get_support_vertex(&self, dir: &Vec2) -> &Vec2 {
        unsafe {
            let support =
                ffi::CircleShape_get_support_vertex(self.get_ptr(), dir);
            assert!(!support.is_null());
            &*support
        }
    }
    fn get_vertex_count(&self) -> uint {
        unsafe {
            ffi::CircleShape_get_vertex_count(self.get_ptr()) as uint
        }
    }
    fn get_vertex(&self, index: uint) -> &Vec2 {
        unsafe {
            let vertex =
                ffi::CircleShape_get_vertex(self.get_ptr(), index as i32);
            assert!(!vertex.is_null());
            &*vertex
        }
    }
}

impl CircleShapeT for CircleShape {}

impl ShapeWrapper for CircleShape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> CircleShape {
        Wrapper::from_ptr(ffi::Shape_as_circle_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::CircleShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::CircleShape_as_shape(self.ptr)
    }
}

impl Shape for CircleShape {}

#[unsafe_destructor]
impl Drop for CircleShape {
    fn drop(&mut self) {
        unsafe {
            ffi::CircleShape_drop(self.get_mut_ptr())
        }
    }
}
