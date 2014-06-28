use ffi;
use math::Vec2;
use super::{ShapeWrapper, Shape};
use WrapStruct;
use Wrapper;

pub type EdgeShape = WrapStruct<ffi::EdgeShape>;

pub trait EdgeShapeT: Wrapper<ffi::EdgeShape> {
    fn new() -> Self {
        unsafe {
            Wrapper::from_ptr(ffi::EdgeShape_new())
        }
    }
    fn set(&mut self, v1: &Vec2, v2: &Vec2) {
        unsafe {
            ffi::EdgeShape_set(self.get_mut_ptr(), v1, v2)
        }
    }
}

impl EdgeShapeT for EdgeShape {}

impl ShapeWrapper for EdgeShape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> EdgeShape {
        Wrapper::from_ptr(ffi::Shape_as_edge_shape(ptr))
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        ffi::EdgeShape_as_shape(self.ptr) as *ffi::Shape
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        ffi::EdgeShape_as_shape(self.ptr)
    }
}

impl Shape for EdgeShape {}

#[unsafe_destructor]
impl Drop for EdgeShape {
    fn drop(&mut self) {
        unsafe {
            ffi::EdgeShape_drop(self.get_mut_ptr())
        }
    }
}
