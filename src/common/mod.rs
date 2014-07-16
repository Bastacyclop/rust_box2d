use std::mem;
use std::ptr;
use std::vec;
use ffi;
use math::{Vec2, Transform};
use self::private::DerivedDraw;

pub mod math;
pub mod settings;

#[allow(visible_private_types)]
pub mod private {
    use ffi;

    pub trait DerivedDraw {
        unsafe fn draw_ptr(&self) -> *const ffi::Draw;
        unsafe fn mut_draw_ptr(&mut self) -> *mut ffi::Draw;
    }
}

pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32
}

#[repr(C)]
#[allow(non_camel_case_types)]
#[deriving(PartialEq, Show)]
pub enum DrawFlags {
    DRAW_SHAPE = 0x0001,
    DRAW_JOINT = 0x0002,
    DRAW_AABB = 0x0004,
    DRAW_PAIR = 0x0008,
    DRAW_CENTER_OF_MASS = 0x0010
}

pub trait Draw {
    fn draw_polygon(&mut self, vertices: Vec<Vec2>, color: &Color);
    fn draw_solid_polygon(&mut self, vertices: Vec<Vec2>, color: &Color);
    fn draw_circle(&mut self, center: &Vec2, radius: f32, color: &Color);
    fn draw_solid_circle(&mut self, center: &Vec2, radius: f32, axis: &Vec2,
                         color: &Color);
    fn draw_segment(&mut self, p1: &Vec2, p2: &Vec2, color: &Color);
    fn draw_transform(&mut self, xf: &Transform);
}

unsafe extern fn draw_polygon(any: ffi::Any, vertices: *const Vec2,
                              count: i32, color: *const Color) {
    assert!(!any.is_null())
    assert!(!color.is_null())
    let draw = mem::transmute::<_, *mut &mut Draw>(any);
    (*draw).draw_polygon(vec::raw::from_buf(vertices, count as uint), &*color)
}
unsafe extern fn draw_solid_polygon(any: ffi::Any, vertices: *const Vec2,
                                    count: i32, color: *const Color) {
    assert!(!any.is_null())
    assert!(!color.is_null())
    let draw = mem::transmute::<_, *mut &mut Draw>(any);
    (*draw).draw_solid_polygon(vec::raw::from_buf(vertices, count as uint),
                              &*color)
}
unsafe extern fn draw_circle(any: ffi::Any, center: *const Vec2,
                             radius: f32, color: *const Color) {
    assert!(!any.is_null())
    assert!(!center.is_null())
    assert!(!color.is_null())
    let draw = mem::transmute::<_, *mut &mut Draw>(any);
    (*draw).draw_circle(&*center, radius, &*color)
}
unsafe extern fn draw_solid_circle(any: ffi::Any, center: *const Vec2,
                                   radius: f32, axis: *const Vec2,
                                   color: *const Color) {
    assert!(!any.is_null())
    assert!(!center.is_null())
    assert!(!axis.is_null())
    assert!(!color.is_null())
    let draw = mem::transmute::<_, *mut &mut Draw>(any);
    (*draw).draw_solid_circle(&*center, radius, &*axis, &*color)
}
unsafe extern fn draw_segment(any: ffi::Any, p1: *const Vec2,
                              p2: *const Vec2, color: *const Color) {
    assert!(!any.is_null())
    assert!(!p1.is_null())
    assert!(!p2.is_null())
    assert!(!color.is_null())
    let draw = mem::transmute::<_, *mut &mut Draw>(any);
    (*draw).draw_segment(&*p1, &*p2, &*color)
}
unsafe extern fn draw_transform(any: ffi::Any, xf: *const Transform) {
    assert!(!any.is_null())
    assert!(!xf.is_null())
    let draw = mem::transmute::<_, *mut &mut Draw>(any);
    (*draw).draw_transform(&*xf)
}

pub struct DrawLink<T> {
    t: T,
    c: *mut ffi::CDraw
}

impl<T: Draw> DrawLink<T> {
    pub fn with(t: T) -> DrawLink<T> {
        let mut link = DrawLink {
            t: t,
            c: ptr::mut_null()
        };
        unsafe {
            link.c = ffi::CDraw_new(mem::transmute(&mut &mut link.t),
                                    draw_polygon,
                                    draw_solid_polygon,
                                    draw_circle,
                                    draw_solid_circle,
                                    draw_segment,
                                    draw_transform);
        }
        link
    }
    pub fn set_flags(&mut self, flags: DrawFlags) {
        unsafe {
            ffi::CDraw_set_flags(self.c, flags)
        }
    }
    pub fn flags(&self) -> DrawFlags {
        unsafe {
            ffi::CDraw_get_flags(self.c as *const ffi::CDraw)
        }
    }
    pub fn add_flags(&mut self, flags: DrawFlags) {
        unsafe {
            ffi::CDraw_append_flags(self.c, flags)
        }
    }
    pub fn remove_flags(&mut self, flags: DrawFlags) {
        unsafe {
            ffi::CDraw_clear_flags(self.c, flags)
        }
    }
}    

impl<T> DerivedDraw for DrawLink<T> {
    unsafe fn draw_ptr(&self) -> *const ffi::Draw {
        ffi::CDraw_as_base(self.c) as *const ffi::Draw
    }
    unsafe fn mut_draw_ptr(&mut self) -> *mut ffi::Draw {
        ffi::CDraw_as_base(self.c)
    }
}

#[unsafe_destructor]
impl<T> Drop for DrawLink<T> {
    fn drop(&mut self) {
        unsafe {
            ffi::CDraw_drop(self.c)
        }
    }
}
