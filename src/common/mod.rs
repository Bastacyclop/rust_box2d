pub mod math;
pub mod settings;

use std::mem;
use common::math::{ Vec2, Transform };

#[repr(C)]
#[derive(Clone, PartialEq, Debug)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32
}

impl Color {
    pub fn as_array(&self) -> &[f32; 4] {
        unsafe { mem::transmute(self) }
    }

    pub fn as_array_mut(&mut self) -> &mut [f32; 4] {
        unsafe { mem::transmute(self) }
    }

    pub fn from_array_ref(array: &[f32; 4]) -> &Vec2 {
        unsafe { mem::transmute(array) }
    }

    pub fn from_array_mut(array: &mut [f32; 4]) -> &mut Vec2 {
        unsafe { mem::transmute(array) }
    }
}

bitflags! {
    #[repr(C)]
    flags DrawFlags: u32 {
        const DRAW_SHAPE = 0x0001,
        const DRAW_JOINT = 0x0002,
        const DRAW_AABB = 0x0004,
        const DRAW_PAIR = 0x0008,
        const DRAW_CENTER_OF_MASS = 0x0010
    }
}

pub trait Draw {
    fn draw_polygon(&mut self, vertices: &[Vec2], color: &Color);
    fn draw_solid_polygon(&mut self, vertices: &[Vec2], color: &Color);
    fn draw_circle(&mut self, center: &Vec2, radius: f32, color: &Color);
    fn draw_solid_circle(&mut self, center: &Vec2, radius: f32, axis: &Vec2, color: &Color);
    fn draw_segment(&mut self, p1: &Vec2, p2: &Vec2, color: &Color);
    fn draw_transform(&mut self, xf: &Transform);
}

#[doc(hidden)]
pub struct DrawLink {
    ptr: *mut ffi::DrawLink
}

impl DrawLink {
    pub fn new() -> DrawLink {
        unsafe {
            DrawLink {
                ptr: ffi::DrawLink_new(
                    ffi::FatAny::null(),
                    draw_polygon,
                    draw_solid_polygon,
                    draw_circle,
                    draw_solid_circle,
                    draw_segment,
                    draw_transform
                )
            }
        }
    }

    pub unsafe fn use_with(&mut self,
                           object: &mut Draw, flags: DrawFlags) -> *mut ffi::Draw {
        ffi::DrawLink_set_object(self.ptr, mem::transmute(object));
        ffi::DrawLink_set_flags(self.ptr, flags);
        ffi::DrawLink_as_base(self.ptr)
    }
}

impl Drop for DrawLink {
    fn drop(&mut self) {
        unsafe {
            ffi::DrawLink_drop(self.ptr)
        }
    }
}

unsafe extern fn draw_polygon(draw: ffi::FatAny, vertices: *const Vec2,
                              count: i32, color: *const Color) {
     // color comes from a C++ &
    let draw = mem::transmute::<_, &mut Draw>(draw);
    let vertices = ::std::slice::from_raw_parts(vertices, count as usize);
    draw.draw_polygon(vertices, &*color)
}

unsafe extern fn draw_solid_polygon(draw: ffi::FatAny, vertices: *const Vec2,
                                    count: i32, color: *const Color) {
     // color comes from a C++ &
    let draw = mem::transmute::<_, &mut Draw>(draw);
    let vertices = ::std::slice::from_raw_parts(vertices, count as usize);
    draw.draw_solid_polygon(vertices, &*color)
}

unsafe extern fn draw_circle(draw: ffi::FatAny, center: *const Vec2,
                             radius: f32, color: *const Color) {
    // center and color are coming from C++ &s
    let draw = mem::transmute::<_, &mut Draw>(draw);
    draw.draw_circle(&*center, radius, &*color)
}

unsafe extern fn draw_solid_circle(draw: ffi::FatAny, center: *const Vec2,
                                   radius: f32, axis: *const Vec2,
                                   color: *const Color) {
    // center, axis and color are coming from C++ &s
    let draw = mem::transmute::<_, &mut Draw>(draw);
    draw.draw_solid_circle(&*center, radius, &*axis, &*color)
}

unsafe extern fn draw_segment(draw: ffi::FatAny, p1: *const Vec2,
                              p2: *const Vec2, color: *const Color) {
    // p1, p2 and color are coming from C++ &s
    let draw = mem::transmute::<_, &mut Draw>(draw);
    draw.draw_segment(&*p1, &*p2, &*color)
}

unsafe extern fn draw_transform(draw: ffi::FatAny, xf: *const Transform) {
    // xf comes from a C++ &
    let draw = mem::transmute::<_, &mut Draw>(draw);
    draw.draw_transform(&*xf)
}

#[doc(hidden)]
pub mod ffi {
    pub use ffi::{ Any, FatAny };
    use common::math::{ Vec2, Transform };
    use super::{ Color, DrawFlags };

    pub enum Draw {}
    pub enum DrawLink {}

    extern {
        pub fn DrawLink_new(
            object: FatAny,
            draw_polygon:
            unsafe extern fn (FatAny, *const Vec2, i32, *const Color),
            draw_solid_polygon:
            unsafe extern fn (FatAny, *const Vec2, i32, *const Color),
            draw_circle:
            unsafe extern fn (FatAny, *const Vec2, f32, *const Color),
            draw_solid_circle:
            unsafe extern fn (FatAny, *const Vec2, f32, *const Vec2, *const Color),
            draw_segment:
            unsafe extern fn (FatAny, *const Vec2, *const Vec2, *const Color),
            draw_transform:
            unsafe extern fn (FatAny, *const Transform)
            ) -> *mut DrawLink;
        pub fn DrawLink_as_base(slf: *mut DrawLink) -> *mut Draw;
        pub fn DrawLink_drop(slf: *mut DrawLink);
        pub fn DrawLink_set_object(slf: *mut DrawLink, object: FatAny);
        pub fn DrawLink_set_flags(slf: *mut DrawLink, flags: DrawFlags);
    }
}
