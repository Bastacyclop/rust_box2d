use math::{Vec2, Transform};

pub mod math;
pub mod settings;

#[repr(C)]
#[deriving(Clone, PartialEq, Show)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32
}

bitflags!(
    #[repr(C)]
    flags DrawFlags: u32 {
    const DRAW_SHAPE = 0x0001,
    const DRAW_JOINT = 0x0002,
    const DRAW_AABB = 0x0004,
    const DRAW_PAIR = 0x0008,
    const DRAW_CENTER_OF_MASS = 0x0010
    }
)

pub trait Draw {
    fn draw_polygon(&mut self, vertices: Vec<Vec2>, color: &Color);
    fn draw_solid_polygon(&mut self, vertices: Vec<Vec2>, color: &Color);
    fn draw_circle(&mut self, center: &Vec2, radius: f32, color: &Color);
    fn draw_solid_circle(&mut self, center: &Vec2, radius: f32, axis: &Vec2, color: &Color);
    fn draw_segment(&mut self, p1: &Vec2, p2: &Vec2, color: &Color);
    fn draw_transform(&mut self, xf: &Transform);
}

pub mod private {
    use std::mem;
    use std::vec;
    use {ffi, MaybeOwned, Owned, Wrapped, BuildWrapped};
    use math::{Vec2, Transform};
    use super::{Draw, DrawFlags, Color};

    unsafe extern fn draw_polygon(any: ffi::FatAny, vertices: *const Vec2,
                                  count: i32, color: *const Color) {
         // color comes from a C++ &
        let draw = mem::transmute::<_, &mut Draw>(any);
        draw.draw_polygon(vec::raw::from_buf(vertices, count as uint), &*color)
    }

    unsafe extern fn draw_solid_polygon(any: ffi::FatAny, vertices: *const Vec2,
                                        count: i32, color: *const Color) {
         // color comes from a C++ &
        let draw = mem::transmute::<_, &mut Draw>(any);
        draw.draw_solid_polygon(vec::raw::from_buf(vertices, count as uint), &*color)
    }

    unsafe extern fn draw_circle(any: ffi::FatAny, center: *const Vec2,
                                 radius: f32, color: *const Color) {
        // center and color are coming from C++ &s
        let draw = mem::transmute::<_, &mut Draw>(any);
        draw.draw_circle(&*center, radius, &*color)
    }

    unsafe extern fn draw_solid_circle(any: ffi::FatAny, center: *const Vec2,
                                       radius: f32, axis: *const Vec2,
                                       color: *const Color) {
        // center, axis and color are coming from C++ &s
        let draw = mem::transmute::<_, &mut Draw>(any);
        draw.draw_solid_circle(&*center, radius, &*axis, &*color)
    }

    unsafe extern fn draw_segment(any: ffi::FatAny, p1: *const Vec2,
                                  p2: *const Vec2, color: *const Color) {
        // p1, p2 and color are coming from C++ &s
        let draw = mem::transmute::<_, &mut Draw>(any);
        draw.draw_segment(&*p1, &*p2, &*color)
    }

    unsafe extern fn draw_transform(any: ffi::FatAny, xf: *const Transform) {
        // xf comes from a C++ &
        let draw = mem::transmute::<_, &mut Draw>(any);
        draw.draw_transform(&*xf)
    }

    wrapped!(ffi::DrawLink into DrawLink)

    impl DrawLink {
        pub fn new() -> DrawLink {
            unsafe {
                BuildWrapped::with(ffi::DrawLink_new(ffi::FatAny::null(),
                                                     draw_polygon,
                                                     draw_solid_polygon,
                                                     draw_circle,
                                                     draw_solid_circle,
                                                     draw_segment,
                                                     draw_transform),
                                   Owned)
            }
        }
        
        pub unsafe fn set_object(&mut self, object: ffi::FatAny) {
            ffi::DrawLink_set_object(self.mut_ptr(), object)
        }
        
        pub fn set_flags(&mut self, flags: DrawFlags) {
            unsafe {
                ffi::DrawLink_set_flags(self.mut_ptr(), flags)
            }
        }
        
        pub fn flags(&self) -> DrawFlags {
            unsafe {
                ffi::DrawLink_get_flags(self.ptr())
            }
        }
        
        pub fn insert_flags(&mut self, flags: DrawFlags) {
            unsafe {
                ffi::DrawLink_append_flags(self.mut_ptr(), flags)
            }
        }
        
        pub fn remove_flags(&mut self, flags: DrawFlags) {
            unsafe {
                ffi::DrawLink_clear_flags(self.mut_ptr(), flags)
            }
        }
    }    

    impl Drop for DrawLink {
        fn drop(&mut self) {
            unsafe {
                if self.mb_owned == Owned {
                    ffi::DrawLink_drop(self.mut_ptr())
                }
            }
        }
    }
}
