use ffi;
use math::Vec2;

pub enum Type {
    Static,
    Kinematic,
    Dynamic,
}

#[allow(dead_code)]
pub struct Def {
    pub body_type: Type,
    pub position: Vec2,
    pub angle: f32,
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub allow_sleep: bool,
    pub awake: bool,
    pub fixed_rotation: bool,
    pub bullet: bool,
    pub active: bool,
    user_data: ffi::UserData,
    pub gravity_scale: f32,
}

pub struct Body<'l> {
    ptr: *mut ffi::Body,
}

impl<'l> Body<'l> {
    pub unsafe fn from_ptr<'l>(ptr: *mut ffi::Body) -> Body<'l> {
        Body { ptr: ptr }
    }
    pub unsafe fn get_ptr(&self) -> *ffi::Body {
        self.ptr as *ffi::Body
    }
    pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::Body {
        self.ptr
    }
}

impl<'l> Drop for Body<'l> {
    fn drop(&mut self) {
    }
}
