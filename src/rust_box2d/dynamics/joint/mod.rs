use ffi;

c_enum!(
    [Type]
    UNKNOWN = 0,
    REVOLUTE = 1,
    PRISMATIC = 2,
    DISTANCE = 3,
    PULLEY = 4,
    MOUSE = 5,
    GEAR = 6,
    WHEEL = 7,
    WELD = 8,
    FRICTION = 9,
    ROPE = 10,
    MOTOR = 11
)

#[allow(dead_code)]
#[packed]
pub struct Def {
    pub joint_type: Type,
    user_data: ffi::UserData,
    body_a: *mut ffi::Body,
    body_b: *mut ffi::Body,
    pub collide_connected: bool,
}

pub struct Unknown<'l> {
    ptr: *mut ffi::Joint,
}

impl<'l> Unknown<'l> {
    pub unsafe fn from_ptr(ptr: *mut ffi::Joint) -> Unknown {
        Unknown { ptr: ptr }
    }
    pub unsafe fn get_ptr(&self) -> *ffi::Joint {
        self.ptr as *ffi::Joint
    }
    pub unsafe fn get_mut_ptr(&mut self) -> *mut ffi::Joint {
        self.ptr
    }
}

impl<'l> Drop for Unknown<'l> {
    fn drop(&mut self) {
    }
}
