use libc::c_void;
use std::ptr;

pub type Any = *mut c_void;
pub type ConstAny = *const c_void;

#[repr(C)]
pub struct FatAny {
    raw1: *mut c_void,
    raw2: *mut c_void
}

impl FatAny {
    pub fn null() -> FatAny {
        FatAny {
            raw1: ptr::null_mut(),
            raw2: ptr::null_mut()
        }
    }
}
