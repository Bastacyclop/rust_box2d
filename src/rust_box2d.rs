#![crate_id = "rust_box2d#2.3.1"]
#![crate_type = "lib"]
#![license = "GPLv3"]

#![feature(macro_rules)]

pub use common::math;
pub use common::settings;

mod ffi;

macro_rules! c_enum(
    ($name:ident with $($element:ident = $value:expr),+) => (
        pub type $name = i32;
        $(
            pub static $element: $name = $value as $name;
        )+
    );
)

macro_rules! wrap(
    ($wrapped:ty into $wrap:ident) => (
        pub struct $wrap {
            ptr: *mut $wrapped
        }
        
        impl Wrapped<$wrapped> for $wrap {
            unsafe fn from_ptr(ptr: *mut $wrapped) -> $wrap {
                assert!(!ptr.is_null())
                $wrap {
                    ptr: ptr
                }   
            }
            unsafe fn ptr(&self) -> *const $wrapped {
                self.ptr as *const $wrapped
            }
            unsafe fn mut_ptr(&mut self) -> *mut $wrapped {
                self.ptr
            }    
        }
    );
)

pub mod dynamics;
pub mod common;
pub mod collision;

trait Wrapped<T> {
    unsafe fn from_ptr(ptr: *mut T) -> Self;
    unsafe fn ptr(&self) -> *const T;
    unsafe fn mut_ptr(&mut self) -> *mut T;
}

fn clone_from_ptr<T: Clone>(t: *const T) -> T {
    unsafe {
        assert!(!t.is_null())
        (*t).clone()
    }
}
