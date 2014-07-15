#![license = "GPLv3"]

#![feature(macro_rules, unsafe_destructor)]

extern crate libc;
pub use common::math;
pub use common::settings;
pub use ffi::Any;

#[link(name = "Box2D")] extern {}
#[link(name = "stdc++")] extern {}

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

mod ffi;
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
