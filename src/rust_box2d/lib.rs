#![crate_id = "rust_box2d#2.3.1"]
#![crate_type = "lib"]
#![license = "GPLv3"]

#![feature(macro_rules)]

pub use common::math;

mod ffi;

#[macro_export]
macro_rules! c_enum(
    ($name:ident with $($element:ident = $value:expr),+) => (
        pub type $name = i32;
        $(
            pub static $element: $name = $value as $name;
        )+
    );
)

#[macro_export]
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
            unsafe fn get_ptr(&self) -> *$wrapped {
                self.ptr as *$wrapped
            }
            unsafe fn get_mut_ptr(&mut self) -> *mut $wrapped {
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
    unsafe fn get_ptr(&self) -> *T;
    unsafe fn get_mut_ptr(&mut self) -> *mut T;
}
