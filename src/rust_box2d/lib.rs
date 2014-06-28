#![crate_id = "rust_box2d#2.3.1"]
#![crate_type = "lib"]
#![license = "GPLv3"]

#![feature(macro_rules)]

pub use common::math;

pub mod ffi;

#[macro_export]
macro_rules! c_enum(
    ([$name:ident] $($element:ident = $value:expr),+) => (
        pub type $name = i32;
        $(
            pub static $element: $name = $value as $name;
        )+
    );
)

pub mod dynamics;
pub mod common;
pub mod collision;

pub struct WrapStruct<T> {
    ptr: *mut T
}

pub trait Wrapper<T> {
    unsafe fn from_ptr(ptr: *mut T) -> Self;
    unsafe fn get_ptr(&self) -> *T;
    unsafe fn get_mut_ptr(&mut self) -> *mut T;
}

impl<T> Wrapper<T> for WrapStruct<T> {
    unsafe fn from_ptr(ptr: *mut T) -> WrapStruct<T> {
        assert!(!ptr.is_null())
        WrapStruct {
            ptr: ptr
        }
    }
    unsafe fn get_ptr(&self) -> *T {
        self.ptr as *T
    }
    unsafe fn get_mut_ptr(&mut self) -> *mut T {
        self.ptr
    }
}
