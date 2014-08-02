#![license = "GPLv3"]

#![feature(macro_rules, globs)]

extern crate libc;

pub use common::math;
pub use common::settings;

#[link(name = "Box2D")] extern {}
#[link(name = "stdc++")] extern {}

macro_rules! wrapped(
    ($wrapped:ty owned into $wrap:ident) => (
        pub struct $wrap {
            ptr: *mut $wrapped
        }
        
        impl Wrapped<$wrapped> for $wrap {
            unsafe fn ptr(&self) -> *const $wrapped {
                self.ptr as *const $wrapped
            }
        }
        
        impl WrappedMut<$wrapped> for $wrap {
            unsafe fn from_ptr(ptr: *mut $wrapped) -> $wrap {
                assert!(!ptr.is_null())
                $wrap {
                    ptr: ptr
                }
            }
            
            unsafe fn mut_ptr(&mut self) -> *mut $wrapped {
                self.ptr
            }
        }
    );
    
    ($wrapped:ty into mut $mut_wrap:ident) => (
        pub struct $mut_wrap<'l> {
            ptr: *mut $wrapped
        }
        
        impl<'l> Wrapped<$wrapped> for $mut_wrap<'l> {
            unsafe fn ptr(&self) -> *const $wrapped {
                self.ptr as *const $wrapped
            }
        }
        
        impl<'l> WrappedMut<$wrapped> for $mut_wrap<'l> {
            unsafe fn from_ptr(ptr: *mut $wrapped) -> $mut_wrap<'l> {
                assert!(!ptr.is_null())
                $mut_wrap {
                    ptr: ptr
                }
            }
            
            unsafe fn mut_ptr(&mut self) -> *mut $wrapped {
                self.ptr
            }
        }
    );
    
    ($wrapped:ty into const $const_wrap:ident) => (
        pub struct $const_wrap<'l> {
            ptr: *const $wrapped
        }
        
        impl<'l> Wrapped<$wrapped> for $const_wrap<'l> {
            unsafe fn ptr(&self) -> *const $wrapped {
                self.ptr
            }
        }
        
        impl<'l> WrappedConst<$wrapped> for $const_wrap<'l> {
            unsafe fn from_ptr(ptr: *const $wrapped) -> $const_wrap<'l> {
                assert!(!ptr.is_null())
                $const_wrap {
                    ptr: ptr
                }
            } 
        }
    );
    
    ($wrapped:ty into $mut_wrap:ident, $const_wrap:ident) => (
        wrapped!($wrapped into mut $mut_wrap)
        wrapped!($wrapped into const $const_wrap)
    );
        
    ($wrapped:ty owned into $wrap:ident with base $base:ty
     << $base_as:path
     >> $as_base:path) => (
     
        wrapped!($wrapped owned into $wrap)
        
        impl WrappedBase<$base> for $wrap {
            unsafe fn base_ptr(&self) -> *const $base {
                $as_base(self.ptr) as *const $base
            }
        }
        
        impl WrappedMutBase<$base> for $wrap {
            unsafe fn from_ptr(ptr: *mut $base) -> $wrap {
                assert!(!ptr.is_null())
                $wrap {
                    ptr: $base_as(ptr)
                }
            }
            
            unsafe fn mut_base_ptr(&mut self) -> *mut $base {
                $as_base(self.ptr)
            }
        }
    );
    
    ($wrapped:ty into mut $mut_wrap:ident with base $base:ty
     << $base_as:path
     >> $as_base:path) => (
     
        wrapped!($wrapped into mut $mut_wrap)
        
        impl<'l> WrappedBase<$base> for $mut_wrap<'l> {
            unsafe fn base_ptr(&self) -> *const $base {
                $as_base(self.ptr) as *const $base
            }
        }
        
        impl<'l> WrappedMutBase<$base> for $mut_wrap<'l> {
            unsafe fn from_ptr(ptr: *mut $base) -> $mut_wrap<'l> {
                assert!(!ptr.is_null())
                $mut_wrap {
                    ptr: $base_as(ptr)
                }
            }
            
            unsafe fn mut_base_ptr(&mut self) -> *mut $base {
                $as_base(self.ptr)
            }
        }
    );
        
    ($wrapped:ty into const $const_wrap:ident
     with base $base:ty
     << $base_as:path
     >> $as_base:path) => (
     
        wrapped!($wrapped into const $const_wrap)
                 
        impl<'l> WrappedBase<$base> for $const_wrap<'l> {
            unsafe fn base_ptr(&self) -> *const $base {
                $as_base(self.ptr as *mut $wrapped) as *const $base
            }
        }
        
        impl<'l> WrappedConstBase<$base> for $const_wrap<'l> {
            unsafe fn from_ptr(ptr: *const $base) -> $const_wrap<'l> {
                assert!(!ptr.is_null())
                $const_wrap {
                    ptr: $base_as(ptr as *mut $base) as *const $wrapped
                }
            }
        }
    );
        
    ($wrapped:ty into $mut_wrap:ident, $const_wrap:ident
     with base $base:ty
     << $base_as:path
     >> $as_base:path) => (
     
        wrapped!($wrapped into mut $mut_wrap with base $base
                 << $base_as
                 >> $as_base
                 )
        wrapped!($wrapped into const $const_wrap with base $base
                 << $base_as
                 >> $as_base
                 )
    );
)

pub mod b2 {
    pub use super::*;
    pub use super::common::*;
    pub use super::dynamics::*;
    pub use super::dynamics::{ConstBody, MutBody};
    pub use super::collision::*;
    pub use super::math::*;
    pub use super::settings::*;
}

mod ffi;
pub mod dynamics;
pub mod common;
pub mod collision;

trait Wrapped<T> {
    unsafe fn ptr(&self) -> *const T;
}

trait WrappedMut<T>: Wrapped<T> {
    unsafe fn from_ptr(ptr: *mut T) -> Self;
    unsafe fn mut_ptr(&mut self) -> *mut T;
}

trait WrappedConst<T>: Wrapped<T> {
    unsafe fn from_ptr(ptr: *const T) -> Self;
}

trait WrappedBase<T> {
    unsafe fn base_ptr(&self) -> *const T;
}

trait WrappedMutBase<T>: WrappedBase<T> {
    unsafe fn from_ptr(ptr: *mut T) -> Self;
    unsafe fn mut_base_ptr(&mut self) -> *mut T;
}

trait WrappedConstBase<T>: WrappedBase<T> {
    unsafe fn from_ptr(ptr: *const T) -> Self;
}
