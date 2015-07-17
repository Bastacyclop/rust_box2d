macro_rules! wrap {
    ($wrapped:ty: custom $wrap:ident) => (
        impl Wrapped<$wrapped> for $wrap {
            unsafe fn ptr(&self) -> *const $wrapped {
                self.ptr as *const $wrapped
            }

            unsafe fn mut_ptr(&mut self) -> *mut $wrapped {
                self.ptr
            }
        }
    );

    ($wrapped:ty: $wrap:ident) => (
        wrap!{ $wrapped: custom $wrap }

        pub struct $wrap {
            ptr: *mut $wrapped,
        }

        impl FromFFI<$wrapped> for $wrap {
            unsafe fn from_ffi(ptr: *mut $wrapped) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: ptr,
                }
            }
        }
    );

    ($wrapped:ty: private $wrap:ident) => (
        wrap!{ $wrapped: custom $wrap }

        struct $wrap {
            ptr: *mut $wrapped,
        }

        impl FromFFI<$wrapped> for $wrap {
            unsafe fn from_ffi(ptr: *mut $wrapped) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: ptr,
                }
            }
        }
    );

    ($wrapped:ty: custom $wrap:ident with base $base:ty
     > $as_base:path
    ) => (

        wrap! { $wrapped: custom $wrap }

        impl WrappedBase<$base> for $wrap {
            unsafe fn base_ptr(&self) -> *const $base {
                $as_base(self.ptr) as *const $base
            }

            unsafe fn mut_base_ptr(&mut self) -> *mut $base {
                $as_base(self.ptr)
            }
        }
    );

    ($wrapped:ty: $wrap:ident with base $base:ty
     > $as_base:path,
     < $base_as:path
    ) => (

        wrap! { $wrapped: $wrap }

        impl WrappedBase<$base> for $wrap {
            unsafe fn base_ptr(&self) -> *const $base {
                $as_base(self.ptr) as *const $base
            }

            unsafe fn mut_base_ptr(&mut self) -> *mut $base {
                $as_base(self.ptr)
            }
        }

        impl FromFFI<$base> for $wrap {
            unsafe fn from_ffi(ptr: *mut $base) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: $base_as(ptr),
                }
            }
        }
    );
}

use std::mem;
use std::marker::PhantomData;
use std::ops::{ Deref, DerefMut };

pub trait Wrapped<T> {
    unsafe fn ptr(&self) -> *const T;
    unsafe fn mut_ptr(&mut self) -> *mut T;
}

pub trait WrappedBase<B> {
    unsafe fn base_ptr(&self) -> *const B;
    unsafe fn mut_base_ptr(&mut self) -> *mut B;
}

pub trait FromFFI<T> {
    unsafe fn from_ffi(ptr: *mut T) -> Self;
}

pub struct RefMut<'a, T> {
    object: Option<T>,
    phantom: PhantomData<&'a ()>
}

impl<'a, T> RefMut<'a, T> {
    pub unsafe fn new(t: T) -> RefMut<'a, T> {
        RefMut {
            object: Some(t),
            phantom: PhantomData
        }
    }
}

impl<'a, T> Deref for RefMut<'a, T> {
    type Target = T;

    fn deref<'b>(&'b self) -> &'b T { self.object.as_ref().unwrap() }
}

impl<'a, T> DerefMut for RefMut<'a, T> {
    fn deref_mut<'b>(&'b mut self) -> &'b mut T { self.object.as_mut().unwrap() }
}

impl<'a, T> Drop for RefMut<'a, T> {
    fn drop(&mut self) {
        mem::forget(self.object.take())
    }
}

pub struct Ref<'a, T> {
    object: Option<T>,
    phantom: PhantomData<&'a ()>
}

impl<'a, T> Ref<'a, T> {
    pub unsafe fn new(t: T) -> Ref<'a, T> {
        Ref {
            object: Some(t),
            phantom: PhantomData
        }
    }
}

impl<'a, T> Deref for Ref<'a, T> {
    type Target = T;

    fn deref<'b>(&'b self) -> &'b T { self.object.as_ref().unwrap() }
}

impl<'a, T> Drop for Ref<'a, T> {
    fn drop(&mut self) {
        mem::forget(self.object.take())
    }
}
