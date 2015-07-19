macro_rules! _from_ffi {
    { $wrap:ident <= $wrapped:ty } => {
        impl FromFFI<$wrapped> for $wrap {
            unsafe fn from_ffi(ptr: *mut $wrapped) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: ptr,
                }
            }
        }
    }
}

macro_rules! wrap {
    { $wrapped:ty => custom $wrap:ident } => {
        impl Wrapped<$wrapped> for $wrap {
            unsafe fn ptr(&self) -> *const $wrapped {
                self.ptr as *const $wrapped
            }

            unsafe fn mut_ptr(&mut self) -> *mut $wrapped {
                self.ptr
            }
        }
    };

    { $wrapped:ty => pub $wrap:ident } => {
        pub struct $wrap {
            ptr: *mut $wrapped,
        }

        wrap! { $wrapped => custom $wrap }
        _from_ffi! { $wrap <= $wrapped }
    };

    { $wrapped:ty => $wrap:ident } => {
        struct $wrap {
            ptr: *mut $wrapped,
        }

        wrap! { $wrapped => custom $wrap }
        _from_ffi! { $wrap <= $wrapped }
    };

    {
        base $base:ty => custom $wrap:ident
        < $as_base:path
    } => {
        impl WrappedBase<$base> for $wrap {
            unsafe fn base_ptr(&self) -> *const $base {
                $as_base(self.ptr) as *const $base
            }

            unsafe fn mut_base_ptr(&mut self) -> *mut $base {
                $as_base(self.ptr)
            }
        }
    };

    {
        $wrapped:ty (base $base:ty) => custom $wrap:ident
        < $as_base:path
        > $base_as:path
    } => {
        wrap! { $wrapped => custom $wrap }
        wrap! {
            base $base => custom $wrap
            < $as_base
        }
    };

    {
        $wrapped:ty (base $base:ty) => pub $wrap:ident
        < $as_base:path
        > $base_as:path
    } => {
        wrap! { $wrapped => pub $wrap }
        wrap! {
            base $base => custom $wrap
            < $as_base
        }

        impl FromFFI<$base> for $wrap {
            unsafe fn from_ffi(ptr: *mut $base) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: $base_as(ptr),
                }
            }
        }
    };
}

use std::mem;
use std::marker::PhantomData;
use std::ops::{ Deref, DerefMut };

#[doc(hidden)]
pub trait Wrapped<T> {
    unsafe fn ptr(&self) -> *const T;
    unsafe fn mut_ptr(&mut self) -> *mut T;
}

#[doc(hidden)]
pub trait WrappedBase<B> {
    unsafe fn base_ptr(&self) -> *const B;
    unsafe fn mut_base_ptr(&mut self) -> *mut B;
}

#[doc(hidden)]
pub trait FromFFI<T> {
    unsafe fn from_ffi(ptr: *mut T) -> Self where Self: Sized;
}

pub struct WrappedRefMut<'a, T> {
    object: Option<T>,
    phantom: PhantomData<&'a ()>
}

impl<'a, T> WrappedRefMut<'a, T> {
    #[doc(hidden)]
    pub unsafe fn new(t: T) -> WrappedRefMut<'a, T> {
        WrappedRefMut {
            object: Some(t),
            phantom: PhantomData
        }
    }
}

impl<'a, T> Deref for WrappedRefMut<'a, T> {
    type Target = T;

    fn deref<'b>(&'b self) -> &'b T { self.object.as_ref().unwrap() }
}

impl<'a, T> DerefMut for WrappedRefMut<'a, T> {
    fn deref_mut<'b>(&'b mut self) -> &'b mut T { self.object.as_mut().unwrap() }
}

impl<'a, T> Drop for WrappedRefMut<'a, T> {
    fn drop(&mut self) {
        mem::forget(self.object.take())
    }
}

pub struct WrappedRef<'a, T> {
    object: Option<T>,
    phantom: PhantomData<&'a ()>
}

impl<'a, T> WrappedRef<'a, T> {
    #[doc(hidden)]
    pub unsafe fn new(t: T) -> WrappedRef<'a, T> {
        WrappedRef {
            object: Some(t),
            phantom: PhantomData
        }
    }
}

impl<'a, T> Deref for WrappedRef<'a, T> {
    type Target = T;

    fn deref<'b>(&'b self) -> &'b T { self.object.as_ref().unwrap() }
}

impl<'a, T> Drop for WrappedRef<'a, T> {
    fn drop(&mut self) {
        mem::forget(self.object.take())
    }
}
