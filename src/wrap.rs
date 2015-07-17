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
        wrap! { $wrapped: custom $wrap }

        pub struct $wrap {
            ptr: *mut $wrapped,
            mb_owned: MaybeOwned
        }

        impl BuildWrapped<$wrapped, MaybeOwned> for $wrap {
            unsafe fn with(ptr: *mut $wrapped, mb_owned: MaybeOwned) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: ptr,
                    mb_owned: mb_owned
                }
            }
        }
    );

    ($wrapped:ty: simple $wrap:ident) => (
        wrap!{ $wrapped: custom $wrap }

        pub struct $wrap {
            ptr: *mut $wrapped,
        }

        impl BuildWrapped<$wrapped, ()> for $wrap {
            unsafe fn with(ptr: *mut $wrapped, _: ()) -> $wrap {
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

        impl BuildWrappedBase<$base, MaybeOwned> for $wrap {
            unsafe fn with(ptr: *mut $base, mb_owned: MaybeOwned) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: $base_as(ptr),
                    mb_owned: mb_owned
                }
            }
        }
    );

    ($wrapped:ty: simple $wrap:ident with base $base:ty
     > $as_base:path,
     < $base_as:path
    ) => (

        wrap! { $wrapped: simple $wrap }

        impl WrappedBase<$base> for $wrap {
            unsafe fn base_ptr(&self) -> *const $base {
                $as_base(self.ptr) as *const $base
            }

            unsafe fn mut_base_ptr(&mut self) -> *mut $base {
                $as_base(self.ptr)
            }
        }

        impl BuildWrappedBase<$base, ()> for $wrap {
            unsafe fn with(ptr: *mut $base, _: ()) -> $wrap {
                assert!(!ptr.is_null());
                $wrap {
                    ptr: $base_as(ptr),
                }
            }
        }
    );
}

use std::ops::{ Deref };

pub use self::MaybeOwned::{ Owned, NotOwned };
#[derive(PartialEq)]
pub enum MaybeOwned {
    Owned,
    NotOwned
}

pub trait Wrapped<T> {
    unsafe fn ptr(&self) -> *const T;
    unsafe fn mut_ptr(&mut self) -> *mut T;
}

pub trait WrappedBase<B> {
    unsafe fn base_ptr(&self) -> *const B;
    unsafe fn mut_base_ptr(&mut self) -> *mut B;
}

pub trait BuildWrapped<T, A> {
    unsafe fn with(ptr: *mut T, a: A) -> Self;
}

pub trait BuildWrappedBase<B, A> {
    unsafe fn with(ptr: *mut B, a: A) -> Self;
}

pub struct Const<T> {
    object: T
}

impl<T> Const<T> {
    pub fn new(t: T) -> Const<T> {
        Const { object: t }
    }
}

impl<T> Deref for Const<T> {
    type Target = T;

    fn deref<'a>(&'a self) -> &'a T {
        &self.object
    }
}
