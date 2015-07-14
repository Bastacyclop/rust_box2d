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

use std::marker::PhantomData;
use std::ops::{ Deref, DerefMut };

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

pub struct RefMut<'l, T> {
    phantom: PhantomData<&'l ()>,
    object: T
}

impl<'l, T> RefMut<'l, T> {
    pub fn new(t: T) -> RefMut<'l, T> {
        RefMut { phantom: PhantomData, object: t }
    }
}

impl<'l, T> Deref for RefMut<'l, T> {
    type Target = T;

    fn deref<'a>(&'a self) -> &'a T {
        &self.object
    }
}

impl<'l, T> DerefMut for RefMut<'l, T> {
    fn deref_mut<'a>(&'a mut self) -> &'a mut T {
        &mut self.object
    }
}

pub struct Ref<'l, T> {
    phantom: PhantomData<&'l ()>,
    object: T
}

impl<'l, T> Ref<'l, T> {
    pub fn new(t: T) -> Ref<'l, T> {
        Ref { phantom: PhantomData, object: t }
    }
}

impl<'l, T> Deref for Ref<'l, T> {
    type Target = T;

    fn deref<'a>(&'a self) -> &'a T {
        &self.object
    }
}
