use std::mem;
use std::any::Any;
use std::ops::{ Deref, DerefMut };
use ffi;
use wrap::*;
use handle::*;
use b2::{ Body, UnknownJoint, Joint, Fixture };

pub struct InternalUserData<O> {
    pub handle: TypedHandle<O>,
    pub custom: Option<Box<Any>>
}

pub trait RawUserData<R> {
    type Internal: Sized;

    unsafe fn get_internal_user_data(ptr: *const R) -> *const InternalUserData<Self::Internal>;

    unsafe fn get_handle(ptr: *const R) -> TypedHandle<Self::Internal> {
        let internal = &*<Self as RawUserData<_>>::get_internal_user_data(ptr);
        internal.handle
    }
}

pub trait UserData {
    fn get_user_data(&self) -> &Option<Box<Any>>;

    #[allow(mutable_transmutes)]
    fn get_user_data_mut(&mut self) -> &mut Option<Box<Any>> {
        unsafe {
            mem::transmute(self.get_user_data())
        }
    }

    fn set_user_data(&mut self, v: Option<Box<Any>>) {
        *self.get_user_data_mut() = v;
    }
}

impl RawUserData<ffi::Body> for Body {
    type Internal = Body;

    unsafe fn get_internal_user_data(ptr: *const ffi::Body) -> *const InternalUserData<Body> {
        ffi::Body_get_user_data(ptr) as *const InternalUserData<Body>
    }
}

impl UserData for Body {
    fn get_user_data(&self) -> &Option<Box<Any>> {
        unsafe {
            let internal = &*<Body as RawUserData<_>>::get_internal_user_data(self.ptr());
            &internal.custom
        }
    }
}

impl<J: Joint> RawUserData<ffi::Joint> for J {
    type Internal = Meta<UnknownJoint>;

    unsafe fn get_internal_user_data(ptr: *const ffi::Joint) -> *const InternalUserData<Meta<UnknownJoint>> {
        ffi::Joint_get_user_data(ptr) as *const InternalUserData<Meta<UnknownJoint>>
    }
}

impl<T: Joint> UserData for T {
    fn get_user_data(&self) -> &Option<Box<Any>> {
        unsafe {
            let internal = &*<T as RawUserData<_>>::get_internal_user_data(self.base_ptr());
            &internal.custom
        }
    }
}

impl RawUserData<ffi::Fixture> for Fixture {
    type Internal = Meta<Fixture>;

    unsafe fn get_internal_user_data(ptr: *const ffi::Fixture) -> *const InternalUserData<Meta<Fixture>> {
        ffi::Fixture_get_user_data(ptr) as *const InternalUserData<Meta<Fixture>>
    }
}

impl UserData for Fixture {
    fn get_user_data(&self) -> &Option<Box<Any>> {
        unsafe {
            let internal = &*<Fixture as RawUserData<_>>::get_internal_user_data(self.ptr());
            &internal.custom
        }
    }
}

pub struct Meta<T> {
    pub object: T,
    pub user_data: Box<InternalUserData<Meta<T>>>
}

impl<T> Deref for Meta<T> {
    type Target = T;

    fn deref(&self) -> &T { &self.object }
}

impl<T> DerefMut for Meta<T> {
    fn deref_mut(&mut self) -> &mut T { &mut self.object }
}
