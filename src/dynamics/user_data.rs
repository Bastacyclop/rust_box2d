use std::any::Any;
use handle::*;
use dynamics::body::MetaBody;
use dynamics::fixture::MetaFixture;
use dynamics::joints::MetaJoint;

#[doc(hidden)]
pub struct InternalUserData<O> {
    pub handle: TypedHandle<O>,
    pub custom: Option<Box<Any>>,
}

#[doc(hidden)]
pub trait RawUserData: Sized {
    type Internal: Sized;

    unsafe fn get_internal_user_data(self) -> *const InternalUserData<Self::Internal>;

    unsafe fn get_handle(self) -> TypedHandle<Self::Internal> {
        let internal = &*self.get_internal_user_data();
        internal.handle
    }
}

#[doc(hidden)]
pub trait RawUserDataMut: RawUserData {
    unsafe fn set_internal_user_data(self, *mut InternalUserData<Self::Internal>);
}

macro_rules! impl_raw_user_data {
    { $raw:ty, $internal:ty, $getter:path, $setter:path } => {
        impl RawUserData for *const $raw {
            type Internal = $internal;

            unsafe fn get_internal_user_data(self) -> *const InternalUserData<Self::Internal> {
                $getter(self) as *const InternalUserData<Self::Internal>
            }
        }

        impl RawUserData for *mut $raw {
            type Internal = $internal;

            unsafe fn get_internal_user_data(self) -> *const InternalUserData<Self::Internal> {
                $getter(self) as *const InternalUserData<Self::Internal>
            }
        }

        impl RawUserDataMut for *mut $raw {
            unsafe fn set_internal_user_data(self, data: *mut InternalUserData<Self::Internal>) {
                $setter(self, data as ffi::Any)
            }
        }
    };
}

impl_raw_user_data! {
    ffi::Body, MetaBody,
    ffi::Body_get_user_data, ffi::Body_set_user_data
}

impl_raw_user_data! {
    ffi::Joint, MetaJoint,
    ffi::Joint_get_user_data, ffi::Joint_set_user_data
}

impl_raw_user_data! {
    ffi::Fixture, MetaFixture,
    ffi::Fixture_get_user_data, ffi::Fixture_set_user_data
}

pub trait UserData {
    fn get_user_data(&self) -> &Option<Box<Any>>;
    fn get_user_data_mut(&mut self) -> &mut Option<Box<Any>>;

    fn set_user_data(&mut self, v: Option<Box<Any>>) {
        *self.get_user_data_mut() = v;
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use ffi::Any;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::fixture::ffi::Fixture;
    pub use dynamics::joints::ffi::Joint;

    extern "C" {
        pub fn Body_get_user_data(slf: *const Body) -> Any;
        pub fn Body_set_user_data(slf: *mut Body, data: Any);
        pub fn Fixture_get_user_data(slf: *const Fixture) -> Any;
        pub fn Fixture_set_user_data(slf: *mut Fixture, data: Any);
        pub fn Joint_get_user_data(slf: *const Joint) -> Any;
        pub fn Joint_set_user_data(slf: *mut Joint, data: Any);
    }
}
