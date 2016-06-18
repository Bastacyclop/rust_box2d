use handle::*;

pub trait UserDataTypes {
    type BodyData: Sized;
    type JointData: Sized;
    type FixtureData: Sized;
}

pub struct NoUserData;

impl UserDataTypes for NoUserData {
    type BodyData = ();
    type JointData = ();
    type FixtureData = ();
}

#[doc(hidden)]
pub struct InternalUserData<T: ?Sized, U> {
    pub handle: TypedHandle<T>,
    pub custom: U,
}

#[doc(hidden)]
pub trait RawUserData: Sized {
    unsafe fn get_internal_user_data<T: ?Sized, U>(self) -> *const InternalUserData<T, U>;

    unsafe fn get_handle<T: ?Sized>(self) -> TypedHandle<T> {
        let internal = &*self.get_internal_user_data::<_, ()>();
        internal.handle
    }
}

#[doc(hidden)]
pub trait RawUserDataMut: RawUserData {
    unsafe fn set_internal_user_data<T: ?Sized, U>(self, *mut InternalUserData<T, U>);
}

macro_rules! impl_raw_user_data {
    { $raw:ty, $getter:path, $setter:path } => {
        impl RawUserData for *const $raw {
            unsafe fn get_internal_user_data<T: ?Sized, U>(self) -> *const InternalUserData<T, U> {
                $getter(self) as *const InternalUserData<T, U>
            }
        }

        impl RawUserData for *mut $raw {
            unsafe fn get_internal_user_data<T: ?Sized, U>(self) -> *const InternalUserData<T, U> {
                $getter(self) as *const InternalUserData<T, U>
            }
        }

        impl RawUserDataMut for *mut $raw {
            unsafe fn set_internal_user_data<T: ?Sized, U>(self,
                                                           data: *mut InternalUserData<T, U>) {
                $setter(self, data as ffi::Any)
            }
        }
    };
}

impl_raw_user_data! {
    ffi::Body, ffi::Body_get_user_data, ffi::Body_set_user_data
}

impl_raw_user_data! {
    ffi::Joint, ffi::Joint_get_user_data, ffi::Joint_set_user_data
}

impl_raw_user_data! {
    ffi::Fixture, ffi::Fixture_get_user_data, ffi::Fixture_set_user_data
}

pub trait UserData<U> {
    fn get_user_data(&self) -> &U;
    fn get_user_data_mut(&mut self) -> &mut U;

    fn set_user_data(&mut self, v: U) {
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
