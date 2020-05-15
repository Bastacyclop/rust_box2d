use std::mem;
use wrap::*;
use common::math::Transform;
use collision::{Manifold, WorldManifold};
use dynamics::body::FixtureHandle;
use dynamics::fixture::Fixture;
use dynamics::world::BodyHandle;
use user_data::RawUserData;

#[repr(C)]
#[doc(hidden)]
pub struct ContactEdge {
    pub other: *mut ffi::Body,
    pub contact: *mut ffi::Contact,
    pub prev: *mut ContactEdge,
    pub next: *mut ContactEdge,
}

wrap! { ffi::Contact => pub Contact }

impl Contact {
    pub fn manifold<'a>(&'a self) -> &'a Manifold {
        unsafe { &*ffi::Contact_get_manifold_const(self.ptr()) }
    }

    pub fn manifold_mut<'a>(&'a mut self) -> &'a mut Manifold {
        unsafe { &mut *ffi::Contact_get_manifold(self.mut_ptr()) }
    }

    pub fn world_manifold<'a>(&'a self) -> WorldManifold {
        unsafe {
            let mut m = mem::MaybeUninit::uninit();
            ffi::Contact_get_world_manifold(self.ptr(), m.as_mut_ptr());
            m.assume_init()
        }
    }

    pub fn is_touching(&self) -> bool {
        unsafe { ffi::Contact_is_touching(self.ptr()) }
    }

    pub fn is_enabled(&self) -> bool {
        unsafe { ffi::Contact_is_enabled(self.ptr()) }
    }

    pub fn fixture_a(&self) -> (BodyHandle, FixtureHandle) {
        unsafe {
            let fixture = ffi::Contact_get_fixture_a_const(self.ptr()) as *mut _;
            let body_handle = WrappedRef::new(Fixture::from_ffi(fixture)).body();
            (body_handle, fixture.handle())
        }
    }

    pub fn child_index_a(&self) -> i32 {
        unsafe { ffi::Contact_get_child_index_a(self.ptr()) }
    }

    pub fn fixture_b(&self) -> (BodyHandle, FixtureHandle) {
        unsafe {
            let fixture = ffi::Contact_get_fixture_b_const(self.ptr()) as *mut _;
            let body_handle = WrappedRef::new(Fixture::from_ffi(fixture)).body();
            (body_handle, fixture.handle())
        }
    }

    pub fn child_index_b(&self) -> i32 {
        unsafe { ffi::Contact_get_child_index_b(self.ptr()) }
    }

    pub fn set_friction(&mut self, friction: f32) {
        unsafe { ffi::Contact_set_friction(self.mut_ptr(), friction) }
    }

    pub fn friction(&self) -> f32 {
        unsafe { ffi::Contact_get_friction(self.ptr()) }
    }

    pub fn reset_friction(&mut self) {
        unsafe { ffi::Contact_reset_friction(self.mut_ptr()) }
    }

    pub fn set_restitution(&mut self, restitution: f32) {
        unsafe { ffi::Contact_set_restitution(self.mut_ptr(), restitution) }
    }

    pub fn restitution(&self) -> f32 {
        unsafe { ffi::Contact_get_restitution(self.ptr()) }
    }

    pub fn reset_restitution(&mut self) {
        unsafe { ffi::Contact_reset_restitution(self.mut_ptr()) }
    }

    pub fn set_tangent_speed(&mut self, speed: f32) {
        unsafe { ffi::Contact_set_tangent_speed(self.mut_ptr(), speed) }
    }

    pub fn tangent_speed(&self) -> f32 {
        unsafe { ffi::Contact_get_tangent_speed(self.ptr()) }
    }

    pub fn evaluate(&mut self, xf_a: &Transform, xf_b: &Transform) -> Manifold {
        unsafe {
            let mut m = mem::MaybeUninit::uninit();
            ffi::Contact_evaluate_virtual(self.mut_ptr(), m.as_mut_ptr(), xf_a, xf_b);
            m.assume_init()
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::body::ffi::Body;
    pub use dynamics::fixture::ffi::Fixture;
    use collision::{Manifold, WorldManifold};
    use common::math::Transform;

    pub enum Contact {}

    extern "C" {
        pub fn Contact_get_manifold(slf: *mut Contact) -> *mut Manifold;
        pub fn Contact_get_manifold_const(slf: *const Contact) -> *const Manifold;
        pub fn Contact_get_world_manifold(slf: *const Contact, wm: *mut WorldManifold);
        pub fn Contact_is_touching(slf: *const Contact) -> bool;
        pub fn Contact_is_enabled(slf: *const Contact) -> bool;
        pub fn Contact_get_next(slf: *mut Contact) -> *mut Contact;
        pub fn Contact_get_next_const(slf: *const Contact) -> *const Contact;
        pub fn Contact_get_fixture_a(slf: *mut Contact) -> *mut Fixture;
        pub fn Contact_get_fixture_a_const(slf: *const Contact) -> *const Fixture;
        pub fn Contact_get_child_index_a(slf: *const Contact) -> i32;
        pub fn Contact_get_fixture_b(slf: *mut Contact) -> *mut Fixture;
        pub fn Contact_get_fixture_b_const(slf: *const Contact) -> *const Fixture;
        pub fn Contact_get_child_index_b(slf: *const Contact) -> i32;
        pub fn Contact_set_friction(slf: *mut Contact, friction: f32);
        pub fn Contact_get_friction(slf: *const Contact) -> f32;
        pub fn Contact_reset_friction(slf: *mut Contact);
        pub fn Contact_set_restitution(slf: *mut Contact, restitution: f32);
        pub fn Contact_get_restitution(slf: *const Contact) -> f32;
        pub fn Contact_reset_restitution(slf: *mut Contact);
        pub fn Contact_set_tangent_speed(slf: *mut Contact, speed: f32);
        pub fn Contact_get_tangent_speed(slf: *const Contact) -> f32;
        pub fn Contact_evaluate_virtual(slf: *mut Contact,
                                        m: *mut Manifold,
                                        xf_a: *const Transform,
                                        xf_b: *const Transform);
    }
}
