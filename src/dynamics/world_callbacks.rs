use std::mem;
use wrap::*;
use common::math::{ Vec2 };
use common::settings::MAX_MANIFOLD_POINTS;
use collision::Manifold;
use dynamics::world::BodyHandle;
use dynamics::body::FixtureHandle;
use dynamics::fixture::Fixture;
use dynamics::contacts::Contact;
use dynamics::user_data::RawUserData;

pub trait ContactFilter {
    fn should_collide(&mut self, fixture_a: &mut Fixture,
                                 fixture_b: &mut Fixture) -> bool;
}

#[doc(hidden)]
pub struct ContactFilterLink {
    ptr: *mut ffi::ContactFilterLink,
    object: Option<Box<ContactFilter>>,
}

wrap! { ffi::ContactFilterLink => custom ContactFilterLink }

impl ContactFilterLink {
    pub unsafe fn new() -> ContactFilterLink {
        ContactFilterLink {
            ptr: ffi::ContactFilterLink_new(ffi::FatAny::null(),
                                            cfl_should_collide),
            object: None
        }
    }

    pub unsafe fn set_object(&mut self, object: Box<ContactFilter>) {
        self.object = Some(object);
        ffi::ContactFilterLink_set_object(
            self.mut_ptr(),
            mem::transmute::<&mut ContactFilter, _>(
                &mut **self.object.as_mut().unwrap()
            )
        )
    }

    pub unsafe fn filter_ptr(&mut self) -> *mut ffi::ContactFilter {
        ffi::ContactFilterLink_as_base(self.mut_ptr())
    }
}

unsafe extern fn cfl_should_collide(any: ffi::FatAny, fixture_a: *mut ffi::Fixture,
                                    fixture_b: *mut ffi::Fixture) -> bool {
    let filter = mem::transmute::<_, &mut ContactFilter>(any);
    let mut fixture_a = WrappedRefMut::new(Fixture::from_ffi(fixture_a));
    let mut fixture_b = WrappedRefMut::new(Fixture::from_ffi(fixture_b));
    filter.should_collide(&mut fixture_a, &mut fixture_b)
}

impl Drop for ContactFilterLink {
    fn drop(&mut self) {
        unsafe {
            ffi::ContactFilterLink_drop(self.mut_ptr())
        }
    }
}

#[repr(C)]
pub struct ContactImpulse {
    pub normal_impulses: [f32; MAX_MANIFOLD_POINTS],
    pub tangent_impulses: [f32; MAX_MANIFOLD_POINTS],
    pub count: i32
}

pub trait ContactListener {
    fn begin_contact(&mut self, contact: &mut Contact);
    fn end_contact(&mut self, contact: &mut Contact);
    fn pre_solve(&mut self, contact: &mut Contact, manifold: &Manifold);
    fn post_solve(&mut self, contact: &mut Contact, impulse: &ContactImpulse);
}

#[doc(hidden)]
pub struct ContactListenerLink {
    ptr: *mut ffi::ContactListenerLink,
    object: Option<Box<ContactListener>>,
}

wrap! { ffi::ContactListenerLink => custom ContactListenerLink }

impl ContactListenerLink {
    pub unsafe fn new() -> ContactListenerLink {
        ContactListenerLink {
            ptr: ffi::ContactListenerLink_new(ffi::FatAny::null(),
                                              cll_begin_contact,
                                              cll_end_contact,
                                              cll_pre_solve,
                                              cll_post_solve),
            object: None
        }
    }

    pub unsafe fn set_object(&mut self, object: Box<ContactListener>) {
        self.object = Some(object);
        ffi::ContactListenerLink_set_object(
            self.mut_ptr(),
            mem::transmute::<&mut ContactListener, _>(
                &mut **self.object.as_mut().unwrap()
            )
        )
    }

    pub unsafe fn listener_ptr(&mut self) -> *mut ffi::ContactListener {
        ffi::ContactListenerLink_as_base(self.mut_ptr())
    }
}

unsafe extern fn cll_begin_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.begin_contact(&mut contact)
}

unsafe extern fn cll_end_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.end_contact(&mut contact)
}

unsafe extern fn cll_pre_solve(any: ffi::FatAny,
                               contact: *mut ffi::Contact,
                               old_manifold: *const Manifold) {
    assert!(!old_manifold.is_null());
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.pre_solve(&mut contact, &*old_manifold)
}

unsafe extern fn cll_post_solve(any: ffi::FatAny,
                                contact: *mut ffi::Contact,
                                impulse: *const ContactImpulse) {
    assert!(!impulse.is_null());
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.post_solve(&mut contact, &*impulse)
}

impl Drop for ContactListenerLink {
    fn drop(&mut self) {
        unsafe {
            ffi::ContactListenerLink_drop(self.mut_ptr())
        }
    }
}

pub trait QueryCallback {
    fn report_fixture(&mut self, body: BodyHandle, fixture: FixtureHandle) -> bool;
}

impl<F> QueryCallback for F
    where F: FnMut(BodyHandle, FixtureHandle) -> bool {

    fn report_fixture(&mut self, body: BodyHandle, fixture: FixtureHandle) -> bool {
        self(body, fixture)
    }
}

wrap! { ffi::QueryCallbackLink => #[doc(hidden)] pub QueryCallbackLink }

impl QueryCallbackLink {
    pub unsafe fn new() -> QueryCallbackLink {
        QueryCallbackLink::from_ffi(
            ffi::QueryCallbackLink_new(ffi::FatAny::null(),
                                       qcl_report_fixture)
        )
    }

    pub unsafe fn use_with(&mut self,
                           callback: &mut QueryCallback) -> *mut ffi::QueryCallback {
        ffi::QueryCallbackLink_set_object(self.mut_ptr(), mem::transmute(callback));
        ffi::QueryCallbackLink_as_base(self.mut_ptr())
    }
}

unsafe extern fn qcl_report_fixture(any: ffi::FatAny,
                                    fixture: *mut ffi::Fixture) -> bool {
    let callback = mem::transmute::<_, &mut QueryCallback>(any);
    let body = WrappedRef::new(Fixture::from_ffi(fixture)).body();
    callback.report_fixture(body, fixture.get_handle())
}

impl Drop for QueryCallbackLink {
    fn drop(&mut self) {
        unsafe {
            ffi::QueryCallbackLink_drop(self.ptr)
        }
    }
}

pub trait RayCastCallback {
    fn report_fixture(&mut self, fixture: FixtureHandle,
                      p: &Vec2, normal: &Vec2, fraction: f32) -> f32;
}

impl<F> RayCastCallback for F
    where F: FnMut(FixtureHandle, &Vec2, &Vec2, f32) -> f32 {

    fn report_fixture(&mut self, fixture: FixtureHandle,
                      p: &Vec2, normal: &Vec2, fraction: f32) -> f32 {
        self(fixture, p, normal, fraction)
    }
}

wrap! { ffi::RayCastCallbackLink => #[doc(hidden)] pub RayCastCallbackLink }

impl RayCastCallbackLink {
    pub unsafe fn new() -> RayCastCallbackLink {
        RayCastCallbackLink::from_ffi(
            ffi::RayCastCallbackLink_new(ffi::FatAny::null(),
                                         rccl_report_fixture)
        )
    }

    pub unsafe fn use_with(&mut self,
                           callback: &mut RayCastCallback
                           ) -> *mut ffi::RayCastCallback {
        ffi::RayCastCallbackLink_set_object(self.mut_ptr(), mem::transmute(callback));
        ffi::RayCastCallbackLink_as_base(self.mut_ptr())
    }
}

unsafe extern fn rccl_report_fixture(any: ffi::FatAny,
                                     fixture: *mut ffi::Fixture,
                                     point: *const Vec2, normal: *const Vec2,
                                     fraction: f32) -> f32 {
    // point and normal are coming from C++ &s
    let callback = mem::transmute::<_, &mut RayCastCallback>(any);
    callback.report_fixture(fixture.get_handle(), &*point, &*normal, fraction)
}

impl Drop for RayCastCallbackLink {
    fn drop(&mut self) {
        unsafe {
            ffi::RayCastCallbackLink_drop(self.mut_ptr())
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use ffi::FatAny;
    pub use collision::shapes::ffi::Shape;
    pub use dynamics::fixture::ffi::Fixture;
    pub use dynamics::joints::ffi::Joint;
    pub use dynamics::contacts::ffi::Contact;
    use common::math::Vec2;
    use collision::Manifold;
    use super::ContactImpulse;

    #[repr(C)] pub struct ContactFilter;
    #[repr(C)] pub struct ContactFilterLink;
    #[repr(C)] pub struct ContactListener;
    #[repr(C)] pub struct ContactListenerLink;
    #[repr(C)] pub struct QueryCallback;
    #[repr(C)] pub struct QueryCallbackLink;
    #[repr(C)] pub struct RayCastCallback;
    #[repr(C)] pub struct RayCastCallbackLink;

    extern {
        pub fn ContactFilterLink_new(
            object: FatAny,
            should_collide:
            unsafe extern fn(FatAny, *mut Fixture, *mut Fixture) -> bool
            ) -> *mut ContactFilterLink;
        pub fn ContactFilterLink_set_object(slf: *mut ContactFilterLink,
                                            object: FatAny);
        pub fn ContactFilterLink_as_base(slf: *mut ContactFilterLink
                                      ) -> *mut ContactFilter;
        pub fn ContactFilterLink_drop(slf: *mut ContactFilterLink);
        pub fn ContactListenerLink_new(
            object: FatAny,
            begin_contact:
            unsafe extern fn(FatAny, *mut Contact),
            end_contact:
            unsafe extern fn(FatAny, *mut Contact),
            pre_solve:
            unsafe extern fn(FatAny, *mut Contact, *const Manifold),
            post_solve:
            unsafe extern fn(FatAny, *mut Contact, *const ContactImpulse),
            ) -> *mut ContactListenerLink;
        pub fn ContactListenerLink_set_object(slf: *mut ContactListenerLink,
                                              object: FatAny);
        pub fn ContactListenerLink_as_base(slf: *mut ContactListenerLink
                                        ) -> *mut ContactListener;
        pub fn ContactListenerLink_drop(slf: *mut ContactListenerLink);
        pub fn QueryCallbackLink_new(
            object: FatAny,
            report_fixture:
            unsafe extern fn(FatAny, *mut Fixture) -> bool,
            ) -> *mut QueryCallbackLink;
        pub fn QueryCallbackLink_set_object(slf: *mut QueryCallbackLink,
                                            object: FatAny);
        pub fn QueryCallbackLink_as_base(slf: *mut QueryCallbackLink
                                      ) -> *mut QueryCallback;
        pub fn QueryCallbackLink_drop(slf: *mut QueryCallbackLink);
        pub fn RayCastCallbackLink_new(
            object: FatAny,
            hit_fixture:
            unsafe extern fn(FatAny,
                             *mut Fixture, *const Vec2, *const Vec2, f32) -> f32,
            ) -> *mut RayCastCallbackLink;
        pub fn RayCastCallbackLink_set_object(slf: *mut RayCastCallbackLink,
                                              object: FatAny);
        pub fn RayCastCallbackLink_as_base(slf: *mut RayCastCallbackLink
                                        ) -> *mut RayCastCallback;
        pub fn RayCastCallbackLink_drop(slf: *mut RayCastCallbackLink);
    }
}
