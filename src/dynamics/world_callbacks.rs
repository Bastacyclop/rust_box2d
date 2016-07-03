use std::mem;
use std::any::Any;
use std::ops::*;
use std::marker::PhantomData;
use wrap::*;
use common::math::Vec2;
use common::settings::MAX_MANIFOLD_POINTS;
use collision::Manifold;
use dynamics::world::BodyHandle;
use dynamics::body::{Body, FixtureHandle};
use dynamics::fixture::Fixture;
use dynamics::contacts::Contact;
use user_data::{InternalUserData, RawUserData, RawUserDataMut, UserData, UserDataTypes};

pub trait ContactFilter<U: UserDataTypes>: Any {
    fn should_collide(&mut self, body_a: BodyAccess<U>, fixture_a: FixtureAccess<U>,
                                 body_b: BodyAccess<U>, fixture_b: FixtureAccess<U>) -> bool;
}

#[doc(hidden)]
pub struct ContactFilterLink {
    ptr: *mut ffi::ContactFilterLink,
    object: Option<Box<Any>>,
}

wrap! { ffi::ContactFilterLink => custom ContactFilterLink }

impl ContactFilterLink {
    pub unsafe fn new() -> Self {
        ContactFilterLink {
            ptr: ffi::ContactFilterLink_alloc(),
            object: None,
        }
    }
    
    pub unsafe fn use_with<F, U>(&mut self, mut filter: Box<F>) -> *mut ffi::ContactFilter
        where F: ContactFilter<U>, U: UserDataTypes
    {
        ffi::ContactFilterLink_bind(self.mut_ptr(),
                                    mem::transmute::<&mut F, _>(&mut *filter),
                                    cfl_should_collide::<F, U>);
        self.object = Some(filter);
        ffi::ContactFilterLink_as_base(self.mut_ptr())
    }
}

unsafe extern "C" fn cfl_should_collide<F, U>(object: ffi::Any,
                                              fixture_a: *mut ffi::Fixture,
                                              fixture_b: *mut ffi::Fixture)
                                              -> bool
        where F: ContactFilter<U>, U: UserDataTypes
{
    let filter = mem::transmute::<_, &mut F>(object);
    body_access(ffi::Fixture_get_body(fixture_a), |ba|
    fixture_access(fixture_a, |fa|
    body_access(ffi::Fixture_get_body(fixture_b), |bb|
    fixture_access(fixture_b, |fb|
        filter.should_collide(ba, fa, bb, fb)))))
}

impl Drop for ContactFilterLink {
    fn drop(&mut self) {
        unsafe { ffi::ContactFilterLink_drop(self.mut_ptr()) }
    }
}

#[repr(C)]
pub struct ContactImpulse {
    pub normal_impulses: [f32; MAX_MANIFOLD_POINTS],
    pub tangent_impulses: [f32; MAX_MANIFOLD_POINTS],
    pub count: i32,
}

pub trait ContactListener<U: UserDataTypes>: Any {
    fn begin_contact(&mut self, _: ContactAccess<U>) {}
    fn end_contact(&mut self, _: ContactAccess<U>) {}
    fn pre_solve(&mut self, _: ContactAccess<U>, _: &Manifold) {}
    fn post_solve(&mut self, _: ContactAccess<U>, _: &ContactImpulse) {}
}

#[doc(hidden)]
pub struct ContactListenerLink {
    ptr: *mut ffi::ContactListenerLink,
    object: Option<Box<Any>>,
}

wrap! { ffi::ContactListenerLink => custom ContactListenerLink }

impl ContactListenerLink {
    pub unsafe fn new() -> Self {
        ContactListenerLink {
            ptr: ffi::ContactListenerLink_alloc(),
            object: None,
        }
    }

    pub unsafe fn use_with<L, U>(&mut self, mut listener: Box<L>) -> *mut ffi::ContactListener
        where L: ContactListener<U>, U: UserDataTypes
    {
        ffi::ContactListenerLink_bind(self.mut_ptr(),
                                      mem::transmute::<&mut L, _>(&mut *listener),             cll_begin_contact::<L, U>,
                                      cll_end_contact::<L, U>,
                                      cll_pre_solve::<L, U>,
                                      cll_post_solve::<L, U>);
        self.object = Some(listener);
        ffi::ContactListenerLink_as_base(self.mut_ptr())
    }
}

unsafe extern "C" fn cll_begin_contact<L, U>(object: ffi::Any,
                                             contact: *mut ffi::Contact)
    where L: ContactListener<U>, U: UserDataTypes
{
    let listener = mem::transmute::<_, &mut L>(object);
    contact_access(contact, |c| listener.begin_contact(c))
}

unsafe extern "C" fn cll_end_contact<L, U>(object: ffi::Any,
                                           contact: *mut ffi::Contact)
    where L: ContactListener<U>, U: UserDataTypes
{
    let listener = mem::transmute::<_, &mut L>(object);
    contact_access(contact, |c| listener.end_contact(c))
}

unsafe extern "C" fn cll_pre_solve<L, U>(object: ffi::Any,
                                         contact: *mut ffi::Contact,
                                         old_manifold: *const Manifold)
    where L: ContactListener<U>, U: UserDataTypes
{
    assert!(!old_manifold.is_null());
    let listener = mem::transmute::<_, &mut L>(object);
    contact_access(contact, |c| listener.pre_solve(c, &*old_manifold))
}

unsafe extern "C" fn cll_post_solve<L, U>(object: ffi::Any,
                                          contact: *mut ffi::Contact,
                                          impulse: *const ContactImpulse)
    where L: ContactListener<U>, U: UserDataTypes
{
    assert!(!impulse.is_null());
    let listener = mem::transmute::<_, &mut L>(object);
    contact_access(contact, |c| listener.post_solve(c, &*impulse))
}

impl Drop for ContactListenerLink {
    fn drop(&mut self) {
        unsafe { ffi::ContactListenerLink_drop(self.mut_ptr()) }
    }
}

pub struct ContactAccess<'a, U: UserDataTypes> {
    pub contact: &'a mut Contact,
    pub body_a: BodyAccess<'a, U>,
    pub fixture_a: FixtureAccess<'a, U>,
    pub body_b: BodyAccess<'a, U>,
    pub fixture_b: FixtureAccess<'a, U>
}

#[inline(always)]
unsafe fn contact_access<F, O, U>(contact: *mut ffi::Contact, f: F) -> O
    where F: for<'a> FnOnce(ContactAccess<'a, U>) -> O,
          U: UserDataTypes
{
    let fixture_a = ffi::Contact_get_fixture_a(contact);
    let fixture_b = ffi::Contact_get_fixture_b(contact);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));

    body_access(ffi::Fixture_get_body(fixture_a), |ba|
    fixture_access(fixture_a, |fa|
    body_access(ffi::Fixture_get_body(fixture_b), |bb|
    fixture_access(fixture_b, |fb|
        f(ContactAccess {
            contact: &mut contact,
            body_a: ba,
            fixture_a: fa,
            body_b: bb,
            fixture_b: fb
        })))))
}

pub struct FixtureAccess<'a, U: UserDataTypes>(&'a mut Fixture, PhantomData<U>);

#[inline(always)]
unsafe fn fixture_access<F, O, U>(fixture: *mut ffi::Fixture, f: F) -> O
    where F: for<'a> FnOnce(FixtureAccess<'a, U>) -> O,
          U: UserDataTypes
{
    let mut fixture = WrappedRefMut::new(Fixture::from_ffi(fixture));
    f(FixtureAccess(&mut fixture, PhantomData))
}

impl<'a, U: UserDataTypes> UserData<U::FixtureData> for FixtureAccess<'a, U> {
    fn user_data(&self) -> &U::FixtureData {
        unsafe {
            let internal: &InternalUserData<(), U::FixtureData> =
                &*self.0.ptr().internal_user_data();
            &internal.custom
        }
    }

    fn user_data_mut(&mut self) -> &mut U::FixtureData {
        unsafe {
            let internal: &mut InternalUserData<(), U::FixtureData> =
                &mut *self.0.mut_ptr().internal_user_data_mut();
            &mut internal.custom
        }
    }
}

impl<'a, U: UserDataTypes> Deref for FixtureAccess<'a, U> {
    type Target = Fixture;

    fn deref(&self) -> &Fixture {
        &self.0
    }
}

impl<'a, U: UserDataTypes> DerefMut for FixtureAccess<'a, U> {
    fn deref_mut(&mut self) -> &mut Fixture {
        &mut self.0
    }
}

pub struct BodyAccess<'a, U: UserDataTypes>(&'a mut Body, PhantomData<U>);

#[inline(always)]
unsafe fn body_access<F, O, U>(body: *mut ffi::Body, f: F) -> O
    where F: for<'a> FnOnce(BodyAccess<'a, U>) -> O,
          U: UserDataTypes
{
    let mut body = WrappedRefMut::new(Body::from_ffi(body));
    f(BodyAccess(&mut body, PhantomData))
}

impl<'a, U: UserDataTypes> UserData<U::BodyData> for BodyAccess<'a, U> {
    fn user_data(&self) -> &U::BodyData {
        unsafe {
            let internal: &InternalUserData<(), U::BodyData> =
                &*self.0.ptr().internal_user_data();
            &internal.custom
        }
    }

    fn user_data_mut(&mut self) -> &mut U::BodyData {
        unsafe {
            let internal: &mut InternalUserData<(), U::BodyData> =
                &mut *self.0.mut_ptr().internal_user_data_mut();
            &mut internal.custom
        }
    }
}

impl<'a, U: UserDataTypes> Deref for BodyAccess<'a, U> {
    type Target = Body;

    fn deref(&self) -> &Body {
        &self.0
    }
}

impl<'a, U: UserDataTypes> DerefMut for BodyAccess<'a, U> {
    fn deref_mut(&mut self) -> &mut Body {
        &mut self.0
    }
}

pub trait QueryCallback {
    fn report_fixture(&mut self, body: BodyHandle, fixture: FixtureHandle) -> bool;
}

impl<F> QueryCallback for F
    where F: FnMut(BodyHandle, FixtureHandle) -> bool
{
    fn report_fixture(&mut self, body: BodyHandle, fixture: FixtureHandle) -> bool {
        self(body, fixture)
    }
}

wrap! { ffi::QueryCallbackLink => #[doc(hidden)] pub QueryCallbackLink }

impl QueryCallbackLink {
    pub unsafe fn new() -> Self {
        QueryCallbackLink::from_ffi(ffi::QueryCallbackLink_alloc())
    }

    pub unsafe fn use_with<C: QueryCallback>(&mut self, callback: &mut C) -> *mut ffi::QueryCallback {
        ffi::QueryCallbackLink_bind(self.mut_ptr(),
                                    mem::transmute(callback),
                                    qcl_report_fixture::<C>);
        ffi::QueryCallbackLink_as_base(self.mut_ptr())
    }
}

unsafe extern "C" fn qcl_report_fixture<C: QueryCallback>(object: ffi::Any,
                                                          fixture: *mut ffi::Fixture)
                                                          -> bool {
    let callback = mem::transmute::<_, &mut C>(object);
    let body_handle = WrappedRef::new(Fixture::from_ffi(fixture)).body();
    callback.report_fixture(body_handle, fixture.handle())
}

impl Drop for QueryCallbackLink {
    fn drop(&mut self) {
        unsafe { ffi::QueryCallbackLink_drop(self.ptr) }
    }
}

pub trait RayCastCallback {
    fn report_fixture(&mut self,
                      body: BodyHandle,
                      fixture: FixtureHandle,
                      p: &Vec2,
                      normal: &Vec2,
                      fraction: f32)
                      -> f32;
}

impl<F> RayCastCallback for F
    where F: FnMut(BodyHandle, FixtureHandle, &Vec2, &Vec2, f32) -> f32
{
    fn report_fixture(&mut self,
                      body: BodyHandle,
                      fixture: FixtureHandle,
                      p: &Vec2,
                      normal: &Vec2,
                      fraction: f32)
                      -> f32 {
        self(body, fixture, p, normal, fraction)
    }
}

wrap! { ffi::RayCastCallbackLink => #[doc(hidden)] pub RayCastCallbackLink }

impl RayCastCallbackLink {
    pub unsafe fn new() -> Self {
        RayCastCallbackLink::from_ffi(ffi::RayCastCallbackLink_alloc())
    }

    pub unsafe fn use_with<C: RayCastCallback>(&mut self, callback: &mut C) -> *mut ffi::RayCastCallback {
        ffi::RayCastCallbackLink_bind(self.mut_ptr(),
                                      mem::transmute(callback),
                                      rccl_report_fixture::<C>);
        ffi::RayCastCallbackLink_as_base(self.mut_ptr())
    }
}

unsafe extern "C" fn rccl_report_fixture<C: RayCastCallback>(object: ffi::Any,
                                                             fixture: *mut ffi::Fixture,
                                                             point: *const Vec2,
                                                             normal: *const Vec2,
                                                             fraction: f32)
                                                             -> f32 {
    // point and normal are coming from C++ &s
    let callback = mem::transmute::<_, &mut C>(object);
    let body_handle = WrappedRef::new(Fixture::from_ffi(fixture)).body();
    callback.report_fixture(body_handle, fixture.handle(), &*point, &*normal, fraction)
}

impl Drop for RayCastCallbackLink {
    fn drop(&mut self) {
        unsafe { ffi::RayCastCallbackLink_drop(self.mut_ptr()) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use ffi::Any;
    pub use collision::shapes::ffi::Shape;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::fixture::ffi::{Fixture, Fixture_get_body};
    pub use dynamics::joints::ffi::Joint;
    pub use dynamics::contacts::ffi::{Contact, Contact_get_fixture_a, Contact_get_fixture_b};
    use common::math::Vec2;
    use collision::Manifold;
    use super::ContactImpulse;

    pub enum ContactFilter {}
    pub enum ContactFilterLink {}
    pub enum ContactListener {}
    pub enum ContactListenerLink {}
    pub enum QueryCallback {}
    pub enum QueryCallbackLink {}
    pub enum RayCastCallback {}
    pub enum RayCastCallbackLink {}

    extern "C" {
        pub fn ContactFilterLink_alloc() -> *mut ContactFilterLink;
        pub fn ContactFilterLink_bind(slf: *mut ContactFilterLink,
                                      object: Any,
                                      should_collide: unsafe extern "C" fn(Any,
                                                                           *mut Fixture,
                                                                           *mut Fixture)
                                                                           -> bool);
        pub fn ContactFilterLink_as_base(slf: *mut ContactFilterLink) -> *mut ContactFilter;
        pub fn ContactFilterLink_drop(slf: *mut ContactFilterLink);
        pub fn ContactListenerLink_alloc() -> *mut ContactListenerLink;
        pub fn ContactListenerLink_bind(slf: *mut ContactListenerLink,
                                        object: Any,
                                        begin_contact: unsafe extern "C" fn(Any, *mut  Contact),
                                        end_contact: unsafe extern "C" fn(Any, *mut  Contact),
                                        pre_solve: unsafe extern "C" fn(Any,
                                                                        *mut Contact,
                                                                        *const Manifold)
                                                                       ,
                                        post_solve: unsafe extern "C" fn(Any,
                                                                        *mut Contact,
                                                                        *const ContactImpulse));
        pub fn ContactListenerLink_as_base(slf: *mut ContactListenerLink) -> *mut ContactListener;
        pub fn ContactListenerLink_drop(slf: *mut ContactListenerLink);
        pub fn QueryCallbackLink_alloc() -> *mut QueryCallbackLink;
        pub fn QueryCallbackLink_bind(slf: *mut QueryCallbackLink,
                                      object: Any,
                                      report_fixture: unsafe extern "C" fn(Any, *mut  Fixture)
                                                                           -> bool);
        pub fn QueryCallbackLink_as_base(slf: *mut QueryCallbackLink) -> *mut QueryCallback;
        pub fn QueryCallbackLink_drop(slf: *mut QueryCallbackLink);
        pub fn RayCastCallbackLink_alloc() -> *mut RayCastCallbackLink;
        pub fn RayCastCallbackLink_bind(slf: *mut RayCastCallbackLink,
                                        object: Any,
                                        hit_fixture: unsafe extern "C" fn(Any,
                                                                          *mut Fixture,
                                                                          *const Vec2,
                                                                          *const Vec2,
                                                                          f32)
                                                                          -> f32);
        pub fn RayCastCallbackLink_as_base(slf: *mut RayCastCallbackLink) -> *mut RayCastCallback;
        pub fn RayCastCallbackLink_drop(slf: *mut RayCastCallbackLink);
    }
}
