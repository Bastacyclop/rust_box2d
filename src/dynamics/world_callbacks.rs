use std::mem;
use std::any::Any;
use wrap::*;
use common::math::Vec2;
use common::settings::MAX_MANIFOLD_POINTS;
use collision::Manifold;
use dynamics::world::BodyHandle;
use dynamics::body::FixtureHandle;
use dynamics::fixture::Fixture;
use dynamics::contacts::Contact;
use user_data::RawUserData;

pub trait ContactFilter: Any {
    fn should_collide(&mut self, fixture_a: &mut Fixture, fixture_b: &mut Fixture) -> bool;
}

#[doc(hidden)]
pub struct ContactFilterLink {
    ptr: *mut ffi::ContactFilterLink,
    object: Option<Box<Any>>,
}

wrap! { ffi::ContactFilterLink => custom ContactFilterLink }

impl ContactFilterLink {
    pub unsafe fn new() -> ContactFilterLink {
        ContactFilterLink {
            ptr: ffi::ContactFilterLink_alloc(),
            object: None,
        }
    }

    pub unsafe fn use_with<F: ContactFilter>(&mut self, mut filter: Box<F>) -> *mut ffi::ContactFilter {
        ffi::ContactFilterLink_bind(self.mut_ptr(),
                                    mem::transmute::<&mut F, _>(&mut *filter),
                                    cfl_should_collide::<F>);
        self.object = Some(filter);
        ffi::ContactFilterLink_as_base(self.mut_ptr())
    }
}

unsafe extern "C" fn cfl_should_collide<F: ContactFilter>(object: ffi::Any,
                                                          fixture_a: *mut ffi::Fixture,
                                                          fixture_b: *mut ffi::Fixture)
                                                          -> bool {
    let filter = mem::transmute::<_, &mut F>(object);
    let mut fixture_a = WrappedRefMut::new(Fixture::from_ffi(fixture_a));
    let mut fixture_b = WrappedRefMut::new(Fixture::from_ffi(fixture_b));
    filter.should_collide(&mut fixture_a, &mut fixture_b)
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

pub trait ContactListener: Any {
    fn begin_contact(&mut self, _: &mut Contact) {}
    fn end_contact(&mut self, _: &mut Contact) {}
    fn pre_solve(&mut self, _: &mut Contact, _: &Manifold) {}
    fn post_solve(&mut self, _: &mut Contact, _: &ContactImpulse) {}
}

#[doc(hidden)]
pub struct ContactListenerLink {
    ptr: *mut ffi::ContactListenerLink,
    object: Option<Box<Any>>,
}

wrap! { ffi::ContactListenerLink => custom ContactListenerLink }

impl ContactListenerLink {
    pub unsafe fn new() -> ContactListenerLink {
        ContactListenerLink {
            ptr: ffi::ContactListenerLink_alloc(),
            object: None,
        }
    }

    pub unsafe fn use_with<L: ContactListener>(&mut self, mut listener: Box<L>) -> *mut ffi::ContactListener {
        ffi::ContactListenerLink_bind(self.mut_ptr(),
                                      mem::transmute::<&mut L, _>(&mut *listener),                   cll_begin_contact::<L>,
                                      cll_end_contact::<L>,
                                      cll_pre_solve::<L>,
                                      cll_post_solve::<L>);
        self.object = Some(listener);
        ffi::ContactListenerLink_as_base(self.mut_ptr())
    }
}

unsafe extern "C" fn cll_begin_contact<L: ContactListener>(object: ffi::Any,
                                                           contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut L>(object);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.begin_contact(&mut contact)
}

unsafe extern "C" fn cll_end_contact<L: ContactListener>(object: ffi::Any,
                                                         contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut L>(object);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.end_contact(&mut contact)
}

unsafe extern "C" fn cll_pre_solve<L: ContactListener>(object: ffi::Any,
                                                       contact: *mut ffi::Contact,
                                                       old_manifold: *const Manifold) {
    assert!(!old_manifold.is_null());
    let listener = mem::transmute::<_, &mut L>(object);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.pre_solve(&mut contact, &*old_manifold)
}

unsafe extern "C" fn cll_post_solve<L: ContactListener>(object: ffi::Any,
                                                        contact: *mut ffi::Contact,
                                                        impulse: *const ContactImpulse) {
    assert!(!impulse.is_null());
    let listener = mem::transmute::<_, &mut L>(object);
    let mut contact = WrappedRefMut::new(Contact::from_ffi(contact));
    listener.post_solve(&mut contact, &*impulse)
}

impl Drop for ContactListenerLink {
    fn drop(&mut self) {
        unsafe { ffi::ContactListenerLink_drop(self.mut_ptr()) }
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
    pub unsafe fn new() -> QueryCallbackLink {
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
    let body = WrappedRef::new(Fixture::from_ffi(fixture)).body();
    callback.report_fixture(body, fixture.handle())
}

impl Drop for QueryCallbackLink {
    fn drop(&mut self) {
        unsafe { ffi::QueryCallbackLink_drop(self.ptr) }
    }
}

pub trait RayCastCallback {
    fn report_fixture(&mut self,
                      fixture: FixtureHandle,
                      p: &Vec2,
                      normal: &Vec2,
                      fraction: f32)
                      -> f32;
}

impl<F> RayCastCallback for F
    where F: FnMut(FixtureHandle, &Vec2, &Vec2, f32) -> f32
{
    fn report_fixture(&mut self,
                      fixture: FixtureHandle,
                      p: &Vec2,
                      normal: &Vec2,
                      fraction: f32)
                      -> f32 {
        self(fixture, p, normal, fraction)
    }
}

wrap! { ffi::RayCastCallbackLink => #[doc(hidden)] pub RayCastCallbackLink }

impl RayCastCallbackLink {
    pub unsafe fn new() -> RayCastCallbackLink {
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
    callback.report_fixture(fixture.handle(), &*point, &*normal, fraction)
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
    pub use dynamics::fixture::ffi::Fixture;
    pub use dynamics::joints::ffi::Joint;
    pub use dynamics::contacts::ffi::Contact;
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
