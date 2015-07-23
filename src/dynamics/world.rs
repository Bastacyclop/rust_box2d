#[path = "world_callbacks.rs"] pub mod callbacks;

use std::mem;
use std::marker::PhantomData;
use std::cell::{ RefCell, Ref, RefMut };
use wrap::*;
use handle::*;
use common::{ Draw, DrawLink, DrawFlags };
use common::math::Vec2;
use collision::AABB;
use dynamics::Profile;
use dynamics::body::{ BodyDef, MetaBody, Body };
use dynamics::joints::{ JointDef, MetaJoint };
use dynamics::contacts::Contact;
use self::callbacks::{
    ContactFilter, ContactFilterLink, ContactListener, ContactListenerLink,
    QueryCallback, QueryCallbackLink, RayCastCallback, RayCastCallbackLink
};

pub type BodyHandle = TypedHandle<MetaBody>;
pub type JointHandle = TypedHandle<MetaJoint>;

pub struct World {
    ptr: *mut ffi::World,
    bodies: HandleMap<MetaBody>,
    joints: HandleMap<MetaJoint>,
    draw_link: DrawLink,
    contact_filter_link: ContactFilterLink,
    contact_listener_link: ContactListenerLink,
    query_callback_link: RefCell<QueryCallbackLink>,
    raycast_callback_link: RefCell<RayCastCallbackLink>
}

wrap! { ffi::World => custom World }

impl World {
    pub fn new(gravity: &Vec2) -> World {
        unsafe {
            World {
                ptr: ffi::World_new(gravity),
                bodies: HandleMap::new(),
                joints: HandleMap::new(),
                draw_link: DrawLink::new(),
                contact_filter_link: ContactFilterLink::new(),
                contact_listener_link: ContactListenerLink::new(),
                query_callback_link: RefCell::new(QueryCallbackLink::new()),
                raycast_callback_link: RefCell::new(RayCastCallbackLink::new())
            }
        }
    }

    pub fn set_contact_filter(&mut self, filter: Box<ContactFilter>) {
        unsafe {
            self.contact_filter_link.set_object(filter);
            ffi::World_set_contact_filter(
                self.mut_ptr(),
                self.contact_filter_link.filter_ptr()
            );
        }
    }

    pub fn set_contact_listener(&mut self, listener: Box<ContactListener>) {
        unsafe {
            self.contact_listener_link.set_object(listener);
            ffi::World_set_contact_listener(
                self.mut_ptr(),
                self.contact_listener_link.listener_ptr()
            );
        }
    }

    pub fn create_body(&mut self, def: &BodyDef) -> BodyHandle {
        unsafe {
            let body = ffi::World_create_body(self.mut_ptr(), def);
            self.bodies.insert_with(|h| MetaBody::new(body, h))
        }
    }

    pub fn get_body(&self, handle: BodyHandle) -> Ref<MetaBody> {
        self.bodies.get(handle).expect("invalid body handle")
    }

    pub fn get_body_mut(&self, handle: BodyHandle) -> RefMut<MetaBody> {
        self.bodies.get_mut(handle).expect("invalid body handle")
    }

    pub fn destroy_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(handle)
            .map(|mut body| {
                World::remove_body_joint_handles(&mut body, &mut self.joints);
                unsafe {
                    ffi::World_destroy_body(self.mut_ptr(), body.mut_ptr());
                }
            });
    }

    fn remove_body_joint_handles(body: &mut Body,
                                 joints: &mut HandleMap<MetaJoint>) {
        let mut joint_edge = body.joints();
        loop {
            match joint_edge {
                None => break,
                Some(je) => {
                    joints.remove(je.joint());
                    joint_edge = je.next();
                }
            }
        }
    }

    pub fn create_joint<JD: JointDef>(&mut self, def: &JD) -> JointHandle {
        unsafe {
            let joint = def.create(self);
            self.joints.insert_with(|h| MetaJoint::new(joint, h))
        }
    }

    pub fn get_joint(&self, handle: JointHandle) -> Ref<MetaJoint> {
        self.joints.get(handle).expect("invalid joint handle")
    }

    pub fn get_joint_mut(&self, handle: JointHandle) -> RefMut<MetaJoint> {
        self.joints.get_mut(handle).expect("invalid joint handle")
    }

    pub fn destroy_joint(&mut self, handle: JointHandle) {
        self.joints.remove(handle)
            .map(|mut meta_joint| { unsafe {
                ffi::World_destroy_joint(self.mut_ptr(), meta_joint.mut_base_ptr());
            } });
    }

    pub fn step(&mut self,
                time_step: f32,
                velocity_iterations: i32,
                position_iterations: i32) {
        unsafe {
            ffi::World_step(self.mut_ptr(),
                            time_step,
                            velocity_iterations,
                            position_iterations)
        }
    }

    pub fn clear_forces(&mut self) {
        unsafe {
            ffi::World_clear_forces(self.mut_ptr())
        }
    }

    pub fn draw_debug_data(&mut self, draw: &mut Draw, flags: DrawFlags) {
        unsafe {
            let draw_ptr = self.draw_link.use_with(draw, flags);
            ffi::World_set_debug_draw(self.mut_ptr(), draw_ptr);
            ffi::World_draw_debug_data(self.mut_ptr());
        }
    }

    pub fn query_aabb(&self, callback: &mut QueryCallback, aabb: &AABB) {
        unsafe {
            let ptr = self.query_callback_link.borrow_mut().use_with(callback);
            ffi::World_query_aabb(self.ptr(), ptr, aabb);
        }
    }

    pub fn ray_cast(&self, callback: &mut RayCastCallback, p1: &Vec2, p2: &Vec2) {
        unsafe {
            let ptr = self.raycast_callback_link.borrow_mut().use_with(callback);
            ffi::World_ray_cast(self.ptr(), ptr, p1, p2);
        }
    }

    pub fn bodies_mut(&mut self) -> HandleIterMut<MetaBody> {
        self.bodies.iter_mut()
    }

    pub fn bodies(&self) -> HandleIter<MetaBody> {
        self.bodies.iter()
    }

    pub fn joints_mut(&mut self) -> HandleIterMut<MetaJoint> {
        self.joints.iter_mut()
    }

    pub fn joints(&mut self) -> HandleIter<MetaJoint> {
        self.joints.iter()
    }

    pub fn contact_list_mut(&mut self) -> ContactIterMut {
        ContactIterMut {
            ptr: unsafe {
                ffi::World_get_contact_list(self.mut_ptr())
            },
            phantom: PhantomData
        }
    }

    pub fn contact_list(&self) -> ContactIter {
        ContactIter {
            ptr: unsafe {
                ffi::World_get_contact_list_const(self.ptr())
            },
            phantom: PhantomData
        }
    }

    pub fn set_sleeping_allowed(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_allow_sleeping(self.mut_ptr(), flag)
        }
    }

    pub fn is_sleeping_allowed(&self) -> bool {
        unsafe {
            ffi::World_get_allow_sleeping(self.ptr())
        }
    }

    pub fn set_warm_starting(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_warm_starting(self.mut_ptr(), flag)
        }
    }

    pub fn is_warm_starting(&self) -> bool {
        unsafe {
            ffi::World_get_warm_starting(self.ptr())
        }
    }

    pub fn set_continuous_physics(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_continuous_physics(self.mut_ptr(), flag)
        }
    }

    pub fn is_continuous_physics(&self) -> bool {
        unsafe {
            ffi::World_get_continuous_physics(self.ptr())
        }
    }

    pub fn set_sub_stepping(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_sub_stepping(self.mut_ptr(), flag)
        }
    }

    pub fn is_sub_stepping(&self) -> bool {
        unsafe {
            ffi::World_get_sub_stepping(self.ptr())
        }
    }

    pub fn proxy_count(&self) -> i32 {
        unsafe {
            ffi::World_get_proxy_count(self.ptr())
        }
    }

    pub fn body_count(&self) -> i32 {
        unsafe {
            ffi::World_get_body_count(self.ptr())
        }
    }

    pub fn joint_count(&self) -> i32 {
        unsafe {
            ffi::World_get_joint_count(self.ptr())
        }
    }

    pub fn contact_count(&self) -> i32 {
        unsafe {
            ffi::World_get_contact_count(self.ptr())
        }
    }

    pub fn tree_height(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_height(self.ptr())
        }
    }

    pub fn tree_balance(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_balance(self.ptr())
        }
    }

    pub fn tree_quality(&self) -> f32 {
        unsafe {
            ffi::World_get_tree_quality(self.ptr())
        }
    }

    pub fn set_gravity(&mut self, gravity: &Vec2) {
        unsafe {
            ffi::World_set_gravity(self.mut_ptr(), gravity)
        }
    }

    pub fn gravity(&self) -> Vec2 {
        unsafe {
            ffi::World_get_gravity(self.ptr())
        }
    }

    pub fn is_locked(&self) -> bool {
        unsafe {
            ffi::World_is_locked(self.ptr())
        }
    }

    pub fn set_auto_clearing_forces(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_auto_clear_forces(self.mut_ptr(), flag)
        }
    }

    pub fn is_auto_clearing_forces(&self) -> bool {
        unsafe {
            ffi::World_get_auto_clear_forces(self.ptr())
        }
    }

    pub fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::World_shift_origin(self.mut_ptr(), origin)
        }
    }

    pub fn profile<'a>(&'a self) -> &'a Profile {
        unsafe {
            &*ffi::World_get_profile(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn dump(&mut self) {
        unsafe {
            ffi::World_dump(self.mut_ptr())
        }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            ffi::World_drop(self.mut_ptr())
        }
    }
}

pub struct ContactIterMut<'a> {
    ptr: *mut ffi::Contact,
    phantom: PhantomData<&'a ()>
}

impl<'a> Iterator for ContactIterMut<'a> {
    type Item = WrappedRefMut<'a, Contact>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr.is_null() {
            None
        } else { unsafe {
            let next = ffi::Contact_get_next(self.ptr);
            Some(WrappedRefMut::new(Contact::from_ffi(
                mem::replace(&mut self.ptr, next)
            )))
        } }
    }
}

pub struct ContactIter<'a> {
    ptr: *const ffi::Contact,
    phantom: PhantomData<&'a ()>
}

impl<'a> Iterator for ContactIter<'a> {
    type Item = WrappedRef<'a, Contact>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr.is_null() {
            None
        } else { unsafe {
            let next = ffi::Contact_get_next_const(self.ptr);
            Some(WrappedRef::new(Contact::from_ffi(
                mem::replace(&mut self.ptr, next) as *mut ffi::Contact
            )))
        } }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use common::ffi::Draw;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    pub use dynamics::contacts::ffi::{
        Contact, Contact_get_next, Contact_get_next_const
    };
    pub use super::callbacks::ffi::{
        ContactFilter, ContactListener, QueryCallback, RayCastCallback
    };
    use common::math::Vec2;
    use collision::AABB;
    use dynamics::Profile;
    use dynamics::body::BodyDef;

    #[repr(C)] pub struct World;

    extern {
        pub fn World_new(gravity: *const Vec2) -> *mut World;
        pub fn World_drop(slf: *mut World);
        pub fn World_set_contact_filter(slf: *mut World, cf: *mut ContactFilter);
        pub fn World_set_contact_listener(slf: *mut World, cl: *mut ContactListener);
        pub fn World_set_debug_draw(slf: *mut World, dd: *mut Draw);
        pub fn World_create_body(slf: *mut World, def: *const BodyDef) -> *mut Body;
        pub fn World_destroy_body(slf: *mut World, body: *mut Body);
        pub fn World_destroy_joint(slf: *mut World, joint: *mut Joint);
        pub fn World_step(slf: *mut World,
                          time_step: f32,
                          velocity_iterations: i32,
                          position_iterations: i32);
        pub fn World_clear_forces(slf: *mut World);
        pub fn World_draw_debug_data(slf: *mut World);
        pub fn World_query_aabb(slf: *const World, qc: *mut QueryCallback,
                                aabb: *const AABB);
        pub fn World_ray_cast(slf: *const World, rcc: *mut RayCastCallback,
                              p1: *const Vec2, p2: *const Vec2);
        //pub fn World_get_body_list(slf: *mut World) -> *mut Body;
        //pub fn World_get_body_list_const(slf: *const World) -> *const Body;
        //pub fn World_get_joint_list(slf: *mut World) -> *mut Joint;
        //pub fn World_get_joint_list_const(slf: *const World) -> *const Joint;
        pub fn World_get_contact_list(slf: *mut World) -> *mut Contact;
        pub fn World_get_contact_list_const(slf: *const World) -> *const Contact;
        pub fn World_set_allow_sleeping(slf: *mut World, flag: bool);
        pub fn World_get_allow_sleeping(slf: *const World) -> bool;
        pub fn World_set_warm_starting(slf: *mut World, flag: bool);
        pub fn World_get_warm_starting(slf: *const World) -> bool;
        pub fn World_set_continuous_physics(slf: *mut World, flag: bool);
        pub fn World_get_continuous_physics(slf: *const World) -> bool;
        pub fn World_set_sub_stepping(slf: *mut World, flag: bool);
        pub fn World_get_sub_stepping(slf: *const World) -> bool;
        pub fn World_get_proxy_count(slf: *const World) -> i32;
        pub fn World_get_body_count(slf: *const World) -> i32;
        pub fn World_get_joint_count(slf: *const World) -> i32;
        pub fn World_get_contact_count(slf: *const World) -> i32;
        pub fn World_get_tree_height(slf: *const World) -> i32;
        pub fn World_get_tree_balance(slf: *const World) -> i32;
        pub fn World_get_tree_quality(slf: *const World)-> f32;
        pub fn World_set_gravity(slf: *mut World, gravity: *const Vec2);
        pub fn World_get_gravity(slf: *const World) -> Vec2;
        pub fn World_is_locked(slf: *const World) -> bool;
        pub fn World_set_auto_clear_forces(slf: *mut World, flag: bool);
        pub fn World_get_auto_clear_forces(slf: *const World) -> bool;
        pub fn World_shift_origin(slf: *mut World, origin: *const Vec2);
        //pub fn World_get_contact_manager(slf: *const World) -> *const ContactManager;
        pub fn World_get_profile(slf: *const World) -> *const Profile;
        pub fn World_dump(slf: *mut World);
    }
}
