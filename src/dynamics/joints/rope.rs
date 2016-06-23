use wrap::*;
use common::math::Vec2;
use user_data::UserDataTypes;
use dynamics::world::{World, BodyHandle};
use dynamics::joints::{Joint, JointType, JointDef, LimitState};

pub struct RopeJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub max_length: f32,
}

impl RopeJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> RopeJointDef {
        RopeJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            local_anchor_a: Vec2 { x: -1., y: 0. },
            local_anchor_b: Vec2 { x: 1., y: 0. },
            max_length: 0.,
        }
    }
}

impl JointDef for RopeJointDef {
    fn joint_type() -> JointType
        where Self: Sized
    {
        JointType::Rope
    }

    unsafe fn create<U: UserDataTypes>(&self, world: &mut World<U>) -> *mut ffi::Joint {
        ffi::World_create_rope_joint(world.mut_ptr(),
                                     world.body_mut(self.body_a).mut_ptr(),
                                     world.body_mut(self.body_b).mut_ptr(),
                                     self.collide_connected,
                                     self.local_anchor_a,
                                     self.local_anchor_b,
                                     self.max_length)
    }
}

wrap_joint! {
    ffi::RopeJoint => RopeJoint (JointType::Rope)
    < ffi::RopeJoint_as_joint
    > ffi::Joint_as_rope_joint
}

impl RopeJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RopeJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RopeJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn max_length(&self) -> f32 {
        unsafe { ffi::RopeJoint_get_max_length(self.ptr()) }
    }

    pub fn limit_state(&self) -> LimitState {
        unsafe { ffi::RopeJoint_get_limit_state(self.ptr()) }
    }

    pub fn set_max_length(&mut self, length: f32) {
        unsafe { ffi::RopeJoint_set_max_length(self.mut_ptr(), length) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;
    use dynamics::joints::LimitState;

    pub enum RopeJoint {}

    extern "C" {
        pub fn World_create_rope_joint(world: *mut World,
                                       body_a: *mut Body,
                                       body_b: *mut Body,
                                       collide_connected: bool,
                                       local_anchor_a: Vec2,
                                       local_anchor_b: Vec2,
                                       max_length: f32)
                                       -> *mut Joint;
        pub fn RopeJoint_as_joint(slf: *mut RopeJoint) -> *mut Joint;
        pub fn Joint_as_rope_joint(slf: *mut Joint) -> *mut RopeJoint;
        pub fn RopeJoint_get_local_anchor_a(slf: *const RopeJoint) -> *const Vec2;
        pub fn RopeJoint_get_local_anchor_b(slf: *const RopeJoint) -> *const Vec2;
        pub fn RopeJoint_set_max_length(slf: *mut RopeJoint, length: f32);
        pub fn RopeJoint_get_max_length(slf: *const RopeJoint) -> f32;
        pub fn RopeJoint_get_limit_state(slf: *const RopeJoint) -> LimitState;
    }
}
