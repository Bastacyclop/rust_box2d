use wrap::*;
use common::math::Vec2;
use dynamics::world::World;
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    LimitState, ToRaw
};

joint_def! {
    RawRopeJointDef => RopeJointDef (JointType::Rope) {
        local_anchor_a: Vec2 => Vec2,
        local_anchor_b: Vec2 => Vec2,
        max_length: f32 => f32
    }
}

impl RopeJointDef {
    pub fn new() -> RopeJointDef {
        RopeJointDef {
            base: JointDefBase::new(),
            local_anchor_a: Vec2 { x: -1., y: 0. },
            local_anchor_b: Vec2 { x: 1., y: 0. },
            max_length: 0.
        }
    }

    fn assert_well_formed(&self) {
        self.base.body_a.expect("RopeJointDef expects some body_a");
        self.base.body_b.expect("RopeJointDef expects some body_b");
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
        unsafe {
            ffi::RopeJoint_get_max_length(self.ptr())
        }
    }

    pub fn limit_state(&self) -> LimitState {
        unsafe {
            ffi::RopeJoint_get_limit_state(self.ptr())
        }
    }

    pub fn set_max_length(&mut self, length: f32) {
        unsafe {
            ffi::RopeJoint_set_max_length(self.mut_ptr(), length)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;
    use dynamics::joints::LimitState;

    #[repr(C)] pub struct RopeJoint;

    extern {
        pub fn RopeJoint_as_joint(slf: *mut RopeJoint) -> *mut Joint;
        pub fn Joint_as_rope_joint(slf: *mut Joint) -> *mut RopeJoint;
        pub fn RopeJoint_get_local_anchor_a(slf: *const RopeJoint) -> *const Vec2;
        pub fn RopeJoint_get_local_anchor_b(slf: *const RopeJoint) -> *const Vec2;
        pub fn RopeJoint_set_max_length(slf: *mut RopeJoint, length: f32);
        pub fn RopeJoint_get_max_length(slf: *const RopeJoint) -> f32;
        pub fn RopeJoint_get_limit_state(slf: *const RopeJoint) -> LimitState;
    }
}
