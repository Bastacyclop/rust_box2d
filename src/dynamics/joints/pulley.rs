use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    ToRaw
};

joint_def! {
    RawPulleyJointDef => PulleyJointDef (JointType::Pulley) {
        ground_anchor_a: Vec2 => Vec2,
        ground_anchor_b: Vec2 => Vec2,
        local_anchor_a: Vec2 => Vec2,
        local_anchor_b: Vec2 => Vec2,
        length_a: f32 => f32,
        length_b: f32 => f32,
        ratio: f32 => f32
    }
}

impl PulleyJointDef {
    pub fn new() -> PulleyJointDef {
        let mut j = PulleyJointDef {
            base: JointDefBase::new(),
            ground_anchor_a: Vec2 { x: -1., y: 1. },
            ground_anchor_b: Vec2 { x: 1., y: 1. },
            local_anchor_a: Vec2 { x: -1., y: 0. },
            local_anchor_b: Vec2 { x: 1., y: 0. },
            length_a: 0.,
            length_b: 0.,
            ratio: 1.
        };
        j.base.collide_connected = true;
        j
    }

    pub fn init(&mut self,
                body_a: BodyHandle,
                body_b: BodyHandle,
                ground_a: Vec2,
                ground_b: Vec2,
                anchor_a: &Vec2,
                anchor_b: &Vec2,
                ratio: f32) {

        self.base.body_a = Some(body_a);
        self.base.body_b = Some(body_b);
        self.ground_anchor_a = ground_a;
        self.ground_anchor_b = ground_b;
        self.length_a = (anchor_a - ground_a).norm();
        self.length_b = (anchor_b - ground_b).norm();
        self.ratio = ratio;
        assert!(ratio > ::std::f32::EPSILON);
    }

    fn assert_well_formed(&self) {
        self.base.body_a.expect("PulleyJointDef expects some body_a");
        self.base.body_b.expect("PulleyJointDef expects some body_b");
    }
}

wrap_joint! {
    ffi::PulleyJoint => PulleyJoint (JointType::Pulley)
    < ffi::PulleyJoint_as_joint
    > ffi::Joint_as_pulley_joint
}

impl PulleyJoint {
    pub fn ground_anchor_a(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_a(self.ptr())
        }
    }

    pub fn ground_anchor_b(&self) -> Vec2 {
        unsafe {
            ffi::PulleyJoint_get_ground_anchor_b(self.ptr())
        }
    }

    pub fn length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_a(self.ptr())
        }
    }

    pub fn length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_length_b(self.ptr())
        }
    }

    pub fn ratio(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_ratio(self.ptr())
        }
    }

    pub fn current_length_a(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_a(self.ptr())
        }
    }

    pub fn current_length_b(&self) -> f32 {
        unsafe {
            ffi::PulleyJoint_get_current_length_b(self.ptr())
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    #[repr(C)] pub struct PulleyJoint;

    extern {
        /*pub fn PulleyJointDef_initialize(slf: *mut PulleyJointDef,
                                         body_a: *mut Body,
                                         body_b: *mut Body,
                                         ground_anchor_a: *const Vec2,
                                         ground_anchor_b: *const Vec2,
                                         anchor_a: *const Vec2,
                                         anchor_b: *const Vec2,
                                         ratio: f32);*/
        pub fn PulleyJoint_as_joint(slf: *mut PulleyJoint) -> *mut Joint;
        pub fn Joint_as_pulley_joint(slf: *mut Joint) -> *mut PulleyJoint;
        pub fn PulleyJoint_get_ground_anchor_a(slf: *const PulleyJoint) -> Vec2;
        pub fn PulleyJoint_get_ground_anchor_b(slf: *const PulleyJoint) -> Vec2;
        pub fn PulleyJoint_get_length_a(slf: *const PulleyJoint) -> f32;
        pub fn PulleyJoint_get_length_b(slf: *const PulleyJoint) -> f32;
        pub fn PulleyJoint_get_ratio(slf: *const PulleyJoint) -> f32;
        pub fn PulleyJoint_get_current_length_a(slf: *const PulleyJoint) -> f32;
        pub fn PulleyJoint_get_current_length_b(slf: *const PulleyJoint) -> f32;
    }
}
