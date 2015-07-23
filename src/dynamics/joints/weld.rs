use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    ToRaw
};

joint_def! {
    RawWeldJointDef => WeldJointDef (JointType::Weld) {
        local_anchor_a: Vec2 => Vec2,
        local_anchor_b: Vec2 => Vec2,
        reference_angle: f32 => f32,
        frequency: f32 => f32,
        damping_ratio: f32 => f32
    }
}

impl WeldJointDef {
    pub fn new() -> WeldJointDef {
        WeldJointDef {
            base: JointDefBase::new(),
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            reference_angle: 0.,
            frequency: 0.,
            damping_ratio: 0.
        }
    }

    pub fn init(&mut self,
                world: &World,
                body_a: BodyHandle,
                body_b: BodyHandle,
                anchor: &Vec2) {
        self.base.body_a = Some(body_a);
        self.base.body_b = Some(body_b);
        let a = world.get_body(body_a);
        let b = world.get_body(body_a);
        self.local_anchor_a = a.local_point(anchor);
        self.local_anchor_b = b.local_point(anchor);
        self.reference_angle = b.angle() - a.angle();
    }

    fn assert_well_formed(&self) {
        self.base.body_a.expect("WeldJointDef expects some body_a");
        self.base.body_b.expect("WeldJointDef expects some body_b");
    }
}

wrap_joint! {
    ffi::WeldJoint => WeldJoint (JointType::Weld)
    < ffi::WeldJoint_as_joint
    > ffi::Joint_as_weld_joint
}

impl WeldJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WeldJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WeldJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn reference_angle(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_reference_angle(self.ptr())
        }
    }

    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_frequency(self.ptr())
        }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WeldJoint_get_damping_ratio(self.ptr())
        }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WeldJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WeldJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    #[repr(C)] pub struct WeldJoint;

    extern {
        /*pub fn WeldJointDef_initialize(slf: *mut WeldJointDef,
                                       body_a: *mut Body,
                                       body_b: *mut Body,
                                       anchor: *const Vec2);*/
        pub fn WeldJoint_as_joint(slf: *mut WeldJoint) -> *mut Joint;
        pub fn Joint_as_weld_joint(slf: *mut Joint) -> *mut WeldJoint;
        pub fn WeldJoint_get_local_anchor_a(slf: *const WeldJoint) -> *const Vec2;
        pub fn WeldJoint_get_local_anchor_b(slf: *const WeldJoint) -> *const Vec2;
        pub fn WeldJoint_get_reference_angle(slf: *const WeldJoint) -> f32;
        pub fn WeldJoint_set_frequency(slf: *mut WeldJoint, frequency: f32);
        pub fn WeldJoint_get_frequency(slf: *const WeldJoint) -> f32;
        pub fn WeldJoint_set_damping_ratio(slf: *mut WeldJoint, ratio: f32);
        pub fn WeldJoint_get_damping_ratio(slf: *const WeldJoint) -> f32;
    }
}
