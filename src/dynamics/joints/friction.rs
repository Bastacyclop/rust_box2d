use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    ToRaw
};

joint_def! {
    RawFrictionJointDef => FrictionJointDef (JointType::Friction) {
        local_anchor_a: Vec2 => Vec2,
        local_anchor_b: Vec2 => Vec2,
        max_force: f32 => f32,
        max_torque: f32 => f32
    }
}

impl FrictionJointDef {
    pub fn new() -> FrictionJointDef {
        FrictionJointDef {
            base: JointDefBase::new(),
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            max_force: 0.,
            max_torque: 0.
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
    }

    fn assert_well_formed(&self) {
        self.base.body_a.expect("FrictionJointDef expects some body_a");
        self.base.body_b.expect("FrictionJointDef expects some body_b");
    }
}

wrap_joint! {
    ffi::FrictionJoint => FrictionJoint (JointType::Friction)
    < ffi::FrictionJoint_as_joint
    > ffi::Joint_as_friction_joint
}

impl FrictionJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::FrictionJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::FrictionJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn max_force(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_force(self.ptr())
        }
    }

    pub fn max_torque(&self) -> f32 {
        unsafe {
            ffi::FrictionJoint_get_max_torque(self.ptr())
        }
    }

    pub fn set_max_force(&mut self, force: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_force(self.mut_ptr(), force)
        }
    }

    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe {
            ffi::FrictionJoint_set_max_torque(self.mut_ptr(), torque)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    #[repr(C)] pub struct FrictionJoint;

    extern {
        /*pub fn FrictionJointDef_initialize(slf: *mut FrictionJointDef,
                                           body_a: *mut Body,
                                           body_b: *mut Body,
                                           anchor: *const Vec2);*/
        pub fn FrictionJoint_as_joint(slf: *mut FrictionJoint) -> *mut Joint;
        pub fn Joint_as_friction_joint(slf: *mut Joint) -> *mut FrictionJoint;
        pub fn FrictionJoint_get_local_anchor_a(slf: *const FrictionJoint
                                                ) -> *const Vec2;
        pub fn FrictionJoint_get_local_anchor_b(slf: *const FrictionJoint
                                                ) -> *const Vec2;
        pub fn FrictionJoint_set_max_force(slf: *mut FrictionJoint, force: f32);
        pub fn FrictionJoint_get_max_force(slf: *const FrictionJoint) -> f32;
        pub fn FrictionJoint_set_max_torque(slf: *mut FrictionJoint, torque: f32);
        pub fn FrictionJoint_get_max_torque(slf: *const FrictionJoint) -> f32;
    }
}
