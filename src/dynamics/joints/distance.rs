use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    ToRaw
};

joint_def! {
    RawDistanceJointDef => DistanceJointDef (JointType::Distance) {
        local_anchor_a: Vec2 => Vec2,
        local_anchor_b: Vec2 => Vec2,
        length: f32 => f32,
        frequency: f32 => f32,
        damping_ratio: f32 => f32
    }
}

impl DistanceJointDef {
    pub fn new() -> DistanceJointDef {
        DistanceJointDef {
            base: JointDefBase::new(),
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            length: 1.,
            frequency: 0.,
            damping_ratio: 0.
        }
    }

    pub fn init(&mut self,
                world: &World,
                body_a: BodyHandle,
                body_b: BodyHandle,
                anchor_a: &Vec2,
                anchor_b: &Vec2) {
        self.base.body_a = Some(body_a);
        self.base.body_b = Some(body_b);
        let a = world.get_body(body_a);
        let b = world.get_body(body_a);
        self.local_anchor_a = a.local_point(anchor_a);
        self.local_anchor_b = b.local_point(anchor_b);
        self.length = (anchor_b - anchor_a).norm();
    }

    fn assert_well_formed(&self) {
        self.base.body_a.expect("DistanceJointDef expects some body_a");
        self.base.body_b.expect("DistanceJointDef expects some body_b");
    }
}

wrap_joint! {
    ffi::DistanceJoint => DistanceJoint (JointType::Distance)
    < ffi::DistanceJoint_as_joint
    > ffi::Joint_as_distance_joint
}

impl DistanceJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::DistanceJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::DistanceJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn length(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_length(self.ptr())
        }
    }

    pub fn frequency(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_frequency(self.ptr())
        }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe {
            ffi::DistanceJoint_get_damping_ratio(self.ptr())
        }
    }

    pub fn set_length(&mut self, length: f32) {
        unsafe {
            ffi::DistanceJoint_set_length(self.mut_ptr(), length)
        }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::DistanceJoint_set_frequency(self.mut_ptr(), frequency)
        }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::DistanceJoint_set_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    #[repr(C)] pub struct DistanceJoint;

    extern {
        /*pub fn DistanceJointDef_initialize(slf: *mut DistanceJointDef,
                                           body_a: *mut Body,
                                           body_b: *mut Body,
                                           anchor_a: *const Vec2,
                                           anchor_b: *const Vec2);*/
        pub fn DistanceJoint_as_joint(slf: *mut DistanceJoint) -> *mut Joint;
        pub fn Joint_as_distance_joint(slf: *mut Joint) -> *mut DistanceJoint;
        pub fn DistanceJoint_get_local_anchor_a(slf: *const DistanceJoint
                                                ) -> *const Vec2;
        pub fn DistanceJoint_get_local_anchor_b(slf: *const DistanceJoint
                                                ) -> *const Vec2;
        pub fn DistanceJoint_set_length(slf: *mut DistanceJoint, length: f32);
        pub fn DistanceJoint_get_length(slf: *const DistanceJoint) -> f32;
        pub fn DistanceJoint_set_frequency(slf: *mut DistanceJoint, hz: f32);
        pub fn DistanceJoint_get_frequency(slf: *const DistanceJoint) -> f32;
        pub fn DistanceJoint_set_damping_ratio(slf: *mut DistanceJoint, ratio: f32);
        pub fn DistanceJoint_get_damping_ratio(slf: *const DistanceJoint) -> f32;
    }
}
