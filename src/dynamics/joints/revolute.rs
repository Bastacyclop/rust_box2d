use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    ToRaw
};

joint_def! {
    RawRevoluteJointDef => RevoluteJointDef (JointType::Revolute) {
        local_anchor_a: Vec2 => Vec2,
        local_anchor_b: Vec2 => Vec2,
        reference_angle: f32 => f32,
        enable_limit: bool => bool,
        lower_angle: f32 => f32,
        upper_angle: f32 => f32,
        enable_motor: bool => bool,
        motor_speed: f32 => f32,
        max_motor_torque: f32 => f32
    }
}

impl RevoluteJointDef {
    pub fn new() -> RevoluteJointDef {
        RevoluteJointDef {
            base: JointDefBase::new(),
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            reference_angle: 0.,
            lower_angle: 0.,
            upper_angle: 0.,
            max_motor_torque: 0.,
            motor_speed: 0.,
            enable_limit: false,
            enable_motor: false
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
        self.base.body_a.expect("RevoluteJointDef expects some body_a");
        self.base.body_b.expect("RevoluteJointDef expects some body_b");
    }
}

wrap_joint! {
    ffi::RevoluteJoint => RevoluteJoint (JointType::Revolute)
    < ffi::RevoluteJoint_as_joint
    > ffi::Joint_as_revolute_joint
}

impl RevoluteJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RevoluteJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::RevoluteJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn reference_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_reference_angle(self.ptr())
        }
    }

    pub fn joint_angle(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_angle(self.ptr())
        }
    }

    pub fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_joint_speed(self.ptr())
        }
    }

    pub fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_limit_enabled(self.ptr())
        }
    }

    pub fn lower_limit(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_lower_limit(self.ptr())
        }
    }

    pub fn upper_limit(&self) -> f32 {
        unsafe {
             ffi::RevoluteJoint_get_upper_limit(self.ptr())
        }
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::RevoluteJoint_is_motor_enabled(self.ptr())
        }
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_speed(self.ptr())
        }
    }

    pub fn max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_max_motor_torque(self.ptr())
        }
    }

    pub fn motor_torque(&self) -> f32 {
        unsafe {
            ffi::RevoluteJoint_get_motor_torque(self.ptr())
        }
    }

    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_limit(self.mut_ptr(), flag)
        }
    }

    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::RevoluteJoint_set_limits(self.mut_ptr(), lower, upper)
        }
    }

    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::RevoluteJoint_enable_motor(self.mut_ptr(), flag)
        }
    }

    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::RevoluteJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }

    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::RevoluteJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    #[repr(C)] pub struct RevoluteJoint;

    extern {
        /*pub fn RevoluteJointDef_initialize(slf: *mut RevoluteJointDef,
                                           body_a: *mut Body,
                                           body_b: *mut Body,
                                           anchor: *const Vec2);*/
        pub fn RevoluteJoint_as_joint(slf: *mut RevoluteJoint) -> *mut Joint;
        pub fn Joint_as_revolute_joint(slf: *mut Joint) -> *mut RevoluteJoint;
        pub fn RevoluteJoint_get_local_anchor_a(slf: *const RevoluteJoint
                                                ) -> *const Vec2;
        pub fn RevoluteJoint_get_local_anchor_b(slf: *const RevoluteJoint
                                                ) -> *const Vec2;
        pub fn RevoluteJoint_get_reference_angle(slf: *const RevoluteJoint) -> f32;
        pub fn RevoluteJoint_get_joint_angle(slf: *const RevoluteJoint) -> f32;
        pub fn RevoluteJoint_get_joint_speed(slf: *const RevoluteJoint) -> f32;
        pub fn RevoluteJoint_is_limit_enabled(slf: *const RevoluteJoint) -> bool;
        pub fn RevoluteJoint_enable_limit(slf: *mut RevoluteJoint, flag: bool);
        pub fn RevoluteJoint_get_lower_limit(slf: *const RevoluteJoint) -> f32;
        pub fn RevoluteJoint_get_upper_limit(slf: *const RevoluteJoint) -> f32;
        pub fn RevoluteJoint_set_limits(slf: *mut RevoluteJoint,
                                        lower: f32, upper: f32);
        pub fn RevoluteJoint_is_motor_enabled(slf: *const RevoluteJoint) -> bool;
        pub fn RevoluteJoint_enable_motor(slf: *mut RevoluteJoint, flag: bool);
        pub fn RevoluteJoint_set_motor_speed(slf: *mut RevoluteJoint, speed: f32);
        pub fn RevoluteJoint_get_motor_speed(slf: *const RevoluteJoint) -> f32;
        pub fn RevoluteJoint_set_max_motor_torque(slf: *mut RevoluteJoint,
                                                  torque: f32);
        pub fn RevoluteJoint_get_max_motor_torque(slf: *const RevoluteJoint) -> f32;
        pub fn RevoluteJoint_get_motor_torque(slf: *const RevoluteJoint) -> f32;
    }
}
