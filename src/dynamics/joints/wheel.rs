use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{
    Joint, JointType, JointDefBase, RawJointDefBase, JointDef,
    ToRaw
};

joint_def! {
    RawWheelJointDef => WheelJointDef (JointType::Wheel) {
        local_anchor_a: Vec2 => Vec2,
        local_anchor_b: Vec2 => Vec2,
        local_axis_a: Vec2 => Vec2,
        enable_motor: bool => bool,
        max_motor_torque: f32 => f32,
        motor_speed: f32 => f32,
        frequency: f32 => f32,
        damping_ratio: f32 => f32
    }
}

impl WheelJointDef {
    pub fn new() -> WheelJointDef {
        WheelJointDef {
            base: JointDefBase::new(),
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            local_axis_a: Vec2 { x: 1., y: 0. },
            enable_motor: false,
            max_motor_torque: 0.,
            motor_speed: 0.,
            frequency: 2.,
            damping_ratio: 0.7
        }
    }

    pub fn init(&mut self,
                world: &World,
                body_a: BodyHandle,
                body_b: BodyHandle,
                anchor: &Vec2,
                axis: &Vec2) {
        self.base.body_a = Some(body_a);
        self.base.body_b = Some(body_b);
        let a = world.get_body(body_a);
        let b = world.get_body(body_a);
        self.local_anchor_a = a.local_point(anchor);
        self.local_anchor_b = b.local_point(anchor);
        self.local_axis_a = a.local_vector(axis);
    }

    fn assert_well_formed(&self) {
        self.base.body_a.expect("WheelJointDef expects some body_a");
        self.base.body_b.expect("WheelJointDef expects some body_b");
    }
}

wrap_joint! {
    ffi::WheelJoint => WheelJoint (JointType::Wheel)
    < ffi::WheelJoint_as_joint
    > ffi::Joint_as_wheel_joint
}

impl WheelJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WheelJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WheelJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_axis_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::WheelJoint_get_local_axis_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn joint_translation(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_translation(self.ptr())
        }
    }

    pub fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_joint_speed(self.ptr())
        }
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::WheelJoint_is_motor_enabled(self.ptr())
        }
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_speed(self.ptr())
        }
    }

    pub fn max_motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_max_motor_torque(self.ptr())
        }
    }

    pub fn motor_torque(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_motor_torque(self.ptr())
        }
    }

    pub fn spring_frequency(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_frequency(self.ptr())
        }
    }

    pub fn spring_damping_ratio(&self) -> f32 {
        unsafe {
            ffi::WheelJoint_get_spring_damping_ratio(self.ptr())
        }
    }

    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::WheelJoint_enable_motor(self.mut_ptr(), flag)
        }
    }

    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::WheelJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }

    pub fn set_max_motor_torque(&mut self, torque: f32) {
        unsafe {
            ffi::WheelJoint_set_max_motor_torque(self.mut_ptr(), torque)
        }
    }

    pub fn set_spring_frequency(&mut self, frequency: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_frequency(self.mut_ptr(), frequency)
        }
    }

    pub fn set_spring_damping_ratio(&mut self, ratio: f32) {
        unsafe {
            ffi::WheelJoint_set_spring_damping_ratio(self.mut_ptr(), ratio)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    #[repr(C)] pub struct WheelJoint;

    extern {
        /*pub fn WheelJointDef_initialize(slf: *mut WheelJointDef,
                                        body_a: *mut Body,
                                        body_b: *mut Body,
                                        anchor: *const Vec2,
                                        axis: *const Vec2);*/
        pub fn WheelJoint_as_joint(slf: *mut WheelJoint) -> *mut Joint;
        pub fn Joint_as_wheel_joint(slf: *mut Joint) -> *mut WheelJoint;
        pub fn WheelJoint_get_local_anchor_a(slf: *const WheelJoint) -> *const Vec2;
        pub fn WheelJoint_get_local_anchor_b(slf: *const WheelJoint) -> *const Vec2;
        pub fn WheelJoint_get_local_axis_a(slf: *const WheelJoint) -> *const Vec2;
        pub fn WheelJoint_get_joint_translation(slf: *const WheelJoint) -> f32;
        pub fn WheelJoint_get_joint_speed(slf: *const WheelJoint) -> f32;
        pub fn WheelJoint_is_motor_enabled(slf: *const WheelJoint) -> bool;
        pub fn WheelJoint_enable_motor(slf: *mut WheelJoint, flag: bool);
        pub fn WheelJoint_set_motor_speed(slf: *mut WheelJoint, speed: f32);
        pub fn WheelJoint_get_motor_speed(slf: *const WheelJoint) -> f32;
        pub fn WheelJoint_set_max_motor_torque(slf: *mut WheelJoint, torque: f32);
        pub fn WheelJoint_get_max_motor_torque(slf: *const WheelJoint) -> f32;
        pub fn WheelJoint_get_motor_torque(slf: *const WheelJoint) -> f32;
        pub fn WheelJoint_set_spring_frequency(slf: *mut WheelJoint, frequency: f32);
        pub fn WheelJoint_get_spring_frequency(slf: *const WheelJoint) -> f32;
        pub fn WheelJoint_set_spring_damping_ratio(slf: *mut WheelJoint, ratio: f32);
        pub fn WheelJoint_get_spring_damping_ratio(slf: *const WheelJoint) -> f32;
    }
}
