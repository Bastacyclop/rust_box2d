use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{ Joint, JointType, JointDef };

pub struct RevoluteJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub reference_angle: f32,
    pub enable_limit: bool,
    pub lower_angle: f32,
    pub upper_angle: f32,
    pub enable_motor: bool,
    pub motor_speed: f32,
    pub max_motor_torque: f32
}

impl RevoluteJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> RevoluteJointDef {
        RevoluteJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            reference_angle: 0.,
            enable_limit: false,
            lower_angle: 0.,
            upper_angle: 0.,
            enable_motor: false,
            motor_speed: 0.,
            max_motor_torque: 0.,
        }
    }

    pub fn init(&mut self,
                world: &World,
                body_a: BodyHandle,
                body_b: BodyHandle,
                anchor: &Vec2) {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.get_body(body_a);
        let b = world.get_body(body_a);
        self.local_anchor_a = a.local_point(anchor);
        self.local_anchor_b = b.local_point(anchor);
        self.reference_angle = b.angle() - a.angle();
    }
}

impl JointDef for RevoluteJointDef {
    fn joint_type() -> JointType where Self: Sized { JointType::Revolute }

    unsafe fn create(&self, world: &mut World) -> *mut ffi::Joint {
        ffi::World_create_revolute_joint(
            world.mut_ptr(),
            world.get_body_mut(self.body_a).mut_ptr(),
            world.get_body_mut(self.body_b).mut_ptr(),
            self.collide_connected,
            self.local_anchor_a,
            self.local_anchor_b,
            self.reference_angle,
            self.enable_limit,
            self.lower_angle,
            self.upper_angle,
            self.enable_motor,
            self.motor_speed,
            self.max_motor_torque
        )
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
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    #[repr(C)] pub struct RevoluteJoint;

    extern {
        pub fn World_create_revolute_joint(
            world: *mut World,
            body_a: *mut Body,
            body_b: *mut Body,
            collide_connected: bool,
            local_anchor_a: Vec2,
            local_anchor_b: Vec2,
            reference_angle: f32,
            enable_limit: bool,
            lower_angle: f32,
            upper_angle: f32,
            enable_motor: bool,
            motor_speed: f32,
            max_motor_torque: f32
        ) -> *mut Joint;
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
