use wrap::*;
use common::math::Vec2;
use user_data::UserDataTypes;
use dynamics::world::{World, BodyHandle};
use dynamics::joints::{Joint, JointType, JointDef};

pub struct MotorJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub linear_offset: Vec2,
    pub angular_offset: f32,
    pub max_force: f32,
    pub max_torque: f32,
    pub correction_factor: f32,
}

impl MotorJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> MotorJointDef {
        MotorJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            linear_offset: Vec2 { x: 0., y: 0. },
            angular_offset: 0.,
            max_force: 1.,
            max_torque: 1.,
            correction_factor: 0.3,
        }
    }

    pub fn init<U: UserDataTypes>(&mut self,
                                  world: &World<U>,
                                  body_a: BodyHandle,
                                  body_b: BodyHandle) {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.body(body_a);
        let b = world.body(body_b);
        self.linear_offset = a.local_point(b.position());
        self.angular_offset = b.angle() - a.angle();
    }

    pub fn try_init<U: UserDataTypes>(&mut self,
                                      world: &World<U>,
                                      body_a: BodyHandle,
                                      body_b: BodyHandle) -> Option<()> {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.try_body_mut(body_a)?;
        let b = world.try_body_mut(body_b)?;
        self.linear_offset = a.local_point(b.position());
        self.angular_offset = b.angle() - a.angle();
        Some(())
    }
}

impl JointDef for MotorJointDef {
    fn joint_type() -> JointType
        where Self: Sized
    {
        JointType::Motor
    }

    unsafe fn create<U: UserDataTypes>(&self, world: &mut World<U>) -> *mut ffi::Joint {
        ffi::World_create_motor_joint(world.mut_ptr(),
                                      world.body_mut(self.body_a).mut_ptr(),
                                      world.body_mut(self.body_b).mut_ptr(),
                                      self.collide_connected,
                                      self.linear_offset,
                                      self.angular_offset,
                                      self.max_force,
                                      self.max_torque,
                                      self.correction_factor)
    }
}

wrap_joint! {
    ffi::MotorJoint => MotorJoint (JointType::Motor)
    < ffi::MotorJoint_as_joint
    > ffi::Joint_as_motor_joint
}

impl MotorJoint {
    pub fn linear_offset<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::MotorJoint_get_linear_offset(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn angular_offset(&self) -> f32 {
        unsafe { ffi::MotorJoint_get_angular_offset(self.ptr()) }
    }

    pub fn max_force(&self) -> f32 {
        unsafe { ffi::MotorJoint_get_max_force(self.ptr()) }
    }

    pub fn max_torque(&self) -> f32 {
        unsafe { ffi::MotorJoint_get_max_torque(self.ptr()) }
    }

    pub fn correction_factor(&self) -> f32 {
        unsafe { ffi::MotorJoint_get_correction_factor(self.ptr()) }
    }

    pub fn set_linear_offset(&mut self, offset: &Vec2) {
        unsafe { ffi::MotorJoint_set_linear_offset(self.mut_ptr(), offset) }
    }

    pub fn set_angular_offset(&mut self, offset: f32) {
        unsafe { ffi::MotorJoint_set_angular_offset(self.mut_ptr(), offset) }
    }

    pub fn set_max_force(&mut self, force: f32) {
        unsafe { ffi::MotorJoint_set_max_force(self.mut_ptr(), force) }
    }

    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe { ffi::MotorJoint_set_max_torque(self.mut_ptr(), torque) }
    }

    pub fn set_correction_factor(&mut self, factor: f32) {
        unsafe { ffi::MotorJoint_set_correction_factor(self.mut_ptr(), factor) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    pub enum MotorJoint {}

    extern "C" {
        pub fn World_create_motor_joint(world: *mut World,
                                        body_a: *mut Body,
                                        body_b: *mut Body,
                                        collide_connected: bool,
                                        linear_offset: Vec2,
                                        angular_offset: f32,
                                        max_force: f32,
                                        max_torque: f32,
                                        correction_factor: f32)
                                        -> *mut Joint;
        pub fn MotorJoint_as_joint(slf: *mut MotorJoint) -> *mut Joint;
        pub fn Joint_as_motor_joint(slf: *mut Joint) -> *mut MotorJoint;
        pub fn MotorJoint_set_linear_offset(slf: *mut MotorJoint, offset: *const Vec2);
        pub fn MotorJoint_get_linear_offset(slf: *const MotorJoint) -> *const Vec2;
        pub fn MotorJoint_set_angular_offset(slf: *mut MotorJoint, offset: f32);
        pub fn MotorJoint_get_angular_offset(slf: *const MotorJoint) -> f32;
        pub fn MotorJoint_set_max_force(slf: *mut MotorJoint, force: f32);
        pub fn MotorJoint_get_max_force(slf: *const MotorJoint) -> f32;
        pub fn MotorJoint_set_max_torque(slf: *mut MotorJoint, torque: f32);
        pub fn MotorJoint_get_max_torque(slf: *const MotorJoint) -> f32;
        pub fn MotorJoint_set_correction_factor(slf: *mut MotorJoint, factor: f32);
        pub fn MotorJoint_get_correction_factor(slf: *const MotorJoint) -> f32;
    }
}
