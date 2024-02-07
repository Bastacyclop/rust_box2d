use wrap::*;
use common::math::Vec2;
use user_data::UserDataTypes;
use dynamics::world::{World, BodyHandle};
use dynamics::joints::{Joint, JointType, JointDef};

pub struct MouseJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub target: Vec2,
    pub max_force: f32,
    pub frequency: f32,
    pub damping_ratio: f32,
}

impl MouseJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> MouseJointDef {
        MouseJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            target: Vec2 { x: 0., y: 0. },
            max_force: 0.,
            frequency: 5.,
            damping_ratio: 0.7,
        }
    }
}

impl JointDef for MouseJointDef {
    fn joint_type() -> JointType
        where Self: Sized
    {
        JointType::Mouse
    }

    unsafe fn create<U: UserDataTypes>(&self, world: &mut World<U>) -> *mut ffi::Joint {
        self.try_create(world).expect("joint create failed: invalid body handle")
    }

    unsafe fn try_create<U: UserDataTypes>(&self, world: &mut World<U>) -> Option<*mut ffi::Joint> {
        Some(ffi::World_create_mouse_joint(world.mut_ptr(),
                                           world.try_body_mut(self.body_a)?.mut_ptr(),
                                           world.try_body_mut(self.body_b)?.mut_ptr(),
                                           self.collide_connected,
                                           self.target,
                                           self.max_force,
                                           self.frequency,
                                           self.damping_ratio))
    }
}

wrap_joint! {
    ffi::MouseJoint => MouseJoint (JointType::Mouse)
    < ffi::MouseJoint_as_joint
    > ffi::Joint_as_mouse_joint
}

impl MouseJoint {
    pub fn target<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::MouseJoint_get_target(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn max_force(&self) -> f32 {
        unsafe { ffi::MouseJoint_get_max_force(self.ptr()) }
    }

    pub fn frequency(&self) -> f32 {
        unsafe { ffi::MouseJoint_get_frequency(self.ptr()) }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe { ffi::MouseJoint_get_damping_ratio(self.ptr()) }
    }

    pub fn set_target(&mut self, target: &Vec2) {
        unsafe { ffi::MouseJoint_set_target(self.mut_ptr(), target) }
    }

    pub fn set_max_force(&mut self, force: f32) {
        unsafe { ffi::MouseJoint_set_max_force(self.mut_ptr(), force) }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe { ffi::MouseJoint_set_frequency(self.mut_ptr(), frequency) }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe { ffi::MouseJoint_set_damping_ratio(self.mut_ptr(), ratio) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    pub enum MouseJoint {}

    extern "C" {
        pub fn World_create_mouse_joint(world: *mut World,
                                        body_a: *mut Body,
                                        body_b: *mut Body,
                                        collide_connected: bool,
                                        target: Vec2,
                                        max_force: f32,
                                        frequency: f32,
                                        damping_ratio: f32)
                                        -> *mut Joint;
        pub fn MouseJoint_as_joint(slf: *mut MouseJoint) -> *mut Joint;
        pub fn Joint_as_mouse_joint(slf: *mut Joint) -> *mut MouseJoint;
        pub fn MouseJoint_set_target(slf: *mut MouseJoint, target: *const Vec2);
        pub fn MouseJoint_get_target(slf: *const MouseJoint) -> *const Vec2;
        pub fn MouseJoint_set_max_force(slf: *mut MouseJoint, force: f32);
        pub fn MouseJoint_get_max_force(slf: *const MouseJoint) -> f32;
        pub fn MouseJoint_set_frequency(slf: *mut MouseJoint, hz: f32);
        pub fn MouseJoint_get_frequency(slf: *const MouseJoint) -> f32;
        pub fn MouseJoint_set_damping_ratio(slf: *mut MouseJoint, ratio: f32);
        pub fn MouseJoint_get_damping_ratio(slf: *const MouseJoint) -> f32;
    }
}
