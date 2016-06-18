use wrap::*;
use common::math::Vec2;
use user_data::UserDataTypes;
use dynamics::world::{World, BodyHandle};
use dynamics::joints::{Joint, JointType, JointDef};

pub struct FrictionJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub max_force: f32,
    pub max_torque: f32,
}

impl FrictionJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> FrictionJointDef {
        FrictionJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            max_force: 0.,
            max_torque: 0.,
        }
    }

    pub fn init<U: UserDataTypes>(&mut self,
                                  world: &World<U>,
                                  body_a: BodyHandle,
                                  body_b: BodyHandle,
                                  anchor: &Vec2) {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.get_body(body_a);
        let b = world.get_body(body_a);
        self.local_anchor_a = a.local_point(anchor);
        self.local_anchor_b = b.local_point(anchor);
    }
}

impl JointDef for FrictionJointDef {
    fn joint_type() -> JointType
        where Self: Sized
    {
        JointType::Friction
    }

    unsafe fn create<U: UserDataTypes>(&self, world: &mut World<U>) -> *mut ffi::Joint {
        ffi::World_create_friction_joint(world.mut_ptr(),
                                         world.get_body_mut(self.body_a).mut_ptr(),
                                         world.get_body_mut(self.body_b).mut_ptr(),
                                         self.collide_connected,
                                         self.local_anchor_a,
                                         self.local_anchor_b,
                                         self.max_force,
                                         self.max_torque)
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
        unsafe { ffi::FrictionJoint_get_max_force(self.ptr()) }
    }

    pub fn max_torque(&self) -> f32 {
        unsafe { ffi::FrictionJoint_get_max_torque(self.ptr()) }
    }

    pub fn set_max_force(&mut self, force: f32) {
        unsafe { ffi::FrictionJoint_set_max_force(self.mut_ptr(), force) }
    }

    pub fn set_max_torque(&mut self, torque: f32) {
        unsafe { ffi::FrictionJoint_set_max_torque(self.mut_ptr(), torque) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    pub enum FrictionJoint {}

    extern "C" {
        pub fn World_create_friction_joint(world: *mut World,
                                           body_a: *mut Body,
                                           body_b: *mut Body,
                                           collide_connected: bool,
                                           local_anchor_a: Vec2,
                                           local_anchor_b: Vec2,
                                           max_force: f32,
                                           max_torque: f32)
                                           -> *mut Joint;
        pub fn FrictionJoint_as_joint(slf: *mut FrictionJoint) -> *mut Joint;
        pub fn Joint_as_friction_joint(slf: *mut Joint) -> *mut FrictionJoint;
        pub fn FrictionJoint_get_local_anchor_a(slf: *const FrictionJoint) -> *const Vec2;
        pub fn FrictionJoint_get_local_anchor_b(slf: *const FrictionJoint) -> *const Vec2;
        pub fn FrictionJoint_set_max_force(slf: *mut FrictionJoint, force: f32);
        pub fn FrictionJoint_get_max_force(slf: *const FrictionJoint) -> f32;
        pub fn FrictionJoint_set_max_torque(slf: *mut FrictionJoint, torque: f32);
        pub fn FrictionJoint_get_max_torque(slf: *const FrictionJoint) -> f32;
    }
}
