use wrap::*;
use common::math::Vec2;
use user_data::UserDataTypes;
use dynamics::world::{World, BodyHandle};
use dynamics::joints::{Joint, JointType, JointDef};

pub struct PulleyJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub ground_anchor_a: Vec2,
    pub ground_anchor_b: Vec2,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub length_a: f32,
    pub length_b: f32,
    pub ratio: f32,
}

impl PulleyJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> PulleyJointDef {
        PulleyJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: true,
            ground_anchor_a: Vec2 { x: -1., y: 1. },
            ground_anchor_b: Vec2 { x: 1., y: 1. },
            local_anchor_a: Vec2 { x: -1., y: 0. },
            local_anchor_b: Vec2 { x: 1., y: 0. },
            length_a: 0.,
            length_b: 0.,
            ratio: 1.,
        }
    }

    pub fn init(&mut self,
                body_a: BodyHandle,
                body_b: BodyHandle,
                ground_a: Vec2,
                ground_b: Vec2,
                anchor_a: &Vec2,
                anchor_b: &Vec2,
                ratio: f32) {
        assert!(ratio > ::std::f32::EPSILON);
        self.body_a = body_a;
        self.body_b = body_b;
        self.ground_anchor_a = ground_a;
        self.ground_anchor_b = ground_b;
        self.length_a = (anchor_a - ground_a).norm();
        self.length_b = (anchor_b - ground_b).norm();
        self.ratio = ratio;
    }
}

impl JointDef for PulleyJointDef {
    fn joint_type() -> JointType
        where Self: Sized
    {
        JointType::Pulley
    }

    unsafe fn create<U: UserDataTypes>(&self, world: &mut World<U>) -> *mut ffi::Joint {
        ffi::World_create_pulley_joint(world.mut_ptr(),
                                       world.body_mut(self.body_a).mut_ptr(),
                                       world.body_mut(self.body_b).mut_ptr(),
                                       self.collide_connected,
                                       self.ground_anchor_a,
                                       self.ground_anchor_b,
                                       self.local_anchor_a,
                                       self.local_anchor_b,
                                       self.length_a,
                                       self.length_b,
                                       self.ratio)
    }

    unsafe fn try_create<U: UserDataTypes>(&self, world: &mut World<U>) -> Option<*mut ffi::Joint> {
        Some(ffi::World_create_pulley_joint(world.mut_ptr(),
                                            world.try_body_mut(self.body_a)?.mut_ptr(),
                                            world.try_body_mut(self.body_b)?.mut_ptr(),
                                            self.collide_connected,
                                            self.ground_anchor_a,
                                            self.ground_anchor_b,
                                            self.local_anchor_a,
                                            self.local_anchor_b,
                                            self.length_a,
                                            self.length_b,
                                            self.ratio))
    }
}

wrap_joint! {
    ffi::PulleyJoint => PulleyJoint (JointType::Pulley)
    < ffi::PulleyJoint_as_joint
    > ffi::Joint_as_pulley_joint
}

impl PulleyJoint {
    pub fn ground_anchor_a(&self) -> Vec2 {
        unsafe { ffi::PulleyJoint_get_ground_anchor_a(self.ptr()) }
    }

    pub fn ground_anchor_b(&self) -> Vec2 {
        unsafe { ffi::PulleyJoint_get_ground_anchor_b(self.ptr()) }
    }

    pub fn length_a(&self) -> f32 {
        unsafe { ffi::PulleyJoint_get_length_a(self.ptr()) }
    }

    pub fn length_b(&self) -> f32 {
        unsafe { ffi::PulleyJoint_get_length_b(self.ptr()) }
    }

    pub fn ratio(&self) -> f32 {
        unsafe { ffi::PulleyJoint_get_ratio(self.ptr()) }
    }

    pub fn current_length_a(&self) -> f32 {
        unsafe { ffi::PulleyJoint_get_current_length_a(self.ptr()) }
    }

    pub fn current_length_b(&self) -> f32 {
        unsafe { ffi::PulleyJoint_get_current_length_b(self.ptr()) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    pub enum PulleyJoint {}

    extern "C" {
        pub fn World_create_pulley_joint(world: *mut World,
                                         body_a: *mut Body,
                                         body_b: *mut Body,
                                         collide_connected: bool,
                                         ground_anchor_a: Vec2,
                                         ground_anchor_b: Vec2,
                                         local_anchor_a: Vec2,
                                         local_anchor_b: Vec2,
                                         length_a: f32,
                                         length_b: f32,
                                         ratio: f32)
                                         -> *mut Joint;
        // pub fn PulleyJointDef_initialize(slf: *mut PulleyJointDef,
        // body_a: *mut Body,
        // body_b: *mut Body,
        // ground_anchor_a: *const Vec2,
        // ground_anchor_b: *const Vec2,
        // anchor_a: *const Vec2,
        // anchor_b: *const Vec2,
        // ratio: f32);
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
