use wrap::*;
use common::math::Vec2;
use user_data::UserDataTypes;
use dynamics::world::{World, BodyHandle};
use dynamics::joints::{Joint, JointType, JointDef};

pub struct WeldJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub reference_angle: f32,
    pub frequency: f32,
    pub damping_ratio: f32,
}

impl WeldJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> WeldJointDef {
        WeldJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            reference_angle: 0.,
            frequency: 0.,
            damping_ratio: 0.,
        }
    }

    pub fn init<U: UserDataTypes>(&mut self,
                                  world: &World<U>,
                                  body_a: BodyHandle,
                                  body_b: BodyHandle,
                                  anchor: &Vec2) {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.body(body_a);
        let b = world.body(body_b);
        self.local_anchor_a = a.local_point(anchor);
        self.local_anchor_b = b.local_point(anchor);
        self.reference_angle = b.angle() - a.angle();
    }

    pub fn try_init<U: UserDataTypes>(&mut self,
                                      world: &World<U>,
                                      body_a: BodyHandle,
                                      body_b: BodyHandle,
                                      anchor: &Vec2) -> Option<()> {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.try_body_mut(body_a)?;
        let b = world.try_body_mut(body_b)?;
        self.local_anchor_a = a.local_point(anchor);
        self.local_anchor_b = b.local_point(anchor);
        self.reference_angle = b.angle() - a.angle();
        Some(())
    }
}

impl JointDef for WeldJointDef {
    fn joint_type() -> JointType
        where Self: Sized
    {
        JointType::Weld
    }

    unsafe fn create<U: UserDataTypes>(&self, world: &mut World<U>) -> *mut ffi::Joint {
        ffi::World_create_weld_joint(world.mut_ptr(),
                                     world.body_mut(self.body_a).mut_ptr(),
                                     world.body_mut(self.body_b).mut_ptr(),
                                     self.collide_connected,
                                     self.local_anchor_a,
                                     self.local_anchor_b,
                                     self.reference_angle,
                                     self.frequency,
                                     self.damping_ratio)
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
        unsafe { ffi::WeldJoint_get_reference_angle(self.ptr()) }
    }

    pub fn frequency(&self) -> f32 {
        unsafe { ffi::WeldJoint_get_frequency(self.ptr()) }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe { ffi::WeldJoint_get_damping_ratio(self.ptr()) }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe { ffi::WeldJoint_set_frequency(self.mut_ptr(), frequency) }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe { ffi::WeldJoint_set_damping_ratio(self.mut_ptr(), ratio) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    pub enum WeldJoint {}

    extern "C" {
        pub fn World_create_weld_joint(world: *mut World,
                                       body_a: *mut Body,
                                       body_b: *mut Body,
                                       collide_connected: bool,
                                       local_anchor_a: Vec2,
                                       local_anchor_b: Vec2,
                                       reference_angle: f32,
                                       frequency: f32,
                                       damping_ratio: f32)
                                       -> *mut Joint;
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
