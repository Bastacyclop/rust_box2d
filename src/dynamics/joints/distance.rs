use wrap::*;
use common::math::Vec2;
use user_data::UserDataTypes;
use dynamics::world::{World, BodyHandle};
use dynamics::joints::{Joint, JointType, JointDef};

pub struct DistanceJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub length: f32,
    pub frequency: f32,
    pub damping_ratio: f32,
}

impl DistanceJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> DistanceJointDef {
        DistanceJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            length: 1.,
            frequency: 0.,
            damping_ratio: 0.,
        }
    }

    pub fn init<U: UserDataTypes>(&mut self,
                                  world: &World<U>,
                                  body_a: BodyHandle,
                                  body_b: BodyHandle,
                                  anchor_a: &Vec2,
                                  anchor_b: &Vec2) {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.body(body_a);
        let b = world.body(body_b);
        self.local_anchor_a = a.local_point(anchor_a);
        self.local_anchor_b = b.local_point(anchor_b);
        self.length = (anchor_b - anchor_a).norm();
    }

    pub fn try_init<U: UserDataTypes>(&mut self,
                                      world: &World<U>,
                                      body_a: BodyHandle,
                                      body_b: BodyHandle,
                                      anchor_a: &Vec2,
                                      anchor_b: &Vec2) -> Option<()> {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.try_body(body_a)?;
        let b = world.try_body(body_b)?;
        self.local_anchor_a = a.local_point(anchor_a);
        self.local_anchor_b = b.local_point(anchor_b);
        self.length = (anchor_b - anchor_a).norm();
        Some(())
    }
}

impl JointDef for DistanceJointDef {
    fn joint_type() -> JointType
        where Self: Sized
    {
        JointType::Distance
    }

    unsafe fn create<U: UserDataTypes>(&self, world: &mut World<U>) -> *mut ffi::Joint {
        ffi::World_create_distance_joint(world.mut_ptr(),
                                         world.body_mut(self.body_a).mut_ptr(),
                                         world.body_mut(self.body_b).mut_ptr(),
                                         self.collide_connected,
                                         self.local_anchor_a,
                                         self.local_anchor_b,
                                         self.length,
                                         self.frequency,
                                         self.damping_ratio)
    }

    unsafe fn try_create<U: UserDataTypes>(&self, world: &mut World<U>) -> Option<*mut ffi::Joint> {
        Some(ffi::World_create_distance_joint(world.mut_ptr(),
                                              world.try_body_mut(self.body_a)?.mut_ptr(),
                                              world.try_body_mut(self.body_b)?.mut_ptr(),
                                              self.collide_connected,
                                              self.local_anchor_a,
                                              self.local_anchor_b,
                                              self.length,
                                              self.frequency,
                                              self.damping_ratio))
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
        unsafe { ffi::DistanceJoint_get_length(self.ptr()) }
    }

    pub fn frequency(&self) -> f32 {
        unsafe { ffi::DistanceJoint_get_frequency(self.ptr()) }
    }

    pub fn damping_ratio(&self) -> f32 {
        unsafe { ffi::DistanceJoint_get_damping_ratio(self.ptr()) }
    }

    pub fn set_length(&mut self, length: f32) {
        unsafe { ffi::DistanceJoint_set_length(self.mut_ptr(), length) }
    }

    pub fn set_frequency(&mut self, frequency: f32) {
        unsafe { ffi::DistanceJoint_set_frequency(self.mut_ptr(), frequency) }
    }

    pub fn set_damping_ratio(&mut self, ratio: f32) {
        unsafe { ffi::DistanceJoint_set_damping_ratio(self.mut_ptr(), ratio) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    pub enum DistanceJoint {}

    extern "C" {
        pub fn World_create_distance_joint(world: *mut World,
                                           body_a: *mut Body,
                                           body_b: *mut Body,
                                           collide_connected: bool,
                                           local_anchor_a: Vec2,
                                           local_anchor_b: Vec2,
                                           length: f32,
                                           frequency: f32,
                                           damping_ratio: f32)
                                           -> *mut Joint;
        pub fn DistanceJoint_as_joint(slf: *mut DistanceJoint) -> *mut Joint;
        pub fn Joint_as_distance_joint(slf: *mut Joint) -> *mut DistanceJoint;
        pub fn DistanceJoint_get_local_anchor_a(slf: *const DistanceJoint) -> *const Vec2;
        pub fn DistanceJoint_get_local_anchor_b(slf: *const DistanceJoint) -> *const Vec2;
        pub fn DistanceJoint_set_length(slf: *mut DistanceJoint, length: f32);
        pub fn DistanceJoint_get_length(slf: *const DistanceJoint) -> f32;
        pub fn DistanceJoint_set_frequency(slf: *mut DistanceJoint, hz: f32);
        pub fn DistanceJoint_get_frequency(slf: *const DistanceJoint) -> f32;
        pub fn DistanceJoint_set_damping_ratio(slf: *mut DistanceJoint, ratio: f32);
        pub fn DistanceJoint_get_damping_ratio(slf: *const DistanceJoint) -> f32;
    }
}
