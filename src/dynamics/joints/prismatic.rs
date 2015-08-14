use wrap::*;
use common::math::Vec2;
use dynamics::world::{ World, BodyHandle };
use dynamics::joints::{ Joint, JointType, JointDef };

pub struct PrismaticJointDef {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub collide_connected: bool,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub local_axis_a: Vec2,
    pub reference_angle: f32,
    pub enable_limit: bool,
    pub lower_translation: f32,
    pub upper_translation: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32
}

impl PrismaticJointDef {
    pub fn new(body_a: BodyHandle, body_b: BodyHandle) -> PrismaticJointDef {
        PrismaticJointDef {
            body_a: body_a,
            body_b: body_b,
            collide_connected: false,
            local_anchor_a: Vec2 { x: 0., y: 0. },
            local_anchor_b: Vec2 { x: 0., y: 0. },
            local_axis_a: Vec2 { x: 1., y: 0. },
            reference_angle: 0.,
            enable_limit: false,
            lower_translation: 0.,
            upper_translation: 0.,
            enable_motor: false,
            max_motor_force: 0.,
            motor_speed: 0.
        }
    }

    pub fn init(&mut self,
                world: &World,
                body_a: BodyHandle,
                body_b: BodyHandle,
                anchor: &Vec2,
                axis: &Vec2) {
        self.body_a = body_a;
        self.body_b = body_b;
        let a = world.get_body(body_a);
        let b = world.get_body(body_a);
        self.local_anchor_a = a.local_point(anchor);
        self.local_anchor_b = b.local_point(anchor);
        self.local_axis_a = a.local_vector(axis);
        self.reference_angle = b.angle() - a.angle();
    }
}

impl JointDef for PrismaticJointDef {
    fn joint_type() -> JointType where Self: Sized { JointType::Prismatic }

    unsafe fn create(&self, world: &mut World) -> *mut ffi::Joint {
        ffi::World_create_prismatic_joint(
            world.mut_ptr(),
            world.get_body_mut(self.body_a).mut_ptr(),
            world.get_body_mut(self.body_b).mut_ptr(),
            self.collide_connected,
            self.local_anchor_a,
            self.local_anchor_b,
            self.local_axis_a,
            self.reference_angle,
            self.enable_limit,
            self.lower_translation,
            self.upper_translation,
            self.enable_motor,
            self.max_motor_force,
            self.motor_speed
        )
    }
}

wrap_joint! {
    ffi::PrismaticJoint => PrismaticJoint (JointType::Prismatic)
    < ffi::PrismaticJoint_as_joint
    > ffi::Joint_as_prismatic_joint
}

impl PrismaticJoint {
    pub fn local_anchor_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::PrismaticJoint_get_local_anchor_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_anchor_b<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::PrismaticJoint_get_local_anchor_b(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_axis_a<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::PrismaticJoint_get_local_axis_a(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn reference_angle(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_reference_angle(self.ptr())
        }
    }

    pub fn joint_translation(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_translation(self.ptr())
        }
    }

    pub fn joint_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_joint_speed(self.ptr())
        }
    }

    pub fn is_limit_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_limit_enabled(self.ptr())
        }
    }

    pub fn lower_limit(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_lower_limit(self.ptr())
        }
    }

    pub fn upper_limit(&self) -> f32 {
        unsafe {
             ffi::PrismaticJoint_get_upper_limit(self.ptr())
        }
    }

    pub fn is_motor_enabled(&self) -> bool {
        unsafe {
            ffi::PrismaticJoint_is_motor_enabled(self.ptr())
        }
    }

    pub fn motor_speed(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_speed(self.ptr())
        }
    }

    pub fn max_motor_force(&self) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_max_motor_force(self.ptr())
        }
    }

    pub fn motor_force(&self, inv_dt: f32) -> f32 {
        unsafe {
            ffi::PrismaticJoint_get_motor_force(self.ptr(), inv_dt)
        }
    }

    pub fn enable_limit(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_limit(self.mut_ptr(), flag)
        }
    }

    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        unsafe {
            ffi::PrismaticJoint_set_limits(self.mut_ptr(), lower, upper)
        }
    }

    pub fn enable_motor(&mut self, flag: bool) {
        unsafe {
            ffi::PrismaticJoint_enable_motor(self.mut_ptr(), flag)
        }
    }

    pub fn set_motor_speed(&mut self, speed: f32) {
        unsafe {
            ffi::PrismaticJoint_set_motor_speed(self.mut_ptr(), speed)
        }
    }

    pub fn set_max_motor_force(&mut self, force: f32) {
        unsafe {
            ffi::PrismaticJoint_set_max_motor_force(self.mut_ptr(), force)
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use dynamics::world::ffi::World;
    pub use dynamics::body::ffi::Body;
    pub use dynamics::joints::ffi::Joint;
    use common::math::Vec2;

    pub enum PrismaticJoint {}

    extern {
        pub fn World_create_prismatic_joint(
            world: *mut World,
            body_a: *mut Body,
            body_b: *mut Body,
            collide_connected: bool,
            local_anchor_a: Vec2,
            local_anchor_b: Vec2,
            local_axis_a: Vec2,
            reference_angle: f32,
            enable_limit: bool,
            lower_translation: f32,
            upper_translation: f32,
            enable_motor: bool,
            max_motor_force: f32,
            motor_speed: f32
        ) -> *mut Joint;
        /*pub fn PrismaticJointDef_initialize(slf: *mut PrismaticJointDef,
                                            body_a: *mut Body,
                                            body_b: *mut Body,
                                            anchor: *const Vec2,
                                            axis: *const Vec2);*/
        pub fn PrismaticJoint_as_joint(slf: *mut PrismaticJoint) -> *mut Joint;
        pub fn Joint_as_prismatic_joint(slf: *mut Joint) -> *mut PrismaticJoint;
        pub fn PrismaticJoint_get_local_anchor_a(slf: *const PrismaticJoint
                                                 ) -> *const Vec2;
        pub fn PrismaticJoint_get_local_anchor_b(slf: *const PrismaticJoint
                                                 ) -> *const Vec2;
        pub fn PrismaticJoint_get_local_axis_a(slf: *const PrismaticJoint
                                               ) -> *const Vec2;
        pub fn PrismaticJoint_get_reference_angle(slf: *const PrismaticJoint) -> f32;
        pub fn PrismaticJoint_get_joint_translation(slf: *const PrismaticJoint
                                                    ) -> f32;
        pub fn PrismaticJoint_get_joint_speed(slf: *const PrismaticJoint) -> f32;
        pub fn PrismaticJoint_is_limit_enabled(slf: *const PrismaticJoint) -> bool;
        pub fn PrismaticJoint_enable_limit(slf: *mut PrismaticJoint, flag: bool);
        pub fn PrismaticJoint_get_lower_limit(slf: *const PrismaticJoint) -> f32;
        pub fn PrismaticJoint_get_upper_limit(slf: *const PrismaticJoint) -> f32;
        pub fn PrismaticJoint_set_limits(slf: *mut PrismaticJoint,
                                         lower: f32, upper: f32);
        pub fn PrismaticJoint_is_motor_enabled(slf: *const PrismaticJoint) -> bool;
        pub fn PrismaticJoint_enable_motor(slf: *mut PrismaticJoint, flag: bool);
        pub fn PrismaticJoint_set_motor_speed(slf: *mut PrismaticJoint, speed: f32);
        pub fn PrismaticJoint_get_motor_speed(slf: *const PrismaticJoint) -> f32;
        pub fn PrismaticJoint_set_max_motor_force(slf: *mut PrismaticJoint,
                                                  force: f32);
        pub fn PrismaticJoint_get_max_motor_force(slf: *const PrismaticJoint) -> f32;
        pub fn PrismaticJoint_get_motor_force(slf: *const PrismaticJoint,
                                              inv_dt: f32) -> f32;

    }
}
