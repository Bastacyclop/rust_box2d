pub use self::joints::{
    UnknownJoint, JointType, JointDefBase, Joint,
    DistanceJointDef, DistanceJoint,
    FrictionJointDef, FrictionJoint,
    GearJointDef, GearJoint,
    MotorJointDef, MotorJoint,
    MouseJointDef, MouseJoint,
    PrismaticJointDef, PrismaticJoint,
    PulleyJointDef, PulleyJoint,
    RevoluteJointDef, RevoluteJoint,
    RopeJointDef, RopeJoint,
    WeldJointDef, WeldJoint,
    WheelJointDef, WheelJoint
};

use ffi;
use math::Vec2;
use dynamics::joints::WrappedJoint;
use Wrapped;

pub mod joints;

wrap!(ffi::World into World)

impl World {
    pub fn new(gravity: Vec2) -> World {
        unsafe {
            Wrapped::from_ptr(ffi::World_new(&gravity))
        }
    }
    pub fn create_body(&mut self, def: BodyDef) -> Body {
        unsafe {
            Wrapped::from_ptr(
                ffi::World_create_body(self.get_mut_ptr(), &def)
                )
        }
    }
    pub fn destroy_body(&mut self, body: Body) {
        unsafe {
            let mut body = body;
            ffi::World_destroy_body(self.get_mut_ptr(), body.get_mut_ptr())
        }
    }
    pub fn create_joint(&mut self,
                        def: JointDefBase) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::World_create_joint(self.get_mut_ptr(), &def)
                )
        }
    }
    pub fn destroy_joint(&mut self, joint: UnknownJoint) {
        unsafe {
            let mut joint = joint;
            ffi::World_destroy_joint(self.get_mut_ptr(), joint.get_mut_joint_ptr())
        }
    }
    pub fn step(&mut self,
                time_step: f32,
                velocity_iterations: i32,
                position_iterations: i32) {
        unsafe {
            ffi::World_step(self.get_mut_ptr(),
                            time_step,
                            velocity_iterations,
                            position_iterations)
        }
    }
    pub fn clear_forces(&mut self) {
        unsafe {
            ffi::World_clear_forces(self.get_mut_ptr())
        }
    }
    pub fn draw_debug_data(&mut self) {
        unsafe {
            ffi::World_draw_debug_data(self.get_mut_ptr())
        }
    }
    pub fn get_mut_body_list(&mut self) -> Vec<Body> {
        unsafe {
            let mut ptr = ffi::World_get_body_list(self.get_mut_ptr());
            
            let mut vec = Vec::new();
            while !ptr.is_null() {
                vec.push(Wrapped::from_ptr(ptr));
                ptr = ffi::Body_get_next(ptr);
            }
            vec
        }
    }
    pub fn set_allow_sleeping(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_allow_sleeping(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_allow_sleeping(&self) -> bool {
        unsafe {
            ffi::World_get_allow_sleeping(self.get_ptr())
        }
    }
    pub fn set_warm_starting(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_warm_starting(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_warm_starting(&self) -> bool {
        unsafe {
            ffi::World_get_warm_starting(self.get_ptr())
        }
    }
    pub fn set_continuous_physics(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_continuous_physics(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_continuous_physics(&self) -> bool {
        unsafe {
            ffi::World_get_continuous_physics(self.get_ptr())
        }
    }
    pub fn set_sub_stepping(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_sub_stepping(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_sub_stepping(&self) -> bool {
        unsafe {
            ffi::World_get_sub_stepping(self.get_ptr())
        }
    }
    pub fn get_proxy_count(&self) -> uint {
        unsafe {
            ffi::World_get_proxy_count(self.get_ptr()) as uint
        }
    }
    pub fn get_body_count(&self) -> uint {
        unsafe {
            ffi::World_get_body_count(self.get_ptr()) as uint
        }
    }
    pub fn get_joint_count(&self) -> uint {
        unsafe {
            ffi::World_get_joint_count(self.get_ptr()) as uint
        }
    }
    pub fn get_contact_count(&self) -> uint {
        unsafe {
            ffi::World_get_contact_count(self.get_ptr()) as uint
        }
    }
    pub fn get_tree_height(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_height(self.get_ptr())
        }
    }
    pub fn get_tree_balance(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_balance(self.get_ptr())
        }
    }
    pub fn get_tree_quality(&self) -> f32 {
        unsafe {
            ffi::World_get_tree_quality(self.get_ptr())
        }
    }
    pub fn set_gravity(&mut self, gravity: Vec2) {
        unsafe {
            ffi::World_set_gravity(self.get_mut_ptr(), &gravity)
        }
    }
    pub fn get_gravity(&self) -> Vec2 {
        unsafe {
            ffi::World_get_gravity(self.get_ptr())
        }
    }
    pub fn is_locked(&self) -> bool {
        unsafe {
            ffi::World_is_locked(self.get_ptr())
        }
    }
    pub fn set_auto_clear_forces(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_auto_clear_forces(self.get_mut_ptr(), flag)
        }
    }
    pub fn get_auto_clear_forces(&self) -> bool {
        unsafe {
            ffi::World_get_auto_clear_forces(self.get_ptr())
        }
    }
    pub fn shift_origin(&mut self, origin: Vec2) {
        unsafe {
            ffi::World_shift_origin(self.get_mut_ptr(), &origin)
        }
    }
    pub fn dump(&mut self) {
        unsafe {
            ffi::World_dump(self.get_mut_ptr())
        }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            ffi::World_drop(self.get_mut_ptr())
        }
    }
}

c_enum!(BodyType with
    STATIC = 0,
    KINEMATIC = 1,
    DYNAMIC = 2
)

#[allow(dead_code)]
pub struct BodyDef {
    pub body_type: BodyType,
    pub position: Vec2,
    pub angle: f32,
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub allow_sleep: bool,
    pub awake: bool,
    pub fixed_rotation: bool,
    pub bullet: bool,
    pub active: bool,
    user_data: ffi::UserData,
    pub gravity_scale: f32,
}

wrap!(ffi::Body into Body)

impl Drop for Body {
    fn drop(&mut self) {
    }
}

#[allow(dead_code)]
pub struct Filter {
    pub category_bits: u16,
    pub mask_bits: u16,
    pub group_index: i16
}

#[allow(dead_code)]
pub struct FixtureDef {
    shape: *const ffi::Shape,
    user_data: ffi::UserData,
    pub friction: f32,
    pub density: f32,
    pub is_sensor: bool,
    pub filter: Filter
}
