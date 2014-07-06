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

use std::ptr;
use ffi;
use math::{Vec2, Transform};
use dynamics::joints::private::{WrappedJoint, JointDef};
use Wrapped;
use clone_from_ptr;
use collision::{RayCastInput, RayCastOutput, AABB};
use collision::shapes::{
    Shape, ShapeType, UnknownShape, MassData
};
use collision::shapes::private::WrappedShape;

pub mod joints;

wrap!(ffi::World into World)

#[packed]
#[deriving(Clone)]
pub struct Profile {
    pub step: f32,
    pub collide: f32,
    pub solve: f32,
    pub solve_init: f32,
    pub solve_velocity: f32,
    pub solve_position: f32,
    pub brad_phase: f32,
    pub solve_TOI: f32
}

impl World {
    pub fn new(gravity: &Vec2) -> World {
        unsafe {
            Wrapped::from_ptr(ffi::World_new(gravity))
        }
    }
    pub fn create_body(&mut self, def: &BodyDef) -> Body {
        unsafe {
            Wrapped::from_ptr(
                ffi::World_create_body(self.get_mut_ptr(), def)
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
                        def: &JointDef) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::World_create_joint(self.get_mut_ptr(),
                                        def.get_joint_def_ptr())
                )
        }
    }
    pub fn destroy_joint<J: Joint>(&mut self, joint: J) {
        unsafe {
            let mut joint = joint;
            ffi::World_destroy_joint(self.get_mut_ptr(),
                                     joint.get_mut_joint_ptr())
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
    /*pub fn get_mut_body_list(&mut self) -> Vec<Body> {
        unsafe {
            let mut ptr = ffi::World_get_body_list(self.get_mut_ptr());
            
            let mut vec = Vec::new();
            while !ptr.is_null() {
                vec.push(Wrapped::from_ptr(ptr));
                ptr = ffi::Body_get_next(ptr);
            }
            vec
        }
    }*/
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
    pub fn set_gravity(&mut self, gravity: &Vec2) {
        unsafe {
            ffi::World_set_gravity(self.get_mut_ptr(), gravity)
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
    pub fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::World_shift_origin(self.get_mut_ptr(), origin)
        }
    }
    pub fn get_profile(&self) -> Profile {
        unsafe {
            clone_from_ptr(ffi::World_get_profile(self.get_ptr()))
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
    STATIC_BODY = 0,
    KINEMATIC_BODY = 1,
    DYNAMIC_BODY = 2
)

#[allow(dead_code)]
#[packed]
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

impl BodyDef {
    pub fn new() -> BodyDef {
        BodyDef {
            body_type: STATIC_BODY,
            position: Vec2 { x:0., y:0. },
            angle: 0.,
            linear_velocity: Vec2 { x:0., y:0. },
            angular_velocity: 0.,
            linear_damping: 0.,
            angular_damping: 0.,
            allow_sleep: true,
            awake: true,
            fixed_rotation: false,
            bullet: false,
            active: true,
            user_data: ptr::mut_null(),
            gravity_scale: 1.
        }
    }
}

wrap!(ffi::Body into Body)

impl Body {
    pub fn create_fixture(&mut self, def: &FixtureDef) -> Fixture {
        unsafe {
            Wrapped::from_ptr(ffi::Body_create_fixture(self.get_mut_ptr(), def))
        }
    }
    pub fn create_fast_fixture(&mut self, shape: &Shape, density: f32) -> Fixture {
        unsafe {
            Wrapped::from_ptr(
                ffi::Body_create_fast_fixture(self.get_mut_ptr(),
                                              shape.get_shape_ptr(),
                                              density)
                )
        }
    }
    pub fn destroy_fixture(&mut self, fixture: Fixture) {
        unsafe {
            let mut fixture = fixture;
            ffi::Body_destroy_fixture(self.get_mut_ptr(), fixture.get_mut_ptr())
        }
    }
    pub fn set_transform(&mut self, pos: &Vec2, angle: f32) {
        unsafe {
            ffi::Body_set_transform(self.get_mut_ptr(), pos, angle)
        }
    }
    pub fn get_transform(&self) -> Transform {
        unsafe {
            clone_from_ptr(ffi::Body_get_transform(self.get_ptr()))
        }
    }
    pub fn get_position(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_position(self.get_ptr()))
        }
    }
    pub fn get_angle(&self) -> f32 {
        unsafe {
            ffi::Body_get_angle(self.get_ptr())
        }
    }
    pub fn get_world_center(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_world_center(self.get_ptr()))
        }
    }
    pub fn get_local_center(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_local_center(self.get_ptr()))
        }
    }
    pub fn set_linear_velocity(&mut self, v: &Vec2) {
        unsafe {
            ffi::Body_set_linear_velocity(self.get_mut_ptr(), v)
        }
    }
    pub fn get_linear_velocity(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_linear_velocity(self.get_ptr()))
        }
    }
    pub fn set_angular_velocity(&mut self, v: f32) {
        unsafe {
            ffi::Body_set_angular_velocity(self.get_mut_ptr(), v)
        }
    }
    pub fn get_angular_velocity(&self) -> f32 {
        unsafe {
            ffi::Body_get_angular_velocity(self.get_ptr())
        }
    }
    pub fn apply_force(&mut self, force: &Vec2, point: &Vec2, wake: bool) {
        unsafe {
            ffi::Body_apply_force(self.get_mut_ptr(), force, point, wake)
        }
    }
    pub fn apply_force_to_center(&mut self, force: &Vec2, wake: bool) {
        unsafe {
            ffi::Body_apply_force_to_center(self.get_mut_ptr(), force, wake)
        }
    }
    pub fn apply_torque(&mut self, torque: f32, wake: bool) {
        unsafe {
            ffi::Body_apply_torque(self.get_mut_ptr(), torque, wake)
        }
    }
    pub fn apply_linear_impulse(&mut self, impulse: &Vec2,
                                point: &Vec2, wake: bool) {
        unsafe {
            ffi::Body_apply_linear_impulse(self.get_mut_ptr(), impulse,
                                           point, wake)
        }
    }
    pub fn apply_angular_impulse(&mut self, impulse: f32, wake: bool) {
        unsafe {
            ffi::Body_apply_angular_impulse(self.get_mut_ptr(), impulse, wake)
        }
    }
    pub fn get_mass(&self) -> f32 {
        unsafe {
            ffi::Body_get_mass(self.get_ptr())
        }
    }
    pub fn get_inertia(&self) -> f32 {
        unsafe {
            ffi::Body_get_inertia(self.get_ptr())
        }
    }
    pub fn get_mass_data(&self) -> MassData {
        unsafe {
            let mut data = MassData::new();
            ffi::Body_get_mass_data(self.get_ptr(), &mut data);
            data
        }
    }
    pub fn set_mass_data(&mut self, data: &MassData) {
        unsafe {
            ffi::Body_set_mass_data(self.get_mut_ptr(), data)
        }
    }
    pub fn reset_mass_data(&mut self) {
        unsafe {
            ffi::Body_reset_mass_data(self.get_mut_ptr())
        }
    }
    pub fn get_world_point(&self, local: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_world_point(self.get_ptr(), local)
        }
    }
    pub fn get_world_vector(&self, local: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_world_vector(self.get_ptr(), local)
        }
    }
    pub fn get_local_point(&self, world: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_local_point(self.get_ptr(), world)
        }
    }
    pub fn get_local_vector(&self, world: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_local_vector(self.get_ptr(), world)
        }
    }
    pub fn get_linear_velocity_from_world_point(&self, world: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_linear_velocity_from_world_point(self.get_ptr(), world)
        }
    }
    pub fn get_linear_velocity_from_local_point(&self, local: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_linear_velocity_from_local_point(self.get_ptr(), local)
        }
    }
    pub fn get_linear_damping(&self) -> f32 {
        unsafe {
            ffi::Body_get_linear_damping(self.get_ptr())
        }
    }
    pub fn set_linear_damping(&mut self, damping: f32) {
        unsafe {
            ffi::Body_set_linear_damping(self.get_mut_ptr(), damping)
        }
    }
    pub fn get_angular_damping(&self) -> f32 {
        unsafe {
            ffi::Body_get_angular_damping(self.get_ptr())
        }
    }
    pub fn set_angular_damping(&mut self, damping: f32) {
        unsafe {
            ffi::Body_set_angular_damping(self.get_mut_ptr(), damping)
        }
    }
    pub fn get_gravity_scale(&self) -> f32 {
        unsafe {
            ffi::Body_get_gravity_scale(self.get_ptr())
        }
    }
    pub fn set_gravity_scale(&mut self, scale: f32) {
        unsafe {
            ffi::Body_set_gravity_scale(self.get_mut_ptr(), scale)
        }
    }
    pub fn set_type(&mut self, typ: BodyType) {
        unsafe {
            ffi::Body_set_type(self.get_mut_ptr(), typ)
        }
    }
    pub fn get_type(&self) -> BodyType {
        unsafe {
            ffi::Body_get_type(self.get_ptr())
        }
    }
    pub fn set_bullet(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_bullet(self.get_mut_ptr(), flag)
        }
    }
    pub fn is_bullet(&self) -> bool {
        unsafe {
            ffi::Body_is_bullet(self.get_ptr())
        }
    }
    pub fn set_sleeping_allowed(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_sleeping_allowed(self.get_mut_ptr(), flag)
        }
    }
    pub fn is_sleeping_allowed(&self) -> bool {
        unsafe {
            ffi::Body_is_sleeping_allowed(self.get_ptr())
        }
    }
    pub fn set_awake(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_awake(self.get_mut_ptr(), flag)
        }
    }
    pub fn is_awake(&self) -> bool {
        unsafe {
            ffi::Body_is_awake(self.get_ptr())
        }
    }
    pub fn set_active(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_active(self.get_mut_ptr(), flag)
        }
    }
    pub fn is_active(&self) -> bool {
        unsafe {
            ffi::Body_is_active(self.get_ptr())
        }
    }
    pub fn set_fixed_rotation(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_fixed_rotation(self.get_mut_ptr(), flag)
        }
    }
    pub fn is_fixed_rotation(&self) -> bool {
        unsafe {
            ffi::Body_is_fixed_rotation(self.get_ptr())
        }
    }
    /*pub fn get_mut_fixture_list(&mut self) -> Vec<Fixture> {
        unsafe {
        
        }
    }*/
    pub fn get_mut_next(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(ffi::Body_get_next(self.get_mut_ptr()))
        }
    }
    pub fn get_mut_world(&mut self) -> World {
        unsafe {
            Wrapped::from_ptr(ffi::Body_get_world(self.get_mut_ptr()))
        }
    }
    pub fn dump(&mut self) {
        unsafe {
            ffi::Body_dump(self.get_mut_ptr())
        }
    }
}

#[allow(dead_code)]
#[deriving(Clone)]
pub struct Filter {
    pub category_bits: u16,
    pub mask_bits: u16,
    pub group_index: i16
}

impl Filter {
    pub fn new() -> Filter {
        Filter {
            category_bits: 0x0001,
            mask_bits: 0xFFFF,
            group_index: 0
        }
    }
}

#[allow(dead_code)]
pub struct FixtureDef {
    shape: *const ffi::Shape,
    user_data: ffi::UserData,
    pub friction: f32,
    pub restitution: f32,
    pub density: f32,
    pub is_sensor: bool,
    pub filter: Filter
}

impl FixtureDef {
    pub fn new(shape: &Shape) -> FixtureDef {
        unsafe {
            FixtureDef {
                shape: shape.get_shape_ptr(),
                user_data: ptr::mut_null(),
                friction: 0.2,
                restitution: 0.,
                density: 0.,
                is_sensor: false,
                filter: Filter::new()
            }
        }
    }
}

wrap!(ffi::Fixture into Fixture)

impl Fixture {
    pub fn get_type(&self) -> ShapeType {
        unsafe {
            ffi::Fixture_get_type(self.get_ptr())
        }
    }
    pub fn get_shape(&mut self) -> UnknownShape {
        unsafe {
            WrappedShape::from_shape_ptr(
                ffi::Fixture_get_shape(self.get_mut_ptr())
                )
        }
    }
    pub fn set_sensor(&mut self, flag: bool) {
        unsafe {
            ffi::Fixture_set_sensor(self.get_mut_ptr(), flag)
        }
    }
    pub fn is_sensor(&self) -> bool {
        unsafe {
            ffi::Fixture_is_sensor(self.get_ptr())
        }
    }
    pub fn set_filter_data(&mut self, filter: &Filter) {
        unsafe {
            ffi::Fixture_set_filter_data(self.get_mut_ptr(), filter)
        }
    }
    pub fn get_filter_data(&self) -> Filter {
        unsafe {
            clone_from_ptr(ffi::Fixture_get_filter_data(self.get_ptr()))
        }
    }
    pub fn refilter(&mut self) {
        unsafe {
            ffi::Fixture_refilter(self.get_mut_ptr())
        }
    }
    pub fn get_mut_body(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(ffi::Fixture_get_body(self.get_mut_ptr()))
        }
    }
    pub fn get_mut_next(&mut self) -> Fixture {
        unsafe {
            Wrapped::from_ptr(ffi::Fixture_get_next(self.get_mut_ptr()))
        }
    }
    pub fn test_point(&self, point: &Vec2) -> bool {
        unsafe {
            ffi::Fixture_test_point(self.get_ptr(), point)
        }
    }
    pub fn ray_cast(&self, input: &RayCastInput, child_index: uint
                    ) -> RayCastOutput {
        unsafe {
            let mut output = RayCastOutput::new();
            ffi::Fixture_ray_cast(self.get_ptr(), &mut output,
                                  input, child_index as i32);
            output
        }
    }
    pub fn get_mass_data(&self) -> MassData {
        unsafe {
            let mut data = MassData::new();
            ffi::Fixture_get_mass_data(self.get_ptr(), &mut data);
            data
        }
    }
    pub fn set_density(&mut self, density: f32) {
        unsafe {
            ffi::Fixture_set_density(self.get_mut_ptr(), density)
        }
    }
    pub fn get_density(&self) -> f32 {
        unsafe {
            ffi::Fixture_get_density(self.get_ptr())
        }
    }
    pub fn get_friction(&self) -> f32 {
        unsafe {
            ffi::Fixture_get_friction(self.get_ptr())
        }
    }
    pub fn set_friction(&mut self, friction: f32) {
        unsafe {
            ffi::Fixture_set_friction(self.get_mut_ptr(), friction)
        }
    }
    pub fn get_restitution(&self) -> f32 {
        unsafe {
            ffi::Fixture_get_restitution(self.get_ptr())
        }
    }
    pub fn set_restitution(&mut self, restitution: f32) {
        unsafe {
            ffi::Fixture_set_restitution(self.get_mut_ptr(), restitution)
        }
    }
    pub fn get_aabb(&self, child_index: uint) -> AABB {
        unsafe {
            clone_from_ptr(ffi::Fixture_get_aabb(self.get_ptr(), child_index as i32))
        }
    }
    pub fn dump(&mut self, child_count: uint) {
        unsafe {
            ffi::Fixture_dump(self.get_mut_ptr(), child_count as i32)
        }
    }
}
