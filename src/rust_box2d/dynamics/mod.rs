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
                ffi::World_create_body(self.mut_ptr(), def)
                )
        }
    }
    pub fn destroy_body(&mut self, body: Body) {
        unsafe {
            let mut body = body;
            ffi::World_destroy_body(self.mut_ptr(), body.mut_ptr())
        }
    }
    pub fn create_joint(&mut self,
                        def: &JointDef) -> UnknownJoint {
        unsafe {
            WrappedJoint::from_joint_ptr(
                ffi::World_create_joint(self.mut_ptr(),
                                        def.joint_def_ptr())
                )
        }
    }
    pub fn destroy_joint<J: Joint>(&mut self, joint: J) {
        unsafe {
            let mut joint = joint;
            ffi::World_destroy_joint(self.mut_ptr(),
                                     joint.mut_joint_ptr())
        }
    }
    pub fn step(&mut self,
                time_step: f32,
                velocity_iterations: i32,
                position_iterations: i32) {
        unsafe {
            ffi::World_step(self.mut_ptr(),
                            time_step,
                            velocity_iterations,
                            position_iterations)
        }
    }
    pub fn clear_forces(&mut self) {
        unsafe {
            ffi::World_clear_forces(self.mut_ptr())
        }
    }
    pub fn draw_debug_data(&mut self) {
        unsafe {
            ffi::World_draw_debug_data(self.mut_ptr())
        }
    }
    /*pub fn mut_body_list(&mut self) -> Vec<Body> {
        unsafe {
            let mut ptr = ffi::World_get_body_list(self.mut_ptr());
            
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
            ffi::World_set_allow_sleeping(self.mut_ptr(), flag)
        }
    }
    pub fn allow_sleeping(&self) -> bool {
        unsafe {
            ffi::World_get_allow_sleeping(self.ptr())
        }
    }
    pub fn set_warm_starting(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_warm_starting(self.mut_ptr(), flag)
        }
    }
    pub fn warm_starting(&self) -> bool {
        unsafe {
            ffi::World_get_warm_starting(self.ptr())
        }
    }
    pub fn set_continuous_physics(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_continuous_physics(self.mut_ptr(), flag)
        }
    }
    pub fn continuous_physics(&self) -> bool {
        unsafe {
            ffi::World_get_continuous_physics(self.ptr())
        }
    }
    pub fn set_sub_stepping(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_sub_stepping(self.mut_ptr(), flag)
        }
    }
    pub fn sub_stepping(&self) -> bool {
        unsafe {
            ffi::World_get_sub_stepping(self.ptr())
        }
    }
    pub fn proxy_count(&self) -> uint {
        unsafe {
            ffi::World_get_proxy_count(self.ptr()) as uint
        }
    }
    pub fn body_count(&self) -> uint {
        unsafe {
            ffi::World_get_body_count(self.ptr()) as uint
        }
    }
    pub fn joint_count(&self) -> uint {
        unsafe {
            ffi::World_get_joint_count(self.ptr()) as uint
        }
    }
    pub fn contact_count(&self) -> uint {
        unsafe {
            ffi::World_get_contact_count(self.ptr()) as uint
        }
    }
    pub fn tree_height(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_height(self.ptr())
        }
    }
    pub fn tree_balance(&self) -> i32 {
        unsafe {
            ffi::World_get_tree_balance(self.ptr())
        }
    }
    pub fn tree_quality(&self) -> f32 {
        unsafe {
            ffi::World_get_tree_quality(self.ptr())
        }
    }
    pub fn set_gravity(&mut self, gravity: &Vec2) {
        unsafe {
            ffi::World_set_gravity(self.mut_ptr(), gravity)
        }
    }
    pub fn gravity(&self) -> Vec2 {
        unsafe {
            ffi::World_get_gravity(self.ptr())
        }
    }
    pub fn is_locked(&self) -> bool {
        unsafe {
            ffi::World_is_locked(self.ptr())
        }
    }
    pub fn set_auto_clear_forces(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_auto_clear_forces(self.mut_ptr(), flag)
        }
    }
    pub fn auto_clear_forces(&self) -> bool {
        unsafe {
            ffi::World_get_auto_clear_forces(self.ptr())
        }
    }
    pub fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::World_shift_origin(self.mut_ptr(), origin)
        }
    }
    pub fn profile(&self) -> Profile {
        unsafe {
            clone_from_ptr(ffi::World_get_profile(self.ptr()))
        }
    }
    pub fn dump(&mut self) {
        unsafe {
            ffi::World_dump(self.mut_ptr())
        }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            ffi::World_drop(self.mut_ptr())
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
            Wrapped::from_ptr(ffi::Body_create_fixture(self.mut_ptr(), def))
        }
    }
    pub fn create_fast_fixture(&mut self, shape: &Shape, density: f32) -> Fixture {
        unsafe {
            Wrapped::from_ptr(
                ffi::Body_create_fast_fixture(self.mut_ptr(),
                                              shape.shape_ptr(),
                                              density)
                )
        }
    }
    pub fn destroy_fixture(&mut self, fixture: Fixture) {
        unsafe {
            let mut fixture = fixture;
            ffi::Body_destroy_fixture(self.mut_ptr(), fixture.mut_ptr())
        }
    }
    pub fn set_transform(&mut self, pos: &Vec2, angle: f32) {
        unsafe {
            ffi::Body_set_transform(self.mut_ptr(), pos, angle)
        }
    }
    pub fn transform(&self) -> Transform {
        unsafe {
            clone_from_ptr(ffi::Body_get_transform(self.ptr()))
        }
    }
    pub fn position(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_position(self.ptr()))
        }
    }
    pub fn angle(&self) -> f32 {
        unsafe {
            ffi::Body_get_angle(self.ptr())
        }
    }
    pub fn world_center(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_world_center(self.ptr()))
        }
    }
    pub fn local_center(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_local_center(self.ptr()))
        }
    }
    pub fn set_linear_velocity(&mut self, v: &Vec2) {
        unsafe {
            ffi::Body_set_linear_velocity(self.mut_ptr(), v)
        }
    }
    pub fn linear_velocity(&self) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::Body_get_linear_velocity(self.ptr()))
        }
    }
    pub fn set_angular_velocity(&mut self, v: f32) {
        unsafe {
            ffi::Body_set_angular_velocity(self.mut_ptr(), v)
        }
    }
    pub fn angular_velocity(&self) -> f32 {
        unsafe {
            ffi::Body_get_angular_velocity(self.ptr())
        }
    }
    pub fn apply_force(&mut self, force: &Vec2, point: &Vec2, wake: bool) {
        unsafe {
            ffi::Body_apply_force(self.mut_ptr(), force, point, wake)
        }
    }
    pub fn apply_force_to_center(&mut self, force: &Vec2, wake: bool) {
        unsafe {
            ffi::Body_apply_force_to_center(self.mut_ptr(), force, wake)
        }
    }
    pub fn apply_torque(&mut self, torque: f32, wake: bool) {
        unsafe {
            ffi::Body_apply_torque(self.mut_ptr(), torque, wake)
        }
    }
    pub fn apply_linear_impulse(&mut self, impulse: &Vec2,
                                point: &Vec2, wake: bool) {
        unsafe {
            ffi::Body_apply_linear_impulse(self.mut_ptr(), impulse,
                                           point, wake)
        }
    }
    pub fn apply_angular_impulse(&mut self, impulse: f32, wake: bool) {
        unsafe {
            ffi::Body_apply_angular_impulse(self.mut_ptr(), impulse, wake)
        }
    }
    pub fn mass(&self) -> f32 {
        unsafe {
            ffi::Body_get_mass(self.ptr())
        }
    }
    pub fn inertia(&self) -> f32 {
        unsafe {
            ffi::Body_get_inertia(self.ptr())
        }
    }
    pub fn mass_data(&self) -> MassData {
        unsafe {
            let mut data = MassData::new();
            ffi::Body_get_mass_data(self.ptr(), &mut data);
            data
        }
    }
    pub fn set_mass_data(&mut self, data: &MassData) {
        unsafe {
            ffi::Body_set_mass_data(self.mut_ptr(), data)
        }
    }
    pub fn reset_mass_data(&mut self) {
        unsafe {
            ffi::Body_reset_mass_data(self.mut_ptr())
        }
    }
    pub fn world_point(&self, local: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_world_point(self.ptr(), local)
        }
    }
    pub fn world_vector(&self, local: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_world_vector(self.ptr(), local)
        }
    }
    pub fn local_point(&self, world: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_local_point(self.ptr(), world)
        }
    }
    pub fn local_vector(&self, world: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_local_vector(self.ptr(), world)
        }
    }
    pub fn linear_velocity_from_world_point(&self, world: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_linear_velocity_from_world_point(self.ptr(), world)
        }
    }
    pub fn linear_velocity_from_local_point(&self, local: &Vec2) -> Vec2 {
        unsafe {
            ffi::Body_get_linear_velocity_from_local_point(self.ptr(), local)
        }
    }
    pub fn linear_damping(&self) -> f32 {
        unsafe {
            ffi::Body_get_linear_damping(self.ptr())
        }
    }
    pub fn set_linear_damping(&mut self, damping: f32) {
        unsafe {
            ffi::Body_set_linear_damping(self.mut_ptr(), damping)
        }
    }
    pub fn angular_damping(&self) -> f32 {
        unsafe {
            ffi::Body_get_angular_damping(self.ptr())
        }
    }
    pub fn set_angular_damping(&mut self, damping: f32) {
        unsafe {
            ffi::Body_set_angular_damping(self.mut_ptr(), damping)
        }
    }
    pub fn gravity_scale(&self) -> f32 {
        unsafe {
            ffi::Body_get_gravity_scale(self.ptr())
        }
    }
    pub fn set_gravity_scale(&mut self, scale: f32) {
        unsafe {
            ffi::Body_set_gravity_scale(self.mut_ptr(), scale)
        }
    }
    pub fn set_type(&mut self, typ: BodyType) {
        unsafe {
            ffi::Body_set_type(self.mut_ptr(), typ)
        }
    }
    pub fn body_type(&self) -> BodyType {
        unsafe {
            ffi::Body_get_type(self.ptr())
        }
    }
    pub fn set_bullet(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_bullet(self.mut_ptr(), flag)
        }
    }
    pub fn is_bullet(&self) -> bool {
        unsafe {
            ffi::Body_is_bullet(self.ptr())
        }
    }
    pub fn set_sleeping_allowed(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_sleeping_allowed(self.mut_ptr(), flag)
        }
    }
    pub fn is_sleeping_allowed(&self) -> bool {
        unsafe {
            ffi::Body_is_sleeping_allowed(self.ptr())
        }
    }
    pub fn set_awake(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_awake(self.mut_ptr(), flag)
        }
    }
    pub fn is_awake(&self) -> bool {
        unsafe {
            ffi::Body_is_awake(self.ptr())
        }
    }
    pub fn set_active(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_active(self.mut_ptr(), flag)
        }
    }
    pub fn is_active(&self) -> bool {
        unsafe {
            ffi::Body_is_active(self.ptr())
        }
    }
    pub fn set_fixed_rotation(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_fixed_rotation(self.mut_ptr(), flag)
        }
    }
    pub fn is_fixed_rotation(&self) -> bool {
        unsafe {
            ffi::Body_is_fixed_rotation(self.ptr())
        }
    }
    /*pub fn mut_fixture_list(&mut self) -> Vec<Fixture> {
        unsafe {
        
        }
    }*/
    pub fn mut_next(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(ffi::Body_get_next(self.mut_ptr()))
        }
    }
    pub fn mut_world(&mut self) -> World {
        unsafe {
            Wrapped::from_ptr(ffi::Body_get_world(self.mut_ptr()))
        }
    }
    pub fn dump(&mut self) {
        unsafe {
            ffi::Body_dump(self.mut_ptr())
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
                shape: shape.shape_ptr(),
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
    pub fn shape_type(&self) -> ShapeType {
        unsafe {
            ffi::Fixture_get_type(self.ptr())
        }
    }
    pub fn shape(&mut self) -> UnknownShape {
        unsafe {
            WrappedShape::from_shape_ptr(
                ffi::Fixture_get_shape(self.mut_ptr())
                )
        }
    }
    pub fn set_sensor(&mut self, flag: bool) {
        unsafe {
            ffi::Fixture_set_sensor(self.mut_ptr(), flag)
        }
    }
    pub fn is_sensor(&self) -> bool {
        unsafe {
            ffi::Fixture_is_sensor(self.ptr())
        }
    }
    pub fn set_filter_data(&mut self, filter: &Filter) {
        unsafe {
            ffi::Fixture_set_filter_data(self.mut_ptr(), filter)
        }
    }
    pub fn filter_data(&self) -> Filter {
        unsafe {
            clone_from_ptr(ffi::Fixture_get_filter_data(self.ptr()))
        }
    }
    pub fn refilter(&mut self) {
        unsafe {
            ffi::Fixture_refilter(self.mut_ptr())
        }
    }
    pub fn mut_body(&mut self) -> Body {
        unsafe {
            Wrapped::from_ptr(ffi::Fixture_get_body(self.mut_ptr()))
        }
    }
    pub fn mut_next(&mut self) -> Fixture {
        unsafe {
            Wrapped::from_ptr(ffi::Fixture_get_next(self.mut_ptr()))
        }
    }
    pub fn test_point(&self, point: &Vec2) -> bool {
        unsafe {
            ffi::Fixture_test_point(self.ptr(), point)
        }
    }
    pub fn ray_cast(&self, input: &RayCastInput, child_index: uint
                    ) -> RayCastOutput {
        unsafe {
            let mut output = RayCastOutput::new();
            ffi::Fixture_ray_cast(self.ptr(), &mut output,
                                  input, child_index as i32);
            output
        }
    }
    pub fn mass_data(&self) -> MassData {
        unsafe {
            let mut data = MassData::new();
            ffi::Fixture_get_mass_data(self.ptr(), &mut data);
            data
        }
    }
    pub fn set_density(&mut self, density: f32) {
        unsafe {
            ffi::Fixture_set_density(self.mut_ptr(), density)
        }
    }
    pub fn density(&self) -> f32 {
        unsafe {
            ffi::Fixture_get_density(self.ptr())
        }
    }
    pub fn friction(&self) -> f32 {
        unsafe {
            ffi::Fixture_get_friction(self.ptr())
        }
    }
    pub fn set_friction(&mut self, friction: f32) {
        unsafe {
            ffi::Fixture_set_friction(self.mut_ptr(), friction)
        }
    }
    pub fn restitution(&self) -> f32 {
        unsafe {
            ffi::Fixture_get_restitution(self.ptr())
        }
    }
    pub fn set_restitution(&mut self, restitution: f32) {
        unsafe {
            ffi::Fixture_set_restitution(self.mut_ptr(), restitution)
        }
    }
    pub fn aabb(&self, child_index: uint) -> AABB {
        unsafe {
            clone_from_ptr(ffi::Fixture_get_aabb(self.ptr(), child_index as i32))
        }
    }
    pub fn dump(&mut self, child_count: uint) {
        unsafe {
            ffi::Fixture_dump(self.mut_ptr(), child_count as i32)
        }
    }
}
