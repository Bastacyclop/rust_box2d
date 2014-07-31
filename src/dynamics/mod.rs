pub use self::joints::{
    JointType, JointDefBase, MutJoint, ConstJoint,
    UnknownJointType, UnknownJointMutRef, UnknownJointRef,
    DistanceJointType, DistanceJointDef, DistanceJointMutRef, FrictionJointRef,
    FrictionJointType, FrictionJointDef, FrictionJointMutRef, FrictionJointRef,
    GearJointType, GearJointDef, GearJointMutRef, GearJointRef,
    MotorJointType, MotorJointDef, MotorJointMutRef, MotorJointRef,
    MouseJointType, MouseJointDef, MouseJointMutRef, MouseJointRef,
    PrismaticJointType, PrismaticJointDef, PrismaticJointMutRef, PrismaticJointRef,
    PulleyJointType, PulleyJointDef, PulleyJointMutRef, PulleyJointRef,
    RevoluteJointType, RevoluteJointDef, RevoluteJointMutRef, RevoluteJointRef,
    RopeJointType, RopeJointDef, RopeJointMutRef, RopeJointRef,
    WeldJointType, WeldJointDef, WeldJointMutRef, WeldJointRef,
    WheelJointType, WheelJointDef, WheelJointMutRef, WheelJointRef
};

use std::ptr;
use std::mem;
use {
    Wrapped, WrappedMut, WrappedConst,
    WrappedBase, WrappedMutBase, WrappedConstBase,
    ffi, settings
};
use common::{Draw, DrawLink};
use math::{Vec2, Transform};
use dynamics::joints::private::{WrappedJoint, JointDef};
use collision::{
    RayCastInput, RayCastOutput, AABB,
    Shape, ShapeType, UnknownShape, MassData
};

pub mod joints;

#[repr(C)]
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

wrapped!(ffi::World owned into World)

impl World {
    pub fn new(gravity: &Vec2) -> World {
        unsafe {
            WrappedMut::from_ptr(ffi::World_new(gravity))
        }
    }
    
    pub fn set_destruction_listener(&mut self, dll: &mut DestructionListenerLink) {
        unsafe {
            ffi::World_set_destruction_listener(self.mut_ptr(),
                                                ffi::CDestructionListener_as_base(dll.mut_ptr()))
        }
    }
    
    pub fn set_contact_filter(&mut self, cfl: &mut ContactFilterLink) {
        unsafe {
            ffi::World_set_contact_filter(self.mut_ptr(),
                                          ffi::CContactFilter_as_base(cfl.mut_ptr()))
        }
    }
    
    pub fn set_contact_listener(&mut self, cll: &mut ContactListenerLink) {
        unsafe {
            ffi::World_set_contact_listener(self.mut_ptr(),
                                            ffi::CContactListener_as_base(cll.mut_ptr()))
        }
    }
    
    pub fn create_body(&mut self, def: &BodyDef) -> BodyMutRef{
        unsafe {
            WrappedMut::from_ptr(
                ffi::World_create_body(self.mut_ptr(), def)
                )
        }
    }
    
    pub fn destroy_body(&mut self, mut body: BodyMutRef) {
        unsafe {
            ffi::World_destroy_body(self.mut_ptr(), body.mut_ptr())
        }
    }
    
    pub fn create_joint<J: MutJoint>(&mut self, def: &JointDef) -> J {
        unsafe {
            let joint: J = WrappedMutBase::from_ptr(
                ffi::World_create_joint(self.mut_ptr(), def.joint_def_ptr())
                );
            assert!(
                joint.joint_type() == WrappedJoint::joint_type(None::<*const J>)
                || self::UnknownJointType == WrappedJoint::joint_type(None::<*const J>)
                )
            joint
        }
    }
    
    pub fn destroy_joint<J: MutJoint>(&mut self, mut joint: J) {
        unsafe {
            ffi::World_destroy_joint(self.mut_ptr(), joint.mut_base_ptr())
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
    
    pub fn draw_debug_data(&mut self, dl: &mut DrawLink, d: &mut Draw) {
        unsafe {
            dl.set_object(mem::transmute(d));
            ffi::World_set_debug_draw(self.mut_ptr(), ffi::DrawLink_as_base(dl.mut_ptr()));
            ffi::World_draw_debug_data(self.mut_ptr());
            ffi::World_set_debug_draw(self.mut_ptr(), ptr::mut_null());
            dl.set_object(ffi::FatAny::null())
        }
    }
    
    pub fn query_aabb(&self, qcl: &mut QueryCallbackLink, aabb: &AABB) {
        unsafe {
            ffi::World_query_aabb(self.ptr(), ffi::CQueryCallback_as_base(qcl.mut_ptr()), aabb)
        }
    }
    pub fn ray_cast(&self, rccl: &mut RayCastCallbackLink, p1: &Vec2, p2: &Vec2) {
        unsafe {
            ffi::World_ray_cast(self.ptr(), ffi::CRayCastCallback_as_base(rccl.mut_ptr()), p1, p2)
        }
    }
    /*pub fn mut_body_list(&mut self) -> Vec<Body> {
        unsafe {
            let mut ptr = ffi::World_get_body_list(self.mut_ptr());
            
            let mut vec = Vec::new();
            while !ptr.is_null() {
                vec.push(WrappedMut::from_ptr(ptr));
                ptr = ffi::Body_get_next(ptr);
            }
            vec
        }
    }*/
    pub fn set_sleeping_allowed(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_allow_sleeping(self.mut_ptr(), flag)
        }
    }
    pub fn is_sleeping_allowed(&self) -> bool {
        unsafe {
            ffi::World_get_allow_sleeping(self.ptr())
        }
    }
    pub fn set_warm_starting(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_warm_starting(self.mut_ptr(), flag)
        }
    }
    pub fn is_warm_starting(&self) -> bool {
        unsafe {
            ffi::World_get_warm_starting(self.ptr())
        }
    }
    pub fn set_continuous_physics(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_continuous_physics(self.mut_ptr(), flag)
        }
    }
    pub fn is_continuous_physics(&self) -> bool {
        unsafe {
            ffi::World_get_continuous_physics(self.ptr())
        }
    }
    pub fn set_sub_stepping(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_sub_stepping(self.mut_ptr(), flag)
        }
    }
    pub fn is_sub_stepping(&self) -> bool {
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
    pub fn set_auto_clearing_forces(&mut self, flag: bool) {
        unsafe {
            ffi::World_set_auto_clear_forces(self.mut_ptr(), flag)
        }
    }
    pub fn is_auto_clearing_forces(&self) -> bool {
        unsafe {
            ffi::World_get_auto_clear_forces(self.ptr())
        }
    }
    pub fn shift_origin(&mut self, origin: &Vec2) {
        unsafe {
            ffi::World_shift_origin(self.mut_ptr(), origin)
        }
    }
    
    pub fn profile<'a>(&'a self) -> &'a Profile {
        unsafe {
            let profile = ffi::World_get_profile(self.ptr());
            assert!(!profile.is_null())
            &*profile
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

#[repr(C)]
#[deriving(PartialEq, Show)]
pub enum BodyType {
    StaticBodyType = 0,
    KinematicBodyType = 1,
    DynamicBodyType = 2
}

#[repr(C)]
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
    user_data: ffi::Any,
    pub gravity_scale: f32,
}

impl BodyDef {
    pub fn new() -> BodyDef {
        BodyDef {
            body_type: StaticBodyType,
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
    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        self.user_data = data as ffi::Any
    }
}

wrapped!(ffi::Body into BodyMutRef, BodyConstRef)

impl<'l> BodyMutRef<'l>{
    pub fn create_fixture(&mut self, def: &FixtureDef) -> FixtureMutRef<'l>{
        unsafe {
            WrappedMut::from_ptr(ffi::Body_create_fixture(self.mut_ptr(), def))
        }
    }
    pub fn create_fast_fixture(&mut self, shape: &Shape, density: f32) -> FixtureMutRef<'l>{
        unsafe {
            WrappedMut::from_ptr(
                ffi::Body_create_fast_fixture(self.mut_ptr(),
                                              shape.base_ptr(),
                                              density)
                )
        }
    }
    pub fn destroy_fixture(&mut self, mut fixture: FixtureMutRef<'l>) {
        unsafe {
            ffi::Body_destroy_fixture(self.mut_ptr(), fixture.mut_ptr())
        }
    }
    pub fn set_transform(&mut self, pos: &Vec2, angle: f32) {
        unsafe {
            ffi::Body_set_transform(self.mut_ptr(), pos, angle)
        }
    }
    
    pub fn transform<'a>(&'a self) -> &'a Transform {
        unsafe {
            let transform = ffi::Body_get_transform(self.ptr());
            assert!(!transform.is_null())
            &*transform
        }
    }
    
    pub fn position<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let position = ffi::Body_get_position(self.ptr());
            assert!(!position.is_null())
            &*position
        }
    }
    
    pub fn angle(&self) -> f32 {
        unsafe {
            ffi::Body_get_angle(self.ptr())
        }
    }
    
    pub fn world_center<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let center = ffi::Body_get_world_center(self.ptr());
            assert!(!center.is_null())
            &*center
        }
    }
    
    pub fn local_center<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let center = ffi::Body_get_local_center(self.ptr());
            assert!(!center.is_null())
            &*center
        }
    }
    
    pub fn set_linear_velocity(&mut self, v: &Vec2) {
        unsafe {
            ffi::Body_set_linear_velocity(self.mut_ptr(), v)
        }
    }
    
    pub fn linear_velocity<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            let velocity = ffi::Body_get_linear_velocity(self.ptr());
            assert!(!velocity.is_null())
            &*velocity
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
    pub fn set_body_type(&mut self, typ: BodyType) {
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
    pub fn set_rotation_fixed(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_fixed_rotation(self.mut_ptr(), flag)
        }
    }
    pub fn is_rotation_fixed(&self) -> bool {
        unsafe {
            ffi::Body_is_fixed_rotation(self.ptr())
        }
    }
    /*pub fn mut_fixture_list(&mut self) -> Vec<Fixture> {
        unsafe {
        
        }
    }*/
    pub fn mut_next(&mut self) -> BodyMutRef{
        unsafe {
            WrappedMut::from_ptr(ffi::Body_get_next(self.mut_ptr()))
        }
    }
    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        ffi::Body_set_user_data(self.mut_ptr(), data as ffi::Any)
    }
    pub unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Body_get_user_data(self.ptr()) as *mut T
    }
    pub fn mut_world(&mut self) -> World {
        unsafe {
            WrappedMut::from_ptr(ffi::Body_get_world(self.mut_ptr()))
        }
    }
    pub fn dump(&mut self) {
        unsafe {
            ffi::Body_dump(self.mut_ptr())
        }
    }
}

#[repr(C)]
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

#[repr(C)]
#[allow(dead_code)]
pub struct FixtureDef {
    shape: *const ffi::Shape,
    user_data: ffi::Any,
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
                shape: shape.base_ptr(),
                user_data: ptr::mut_null(),
                friction: 0.2,
                restitution: 0.,
                density: 0.,
                is_sensor: false,
                filter: Filter::new()
            }
        }
    }
    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        self.user_data = data as ffi::Any
    }
}

wrapped!(ffi::Fixture into FixtureMutRef, FixtureConstRef)

impl<'l> FixtureMutRef<'l>{
    pub fn shape_type(&self) -> ShapeType {
        unsafe {
            ffi::Fixture_get_type(self.ptr())
        }
    }
    pub fn shape(&mut self) -> UnknownShape {
        unsafe {
            WrappedMutBase::from_ptr(
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
    
    pub fn filter_data<'a>(&'a self) -> &'a Filter {
        unsafe {
            let data = ffi::Fixture_get_filter_data(self.ptr());
            assert!(!data.is_null())
            &*data
        }
    }
    
    pub fn refilter(&mut self) {
        unsafe {
            ffi::Fixture_refilter(self.mut_ptr())
        }
    }
    pub fn mut_body(&mut self) -> BodyMutRef<'l>{
        unsafe {
            WrappedMut::from_ptr(ffi::Fixture_get_body(self.mut_ptr()))
        }
    }
    pub fn mut_next(&mut self) -> FixtureMutRef<'l>{
        unsafe {
            WrappedMut::from_ptr(ffi::Fixture_get_next(self.mut_ptr()))
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
    
    pub fn aabb<'a>(&'a self, child_index: uint) -> &'a AABB {
        unsafe {
            let aabb = ffi::Fixture_get_aabb(self.ptr(), child_index as i32);
            assert!(!aabb.is_null())
            &*aabb
        }
    }
    
    pub unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Fixture_get_user_data(self.ptr()) as *mut T
    }
    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        ffi::Fixture_set_user_data(self.mut_ptr(), data as ffi::Any)
    }
    pub fn dump(&mut self, child_count: uint) {
        unsafe {
            ffi::Fixture_dump(self.mut_ptr(), child_count as i32)
        }
    }
}

#[repr(C)]
pub struct ContactImpulse {
    pub normal_impulses: [f32, ..settings::MAX_MANIFOLD_POINTS],
    pub tangent_impulses: [f32, ..settings::MAX_MANIFOLD_POINTS],
    pub count: i32
}

#[repr(C)]
#[deriving(PartialEq, Show)]
pub enum ManifoldType {
    CirclesManifoldType = 0,
    FaceAManifoldType = 1,
    FaceBManifoldType = 2
}

#[repr(C)]
pub struct Manifold {
    pub points: [ManifoldPoint, ..settings::MAX_MANIFOLD_POINTS],
    pub local_normal: Vec2,
    pub local_point: Vec2,
    pub manifold_type: ManifoldType,
    pub count: i32
}

#[repr(C)]
pub struct ManifoldPoint {
    pub local_point: Vec2,
    pub normal_impulse: f32,
    pub tangent_impulse: f32,
    pub id: u32
}

wrapped!(ffi::Contact into ContactMutRef, ContactConstRef)

pub trait DestructionListener {
    fn goodbye_joint(&mut self, joint: UnknownJointMutRef);
    fn goodbye_fixture(&mut self, fixture: FixtureMutRef);
}

pub trait ContactFilter {
    fn should_collide(&mut self, fixture_a: FixtureMutRef, fixture_b: FixtureMutRef) -> bool;
}

pub trait ContactListener {
    fn begin_contact(&mut self, contact: ContactMutRef);
    fn end_contact(&mut self, contact: ContactMutRef);
    fn pre_solve(&mut self, contact: ContactMutRef, manifold: &Manifold);
    fn post_solve(&mut self, contact: ContactMutRef, impulse: &ContactImpulse);
}

pub trait QueryCallback {
    fn report_fixture(&mut self, fixture: FixtureMutRef) -> bool;
}

pub trait RayCastCallback {
    fn report_fixture(&mut self, fixture: FixtureMutRef, p: &Vec2, normal: &Vec2,
                      fraction: f32) -> f32;
}

unsafe extern fn goodbye_joint(any: ffi::FatAny, joint: *mut ffi::Joint) {
    let listener = mem::transmute::<_, &mut DestructionListener>(any);
    listener.goodbye_joint(WrappedMutBase::from_ptr(joint))
}
unsafe extern fn goodbye_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture) {
    let listener = mem::transmute::<_, &mut DestructionListener>(any);
    listener.goodbye_fixture(WrappedMut::from_ptr(fixture))
}

unsafe extern fn should_collide(any: ffi::FatAny, fixture_a: *mut ffi::Fixture,
                                fixture_b: *mut ffi::Fixture) -> bool {
    let filter = mem::transmute::<_, &mut ContactFilter>(any);
    filter.should_collide(WrappedMut::from_ptr(fixture_a),
                          WrappedMut::from_ptr(fixture_b))
}

unsafe extern fn begin_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.begin_contact(WrappedMut::from_ptr(contact))
}
unsafe extern fn end_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.end_contact(WrappedMut::from_ptr(contact))
}
unsafe extern fn pre_solve(any: ffi::FatAny, contact: *mut ffi::Contact,
                           old_manifold: *const Manifold) {
    assert!(!old_manifold.is_null())
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.pre_solve(WrappedMut::from_ptr(contact), &*old_manifold)
}
unsafe extern fn post_solve(any: ffi::FatAny, contact: *mut ffi::Contact,
                            impulse: *const ContactImpulse) {
    assert!(!impulse.is_null())
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.post_solve(WrappedMut::from_ptr(contact), &*impulse)
}

unsafe extern fn qc_report_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture
                                   ) -> bool {
    let callback = mem::transmute::<_, &mut QueryCallback>(any);
    callback.report_fixture(WrappedMut::from_ptr(fixture))
}

unsafe extern fn rcc_report_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture,
                                    point: *const Vec2, normal: *const Vec2,
                                    fraction: f32) -> f32 {
    assert!(!point.is_null())
    assert!(!normal.is_null())
    let callback = mem::transmute::<_, &mut RayCastCallback>(any);
    callback.report_fixture(WrappedMut::from_ptr(fixture), &*point, &*normal,
                            fraction)
}

wrapped!(ffi::CDestructionListener owned into DestructionListenerLink)
wrapped!(ffi::CContactFilter owned into ContactFilterLink)
wrapped!(ffi::CContactListener owned into ContactListenerLink)
wrapped!(ffi::CQueryCallback owned into QueryCallbackLink)
wrapped!(ffi::CRayCastCallback owned into RayCastCallbackLink)

impl DestructionListenerLink {
    pub fn with(t: &mut DestructionListener) -> DestructionListenerLink {
        unsafe {
            WrappedMut::from_ptr(
                ffi::CDestructionListener_new(mem::transmute(t),
                                              goodbye_joint,
                                              goodbye_fixture
                                              ))
        }
    }
}

impl ContactFilterLink {
    pub fn with(t: &mut ContactFilter) -> ContactFilterLink {
        unsafe {
            WrappedMut::from_ptr(
                ffi::CContactFilter_new(mem::transmute(t),
                                        should_collide))
        }
    }
}

impl ContactListenerLink {
    pub fn with(t: &mut ContactListener) -> ContactListenerLink {
        unsafe {
            WrappedMut::from_ptr(
                ffi::CContactListener_new(mem::transmute(t),
                                          begin_contact,
                                          end_contact,
                                          pre_solve,
                                          post_solve))
        }          
    }
}

impl QueryCallbackLink {
    pub fn with(t: &mut QueryCallback) -> QueryCallbackLink {
        unsafe {
            WrappedMut::from_ptr(
                ffi::CQueryCallback_new(mem::transmute(t),
                                        qc_report_fixture))
        }
    }
}

impl RayCastCallbackLink {
    pub fn with(t: &mut RayCastCallback) -> RayCastCallbackLink {
        unsafe {
            WrappedMut::from_ptr(
                ffi::CRayCastCallback_new(mem::transmute(t),
                                          rcc_report_fixture))
        }
    }
}

impl Drop for DestructionListenerLink {
    fn drop(&mut self) {
        unsafe {
            ffi::CDestructionListener_drop(self.mut_ptr())
        }
    }
}

impl Drop for ContactFilterLink {
    fn drop(&mut self) {
        unsafe {
            ffi::CContactFilter_drop(self.mut_ptr())
        }
    }
}

impl Drop for ContactListenerLink {
    fn drop(&mut self) {
        unsafe {
            ffi::CContactListener_drop(self.mut_ptr())
        }
    }
}

impl Drop for QueryCallbackLink {
    fn drop(&mut self) {
        unsafe {
            ffi::CQueryCallback_drop(self.mut_ptr())
        }
    }
}

impl Drop for RayCastCallbackLink {
    fn drop(&mut self) {
        unsafe  {
            ffi::CRayCastCallback_drop(self.mut_ptr())
        }
    }
}
