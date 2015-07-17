pub mod joints;
pub use self::joints::{
    JointType, JointDef, JointDefBase, Joint, JointEdge, UnknownJoint,
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
use std::mem;
use std::collections::HashMap;
use { ffi, settings };
use wrap::*;
use handle::*;
use common::{ Draw, DrawFlags, Color };
use math::{ Vec2, Transform };
use collision::{
    RayCastInput, RayCastOutput, AABB,
    Shape, ShapeType, UnknownShape, MassData
};

#[repr(C)]
#[derive(Clone)]
pub struct Profile {
    pub step: f32,
    pub collide: f32,
    pub solve: f32,
    pub solve_init: f32,
    pub solve_velocity: f32,
    pub solve_position: f32,
    pub brad_phase: f32,
    pub solve_toi: f32
}

pub type BodyHandle = TypedHandle<Body>;
pub type JointHandle = TypedHandle<UnknownJoint>;
pub type FixtureHandle = TypedHandle<Fixture>;

pub struct World {
    ptr: *mut ffi::World,
    bodies: HandleMap<Body>,
    mem_to_body: HashMap<usize, BodyHandle>,
    joints: HandleMap<UnknownJoint>,
    mem_to_joint: HashMap<usize, JointHandle>,
    draw_link: DrawLink,
    destruction_listener_link: DestructionListenerLink,
    contact_filter_link: ContactFilterLink,
    contact_listener_link: ContactListenerLink,
}

wrap! { ffi::World: custom World }

impl World {
    pub fn new(gravity: &Vec2) -> World {
        unsafe {
            World {
                ptr: ffi::World_new(gravity),
                bodies: HandleMap::new(),
                mem_to_body: HashMap::new(),
                joints: HandleMap::new(),
                mem_to_joint: HashMap::new(),
                draw_link: DrawLink::new(),
                destruction_listener_link: DestructionListenerLink::new(),
                contact_filter_link: ContactFilterLink::new(),
                contact_listener_link: ContactListenerLink::new(),
            }
        }
    }

    pub fn set_destruction_listener(&mut self,
                                    listener: Box<DestructionListener>) {
        unsafe {
            self.destruction_listener_link.set_object(listener);
            ffi::World_set_destruction_listener(
                self.mut_ptr(),
                ffi::DestructionListenerLink_as_base(
                    self.destruction_listener_link.mut_ptr()
                )
            );
        }
    }

    pub fn set_contact_filter(&mut self, filter: Box<ContactFilter>) {
        unsafe {
            self.contact_filter_link.set_object(filter);
            ffi::World_set_contact_filter(
                self.mut_ptr(),
                ffi::ContactFilterLink_as_base(self.contact_filter_link.mut_ptr())
            );
        }
    }

    pub fn set_contact_listener(&mut self, listener: Box<ContactListener>) {
        unsafe {
            self.contact_listener_link.set_object(listener);
            ffi::World_set_contact_listener(
                self.mut_ptr(),
                ffi::ContactListenerLink_as_base(
                    self.contact_listener_link.mut_ptr()
                )
            );
        }
    }

    pub fn create_body(&mut self, def: &BodyDef) -> BodyHandle {
        let body = unsafe {
            ffi::World_create_body(self.mut_ptr(), def)
        };
        let handle = self.bodies.insert(Body {
            ptr: body,
            fixtures: HandleMap::new()
        });
        self.mem_to_body.insert(body as usize, handle);
        handle
    }

    pub fn get_body_handle(&self, mem: usize) -> Option<BodyHandle> {
        self.mem_to_body.get(&mem).map(|h| *h)
    }

    pub fn get_body(&self, handle: BodyHandle) -> Option<&Body> {
        self.bodies.get(handle)
    }

    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Option<&mut Body> {
        self.bodies.get_mut(handle)
    }

    pub fn destroy_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(handle)
            .map(|mut body| { unsafe {
                self.mem_to_body.remove(& (body.ptr() as usize));
                ffi::World_destroy_body(self.mut_ptr(), body.mut_ptr());
            } });
    }

    pub fn create_joint<J: Joint>(&mut self, def: &JointDef) -> JointHandle {
        let assumed = J::assumed_joint_type();
        assert!(def.joint_type() == assumed ||
                assumed == JointType::Unknown);
        unsafe {
            let joint = ffi::World_create_joint(self.mut_ptr(), def.base_ptr());
            let handle = self.joints.insert(UnknownJoint::from_ffi(joint));
            self.mem_to_joint.insert(joint as usize, handle);
            handle
        }
    }

    pub fn get_joint_handle(self, mem: usize) -> Option<JointHandle> {
        self.mem_to_joint.get(&mem).map(|h| *h)
    }

    pub fn get_joint(&self, handle: JointHandle) -> Option<&UnknownJoint> {
        self.joints.get(handle)
    }

    pub fn get_joint_mut(&mut self, handle: JointHandle) -> Option<&mut UnknownJoint> {
        self.joints.get_mut(handle)
    }

    pub fn destroy_joint(&mut self, handle: JointHandle) {
        self.joints.remove(handle)
            .map(|mut joint| { unsafe {
                self.mem_to_joint.remove(& (joint.base_ptr() as usize));
                ffi::World_destroy_joint(self.mut_ptr(), joint.mut_base_ptr());
            } });
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

    pub fn draw_debug_data(&mut self, draw: &mut Draw, draw_flags: DrawFlags) {
        unsafe {
            self.draw_link.set_object(mem::transmute(draw));
            self.draw_link.set_flags(draw_flags);
            ffi::World_set_debug_draw(self.mut_ptr(),
                ffi::DrawLink_as_base(self.draw_link.mut_ptr())
            );
            ffi::World_draw_debug_data(self.mut_ptr());
            ffi::World_set_debug_draw(self.mut_ptr(), ptr::null_mut());
            self.draw_link.set_object(ffi::FatAny::null());
        }
    }

    pub fn query_aabb(&self, callback: &mut QueryCallback, aabb: &AABB) {
        unsafe {
            let mut query_callback_link = QueryCallbackLink::new(callback);
            ffi::World_query_aabb(self.ptr(),
                ffi::QueryCallbackLink_as_base(query_callback_link.mut_ptr()),
                aabb
            );
        }
    }

    pub fn ray_cast(&self, callback: &mut RayCastCallback, p1: &Vec2, p2: &Vec2) {
        unsafe {
            let mut ray_cast_callback_link = RayCastCallbackLink::new(callback);
            ffi::World_ray_cast(self.ptr(),
                ffi::RayCastCallbackLink_as_base(ray_cast_callback_link.mut_ptr()),
                p1, p2
            );
        }
    }

    pub fn bodies_mut(&mut self) -> HandleIterMut<Body> {
        self.bodies.iter_mut()
    }

    pub fn bodies(&self) -> HandleIter<Body> {
        self.bodies.iter()
    }

    pub fn joints_mut(&mut self) -> HandleIterMut<UnknownJoint> {
        self.joints.iter_mut()
    }

    pub fn joints(&mut self) -> HandleIter<UnknownJoint> {
        self.joints.iter()
    }

    pub fn contact_list_mut(&mut self) -> Option<&mut ContactEdge> {
        unsafe {
            ffi::World_get_contact_list(self.mut_ptr()).as_mut()
        }
    }

    pub fn contact_list(&self) -> Option<&ContactEdge> {
        unsafe {
            ffi::World_get_contact_list_const(self.ptr()).as_ref()
        }
    }

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

    pub fn proxy_count(&self) -> i32 {
        unsafe {
            ffi::World_get_proxy_count(self.ptr())
        }
    }

    pub fn body_count(&self) -> i32 {
        unsafe {
            ffi::World_get_body_count(self.ptr())
        }
    }

    pub fn joint_count(&self) -> i32 {
        unsafe {
            ffi::World_get_joint_count(self.ptr())
        }
    }

    pub fn contact_count(&self) -> i32 {
        unsafe {
            ffi::World_get_contact_count(self.ptr())
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
            &*ffi::World_get_profile(self.ptr()) // Comes from a C++ &
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
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum BodyType {
    Static = 0,
    Kinematic = 1,
    Dynamic = 2
}

#[repr(C)]
#[derive(Clone)]
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
            body_type: BodyType::Static,
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
            user_data: ptr::null_mut(),
            gravity_scale: 1.
        }
    }

    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        self.user_data = data as ffi::Any
    }
}


pub struct Body {
    ptr: *mut ffi::Body,
    fixtures: HandleMap<Fixture>
}

wrap! { ffi::Body: custom Body }

impl Body {
    pub fn transform<'a>(&'a self) -> &'a Transform {
        unsafe {
            &*ffi::Body_get_transform(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn position<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::Body_get_position(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn angle(&self) -> f32 {
        unsafe {
            ffi::Body_get_angle(self.ptr())
        }
    }

    pub fn world_center<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::Body_get_world_center(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn local_center<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::Body_get_local_center(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn linear_velocity<'a>(&'a self) -> &'a Vec2 {
        unsafe {
            &*ffi::Body_get_linear_velocity(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn angular_velocity(&self) -> f32 {
        unsafe {
            ffi::Body_get_angular_velocity(self.ptr())
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

    pub fn angular_damping(&self) -> f32 {
        unsafe {
            ffi::Body_get_angular_damping(self.ptr())
        }
    }

    pub fn gravity_scale(&self) -> f32 {
        unsafe {
            ffi::Body_get_gravity_scale(self.ptr())
        }
    }

    pub fn body_type(&self) -> BodyType {
        unsafe {
            ffi::Body_get_type(self.ptr())
        }
    }

    pub fn is_bullet(&self) -> bool {
        unsafe {
            ffi::Body_is_bullet(self.ptr())
        }
    }

    pub fn is_sleeping_allowed(&self) -> bool {
        unsafe {
            ffi::Body_is_sleeping_allowed(self.ptr())
        }
    }

    pub fn is_awake(&self) -> bool {
        unsafe {
            ffi::Body_is_awake(self.ptr())
        }
    }

    pub fn is_active(&self) -> bool {
        unsafe {
            ffi::Body_is_active(self.ptr())
        }
    }

    pub fn is_rotation_fixed(&self) -> bool {
        unsafe {
            ffi::Body_is_fixed_rotation(self.ptr())
        }
    }

    pub fn fixtures_mut(&mut self) -> HandleIterMut<Fixture> {
        self.fixtures.iter_mut()
    }

    pub fn fixtures(&self) -> HandleIter<Fixture> {
        self.fixtures.iter()
    }

    pub fn joints_mut<'a>(&'a mut self) -> Option<&'a mut JointEdge> {
        unsafe {
            ffi::Body_get_joint_list(self.mut_ptr()).as_mut()
        }
    }

    pub fn joints<'a>(&'a self) -> Option<&'a JointEdge> {
        unsafe {
            ffi::Body_get_joint_list_const(self.ptr()).as_ref()
        }
    }

    pub fn contacts_mut<'a>(&'a mut self) -> Option<&'a mut ContactEdge> {
        unsafe {
            ffi::Body_get_contact_list(self.mut_ptr()).as_mut()
        }
    }

    pub fn contacts<'a>(&'a self) -> Option<&'a ContactEdge> {
        unsafe {
            ffi::Body_get_contact_list_const(self.ptr()).as_ref()
        }
    }

    pub unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Body_get_user_data(self.ptr()) as *mut T
    }

    pub fn create_fixture(&mut self,
                          shape: &Shape,
                          def: &mut FixtureDef) -> FixtureHandle {
        let fixture;
        unsafe {
            def.shape = shape.base_ptr();
            fixture = Fixture::from_ffi(
                ffi::Body_create_fixture(self.mut_ptr(), def)
            );
            def.shape = ptr::null();
        }
        self.fixtures.insert(fixture)
    }

    pub fn create_fast_fixture(&mut self,
                               shape: &Shape,
                               density: f32) -> FixtureHandle {
        let fixture = unsafe {
            Fixture::from_ffi(
                ffi::Body_create_fast_fixture(
                    self.mut_ptr(),
                    shape.base_ptr(),
                    density
                )
            )
        };
        self.fixtures.insert(fixture)
    }

    pub fn get_fixture(&self, handle: FixtureHandle) -> Option<&Fixture> {
        self.fixtures.get(handle)
    }

    pub fn get_fixture_mut(&mut self, handle: FixtureHandle) -> Option<&mut Fixture> {
        self.fixtures.get_mut(handle)
    }

    pub fn destroy_fixture(&mut self, handle: FixtureHandle) {
        self.fixtures.remove(handle)
            .map(|mut fixture| {
                unsafe {
                    ffi::Body_destroy_fixture(self.mut_ptr(), fixture.mut_ptr());
                }
            });
    }

    pub fn set_transform(&mut self, pos: &Vec2, angle: f32) {
        unsafe {
            ffi::Body_set_transform(self.mut_ptr(), pos, angle)
        }
    }

    pub fn set_linear_velocity(&mut self, v: &Vec2) {
        unsafe {
            ffi::Body_set_linear_velocity(self.mut_ptr(), v)
        }
    }

    pub fn set_angular_velocity(&mut self, v: f32) {
        unsafe {
            ffi::Body_set_angular_velocity(self.mut_ptr(), v)
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

    pub fn apply_linear_impulse(&mut self, impulse: &Vec2, point: &Vec2, wake: bool) {
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

    pub fn set_linear_damping(&mut self, damping: f32) {
        unsafe {
            ffi::Body_set_linear_damping(self.mut_ptr(), damping)
        }
    }

    pub fn set_angular_damping(&mut self, damping: f32) {
        unsafe {
            ffi::Body_set_angular_damping(self.mut_ptr(), damping)
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

    pub fn set_bullet(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_bullet(self.mut_ptr(), flag)
        }
    }

    pub fn set_sleeping_allowed(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_sleeping_allowed(self.mut_ptr(), flag)
        }
    }

    pub fn set_awake(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_awake(self.mut_ptr(), flag)
        }
    }

    pub fn set_active(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_active(self.mut_ptr(), flag)
        }
    }

    pub fn set_rotation_fixed(&mut self, flag: bool) {
        unsafe {
            ffi::Body_set_fixed_rotation(self.mut_ptr(), flag)
        }
    }

    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        ffi::Body_set_user_data(self.mut_ptr(), data as ffi::Any)
    }

    pub fn dump(&mut self) {
        unsafe {
            ffi::Body_dump(self.mut_ptr())
        }
    }
}

#[repr(C)]
#[derive(Clone)]
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
pub struct FixtureDef {
    shape: *const ffi::Shape,
    user_data: ffi::Any,
    pub friction: f32,
    pub restitution: f32,
    pub density: f32,
    pub is_sensor: bool,
    pub filter: Filter,
}

impl FixtureDef {
    pub fn new() -> FixtureDef {
        FixtureDef {
            shape: ptr::null(), // needs to be specified later
            user_data: ptr::null_mut(),
            friction: 0.2,
            restitution: 0.,
            density: 0.,
            is_sensor: false,
            filter: Filter::new()
        }
    }

    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        self.user_data = data as ffi::Any
    }
}

wrap! { ffi::Fixture: Fixture }

impl Fixture {
    pub fn shape_type(&self) -> ShapeType {
        unsafe {
            ffi::Fixture_get_type(self.ptr())
        }
    }

    pub fn is_sensor(&self) -> bool {
        unsafe {
            ffi::Fixture_is_sensor(self.ptr())
        }
    }

    pub fn filter_data<'a>(&'a self) -> &'a Filter {
        unsafe {
            &*ffi::Fixture_get_filter_data(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn body(&self) -> usize {
        unsafe {
            ffi::Fixture_get_body_const(self.ptr()) as usize
        }
    }

    pub fn test_point(&self, point: &Vec2) -> bool {
        unsafe {
            ffi::Fixture_test_point(self.ptr(), point)
        }
    }

    pub fn ray_cast(&self, input: &RayCastInput,
                           child_index: i32) -> RayCastOutput {
        unsafe {
            let mut output = RayCastOutput::new();
            ffi::Fixture_ray_cast(self.ptr(),
                &mut output,
                input,
                child_index
            );
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

    pub fn restitution(&self) -> f32 {
        unsafe {
            ffi::Fixture_get_restitution(self.ptr())
        }
    }

    pub fn aabb<'a>(&'a self, child_index: i32) -> &'a AABB {
        unsafe {
            &*ffi::Fixture_get_aabb(self.ptr(), child_index) // Comes from a C++ &
        }
    }

    pub unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Fixture_get_user_data(self.ptr()) as *mut T
    }

    pub fn shape_mut<'a>(&'a mut self) -> RefMut<'a, UnknownShape> {
        unsafe {
            RefMut::new(UnknownShape::from_ffi(
                ffi::Fixture_get_shape(self.mut_ptr())
            ))
        }
    }

    pub fn shape<'a>(&'a self) -> Ref<'a, UnknownShape> {
        unsafe {
            Ref::new(UnknownShape::from_ffi(
                ffi::Fixture_get_shape_const(self.ptr()) as *mut ffi::Shape
            ))
        }
    }

    pub fn set_sensor(&mut self, flag: bool) {
        unsafe {
            ffi::Fixture_set_sensor(self.mut_ptr(), flag)
        }
    }

    pub fn set_filter_data(&mut self, filter: &Filter) {
        unsafe {
            ffi::Fixture_set_filter_data(self.mut_ptr(), filter)
        }
    }

    pub fn refilter(&mut self) {
        unsafe {
            ffi::Fixture_refilter(self.mut_ptr())
        }
    }

    pub fn set_density(&mut self, density: f32) {
        unsafe {
            ffi::Fixture_set_density(self.mut_ptr(), density)
        }
    }

    pub fn set_friction(&mut self, friction: f32) {
        unsafe {
            ffi::Fixture_set_friction(self.mut_ptr(), friction)
        }
    }

    pub fn set_restitution(&mut self, restitution: f32) {
        unsafe {
            ffi::Fixture_set_restitution(self.mut_ptr(), restitution)
        }
    }

    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        ffi::Fixture_set_user_data(self.mut_ptr(), data as ffi::Any)
    }

    pub fn dump(&mut self, child_count: i32) {
        unsafe {
            ffi::Fixture_dump(self.mut_ptr(), child_count)
        }
    }
}

#[repr(C)]
pub struct ContactImpulse {
    pub normal_impulses: [f32; settings::MAX_MANIFOLD_POINTS],
    pub tangent_impulses: [f32; settings::MAX_MANIFOLD_POINTS],
    pub count: i32
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ManifoldType {
    Circles = 0,
    FaceA = 1,
    FaceB = 2
}

#[repr(C)]
pub struct Manifold {
    pub points: [ManifoldPoint; settings::MAX_MANIFOLD_POINTS],
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

wrap! { ffi::Contact: Contact }

#[repr(C)]
pub struct ContactEdge {
    other: *mut ffi::Body,
    contact: *mut ffi::Contact,
    prev: *mut ContactEdge,
    next: *mut ContactEdge
}

impl ContactEdge {
    pub fn other(&self) -> usize {
        self.other as usize
    }

    pub fn contact_mut<'a>(&'a mut self) -> RefMut<'a, Contact> {
        unsafe {
            RefMut::new(Contact::from_ffi(self.contact))
        }
    }

    pub fn contact<'a>(&'a self) -> Ref<'a, Contact> {
        unsafe {
            Ref::new(Contact::from_ffi(self.contact))
        }
    }

    pub fn prev_mut<'a>(&'a mut self) -> Option<&'a mut ContactEdge> {
        unsafe {
            self.prev.as_mut()
        }
    }

    pub fn prev<'a>(&'a self) -> Option<&'a ContactEdge> {
        unsafe {
            self.prev.as_ref()
        }
    }

    pub fn next_mut<'a>(&'a mut self) -> Option<&'a mut ContactEdge> {
        unsafe {
            self.next.as_mut()
        }
    }

    pub fn next<'a>(&'a self) -> Option<&'a ContactEdge> {
        unsafe {
            self.next.as_ref()
        }
    }
}

pub trait DestructionListener {
    fn goodbye_joint(&mut self, joint: &mut UnknownJoint);
    fn goodbye_fixture(&mut self, fixture: &mut Fixture);
}

pub trait ContactFilter {
    fn should_collide(&mut self, fixture_a: &mut Fixture,
                                 fixture_b: &mut Fixture) -> bool;
}

pub trait ContactListener {
    fn begin_contact(&mut self, contact: &mut Contact);
    fn end_contact(&mut self, contact: &mut Contact);
    fn pre_solve(&mut self, contact: &mut Contact, manifold: &Manifold);
    fn post_solve(&mut self, contact: &mut Contact, impulse: &ContactImpulse);
}

pub trait QueryCallback {
    fn report_fixture(&mut self, fixture: &mut Fixture) -> bool;
}

pub trait RayCastCallback {
    fn report_fixture(&mut self, fixture: &mut Fixture, p: &Vec2, normal: &Vec2,
                      fraction: f32) -> f32;
}

unsafe extern fn goodbye_joint(any: ffi::FatAny, joint: *mut ffi::Joint) {
    let listener = mem::transmute::<_, &mut DestructionListener>(any);
    let mut joint = RefMut::new(UnknownJoint::from_ffi(joint));
    listener.goodbye_joint(&mut joint)
}

unsafe extern fn goodbye_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture) {
    let listener = mem::transmute::<_, &mut DestructionListener>(any);
    let mut fixture = RefMut::new(Fixture::from_ffi(fixture));
    listener.goodbye_fixture(&mut fixture)
}

unsafe extern fn should_collide(any: ffi::FatAny, fixture_a: *mut ffi::Fixture,
                                fixture_b: *mut ffi::Fixture) -> bool {
    let filter = mem::transmute::<_, &mut ContactFilter>(any);
    let mut fixture_a = RefMut::new(Fixture::from_ffi(fixture_a));
    let mut fixture_b = RefMut::new(Fixture::from_ffi(fixture_b));
    filter.should_collide(&mut fixture_a, &mut fixture_b)
}

unsafe extern fn begin_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = RefMut::new(Contact::from_ffi(contact));
    listener.begin_contact(&mut contact)
}

unsafe extern fn end_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = RefMut::new(Contact::from_ffi(contact));
    listener.end_contact(&mut contact)
}

unsafe extern fn pre_solve(any: ffi::FatAny, contact: *mut ffi::Contact,
                           old_manifold: *const Manifold) {
    assert!(!old_manifold.is_null());
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = RefMut::new(Contact::from_ffi(contact));
    listener.pre_solve(&mut contact, &*old_manifold)
}

unsafe extern fn post_solve(any: ffi::FatAny, contact: *mut ffi::Contact,
                            impulse: *const ContactImpulse) {
    assert!(!impulse.is_null());
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    let mut contact = RefMut::new(Contact::from_ffi(contact));
    listener.post_solve(&mut contact, &*impulse)
}

unsafe extern fn qc_report_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture
                                   ) -> bool {
    let callback = mem::transmute::<_, &mut QueryCallback>(any);
    let mut fixture = RefMut::new(Fixture::from_ffi(fixture));
    callback.report_fixture(&mut fixture)
}

unsafe extern fn rcc_report_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture,
                                    point: *const Vec2, normal: *const Vec2,
                                    fraction: f32) -> f32 {
    // point and normal are coming from C++ &s
    let callback = mem::transmute::<_, &mut RayCastCallback>(any);
    let mut fixture = RefMut::new(Fixture::from_ffi(fixture));
    callback.report_fixture(&mut fixture, &*point, &*normal, fraction)
}

struct DestructionListenerLink {
    ptr: *mut ffi::DestructionListenerLink,
    object: Option<Box<DestructionListener>>,
}

struct ContactFilterLink {
    ptr: *mut ffi::ContactFilterLink,
    object: Option<Box<ContactFilter>>,
}

struct ContactListenerLink {
    ptr: *mut ffi::ContactListenerLink,
    object: Option<Box<ContactListener>>,
}

wrap! { ffi::DestructionListenerLink: custom DestructionListenerLink }
wrap! { ffi::ContactFilterLink: custom ContactFilterLink }
wrap! { ffi::ContactListenerLink: custom ContactListenerLink }

impl DestructionListenerLink {
    fn new() -> DestructionListenerLink {
        unsafe {
            DestructionListenerLink {
                ptr: ffi::DestructionListenerLink_new(ffi::FatAny::null(),
                                                      goodbye_joint,
                                                      goodbye_fixture),
                object: None
            }
        }
    }

    fn set_object(&mut self, object: Box<DestructionListener>) {
        unsafe {
            self.object = Some(object);
            ffi::DestructionListenerLink_set_object(
                self.mut_ptr(),
                mem::transmute::<&mut DestructionListener, _>(
                    &mut **self.object.as_mut().unwrap()
                )
            )
        }
    }
}

impl ContactFilterLink {
    unsafe fn new() -> ContactFilterLink {
        ContactFilterLink {
            ptr: ffi::ContactFilterLink_new(ffi::FatAny::null(),
                                            should_collide),
            object: None
        }
    }

    unsafe fn set_object(&mut self, object: Box<ContactFilter>) {
        self.object = Some(object);
        ffi::ContactFilterLink_set_object(
            self.mut_ptr(),
            mem::transmute::<&mut ContactFilter, _>(
                &mut **self.object.as_mut().unwrap()
            )
        )
    }
}

impl ContactListenerLink {
    unsafe fn new() -> ContactListenerLink {
        ContactListenerLink {
            ptr: ffi::ContactListenerLink_new(ffi::FatAny::null(),
                                              begin_contact,
                                              end_contact,
                                              pre_solve,
                                              post_solve),
            object: None
        }
    }

    unsafe fn set_object(&mut self, object: Box<ContactListener>) {
        self.object = Some(object);
        ffi::ContactListenerLink_set_object(
            self.mut_ptr(),
            mem::transmute::<&mut ContactListener, _>(
                &mut **self.object.as_mut().unwrap()
            )
        )
    }
}

wrap! { ffi::QueryCallbackLink: private QueryCallbackLink }
wrap! { ffi::RayCastCallbackLink: private RayCastCallbackLink }

impl QueryCallbackLink {
    unsafe fn new(object: &mut QueryCallback) -> QueryCallbackLink {
        QueryCallbackLink::from_ffi(
            ffi::QueryCallbackLink_new(mem::transmute(object),
                                       qc_report_fixture)
        )
    }
}

impl RayCastCallbackLink {
    unsafe fn new(object: &mut RayCastCallback) -> RayCastCallbackLink {
        RayCastCallbackLink::from_ffi(
            ffi::RayCastCallbackLink_new(mem::transmute(object),
                                         rcc_report_fixture)
        )
    }
}

impl Drop for DestructionListenerLink {
    fn drop(&mut self) {
        unsafe {
            ffi::DestructionListenerLink_drop(self.mut_ptr())
        }
    }
}

impl Drop for ContactFilterLink {
    fn drop(&mut self) {
        unsafe {
            ffi::ContactFilterLink_drop(self.mut_ptr())
        }
    }
}

impl Drop for ContactListenerLink {
    fn drop(&mut self) {
        unsafe {
            ffi::ContactListenerLink_drop(self.mut_ptr())
        }
    }
}

impl Drop for QueryCallbackLink {
    fn drop(&mut self) {
        unsafe {
            ffi::QueryCallbackLink_drop(self.mut_ptr())
        }
    }
}

impl Drop for RayCastCallbackLink {
    fn drop(&mut self) {
        unsafe  {
            ffi::RayCastCallbackLink_drop(self.mut_ptr())
        }
    }
}

unsafe extern fn draw_polygon(any: ffi::FatAny, vertices: *const Vec2,
                              count: i32, color: *const Color) {
     // color comes from a C++ &
    let draw = mem::transmute::<_, &mut Draw>(any);
    let vertices = ::std::slice::from_raw_parts(vertices, count as usize);
    draw.draw_polygon(vertices, &*color)
}

unsafe extern fn draw_solid_polygon(any: ffi::FatAny, vertices: *const Vec2,
                                    count: i32, color: *const Color) {
     // color comes from a C++ &
    let draw = mem::transmute::<_, &mut Draw>(any);
    let vertices = ::std::slice::from_raw_parts(vertices, count as usize);
    draw.draw_solid_polygon(vertices, &*color)
}

unsafe extern fn draw_circle(any: ffi::FatAny, center: *const Vec2,
                             radius: f32, color: *const Color) {
    // center and color are coming from C++ &s
    let draw = mem::transmute::<_, &mut Draw>(any);
    draw.draw_circle(&*center, radius, &*color)
}

unsafe extern fn draw_solid_circle(any: ffi::FatAny, center: *const Vec2,
                                   radius: f32, axis: *const Vec2,
                                   color: *const Color) {
    // center, axis and color are coming from C++ &s
    let draw = mem::transmute::<_, &mut Draw>(any);
    draw.draw_solid_circle(&*center, radius, &*axis, &*color)
}

unsafe extern fn draw_segment(any: ffi::FatAny, p1: *const Vec2,
                              p2: *const Vec2, color: *const Color) {
    // p1, p2 and color are coming from C++ &s
    let draw = mem::transmute::<_, &mut Draw>(any);
    draw.draw_segment(&*p1, &*p2, &*color)
}

unsafe extern fn draw_transform(any: ffi::FatAny, xf: *const Transform) {
    // xf comes from a C++ &
    let draw = mem::transmute::<_, &mut Draw>(any);
    draw.draw_transform(&*xf)
}

wrap! { ffi::DrawLink: private DrawLink }

impl DrawLink {
    fn new() -> DrawLink {
        unsafe {
            DrawLink::from_ffi(
                ffi::DrawLink_new(ffi::FatAny::null(),
                                  draw_polygon,
                                  draw_solid_polygon,
                                  draw_circle,
                                  draw_solid_circle,
                                  draw_segment,
                                  draw_transform)
            )
        }
    }

    unsafe fn set_object(&mut self, object: ffi::FatAny) {
        ffi::DrawLink_set_object(self.mut_ptr(), object)
    }

    fn set_flags(&mut self, flags: DrawFlags) {
        unsafe {
            ffi::DrawLink_set_flags(self.mut_ptr(), flags)
        }
    }

    fn flags(&self) -> DrawFlags {
        unsafe {
            ffi::DrawLink_get_flags(self.ptr())
        }
    }

    fn insert_flags(&mut self, flags: DrawFlags) {
        unsafe {
            ffi::DrawLink_append_flags(self.mut_ptr(), flags)
        }
    }

    fn remove_flags(&mut self, flags: DrawFlags) {
        unsafe {
            ffi::DrawLink_clear_flags(self.mut_ptr(), flags)
        }
    }
}

impl Drop for DrawLink {
    fn drop(&mut self) {
        unsafe {
            ffi::DrawLink_drop(self.mut_ptr())
        }
    }
}
