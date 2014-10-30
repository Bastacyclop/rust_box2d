pub use self::joints::{
    JointType, JointDef, JointDefBase, Joint, JointEdge,
    UnknownJointType, UnknownJoint,
    DistanceJointType, DistanceJointDef, DistanceJoint,
    FrictionJointType, FrictionJointDef, FrictionJoint,
    GearJointType, GearJointDef, GearJoint,
    MotorJointType, MotorJointDef, MotorJoint,
    MouseJointType, MouseJointDef, MouseJoint,
    PrismaticJointType, PrismaticJointDef, PrismaticJoint,
    PulleyJointType, PulleyJointDef, PulleyJoint,
    RevoluteJointType, RevoluteJointDef, RevoluteJoint,
    RopeJointType, RopeJointDef, RopeJoint,
    WeldJointType, WeldJointDef, WeldJoint,
    WheelJointType, WheelJointDef, WheelJoint
};

use std::ptr;
use std::mem;
use {
    ffi, settings, MaybeOwned, Owned, NotOwned,
    Wrapped, BuildWrapped, WrappedBase, BuildWrappedBase, Ref, RefMut
};
use common::{Draw, DrawFlags};
use common::private::DrawLink;
use math::{Vec2, Transform};
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
    pub solve_toi: f32
}

pub struct World {
    ptr: *mut ffi::World,
    draw_link: DrawLink,
    destruction_listener_link: DestructionListenerLink,
    contact_filter_link: ContactFilterLink,
    contact_listener_link: ContactListenerLink,
    mb_owned: MaybeOwned
}

wrapped!(ffi::World into custom World)

impl BuildWrapped<ffi::World, MaybeOwned> for World {
    fn with(ptr: *mut ffi::World, mb_owned: MaybeOwned) -> World {
        World {
            ptr: ptr,
            draw_link: DrawLink::new(),
            destruction_listener_link: DestructionListenerLink::new(),
            contact_filter_link: ContactFilterLink::new(),
            contact_listener_link: ContactListenerLink::new(),
            mb_owned: mb_owned
        }
    }
}

impl World {
    pub fn new(gravity: &Vec2) -> World {
        unsafe {
            BuildWrapped::with(ffi::World_new(gravity), Owned)
        }
    }
    
    /// __VERIFY__ that there is no problem when the Box is copied in C++
    pub fn set_destruction_listener(&mut self, destruction_listener: Box<DestructionListener>) {
        unsafe {
            self.destruction_listener_link.set_object(mem::transmute(destruction_listener));
            ffi::World_set_destruction_listener(self.mut_ptr(),
                ffi::DestructionListenerLink_as_base(self.destruction_listener_link.mut_ptr())
                );
        }
    }
    
    /// __VERIFY__ that there is no problem when the Box is copied in C++
    pub fn set_contact_filter(&mut self, contact_filter: Box<ContactFilter>) {
        unsafe {
            self.contact_filter_link.set_object(mem::transmute(contact_filter));
            ffi::World_set_contact_filter(self.mut_ptr(),
                ffi::ContactFilterLink_as_base(self.contact_filter_link.mut_ptr())
                );
        }
    }
    
    /// __VERIFY__ that there is no problem when the Box is copied in C++
    pub fn set_contact_listener(&mut self, contact_listener: Box<ContactListener>) {
        unsafe {
            self.contact_listener_link.set_object(mem::transmute(contact_listener));
            ffi::World_set_contact_listener(self.mut_ptr(),
                ffi::ContactListenerLink_as_base(self.contact_listener_link.mut_ptr())
                );
        }
    }
    
    pub fn create_body<'a, 'b>(&'a mut self, def: &BodyDef) -> RefMut<'b, Body> {
        unsafe {
            RefMut::new(BuildWrapped::with(ffi::World_create_body(self.mut_ptr(), def), ()))
        }
    }
    
    pub unsafe fn destroy_body(&mut self, mut body: RefMut<Body>) {
        ffi::World_destroy_body(self.mut_ptr(), body.mut_ptr())
    }
    
    pub fn create_joint<'a, 'b, J: Joint>(&'a mut self, def: &JointDef) -> RefMut<'b, J> {
        unsafe {
            let required_joint_type = JointType::of::<J>();
            assert!(def.joint_type() == required_joint_type ||
                    required_joint_type == UnknownJointType)
            let joint: J = BuildWrappedBase::with(
                ffi::World_create_joint(self.mut_ptr(), def.base_ptr()), ()
                );
            // Probably too much:
            // assert!(joint.joint_type() == required_joint_type ||
            //        required_joint_type == UnknownJointType)
            RefMut::new(joint)
        }
    }
    
    pub unsafe fn destroy_joint<J: Joint>(&mut self, mut joint: RefMut<J>) {
        ffi::World_destroy_joint(self.mut_ptr(), joint.mut_base_ptr())
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
                                      ffi::DrawLink_as_base(self.draw_link.mut_ptr()));
            ffi::World_draw_debug_data(self.mut_ptr());
            ffi::World_set_debug_draw(self.mut_ptr(), ptr::null_mut());
            self.draw_link.set_object(ffi::FatAny::null());
        }
    }
    
    pub fn query_aabb(&self, query_callback: &mut QueryCallback, aabb: &AABB) {
        unsafe {
            let mut query_callback_link = QueryCallbackLink::new();
            query_callback_link.set_object(mem::transmute(query_callback));
            ffi::World_query_aabb(self.ptr(),
                ffi::QueryCallbackLink_as_base(query_callback_link.mut_ptr()),
                aabb
                );
        }
    }
    
    pub fn ray_cast(&self, ray_cast_callback: &mut RayCastCallback, p1: &Vec2, p2: &Vec2) {
        unsafe {
            let mut ray_cast_callback_link = RayCastCallbackLink::new();
            ray_cast_callback_link.set_object(mem::transmute(ray_cast_callback));
            ffi::World_ray_cast(self.ptr(),
                ffi::RayCastCallbackLink_as_base(ray_cast_callback_link.mut_ptr()),
                p1, p2
                );
        }
    }
    
    pub unsafe fn mut_body_list<'a>(&'a mut self) -> Vec<RefMut<'a, Body>> {
        let mut ptr = ffi::World_get_body_list(self.mut_ptr());
            
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push(RefMut::new(BuildWrapped::with(ptr, ())));
            ptr = ffi::Body_get_next(ptr);
        }
        vec
    }
    
    pub unsafe fn body_list<'a>(&'a mut self) -> Vec<Ref<'a, Body>> {
        let mut ptr = ffi::World_get_body_list_const(self.ptr());
            
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push(Ref::new(BuildWrapped::with(ptr as *mut ffi::Body, ())));
            ptr = ffi::Body_get_next_const(ptr);
        }
        vec
    }
    
    pub unsafe fn mut_joint_list<'a>(&'a mut self) -> Vec<RefMut<'a, UnknownJoint>> {
        let mut ptr = ffi::World_get_joint_list(self.mut_ptr());
            
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push(RefMut::new(BuildWrappedBase::with(ptr, ())));
            ptr = ffi::Joint_get_next(ptr);
        }
        vec
    }
    
    pub unsafe fn joint_list<'a>(&'a mut self) -> Vec<Ref<'a, UnknownJoint>> {
        let mut ptr = ffi::World_get_joint_list_const(self.ptr());
            
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push(Ref::new(BuildWrappedBase::with(ptr as *mut ffi::Joint, ())));
            ptr = ffi::Joint_get_next_const(ptr);
        }
        vec
    }
    
    pub unsafe fn mut_contact_list<'a>(&'a mut self) -> *mut ContactEdge {
        ffi::World_get_contact_list(self.mut_ptr())
    }
    
    pub unsafe fn contact_list<'a>(&'a mut self) -> *const ContactEdge {
        ffi::World_get_contact_list_const(self.ptr())
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
            if self.mb_owned == Owned {
                ffi::World_drop(self.mut_ptr())
            }
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
            user_data: ptr::null_mut(),
            gravity_scale: 1.
        }
    }
    
    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        self.user_data = data as ffi::Any
    }
}

wrapped!(ffi::Body into simple Body)

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
    
    pub unsafe fn fixture_list<'a>(&'a self) -> Vec<Ref<'a, Fixture>> {
        let mut ptr = ffi::Body_get_fixture_list_const(self.ptr());
            
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push(Ref::new(BuildWrapped::with(ptr as *mut ffi::Fixture, ())));
            ptr = ffi::Fixture_get_next_const(ptr);
        }
        vec
    }
    
    pub unsafe fn mut_joint_list<'a>(&'a mut self) -> Vec<RefMut<'a, UnknownJoint>> {
        let mut ptr = ffi::Body_get_joint_list(self.mut_ptr());
            
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push((*ptr).mut_joint());
            ptr = (*ptr).mut_next();
        }
        vec
    }
    
    pub unsafe fn joint_list<'a>(&'a self) -> Vec<Ref<'a, UnknownJoint>> {
        let mut ptr = ffi::Body_get_joint_list_const(self.ptr());
        
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push((*ptr).joint());
            ptr = (*ptr).next();
        }
        vec
    }
    
    pub unsafe fn mut_contact_list<'a>(&'a mut self) -> *mut ContactEdge {
        ffi::Body_get_contact_list(self.mut_ptr())
    }
    
    pub unsafe fn contact_list<'a>(&'a mut self) -> *const ContactEdge {
        ffi::Body_get_contact_list_const(self.ptr())
    }
    
    pub unsafe fn next<'a>(&'a self) -> Ref<'a, Body> {
        Ref::new(BuildWrapped::with(ffi::Body_get_next_const(self.ptr()) as *mut ffi::Body, ()))
    }
    
    pub unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Body_get_user_data(self.ptr()) as *mut T
    }

    pub fn create_fixture<'a, 'b>(&'a mut self, shape: &Shape, mut def: FixtureDef
                                  ) -> RefMut<'b, Fixture> {
        unsafe {
            def.shape = shape.base_ptr();
            RefMut::new(BuildWrapped::with(ffi::Body_create_fixture(self.mut_ptr(), &def), ()))
        }
    }
    
    pub fn create_fast_fixture<'a, 'b>(&'a mut self, shape: &Shape, density: f32
                                       ) -> RefMut<'b, Fixture> {
        unsafe {
            RefMut::new(BuildWrapped::with(
                ffi::Body_create_fast_fixture(self.mut_ptr(), shape.base_ptr(), density), ()
                ))
        }
    }
    
    pub unsafe fn destroy_fixture(&mut self, mut fixture: RefMut<Fixture>) {
        ffi::Body_destroy_fixture(self.mut_ptr(), fixture.mut_ptr())
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
    
    pub unsafe fn mut_fixture_list<'a>(&'a mut self) -> Vec<RefMut<'a, Fixture>> {
        let mut ptr = ffi::Body_get_fixture_list(self.mut_ptr());
            
        let mut vec = Vec::new();
        while !ptr.is_null() {
            vec.push(RefMut::new(BuildWrapped::with(ptr, ())));
            ptr = ffi::Fixture_get_next(ptr);
        }
        vec
    }
    
    pub unsafe fn mut_next<'a>(&'a mut self) -> RefMut<'a, Body> {
        RefMut::new(BuildWrapped::with(ffi::Body_get_next(self.mut_ptr()), ()))
    }
    
    pub unsafe fn set_user_data<T>(&mut self, data: *mut T) {
        ffi::Body_set_user_data(self.mut_ptr(), data as ffi::Any)
    }
    
    pub unsafe fn mut_world<'a>(&'a mut self) -> RefMut<'a, World> {
        RefMut::new(BuildWrapped::with(ffi::Body_get_world(self.mut_ptr()), NotOwned))
    }
    
    pub unsafe fn world<'a>(&'a self) -> Ref<'a, World> {
        Ref::new(BuildWrapped::with(
            ffi::Body_get_world_const(self.ptr()) as *mut ffi::World, NotOwned
            ))
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
    pub filter: Filter,
}

impl FixtureDef {
    pub fn new() -> FixtureDef {
        FixtureDef {
            shape: ptr::null(), // need to be specified later
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

wrapped!(ffi::Fixture into simple Fixture)

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
    
    pub unsafe fn body<'a>(&'a mut self) -> Ref<'a, Body>{
        Ref::new(BuildWrapped::with(
            ffi::Fixture_get_body_const(self.ptr()) as *mut ffi::Body, ()
            ))
    }
    
    pub unsafe fn next<'a>(&'a self) -> Ref<'a, Fixture>{
        Ref::new(BuildWrapped::with(
            ffi::Fixture_get_next_const(self.ptr()) as *mut ffi::Fixture, ()
            ))
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
    
    pub fn aabb<'a>(&'a self, child_index: uint) -> &'a AABB {
        unsafe {
            &*ffi::Fixture_get_aabb(self.ptr(), child_index as i32) // Comes from a C++ &
        }
    }
    
    pub unsafe fn user_data<T>(&self) -> *mut T {
        ffi::Fixture_get_user_data(self.ptr()) as *mut T
    }

    pub fn mut_shape<'a>(&'a mut self) -> RefMut<'a, UnknownShape> {
        unsafe {
            RefMut::new(BuildWrappedBase::with(ffi::Fixture_get_shape(self.mut_ptr()), NotOwned))
        }
    }
    
    pub fn shape<'a>(&'a self) -> Ref<'a, UnknownShape> {
        unsafe {
            Ref::new(BuildWrappedBase::with(
                ffi::Fixture_get_shape_const(self.ptr()) as *mut ffi::Shape, NotOwned
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
    
    pub unsafe fn mut_body<'a>(&'a mut self) -> RefMut<'a, Body>{
        RefMut::new(BuildWrapped::with(ffi::Fixture_get_body(self.mut_ptr()), ()))
    }
    
    pub unsafe fn mut_next<'a>(&'a mut self) -> RefMut<'a, Fixture>{
        RefMut::new(BuildWrapped::with(ffi::Fixture_get_next(self.mut_ptr()), ()))
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

wrapped!(ffi::Contact into simple Contact)

#[repr(C)]
pub struct ContactEdge {
    other: *mut ffi::Body,
    contact: *mut ffi::Contact,
    prev: *mut ContactEdge,
    next: *mut ContactEdge
}

impl ContactEdge {
    pub unsafe fn mut_other<'a>(&'a mut self) -> RefMut<'a, Body> {
        RefMut::new(BuildWrapped::with(self.other, ()))
    }
    
    pub unsafe fn other<'a>(&'a self) -> Ref<'a, Body> {
        Ref::new(BuildWrapped::with(self.other, ()))
    }
    
    pub unsafe fn mut_contact<'a>(&'a mut self) -> RefMut<'a, Contact> {
        RefMut::new(BuildWrapped::with(self.contact, ()))
    }
    
    pub unsafe fn contact<'a>(&'a self) -> Ref<'a, Contact> {
        Ref::new(BuildWrapped::with(self.contact, ()))
    }
    
    pub unsafe fn mut_prev(&mut self) -> *mut ContactEdge {
        self.prev
    }
    
    pub unsafe fn prev(&self) -> *const ContactEdge {
        self.prev as *const ContactEdge
    }
    
    pub unsafe fn mut_next(&mut self) -> *mut ContactEdge {
        self.next
    }
    
    pub unsafe fn next(&self) -> *const ContactEdge {
        self.next as *const ContactEdge
    }
}

pub trait DestructionListener {
    fn goodbye_joint(&mut self, joint: RefMut<UnknownJoint>);
    fn goodbye_fixture(&mut self, fixture: RefMut<Fixture>);
}

pub trait ContactFilter {
    fn should_collide(&mut self, fixture_a: RefMut<Fixture>, fixture_b: RefMut<Fixture>) -> bool;
}

pub trait ContactListener {
    fn begin_contact(&mut self, contact: RefMut<Contact>);
    fn end_contact(&mut self, contact: RefMut<Contact>);
    fn pre_solve(&mut self, contact: RefMut<Contact>, manifold: &Manifold);
    fn post_solve(&mut self, contact: RefMut<Contact>, impulse: &ContactImpulse);
}

pub trait QueryCallback {
    fn report_fixture(&mut self, fixture: RefMut<Fixture>) -> bool;
}

pub trait RayCastCallback {
    fn report_fixture(&mut self, fixture: RefMut<Fixture>, p: &Vec2, normal: &Vec2,
                      fraction: f32) -> f32;
}

unsafe extern fn goodbye_joint(any: ffi::FatAny, joint: *mut ffi::Joint) {
    let listener = mem::transmute::<_, &mut DestructionListener>(any);
    listener.goodbye_joint(RefMut::new(BuildWrappedBase::with(joint, ())))
}
unsafe extern fn goodbye_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture) {
    let listener = mem::transmute::<_, &mut DestructionListener>(any);
    listener.goodbye_fixture(RefMut::new(BuildWrapped::with(fixture, ())))
}

unsafe extern fn should_collide(any: ffi::FatAny, fixture_a: *mut ffi::Fixture,
                                fixture_b: *mut ffi::Fixture) -> bool {
    let filter = mem::transmute::<_, &mut ContactFilter>(any);
    filter.should_collide(RefMut::new(BuildWrapped::with(fixture_a, ())),
                          RefMut::new(BuildWrapped::with(fixture_b, ())))
}

unsafe extern fn begin_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.begin_contact(RefMut::new(BuildWrapped::with(contact, ())))
}
unsafe extern fn end_contact(any: ffi::FatAny, contact: *mut ffi::Contact) {
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.end_contact(RefMut::new(BuildWrapped::with(contact, ())))
}
unsafe extern fn pre_solve(any: ffi::FatAny, contact: *mut ffi::Contact,
                           old_manifold: *const Manifold) {
    assert!(!old_manifold.is_null())
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.pre_solve(RefMut::new(BuildWrapped::with(contact, ())), &*old_manifold)
}
unsafe extern fn post_solve(any: ffi::FatAny, contact: *mut ffi::Contact,
                            impulse: *const ContactImpulse) {
    assert!(!impulse.is_null())
    let listener = mem::transmute::<_, &mut ContactListener>(any);
    listener.post_solve(RefMut::new(BuildWrapped::with(contact, ())), &*impulse)
}

unsafe extern fn qc_report_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture
                                   ) -> bool {
    let callback = mem::transmute::<_, &mut QueryCallback>(any);
    callback.report_fixture(RefMut::new(BuildWrapped::with(fixture, ())))
}

unsafe extern fn rcc_report_fixture(any: ffi::FatAny, fixture: *mut ffi::Fixture,
                                    point: *const Vec2, normal: *const Vec2,
                                    fraction: f32) -> f32 {
    // point and normal are coming from C++ &s
    let callback = mem::transmute::<_, &mut RayCastCallback>(any);
    callback.report_fixture(RefMut::new(BuildWrapped::with(fixture, ())), &*point, &*normal,
                            fraction)
}

wrapped!(ffi::DestructionListenerLink into simple DestructionListenerLink)
wrapped!(ffi::ContactFilterLink into simple ContactFilterLink)
wrapped!(ffi::ContactListenerLink into simple ContactListenerLink)
wrapped!(ffi::QueryCallbackLink into simple QueryCallbackLink)
wrapped!(ffi::RayCastCallbackLink into simple RayCastCallbackLink)

impl DestructionListenerLink {
    fn new() -> DestructionListenerLink {
        unsafe {
            BuildWrapped::with(ffi::DestructionListenerLink_new(ffi::FatAny::null(),
                                                                goodbye_joint,
                                                                goodbye_fixture),
                               ())
        }
    }
    
    unsafe fn set_object(&mut self, object: ffi::FatAny) {
        ffi::DestructionListenerLink_set_object(self.mut_ptr(), object);
    }
}

impl ContactFilterLink {
    fn new() -> ContactFilterLink {
        unsafe {
            BuildWrapped::with(ffi::ContactFilterLink_new(ffi::FatAny::null(),
                                                          should_collide),
                               ())
        }
    }
    
    unsafe fn set_object(&mut self, object: ffi::FatAny) {
        ffi::ContactFilterLink_set_object(self.mut_ptr(), object);
    }
}

impl ContactListenerLink {
    fn new() -> ContactListenerLink {
        unsafe {
            BuildWrapped::with(ffi::ContactListenerLink_new(ffi::FatAny::null(),
                                                            begin_contact,
                                                            end_contact,
                                                            pre_solve,
                                                            post_solve),
                               ())
        }
    }
    
    unsafe fn set_object(&mut self, object: ffi::FatAny) {
        ffi::ContactListenerLink_set_object(self.mut_ptr(), object);
    }
}

impl QueryCallbackLink {
    fn new() -> QueryCallbackLink {
        unsafe {
            BuildWrapped::with(ffi::QueryCallbackLink_new(ffi::FatAny::null(),
                                                          qc_report_fixture),
                               ())
        }
    }
    
    unsafe fn set_object(&mut self, object: ffi::FatAny) {
        ffi::QueryCallbackLink_set_object(self.mut_ptr(), object);
    }
}

impl RayCastCallbackLink {
    fn new() -> RayCastCallbackLink {
        unsafe {
            BuildWrapped::with(ffi::RayCastCallbackLink_new(ffi::FatAny::null(),
                                                            rcc_report_fixture),
                               ())
        }
    }
    
    unsafe fn set_object(&mut self, object: ffi::FatAny) {
        ffi::RayCastCallbackLink_set_object(self.mut_ptr(), object);
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
