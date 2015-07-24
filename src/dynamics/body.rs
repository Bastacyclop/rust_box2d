use std::mem;
use std::ptr;
use std::any::Any;
use std::ops::{ Deref, DerefMut };
use std::cell::{ Ref, RefMut };
use wrap::*;
use handle::*;
use common::math::{ Vec2, Transform };
use collision::shapes::{ MassData, Shape };
use dynamics::world::BodyHandle;
use dynamics::joints::JointEdge;
use dynamics::fixture::{ MetaFixture, FixtureDef };
use dynamics::contacts::ContactEdge;
use dynamics::user_data::{ UserData, InternalUserData, RawUserDataMut };

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
    #[doc(hidden)] pub user_data: ffi::Any,
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
}

pub type FixtureHandle = TypedHandle<MetaFixture>;

pub struct MetaBody {
    body: Body,
    fixtures: HandleMap<MetaFixture>,
    user_data: Box<InternalUserData<MetaBody>>
}

impl MetaBody {
    #[doc(hidden)]
    pub unsafe fn new(ptr: *mut ffi::Body, handle: BodyHandle) -> MetaBody {
        let mut b = MetaBody {
            body: Body::from_ffi(ptr),
            fixtures: HandleMap::new(),
            user_data: Box::new(InternalUserData{
                handle: handle,
                custom: None
            })
        };
        b.mut_ptr().set_internal_user_data(&mut *b.user_data);
        b
    }

    pub fn create_fixture(&mut self,
                          shape: &Shape,
                          def: &mut FixtureDef) -> FixtureHandle {
        unsafe {
            def.shape = shape.base_ptr();
            let fixture = ffi::Body_create_fixture(self.mut_ptr(), def);
            self.fixtures.insert_with(|h| MetaFixture::new(fixture, h))
        }
    }

    pub fn create_fast_fixture(&mut self,
                               shape: &Shape,
                               density: f32) -> FixtureHandle {
        unsafe {
            let fixture = ffi::Body_create_fast_fixture(
                self.mut_ptr(),
                shape.base_ptr(),
                density
            );
            self.fixtures.insert_with(|h| MetaFixture::new(fixture, h))
        }
    }

    pub fn get_fixture(&self, handle: FixtureHandle) -> Ref<MetaFixture> {
        self.fixtures.get(handle).expect("invalid fixture handle")
    }

    pub fn get_fixture_mut(&self, handle: FixtureHandle) -> RefMut<MetaFixture> {
        self.fixtures.get_mut(handle).expect("invalid fixture handle")
    }

    pub fn destroy_fixture(&mut self, handle: FixtureHandle) {
        self.fixtures.remove(handle)
            .map(|mut meta_fixture| {
                unsafe {
                    ffi::Body_destroy_fixture(self.mut_ptr(), meta_fixture.mut_ptr());
                }
            });
    }

    pub fn fixtures(&self) -> HandleIter<MetaFixture> {
        self.fixtures.iter()
    }
}

impl UserData for MetaBody {
    fn get_user_data(&self) -> &Option<Box<Any>> {
        &self.user_data.custom
    }

    fn get_user_data_mut(&mut self) -> &mut Option<Box<Any>> {
        &mut self.user_data.custom
    }
}

impl Deref for MetaBody {
    type Target = Body;

    fn deref(&self) -> &Body { &self.body }
}

impl DerefMut for MetaBody {
    fn deref_mut(&mut self) -> &mut Body { &mut self.body }
}

wrap! { ffi::Body => pub Body }

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
            let mut data = mem::zeroed();
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

    pub fn joints<'a>(&'a self) -> Option<&'a JointEdge> {
        unsafe {
            ffi::Body_get_joint_list_const(self.ptr()).as_ref()
        }
    }

    pub fn joints_mut<'a>(&'a mut self) -> Option<&'a mut JointEdge> {
        unsafe {
            ffi::Body_get_joint_list(self.mut_ptr()).as_mut()
        }
    }

    pub fn contacts<'a>(&'a self) -> Option<&'a ContactEdge> {
        unsafe {
            ffi::Body_get_contact_list_const(self.ptr()).as_ref()
        }
    }

    pub fn contacts_mut<'a>(&'a mut self) -> Option<&'a mut ContactEdge> {
        unsafe {
            ffi::Body_get_contact_list(self.mut_ptr()).as_mut()
        }
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

    pub fn dump(&mut self) {
        unsafe {
            ffi::Body_dump(self.mut_ptr())
        }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use ffi::Any;
    pub use collision::shapes::ffi::Shape;
    pub use dynamics::fixture::ffi::Fixture;
    use common::math::{ Vec2, Transform };
    use collision::shapes::MassData;
    use dynamics::fixture::FixtureDef;
    use dynamics::joints::JointEdge;
    use dynamics::contacts::ContactEdge;
    use super::BodyType;

    #[repr(C)] pub struct Body;

    extern {
        pub fn Body_create_fixture(slf: *mut Body, def: *const FixtureDef
                                   ) -> *mut Fixture;
        pub fn Body_create_fast_fixture(slf: *mut Body, shape: *const Shape,
                                        density: f32) -> *mut Fixture;
        pub fn Body_destroy_fixture(slf: *mut Body, fixture: *mut Fixture);
        pub fn Body_set_transform(slf: *mut Body, pos: *const Vec2, angle: f32);
        pub fn Body_get_transform(slf: *const Body) -> *const Transform;
        pub fn Body_get_position(slf: *const Body) -> *const Vec2;
        pub fn Body_get_angle(slf: *const Body) -> f32;
        pub fn Body_get_world_center(slf: *const Body) -> *const Vec2;
        pub fn Body_get_local_center(slf: *const Body) -> *const Vec2;
        pub fn Body_set_linear_velocity(slf: *mut Body, v: *const Vec2);
        pub fn Body_get_linear_velocity(slf: *const Body) -> *const Vec2;
        pub fn Body_set_angular_velocity(slf: *mut Body, omega: f32);
        pub fn Body_get_angular_velocity(slf: *const Body) -> f32;
        pub fn Body_apply_force(slf: *mut Body, force: *const Vec2,
                                point: *const Vec2, wake: bool);
        pub fn Body_apply_force_to_center(slf: *mut Body, force: *const Vec2,
                                          wake: bool);
        pub fn Body_apply_torque(slf: *mut Body, torque: f32, wake: bool);
        pub fn Body_apply_linear_impulse(slf: *mut Body, impulse: *const Vec2,
                                         point: *const Vec2, wake: bool);
        pub fn Body_apply_angular_impulse(slf: *mut Body, impulse: f32,
                                          wake: bool);
        pub fn Body_get_mass(slf: *const Body) -> f32;
        pub fn Body_get_inertia(slf: *const Body) -> f32;
        pub fn Body_get_mass_data(slf: *const Body, data: *mut MassData);
        pub fn Body_set_mass_data(slf: *mut Body, data: *const MassData);
        pub fn Body_reset_mass_data(slf: *mut Body);
        pub fn Body_get_world_point(slf: *const Body, local: *const Vec2) -> Vec2;
        pub fn Body_get_world_vector(slf: *const Body, local: *const Vec2) -> Vec2;
        pub fn Body_get_local_point(slf: *const Body, world: *const Vec2) -> Vec2;
        pub fn Body_get_local_vector(slf: *const Body, world: *const Vec2) -> Vec2;
        pub fn Body_get_linear_velocity_from_world_point(slf: *const Body,
                                                         point: *const Vec2
                                                         ) -> Vec2;
        pub fn Body_get_linear_velocity_from_local_point(slf: *const Body,
                                                         point: *const Vec2
                                                         ) -> Vec2;
        pub fn Body_get_linear_damping(slf: *const Body) -> f32;
        pub fn Body_set_linear_damping(slf: *mut Body, damping: f32);
        pub fn Body_get_angular_damping(slf: *const Body) -> f32;
        pub fn Body_set_angular_damping(slf: *mut Body, damping: f32);
        pub fn Body_get_gravity_scale(slf: *const Body) -> f32;
        pub fn Body_set_gravity_scale(slf: *mut Body, scale: f32);
        pub fn Body_set_type(slf: *mut Body, typ: BodyType);
        pub fn Body_get_type(slf: *const Body) -> BodyType;
        pub fn Body_set_bullet(slf: *mut Body, flag: bool);
        pub fn Body_is_bullet(slf: *const Body) -> bool;
        pub fn Body_set_sleeping_allowed(slf: *mut Body, flag: bool);
        pub fn Body_is_sleeping_allowed(slf: *const Body) -> bool;
        pub fn Body_set_awake(slf: *mut Body, flag: bool);
        pub fn Body_is_awake(slf: *const Body) -> bool;
        pub fn Body_set_active(slf: *mut Body, flag: bool);
        pub fn Body_is_active(slf: *const Body) -> bool;
        pub fn Body_set_fixed_rotation(slf: *mut Body, flag: bool);
        pub fn Body_is_fixed_rotation(slf: *const Body) -> bool;
        //pub fn Body_get_fixture_list(slf: *mut Body) -> *mut Fixture;
        //pub fn Body_get_fixture_list_const(slf: *const Body) -> *const Fixture;
        pub fn Body_get_joint_list(slf: *mut Body) -> *mut JointEdge;
        pub fn Body_get_joint_list_const(slf: *const Body) -> *const JointEdge;
        pub fn Body_get_contact_list(slf: *mut Body) -> *mut ContactEdge;
        pub fn Body_get_contact_list_const(slf: *const Body) -> *const ContactEdge;
        //pub fn Body_get_next(slf: *mut Body) -> *mut Body;
        //pub fn Body_get_next_const(slf: *const Body) -> *const Body;
        //pub fn Body_get_world(slf: *mut Body) -> *mut World;
        //pub fn Body_get_world_const(slf: *const Body) -> *const World;
        pub fn Body_dump(slf: *mut Body);
    }
}
