use std::mem;
use std::ptr;
use std::ops::{Deref, DerefMut};
use wrap::*;
use common::math::Vec2;
use collision::{AABB, RayCastInput, RayCastOutput};
use collision::shapes::{MassData, ShapeType, UnknownShape};
use dynamics::world::BodyHandle;
use dynamics::body::FixtureHandle;
use user_data::{UserDataTypes, UserData, RawUserData, RawUserDataMut, InternalUserData};

#[repr(C)]
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub struct Filter {
    pub category_bits: u16,
    pub mask_bits: u16,
    pub group_index: i16,
}

impl Filter {
    pub fn new() -> Filter {
        Filter {
            category_bits: 0x0001,
            mask_bits: 0xFFFF,
            group_index: 0,
        }
    }
}

#[repr(C)]
#[derive(Clone)]
pub struct FixtureDef {
    #[doc(hidden)]
    pub shape: *const ffi::Shape,
    #[doc(hidden)]
    pub user_data: ffi::Any,
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
            filter: Filter::new(),
        }
    }
}

pub struct MetaFixture<U: UserDataTypes> {
    fixture: Fixture,
    user_data: Box<InternalUserData<Fixture, U::FixtureData>>,
}

impl<U: UserDataTypes> MetaFixture<U> {
    #[doc(hidden)]
    pub unsafe fn new(ptr: *mut ffi::Fixture,
                      handle: FixtureHandle,
                      custom: U::FixtureData)
                      -> Self {
        let mut f = MetaFixture {
            fixture: Fixture::from_ffi(ptr),
            user_data: Box::new(InternalUserData {
                handle: handle,
                custom: custom,
            }),
        };
        f.mut_ptr().set_internal_user_data(&mut *f.user_data);
        f
    }
}

impl<U: UserDataTypes> UserData<U::FixtureData> for MetaFixture<U> {
    fn user_data(&self) -> &U::FixtureData {
        &self.user_data.custom
    }

    fn user_data_mut(&mut self) -> &mut U::FixtureData {
        &mut self.user_data.custom
    }
}

impl<U: UserDataTypes> Deref for MetaFixture<U> {
    type Target = Fixture;

    fn deref(&self) -> &Fixture {
        &self.fixture
    }
}

impl<U: UserDataTypes> DerefMut for MetaFixture<U> {
    fn deref_mut(&mut self) -> &mut Fixture {
        &mut self.fixture
    }
}

wrap! { ffi::Fixture => pub Fixture }

impl Fixture {
    pub fn handle(&self) -> FixtureHandle {
        unsafe { self.ptr().handle() }
    }
        
    pub fn shape_type(&self) -> ShapeType {
        unsafe { ffi::Fixture_get_type(self.ptr()) }
    }

    pub fn is_sensor(&self) -> bool {
        unsafe { ffi::Fixture_is_sensor(self.ptr()) }
    }

    pub fn filter_data<'a>(&'a self) -> &'a Filter {
        unsafe {
            &*ffi::Fixture_get_filter_data(self.ptr()) // Comes from a C++ &
        }
    }

    pub fn body(&self) -> BodyHandle {
        unsafe { ffi::Fixture_get_body_const(self.ptr()).handle() }
    }

    pub fn test_point(&self, point: &Vec2) -> bool {
        unsafe { ffi::Fixture_test_point(self.ptr(), point) }
    }

    pub fn ray_cast(&self, input: &RayCastInput, child_index: i32) -> RayCastOutput {
        unsafe {
            let mut output = mem::zeroed();
            ffi::Fixture_ray_cast(self.ptr(), &mut output, input, child_index);
            output
        }
    }

    pub fn mass_data(&self) -> MassData {
        unsafe {
            let mut data = mem::zeroed();
            ffi::Fixture_get_mass_data(self.ptr(), &mut data);
            data
        }
    }

    pub fn density(&self) -> f32 {
        unsafe { ffi::Fixture_get_density(self.ptr()) }
    }

    pub fn friction(&self) -> f32 {
        unsafe { ffi::Fixture_get_friction(self.ptr()) }
    }

    pub fn restitution(&self) -> f32 {
        unsafe { ffi::Fixture_get_restitution(self.ptr()) }
    }

    pub fn aabb<'a>(&'a self, child_index: i32) -> &'a AABB {
        unsafe {
            &*ffi::Fixture_get_aabb(self.ptr(), child_index) // Comes from a C++ &
        }
    }

    pub fn shape_mut<'a>(&'a mut self) -> WrappedRefMut<'a, UnknownShape> {
        unsafe {
            WrappedRefMut::new(UnknownShape::from_ffi(ffi::Fixture_get_shape(self.mut_ptr())))
        }
    }

    pub fn shape<'a>(&'a self) -> WrappedRef<'a, UnknownShape> {
        unsafe {
            WrappedRef::new(UnknownShape::from_ffi(
                ffi::Fixture_get_shape_const(self.ptr()) as *mut ffi::Shape
            ))
        }
    }

    pub fn set_sensor(&mut self, flag: bool) {
        unsafe { ffi::Fixture_set_sensor(self.mut_ptr(), flag) }
    }

    pub fn set_filter_data(&mut self, filter: &Filter) {
        unsafe { ffi::Fixture_set_filter_data(self.mut_ptr(), filter) }
    }

    pub fn refilter(&mut self) {
        unsafe { ffi::Fixture_refilter(self.mut_ptr()) }
    }

    pub fn set_density(&mut self, density: f32) {
        unsafe { ffi::Fixture_set_density(self.mut_ptr(), density) }
    }

    pub fn set_friction(&mut self, friction: f32) {
        unsafe { ffi::Fixture_set_friction(self.mut_ptr(), friction) }
    }

    pub fn set_restitution(&mut self, restitution: f32) {
        unsafe { ffi::Fixture_set_restitution(self.mut_ptr(), restitution) }
    }

    pub fn dump(&mut self, child_count: i32) {
        unsafe { ffi::Fixture_dump(self.mut_ptr(), child_count) }
    }
}

#[doc(hidden)]
pub mod ffi {
    pub use ffi::Any;
    pub use collision::shapes::ffi::Shape;
    pub use dynamics::body::ffi::Body;
    use common::math::Vec2;
    use collision::{AABB, RayCastInput, RayCastOutput};
    use collision::shapes::{MassData, ShapeType};
    use super::Filter;

    pub enum Fixture {}

    extern "C" {
        pub fn Fixture_get_type(slf: *const Fixture) -> ShapeType;
        pub fn Fixture_get_shape(slf: *mut Fixture) -> *mut Shape;
        pub fn Fixture_get_shape_const(slf: *const Fixture) -> *const Shape;
        pub fn Fixture_set_sensor(slf: *mut Fixture, flag: bool);
        pub fn Fixture_is_sensor(slf: *const Fixture) -> bool;
        pub fn Fixture_set_filter_data(slf: *mut Fixture, filter: *const Filter);
        pub fn Fixture_get_filter_data(slf: *const Fixture) -> *const Filter;
        pub fn Fixture_refilter(slf: *mut Fixture);
        pub fn Fixture_get_body(slf: *mut Fixture) -> *mut Body;
        pub fn Fixture_get_body_const(slf: *const Fixture) -> *const Body;
        // pub fn Fixture_get_next(slf: *mut Fixture) -> *mut Fixture;
        // pub fn Fixture_get_next_const(slf: *const Fixture) -> *const Fixture;
        pub fn Fixture_test_point(slf: *const Fixture, p: *const Vec2) -> bool;
        pub fn Fixture_ray_cast(slf: *const Fixture,
                                output: *mut RayCastOutput,
                                input: *const RayCastInput,
                                child_index: i32)
                                -> bool;
        pub fn Fixture_get_mass_data(slf: *const Fixture, data: *mut MassData);
        pub fn Fixture_set_density(slf: *mut Fixture, density: f32);
        pub fn Fixture_get_density(slf: *const Fixture) -> f32;
        pub fn Fixture_get_friction(slf: *const Fixture) -> f32;
        pub fn Fixture_set_friction(slf: *mut Fixture, friction: f32);
        pub fn Fixture_get_restitution(slf: *const Fixture) -> f32;
        pub fn Fixture_set_restitution(slf: *mut Fixture, restitution: f32);
        pub fn Fixture_get_aabb(slf: *const Fixture, child_id: i32) -> *const AABB;
        pub fn Fixture_dump(slf: *mut Fixture, body_id: i32);
    }
}
