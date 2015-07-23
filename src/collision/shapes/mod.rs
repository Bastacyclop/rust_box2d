use std::mem;
use wrap::*;
use common::math::{ Vec2, Transform };
use collision::{ RayCastInput, RayCastOutput, AABB };

macro_rules! wrap_shape {
    {
        $wrapped:ty => $wrap:ident
        < $as_base:path
        > $base_as:path
    } => {
        wrap! {
            $wrapped (base ffi::Shape) => pub $wrap
            < $as_base
            > $base_as
        }

        impl Shape for $wrap {}
    };
}

pub mod chain;
pub mod edge;
pub mod circle;
pub mod polygon;

pub use self::chain::ChainShape;
pub use self::circle::CircleShape;
pub use self::edge::EdgeShape;
pub use self::polygon::PolygonShape;

#[repr(C)]
#[derive(Clone, Debug)]
pub struct MassData {
    pub mass: f32,
    pub center: Vec2,
    pub inertia: f32,
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ShapeType {
    Circle,
    Edge,
    Polygon,
    Chain,
    Count
}

pub trait Shape: WrappedBase<ffi::Shape> {
    fn shape_type(&self) -> ShapeType {
        unsafe {
            ffi::Shape_get_type(self.base_ptr())
        }
    }

    fn child_count(&self) -> i32 {
        unsafe {
            ffi::Shape_get_child_count_virtual(self.base_ptr())
        }
    }

    fn test_point(&self, xf: &Transform, p: &Vec2) -> bool {
        unsafe {
            ffi::Shape_test_point_virtual(self.base_ptr(), xf, p)
        }
    }

    fn ray_cast(&self, input: &RayCastInput,
                       transform: &Transform,
                       child_index: i32) -> RayCastOutput {
        unsafe {
            let mut output = mem::zeroed();
            ffi::Shape_ray_cast_virtual(self.base_ptr(),
                                        &mut output,
                                        input,
                                        transform,
                                        child_index);
            output
        }
    }

    fn compute_aabb(&self, xf: &Transform, child_index: i32) -> AABB {
        unsafe {
            let mut aabb = mem::zeroed();
            ffi::Shape_compute_aabb_virtual(self.base_ptr(),
                                            &mut aabb,
                                            xf,
                                            child_index);
            aabb
        }
    }

    fn compute_mass(&self, density: f32) -> MassData {
        unsafe {
            let mut mass_data = mem::zeroed();
            ffi::Shape_compute_mass_virtual(self.base_ptr(),
                                            &mut mass_data,
                                            density);
            mass_data
        }
    }
}

pub enum UnknownShape {
    Unknown,
    Circle(CircleShape),
    Edge(EdgeShape),
    Polygon(PolygonShape),
    Chain(ChainShape),
}

impl WrappedBase<ffi::Shape> for UnknownShape {
    unsafe fn base_ptr(&self) -> *const ffi::Shape {
        use self::UnknownShape::*;
        match self {
            &Circle(ref x) => x.base_ptr(),
            &Edge(ref x) => x.base_ptr(),
            &Polygon(ref x) => x.base_ptr(),
            &Chain(ref x) => x.base_ptr(),
            _ => panic!("Truly unknown shape")
        }
    }

    unsafe fn mut_base_ptr(&mut self) -> *mut ffi::Shape {
        use self::UnknownShape::*;
        match self {
            &mut Circle(ref mut x) => x.mut_base_ptr(),
            &mut Edge(ref mut x) => x.mut_base_ptr(),
            &mut Polygon(ref mut x) => x.mut_base_ptr(),
            &mut Chain(ref mut x) => x.mut_base_ptr(),
            _ => panic!("Truly unknown shape")
        }
    }
}

impl FromFFI<ffi::Shape> for UnknownShape {
    unsafe fn from_ffi(ptr: *mut ffi::Shape) -> UnknownShape {
        use self::UnknownShape::*;
        assert!(!ptr.is_null());
        let shape_type = ffi::Shape_get_type(ptr as *const ffi::Shape);
        match shape_type {
            ShapeType::Circle => Circle(CircleShape::from_ffi(ptr)),
            ShapeType::Edge => Edge(EdgeShape::from_ffi(ptr)),
            ShapeType::Polygon => Polygon(PolygonShape::from_ffi(ptr)),
            ShapeType::Chain => Chain(ChainShape::from_ffi(ptr)),
            _ => Unknown,
        }
    }

}

impl Shape for UnknownShape {}

#[doc(hidden)]
pub mod ffi {
    use common::math::{ Vec2, Transform };
    use collision::{ AABB, RayCastInput, RayCastOutput };
    use super::{ ShapeType, MassData };

    #[repr(C)] pub struct Shape;

    extern {
        pub fn Shape_get_type(slf: *const Shape) -> ShapeType;
        pub fn Shape_get_child_count_virtual(slf: *const Shape) -> i32;
        pub fn Shape_test_point_virtual(slf: *const Shape,
                                        xf: *const Transform,
                                        p: *const Vec2) -> bool;
        pub fn Shape_ray_cast_virtual(slf: *const Shape,
                                      output: *mut RayCastOutput,
                                      input: *const RayCastInput,
                                      transform: *const Transform,
                                      child_index: i32) -> bool;
        pub fn Shape_compute_aabb_virtual(slf: *const Shape,
                                          aabb: *mut AABB,
                                          xf: *const Transform,
                                          child_id: i32);
        pub fn Shape_compute_mass_virtual(slf: *const Shape,
                                          data: *mut MassData,
                                          density: f32);
        pub fn Shape_get_radius(slf: *const Shape) -> f32;
        pub fn Shape_set_radius(slf: *mut Shape, radius: f32);
    }
}
