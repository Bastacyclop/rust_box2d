use {
    ffi, MaybeOwned, Owned, NotOwned, Ref,
    Wrapped, BuildWrapped, WrappedBase, BuildWrappedBase
};
use math::{Vec2, Transform};
use collision::{RayCastInput, RayCastOutput, AABB};

macro_rules! wrap_shape {
    ($wrapped:ty: $wrap:ident
     > $as_base:path,
     < $base_as:path
    ) => (
        wrap! {
            $wrapped: $wrap with base ffi::Shape
            > $as_base, < $base_as
        }

        impl Shape for $wrap {}
    );
}

#[repr(C)]
#[derive(Clone)]
pub struct MassData {
    pub mass: f32,
    pub center: Vec2,
    pub inertia: f32,
}

impl MassData {
    pub fn new() -> MassData {
        MassData {
            mass: 0.,
            center: Vec2 { x:0., y:0. },
            inertia: 0.,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ShapeType {
    Circle = 0,
    Edge = 1,
    Polygon = 2,
    Chain = 3,
    Count = 4
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

    fn ray_cast(&self, input: &RayCastInput, transform: &Transform,
                child_index: i32) -> RayCastOutput {
        unsafe {
            let mut output = RayCastOutput::new();
            ffi::Shape_ray_cast_virtual(self.base_ptr(),
                                        &mut output, input,
                                        transform, child_index);
            output
        }
    }

    fn compute_aabb(&self, xf: &Transform, child_index: i32) -> AABB {
        unsafe {
            let mut aabb = AABB::new();
            ffi::Shape_compute_aabb_virtual(self.base_ptr(), &mut aabb,
                                            xf, child_index);
            aabb
        }
    }

    fn compute_mass(&self, density: f32) -> MassData {
        unsafe {
            let mut mass_data = MassData::new();
            ffi::Shape_compute_mass_virtual(self.base_ptr(),
                                            &mut mass_data, density);
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
        use super::UnknownShape::*;
        match self {
            &Circle(ref x) => x.base_ptr(),
            &Edge(ref x) => x.base_ptr(),
            &Polygon(ref x) => x.base_ptr(),
            &Chain(ref x) => x.base_ptr(),
            _ => panic!("Truly unknown shape")
        }
    }

    unsafe fn mut_base_ptr(&mut self) -> *mut ffi::Shape {
        use super::UnknownShape::*;
        match self {
            &mut Circle(ref mut x) => x.mut_base_ptr(),
            &mut Edge(ref mut x) => x.mut_base_ptr(),
            &mut Polygon(ref mut x) => x.mut_base_ptr(),
            &mut Chain(ref mut x) => x.mut_base_ptr(),
            _ => panic!("Truly unknown shape")
        }
    }
}

impl BuildWrappedBase<ffi::Shape, MaybeOwned> for UnknownShape {
    unsafe fn with(ptr: *mut ffi::Shape, mb_owned: MaybeOwned) -> UnknownShape {
        use super::UnknownShape::*;
        assert!(!ptr.is_null());
        let shape_type = ffi::Shape_get_type(ptr as *const ffi::Shape);
        match shape_type {
            ShapeType::Circle => Circle(
                BuildWrappedBase::with(ptr, mb_owned)
                ),
            ShapeType::Edge => Edge(
                BuildWrappedBase::with(ptr, mb_owned)
                ),
            ShapeType::Polygon => Polygon(
                BuildWrappedBase::with(ptr, mb_owned)
                ),
            ShapeType::Chain => Chain(
                BuildWrappedBase::with(ptr, mb_owned)
                ),
            _ => Unknown,
        }
    }

}

impl Shape for UnknownShape {}

wrap_shape! {
    ffi::ChainShape: ChainShape
    > ffi::ChainShape_as_shape,
    < ffi::Shape_as_chain_shape
}

wrap_shape! {
    ffi::CircleShape: CircleShape
    > ffi::CircleShape_as_shape,
    < ffi::Shape_as_circle_shape
}

wrap_shape! {
    ffi::EdgeShape: EdgeShape
    > ffi::EdgeShape_as_shape,
    < ffi::Shape_as_edge_shape
}

wrap_shape! {
    ffi::PolygonShape: PolygonShape
    > ffi::PolygonShape_as_shape,
    < ffi::Shape_as_polygon_shape
}

impl ChainShape {
    pub fn new() -> ChainShape {
        unsafe {
            BuildWrapped::with(ffi::ChainShape_new(), Owned)
        }
    }

    pub fn clear(&mut self) {
        unsafe {
            ffi::ChainShape_clear(self.mut_ptr())
        }
    }

    pub fn create_loop(&mut self, vertices: Vec<Vec2>) {
        unsafe {
            ffi::ChainShape_create_loop(self.mut_ptr(),
                                        vertices.as_ptr(),
                                        vertices.len() as i32
                                        )
        }
    }

    pub fn create_chain(&mut self, vertices: Vec<Vec2>) {
        unsafe {
            ffi::ChainShape_create_chain(self.mut_ptr(),
                                         vertices.as_ptr(),
                                         vertices.len() as i32
                                         )
        }
    }

    pub fn set_prev_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_prev_vertex(self.mut_ptr(), vertex)
        }
    }

    pub fn set_next_vertex(&mut self, vertex: &Vec2) {
        unsafe {
            ffi::ChainShape_set_next_vertex(self.mut_ptr(), vertex)
        }
    }

    pub fn child_edge<'a>(&'a self, index: i32) -> Ref<'a, EdgeShape> {
        unsafe {
            let edge = ffi::EdgeShape_new();
            ffi::ChainShape_get_child_edge(self.ptr(), edge, index);
            Ref::new(BuildWrapped::with(edge, NotOwned))
        }
    }
}

impl CircleShape {
    pub fn new() -> CircleShape {
        unsafe {
            BuildWrapped::with(ffi::CircleShape_new(), Owned)
        }
    }

    pub fn support(&self, dir: &Vec2) -> i32 {
        unsafe {
            ffi::CircleShape_get_support(self.ptr(), dir)
        }
    }

    pub fn support_vertex<'a>(&'a self, dir: &Vec2) -> &'a Vec2 {
        unsafe {
            &*ffi::CircleShape_get_support_vertex(self.ptr(), dir) // Comes from a C++ &
        }
    }

    pub fn vertex_count(&self) -> i32 {
        unsafe {
            ffi::CircleShape_get_vertex_count(self.ptr())
        }
    }

    pub fn vertex<'a>(&'a self, index: i32) -> &'a Vec2 {
        unsafe {
            &*ffi::CircleShape_get_vertex(self.ptr(), index) // Comes from a C++ &
        }
    }

    pub fn radius(&self) -> f32 {
        unsafe {
            ffi::Shape_get_radius(self.base_ptr())
        }
    }

    pub fn set_radius(&mut self, radius: f32) {
        unsafe {
            ffi::Shape_set_radius(self.mut_base_ptr(), radius)
        }
    }

    pub fn position(&self) -> Vec2 {
        unsafe {
            ffi::CircleShape_get_pos(self.ptr())
        }
    }

    pub fn set_position(&mut self, pos: Vec2) {
        unsafe {
            ffi::CircleShape_set_pos(self.mut_ptr(), pos)
        }
    }
}

impl EdgeShape {
    pub fn new() -> EdgeShape {
        unsafe {
            BuildWrapped::with(ffi::EdgeShape_new(), Owned)
        }
    }

    pub fn set(&mut self, v1: &Vec2, v2: &Vec2) {
        unsafe {
            ffi::EdgeShape_set(self.mut_ptr(), v1, v2)
        }
    }
}

impl PolygonShape {
    pub fn new() -> PolygonShape {
        unsafe {
            BuildWrapped::with(ffi::PolygonShape_new(), Owned)
        }
    }

    pub fn set(&mut self, points: Vec<Vec2>) {
        unsafe {
            ffi::PolygonShape_set(self.mut_ptr(),
                                  points.as_ptr(),
                                  points.len() as i32)
        }
    }

    pub fn set_as_box(&mut self, hw: f32, hh: f32) {
        unsafe {
            ffi::PolygonShape_set_as_box(self.mut_ptr(), hw, hh)
        }
    }

    pub fn set_as_oriented_box(&mut self, hw: f32, hh: f32,
                               center: &Vec2, angle: f32) {
        unsafe {
            ffi::PolygonShape_set_as_oriented_box(self.mut_ptr(),
                                                  hw, hh,
                                                  center, angle)
        }
    }

    pub fn vertex_count(&self) -> i32 {
        unsafe {
            ffi::PolygonShape_get_vertex_count(self.ptr())
        }
    }

    pub fn vertex<'a>(&'a self, index: i32) -> &'a Vec2 {
        unsafe {
            &*ffi::PolygonShape_get_vertex(self.ptr(), index) // Comes from a C++ &
        }
    }

    pub fn validate(&self) -> bool {
        unsafe {
            ffi::PolygonShape_validate(self.ptr())
        }
    }
}

impl Drop for ChainShape {
    fn drop(&mut self) {
        unsafe {
            if self.mb_owned == Owned {
                ffi::ChainShape_drop(self.mut_ptr())
            }
        }
    }
}

impl Drop for CircleShape {
    fn drop(&mut self) {
        unsafe {
            if self.mb_owned == Owned {
                ffi::CircleShape_drop(self.mut_ptr())
            }
        }
    }
}

impl Drop for EdgeShape {
    fn drop(&mut self) {
        unsafe {
            if self.mb_owned == Owned {
                ffi::EdgeShape_drop(self.mut_ptr())
            }
        }
    }
}

impl Drop for PolygonShape {
    fn drop(&mut self) {
        unsafe {
            if self.mb_owned == Owned {
                ffi::PolygonShape_drop(self.mut_ptr())
            }
        }
    }
}
