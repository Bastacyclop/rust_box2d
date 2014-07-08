use {ffi, Wrapped, clone_from_ptr};
use math::{Vec2, Transform};
use collision::{RayCastInput, RayCastOutput, AABB};
use self::private::WrappedShape;

macro_rules! impl_shape(
    (for $wrap:ty << $shape_as:path >> $as_shape:path) => (
        impl WrappedShape for $wrap {
            unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> $wrap {
                Wrapped::from_ptr($shape_as(ptr))
            }
            unsafe fn shape_ptr(&self) -> *const ffi::Shape {
                $as_shape(self.ptr) as *const ffi::Shape
            }
            unsafe fn mut_shape_ptr(&mut self) -> *mut ffi::Shape {
                $as_shape(self.ptr)
            }
        }
        
        impl Shape for $wrap {}
    );
)

#[allow(visible_private_types)]
pub mod private {
    use ffi;
    pub trait WrappedShape {
        unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Self;
        unsafe fn shape_ptr(&self) -> *const ffi::Shape;
        unsafe fn mut_shape_ptr(&mut self) -> *mut ffi::Shape;
    }
}

c_enum!(ShapeType with
    CIRCLE_SHAPE = 0,
    EDGE_SHAPE = 1,
    POLYGON_SHAPE = 2,
    CHAIN_SHAPE = 3,
    COUNT_SHAPE = 4
)

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

pub trait Shape: WrappedShape {
    /*fn clone(&self, alloc: &mut ffi::BlockAllocator) -> Self {
        WrappedShape::from_shape_ptr(
            ffi::Shape_clone_virtual(self.shape_ptr(), alloc)
            )
    }*/
        
    fn shape_type(&self) -> ShapeType {
        unsafe {
            ffi::Shape_get_type(self.shape_ptr())
        }
    }
    fn child_count(&self) -> uint {
        unsafe {
            ffi::Shape_get_child_count_virtual(self.shape_ptr())
            as uint 
        }
    }
    fn test_point(&self, xf: &Transform, p: &Vec2) -> bool {
        unsafe {
            ffi::Shape_test_point_virtual(self.shape_ptr(), xf, p)
        }
    }
    fn ray_cast(&self, input: &RayCastInput,
                transform: &Transform, child_index: uint) -> RayCastOutput {
        unsafe {
            let mut output = RayCastOutput::new();
            ffi::Shape_ray_cast_virtual(self.shape_ptr(),
                                        &mut output, input,
                                        transform, child_index as i32);
            output
        }
    }
    fn compute_aabb(&self, xf: &Transform, child_index: uint) -> AABB {
        unsafe {
            let mut aabb = AABB::new();
            ffi::Shape_compute_aabb_virtual(self.shape_ptr(), &mut aabb,
                                            xf, child_index as i32);
            aabb
        }
    }
    fn compute_mass(&self, density: f32) -> MassData {
        unsafe {
            let mut mass_data = MassData::new();
            ffi::Shape_compute_mass_virtual(self.shape_ptr(),
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

impl WrappedShape for UnknownShape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> UnknownShape {
        assert!(!ptr.is_null())
        let shape_type = ffi::Shape_get_type(ptr as *const ffi::Shape);
        match shape_type {
            CIRCLE_SHAPE => Circle(
                WrappedShape::from_shape_ptr(ptr)
                ),
            EDGE_SHAPE => Edge(
                WrappedShape::from_shape_ptr(ptr)
                ),
            POLYGON_SHAPE => Polygon(
                WrappedShape::from_shape_ptr(ptr)
                ),
            CHAIN_SHAPE => Chain(
                WrappedShape::from_shape_ptr(ptr)
                ),
            _ => Unknown,
        } 
    }
    unsafe fn shape_ptr(&self) -> *const ffi::Shape {
        match self {
            &Circle(ref x) => x.shape_ptr(),
            &Edge(ref x) => x.shape_ptr(),
            &Polygon(ref x) => x.shape_ptr(),
            &Chain(ref x) => x.shape_ptr(),
            _ => fail!("Truly unknown shape")
        }
    }
    unsafe fn mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        match self {
            &Circle(ref mut x) => x.mut_shape_ptr(),
            &Edge(ref mut x) => x.mut_shape_ptr(),
            &Polygon(ref mut x) => x.mut_shape_ptr(),
            &Chain(ref mut x) => x.mut_shape_ptr(),
            _ => fail!("Truly unknown shape")
        }
    }
}

impl Shape for UnknownShape {}

wrap!(ffi::ChainShape into ChainShape)
wrap!(ffi::CircleShape into CircleShape)
wrap!(ffi::EdgeShape into EdgeShape)
wrap!(ffi::PolygonShape into PolygonShape)

impl ChainShape {
    pub fn new() -> ChainShape {
        unsafe {
            Wrapped::from_ptr(ffi::ChainShape_new())
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
    pub fn child_edge(&self, index: i32) -> EdgeShape {
        unsafe {
            let edge = ffi::EdgeShape_new();
            ffi::ChainShape_get_child_edge(self.ptr(), edge, index);
            Wrapped::from_ptr(edge)
        }
    }
}

impl CircleShape {
    pub fn new() -> CircleShape {
        unsafe {
            Wrapped::from_ptr(ffi::CircleShape_new())
        }
    }
    pub fn support(&self, dir: &Vec2) -> uint {
        unsafe {
            ffi::CircleShape_get_support(self.ptr(), dir) as uint
        }
    }
    pub fn support_vertex(&self, dir: &Vec2) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::CircleShape_get_support_vertex(self.ptr(), dir))
        }
    }
    pub fn vertex_count(&self) -> uint {
        unsafe {
            ffi::CircleShape_get_vertex_count(self.ptr()) as uint
        }
    }
    pub fn vertex(&self, index: uint) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::CircleShape_get_vertex(self.ptr(), index as i32))
        }
    }
}

impl EdgeShape {
    pub fn new() -> EdgeShape {
        unsafe {
            Wrapped::from_ptr(ffi::EdgeShape_new())
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
            Wrapped::from_ptr(ffi::PolygonShape_new())
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
    pub fn vertex_count(&self) -> uint {
        unsafe {
            ffi::PolygonShape_get_vertex_count(self.ptr()) as uint
        }
    }
    pub fn vertex(&self, index: uint) -> Vec2 {
        unsafe {
            clone_from_ptr(ffi::PolygonShape_get_vertex(self.ptr(), index as i32))
        }
    }
    pub fn validate(&self) -> bool {
        unsafe {
            ffi::PolygonShape_validate(self.ptr())
        }
    }
}

impl_shape!(for ChainShape
    << ffi::Shape_as_chain_shape
    >> ffi::ChainShape_as_shape
    )
impl_shape!(for CircleShape
    << ffi::Shape_as_circle_shape
    >> ffi::CircleShape_as_shape
    )
impl_shape!(for EdgeShape
    << ffi::Shape_as_edge_shape
    >> ffi::EdgeShape_as_shape
    )
impl_shape!(for PolygonShape
    << ffi::Shape_as_polygon_shape
    >> ffi::PolygonShape_as_shape
    )

impl Drop for ChainShape {
    fn drop(&mut self) {
        unsafe {
            ffi::ChainShape_drop(self.mut_ptr())
        }
    }
}

impl Drop for CircleShape {
    fn drop(&mut self) {
        unsafe {
            ffi::CircleShape_drop(self.mut_ptr())
        }
    }
}

impl Drop for EdgeShape {
    fn drop(&mut self) {
        unsafe {
            ffi::EdgeShape_drop(self.mut_ptr())
        }
    }
}

impl Drop for PolygonShape {
    fn drop(&mut self) {
        unsafe {
            ffi::PolygonShape_drop(self.mut_ptr())
        }
    }
}
