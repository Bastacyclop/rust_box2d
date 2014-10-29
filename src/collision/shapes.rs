use {ffi, Wrapped, BuildWrapped, WrappedBase, BuildWrappedBase, Owned, RefMut};
use math::{Vec2, Transform};
use collision::{RayCastInput, RayCastOutput, AABB};

macro_rules! wrapped_shape(
    ($wrapped:ty into $wrap:ident
     << $base_as:path
     >> $as_base:path) => (
     
        wrapped!($wrapped into $wrap with base ffi::Shape
                 << $base_as
                 >> $as_base)
                 
        impl Shape for $wrap {}
    );
)

#[repr(C)]
#[deriving(PartialEq, Show)]
pub enum ShapeType {
    CircleShapeType = 0,
    EdgeShapeType = 1,
    PolygonShapeType = 2,
    ChainShapeType = 3,
    CountShapeType = 4
}

#[repr(C)]
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

pub trait Shape: WrappedBase<ffi::Shape> {        
    fn shape_type(&self) -> ShapeType {
        unsafe {
            ffi::Shape_get_type(self.base_ptr())
        }
    }
    
    fn child_count(&self) -> uint {
        unsafe {
            ffi::Shape_get_child_count_virtual(self.base_ptr())
            as uint 
        }
    }
    
    fn test_point(&self, xf: &Transform, p: &Vec2) -> bool {
        unsafe {
            ffi::Shape_test_point_virtual(self.base_ptr(), xf, p)
        }
    }
    
    fn ray_cast(&self, input: &RayCastInput, transform: &Transform,
                child_index: uint) -> RayCastOutput {
        unsafe {
            let mut output = RayCastOutput::new();
            ffi::Shape_ray_cast_virtual(self.base_ptr(),
                                        &mut output, input,
                                        transform, child_index as i32);
            output
        }
    }
    
    fn compute_aabb(&self, xf: &Transform, child_index: uint) -> AABB {
        unsafe {
            let mut aabb = AABB::new();
            ffi::Shape_compute_aabb_virtual(self.base_ptr(), &mut aabb,
                                            xf, child_index as i32);
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
        match self {
            &Circle(ref x) => x.base_ptr(),
            &Edge(ref x) => x.base_ptr(),
            &Polygon(ref x) => x.base_ptr(),
            &Chain(ref x) => x.base_ptr(),
            _ => fail!("Truly unknown shape")
        }
    }
    
    unsafe fn mut_base_ptr(&mut self) -> *mut ffi::Shape {
        match self {
            &Circle(ref mut x) => x.mut_base_ptr(),
            &Edge(ref mut x) => x.mut_base_ptr(),
            &Polygon(ref mut x) => x.mut_base_ptr(),
            &Chain(ref mut x) => x.mut_base_ptr(),
            _ => fail!("Truly unknown shape")
        }
    }
}

impl BuildWrappedBase<ffi::Shape, ()> for UnknownShape {
    unsafe fn with(ptr: *mut ffi::Shape, _: ()) -> UnknownShape {
        assert!(!ptr.is_null())
        let shape_type = ffi::Shape_get_type(ptr as *const ffi::Shape);
        match shape_type {
            CircleShapeType => Circle(
                BuildWrappedBase::with(ptr, ())
                ),
            EdgeShapeType => Edge(
                BuildWrappedBase::with(ptr, ())
                ),
            PolygonShapeType => Polygon(
                BuildWrappedBase::with(ptr, ())
                ),
            ChainShapeType => Chain(
                BuildWrappedBase::with(ptr, ())
                ),
            _ => Unknown,
        }
    }
    
}

impl Shape for UnknownShape {}

wrapped_shape!(ffi::ChainShape into ChainShape
               << ffi::Shape_as_chain_shape
               >> ffi::ChainShape_as_shape
               )
wrapped_shape!(ffi::CircleShape into CircleShape
               << ffi::Shape_as_circle_shape
               >> ffi::CircleShape_as_shape
               )
wrapped_shape!(ffi::EdgeShape into EdgeShape
               << ffi::Shape_as_edge_shape
               >> ffi::EdgeShape_as_shape
               )
wrapped_shape!(ffi::PolygonShape into PolygonShape
               << ffi::Shape_as_polygon_shape
               >> ffi::PolygonShape_as_shape
               )

impl ChainShape {
    pub fn new() -> Owned<ChainShape> {
        unsafe {
            Owned::new(BuildWrapped::with(ffi::ChainShape_new(), ()))
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
    
    pub fn child_edge<'a>(&'a self, index: i32) -> RefMut<'a, EdgeShape> {
        unsafe {
            let edge = ffi::EdgeShape_new();
            ffi::ChainShape_get_child_edge(self.ptr(), edge, index);
            RefMut::new(BuildWrapped::with(edge, ()))
        }
    }
}

impl CircleShape {
    pub fn new() -> Owned<CircleShape> {
        unsafe {
            Owned::new(BuildWrapped::with(ffi::CircleShape_new(), ()))
        }
    }
    
    pub fn support(&self, dir: &Vec2) -> uint {
        unsafe {
            ffi::CircleShape_get_support(self.ptr(), dir) as uint
        }
    }
    
    pub fn support_vertex<'a>(&'a self, dir: &Vec2) -> &'a Vec2 {
        unsafe {
            &*ffi::CircleShape_get_support_vertex(self.ptr(), dir) // Comes from a C++ &
        }
    }
    
    pub fn vertex_count(&self) -> uint {
        unsafe {
            ffi::CircleShape_get_vertex_count(self.ptr()) as uint
        }
    }
    
    pub fn vertex<'a>(&'a self, index: uint) -> &'a Vec2 {
        unsafe {
            &*ffi::CircleShape_get_vertex(self.ptr(), index as i32) // Comes from a C++ &
        }
    }
}

impl EdgeShape {
    pub fn new() -> Owned<EdgeShape> {
        unsafe {
            Owned::new(BuildWrapped::with(ffi::EdgeShape_new(), ()))
        }
    }
    
    pub fn set(&mut self, v1: &Vec2, v2: &Vec2) {
        unsafe {
            ffi::EdgeShape_set(self.mut_ptr(), v1, v2)
        }
    }
}

impl PolygonShape {
    pub fn new() -> Owned<PolygonShape> {
        unsafe {
            Owned::new(BuildWrapped::with(ffi::PolygonShape_new(), ()))
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
    
    pub fn vertex<'a>(&'a self, index: uint) -> &'a Vec2 {
        unsafe {
            &*ffi::PolygonShape_get_vertex(self.ptr(), index as i32) // Comes from a C++ &
        }
    }
    
    pub fn validate(&self) -> bool {
        unsafe {
            ffi::PolygonShape_validate(self.ptr())
        }
    }
}

#[unsafe_destructor]
impl Drop for Owned<ChainShape> {
    fn drop(&mut self) {
        unsafe {
            ffi::ChainShape_drop(self.mut_ptr())
        }
    }
}

#[unsafe_destructor]
impl Drop for Owned<CircleShape> {
    fn drop(&mut self) {
        unsafe {
            ffi::CircleShape_drop(self.mut_ptr())
        }
    }
}

#[unsafe_destructor]
impl Drop for Owned<EdgeShape> {
    fn drop(&mut self) {
        unsafe {
            ffi::EdgeShape_drop(self.mut_ptr())
        }
    }
}

#[unsafe_destructor]
impl Drop for Owned<PolygonShape> {
    fn drop(&mut self) {
        unsafe {
            ffi::PolygonShape_drop(self.mut_ptr())
        }
    }
}
