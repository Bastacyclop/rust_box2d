use ffi;
use math::Vec2;
use math::Transform;

pub use self::chain::ChainStruct;
pub use self::chain::Chain;
pub use self::edge::Edge;
pub use self::circle::Circle;
pub use self::polygon::Polygon;

pub mod chain;
pub mod edge;
pub mod circle;
pub mod polygon;

c_enum!(
    [Type]
    CIRCLE = 0,
    EDGE = 1,
    POLYGON = 2,
    CHAIN = 3,
    COUNT = 4
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

pub trait Shape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Self;
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape;
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape;
    
    unsafe fn clone(&self, alloc: &mut ffi::BlockAllocator) -> Self {
        Shape::from_shape_ptr(
            ffi::Shape_clone_virtual(self.get_shape_ptr(), alloc)
            )
    }
        
    fn get_type(&self) -> Type {
        unsafe {
            ffi::Shape_get_type(self.get_shape_ptr())
        }
    }
    fn get_child_count(&self) -> uint {
        unsafe {
            ffi::Shape_get_child_count_virtual(self.get_shape_ptr())
            as uint 
        }
    }
    fn test_point(&self, xf: &Transform, p: &Vec2) -> bool {
        unsafe {
            ffi::Shape_test_point_virtual(self.get_shape_ptr(), xf, p)
        }
    }
    fn compute_mass(&self, density: f32) -> MassData {
        unsafe {
            let mut mass_data = MassData::new();
            ffi::Shape_compute_mass_virtual(self.get_shape_ptr(),
                                            &mut mass_data, density);
            mass_data
        }
    }
}

pub enum Unknown {
    None,
    SomeCircle(Circle),
    SomeEdge(Edge),
    SomePolygon(Polygon),
    SomeChain(ChainStruct),
}

impl Unknown {
    pub unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Unknown {
        assert!(!ptr.is_null())
        let shape_type = ffi::Shape_get_type(ptr as *ffi::Shape);
        match shape_type {
            CIRCLE => SomeCircle(
                Shape::from_shape_ptr(ptr)
                ),
            EDGE => SomeEdge(
                Shape::from_shape_ptr(ptr)
                ),
            POLYGON => SomePolygon(
                Shape::from_shape_ptr(ptr)
                ),
            CHAIN => SomeChain(
                Shape::from_shape_ptr(ptr)
                ),
            _ => None,
        } 
    }
}
