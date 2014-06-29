use ffi;
use math::Vec2;
use math::Transform;

pub use self::chain::ChainShape;
pub use self::edge::EdgeShape;
pub use self::circle::CircleShape;
pub use self::polygon::PolygonShape;

#[macro_export]
macro_rules! impl_shape(
    (for $wrap:ty << $shape_as:path >> $as_shape:path) => (
        impl WrappedShape for $wrap {
            unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> $wrap {
                Wrapped::from_ptr($shape_as(ptr))
            }
            unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
                $as_shape(self.ptr) as *ffi::Shape
            }
            unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
                $as_shape(self.ptr)
            }
        }
        
        impl Shape for $wrap {}
    );
)

pub mod chain;
pub mod edge;
pub mod circle;
pub mod polygon;

c_enum!(ShapeType with
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

trait WrappedShape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Self;
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape;
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape;
}

pub trait Shape: WrappedShape {
    /*fn clone(&self, alloc: &mut ffi::BlockAllocator) -> Self {
        WrappedShape::from_shape_ptr(
            ffi::Shape_clone_virtual(self.get_shape_ptr(), alloc)
            )
    }*/
        
    fn get_type(&self) -> ShapeType {
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

pub enum UnknownShape {
    None,
    Circle(CircleShape),
    Edge(EdgeShape),
    Polygon(PolygonShape),
    Chain(ChainShape),
}

impl WrappedShape for UnknownShape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> UnknownShape {
        assert!(!ptr.is_null())
        let shape_type = ffi::Shape_get_type(ptr as *ffi::Shape);
        match shape_type {
            CIRCLE => Circle(
                WrappedShape::from_shape_ptr(ptr)
                ),
            EDGE => Edge(
                WrappedShape::from_shape_ptr(ptr)
                ),
            POLYGON => Polygon(
                WrappedShape::from_shape_ptr(ptr)
                ),
            CHAIN => Chain(
                WrappedShape::from_shape_ptr(ptr)
                ),
            _ => None,
        } 
    }
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape {
        match self {
            &Circle(ref x) => x.get_shape_ptr(),
            &Edge(ref x) => x.get_shape_ptr(),
            &Polygon(ref x) => x.get_shape_ptr(),
            &Chain(ref x) => x.get_shape_ptr(),
            _ => fail!("Truly unknown shape")
        }
    }
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape {
        self.get_shape_ptr() as *mut ffi::Shape
    }
}

/*#[unsafe_destructor]
impl Drop for UnknownShape {
    fn drop(&mut self) {
        unsafe {
            ffi::Shape_drop_virtual(self.get_mut_shape_ptr())
        }
    }    
}*/
