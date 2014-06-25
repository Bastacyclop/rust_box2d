use ffi;
use math::Vec2;
use math::Transform;

pub use self::chain::Chain;

pub mod chain;

pub enum Type {
    Circle,
    Edge,
    Polygon,
    Chain,
    Count,
}

pub struct MassData {
    pub mass: f32,
    pub center: Vec2,
    pub I: f32,
}

impl MassData {
    pub fn new() -> MassData {
        MassData {
            mass: 0.,
            center: Vec2 { x:0., y:0. },
            I: 0.,
        }
    }
}

pub trait Shape {
    unsafe fn from_shape_ptr(ptr: *mut ffi::Shape) -> Self;
    unsafe fn get_shape_ptr(&self) -> *ffi::Shape;
    unsafe fn get_mut_shape_ptr(&mut self) -> *mut ffi::Shape;
    
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
    fn test_point(&self, xf: &Transform, p: Vec2) -> bool {
        unsafe {
            ffi::Shape_test_point_virtual(self.get_shape_ptr(), xf, &p)
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
