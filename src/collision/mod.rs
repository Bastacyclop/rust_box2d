pub use self::shapes::{
    Shape, UnknownShape,
    PolygonShape, EdgeShape,  CircleShape, ChainShape,
    MassData, ShapeType,
    PolygonShapeType, EdgeShapeType, CircleShapeType, ChainShapeType, ShapeTypeCount
};

use math::Vec2;

pub mod shapes;

#[repr(C)]
#[deriving(Clone)]
pub struct RayCastInput {
    pub p1: Vec2,
    pub p2: Vec2,
    pub max_fraction: f32
}

#[repr(C)]
#[deriving(Clone)]
pub struct RayCastOutput {
    pub normal: Vec2,
    pub fraction: f32
}

impl RayCastOutput {
    pub fn new() -> RayCastOutput {
        RayCastOutput {
            normal: Vec2 {x:0., y:0.},
            fraction: 0.
        }
    }
}

#[repr(C)]
#[deriving(Clone)]
pub struct AABB {
    pub lower_bound: Vec2,
    pub upper_bound: Vec2
}

impl AABB {
    pub fn new() -> AABB {
        AABB {
            lower_bound: Vec2 { x:0., y:0. },
            upper_bound: Vec2 { x:0., y:0. }
        }
    }
}
