pub use self::shapes::{
    Shape, UnknownShape,
    PolygonShape, EdgeShape,
    CircleShape, ChainShape,
    MassData, ShapeType,
    POLYGON, EDGE, CIRCLE, CHAIN, COUNT
};

use math::Vec2;

pub mod shapes;

#[packed]
#[deriving(Clone)]
pub struct RayCastInput {
    pub p1: Vec2,
    pub p2: Vec2,
    pub max_fraction: f32
}

#[packed]
#[deriving(Clone)]
pub struct RayCastOutput {
    pub normal: Vec2,
    pub fraction: f32
}

#[packed]
#[deriving(Clone)]
pub struct AABB {
    pub lower_bound: Vec2,
    pub upper_bound: Vec2
}
