pub mod math;
pub mod settings;

use math::{ Vec2, Transform };

#[repr(C)]
#[derive(Clone, PartialEq, Debug)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32
}

bitflags! {
    #[repr(C)]
    flags DrawFlags: u32 {
        const DRAW_SHAPE = 0x0001,
        const DRAW_JOINT = 0x0002,
        const DRAW_AABB = 0x0004,
        const DRAW_PAIR = 0x0008,
        const DRAW_CENTER_OF_MASS = 0x0010
    }
}

pub trait Draw {
    fn draw_polygon(&mut self, vertices: &[Vec2], color: &Color);
    fn draw_solid_polygon(&mut self, vertices: &[Vec2], color: &Color);
    fn draw_circle(&mut self, center: &Vec2, radius: f32, color: &Color);
    fn draw_solid_circle(&mut self, center: &Vec2, radius: f32, axis: &Vec2, color: &Color);
    fn draw_segment(&mut self, p1: &Vec2, p2: &Vec2, color: &Color);
    fn draw_transform(&mut self, xf: &Transform);
}
