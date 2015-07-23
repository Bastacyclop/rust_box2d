pub mod world;
pub mod body;
pub mod fixture;
pub mod joints;
pub mod contacts;
pub mod user_data;

#[repr(C)]
#[derive(Clone)]
pub struct Profile {
    pub step: f32,
    pub collide: f32,
    pub solve: f32,
    pub solve_init: f32,
    pub solve_velocity: f32,
    pub solve_position: f32,
    pub broad_phase: f32,
    pub solve_toi: f32
}
