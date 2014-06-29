use ffi;
use math::Vec2;
use Wrapped;

pub enum Type {
    Static,
    Kinematic,
    Dynamic,
}

#[allow(dead_code)]
pub struct Def {
    pub body_type: Type,
    pub position: Vec2,
    pub angle: f32,
    pub linear_velocity: Vec2,
    pub angular_velocity: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub allow_sleep: bool,
    pub awake: bool,
    pub fixed_rotation: bool,
    pub bullet: bool,
    pub active: bool,
    user_data: ffi::UserData,
    pub gravity_scale: f32,
}

wrap!(ffi::Body into Body)

impl Drop for Body {
    fn drop(&mut self) {
    }
}
