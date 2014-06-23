use math::Vec2;

use dynamics::body;
use dynamics::joint;

pub struct World;
pub struct DestructionListener;
pub struct ContactFilter;
pub struct ContactListener;
pub struct Draw;
pub struct Body;
pub struct Joint;
pub struct QueryCallback;
pub struct AABB;
pub struct RayCastCallback;
pub struct Contact;
pub struct ContactManager;
pub struct Profile;

pub type UserData = *i32;

#[link(name = "c_box2d", kind = "static")]
extern {
    pub static SIZEOF_BODY: u32;
    
    pub fn World_new(gravity: *Vec2) -> *mut World;
    pub fn World_drop(slf: *mut World);
    pub fn World_set_destruction_listener(slf: *mut World,
                                          dl: *mut DestructionListener);
    pub fn World_set_contact_filter(slf: *mut World, cf: *mut ContactFilter);
    pub fn World_set_contact_listener(slf: *mut World, cl: *mut ContactListener);
    pub fn World_set_debug_draw(slf: *mut World, dd: *mut Draw);
    pub fn World_create_body(slf: *mut World, def: *body::Def) -> *mut Body;
    pub fn World_destroy_body(slf: *mut World, body: *mut Body);
    pub fn World_create_joint(slf: *mut World, def: *joint::Def) -> *mut Joint;
    pub fn World_destroy_joint(slf: *mut World, joint: *mut Joint);
    pub fn World_step(slf: *mut World,
                      time_step: f32,
                      velocity_iterations: f32,
                      position_iterations: f32);
    pub fn World_clear_forces(slf: *mut World);
    pub fn World_draw_debug_data(slf: *mut World);
    pub fn World_query_aabb(slf: *World, qc: *mut QueryCallback, aabb: *AABB);
    pub fn World_ray_cast(slf: *World, rccb: *RayCastCallback, p1: *Vec2, p2: *Vec2);
    pub fn World_get_body_list(slf: *mut World) -> *mut Body;
    pub fn World_get_body_list_const(slf: *World) -> *Body;
    pub fn World_get_joint_list(slf: *mut World) -> *mut Joint;
    pub fn World_get_joint_list_const(slf: *World) -> *Joint;
    pub fn World_get_contact_list(slf: *mut World) -> *mut Contact;
    pub fn World_get_contact_list_const(slf: *World) -> *Contact;
    pub fn World_set_allow_sleeping(slf: *mut World, flag: bool);
    pub fn World_get_allow_sleeping(slf: *World) -> bool;
    pub fn World_set_warm_starting(slf: *mut World, flag: bool);
    pub fn World_get_warm_starting(slf: *World) -> bool;
    pub fn World_set_continuous_physics(slf: *mut World, flag: bool);
    pub fn World_get_continuous_physics(slf: *World) -> bool;
    pub fn World_set_sub_stepping(slf: *mut World, flag: bool);
    pub fn World_get_sub_stepping(slf: *World) -> bool;
    pub fn World_get_proxy_count(slf: *World) -> i32;
    pub fn World_get_body_count(slf: *World) -> i32;
    pub fn World_get_joint_count(slf: *World) -> i32;
    pub fn World_get_contact_count(slf: *World) -> i32;
    pub fn World_get_tree_height(slf: *World) -> i32;
    pub fn World_get_tree_balance(slf: *World) -> i32;
    pub fn World_get_tree_quality(slf: *World)-> f32;
    pub fn World_set_gravity(slf: *mut World, gravity: *Vec2);
    pub fn World_get_gravity(slf: *World) -> Vec2;
    pub fn World_is_locked(slf: *World) -> bool;
    pub fn World_set_auto_clear_forces(slf: *mut World, flag: bool);
    pub fn World_get_auto_clear_forces(slf: *World) -> bool;
    pub fn World_shift_origin(slf: *mut World, origin: *Vec2);
    pub fn World_get_contact_manager(slf: *World) -> *ContactManager;
    pub fn World_get_profile(slf: *World) -> *Profile;
    pub fn World_dump(slf: *mut World);
}
