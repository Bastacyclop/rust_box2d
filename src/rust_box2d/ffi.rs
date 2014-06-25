use math::Vec2;
use math::Transform;

use dynamics::body;
use dynamics::joint;
use collision::shape;

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
pub struct Shape;
pub struct RayCastInput;
pub struct RayCastOutput;
pub struct BlockAllocator;
pub struct ChainShape;
pub struct EdgeShape;
pub struct CircleShape;

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
    
    pub fn Shape_drop_virtual(slf: *mut Shape);
    pub fn Shape_clone_cirtual(slf: *Shape,
                               alloc: *mut BlockAllocator) -> *mut Shape;
    pub fn Shape_get_type(slf: *Shape) -> shape::Type;
    pub fn Shape_get_child_count_virtual(slf: *Shape) -> i32;
    pub fn Shape_test_point_virtual(slf: *Shape,
                                    xf: *Transform,
                                    p: *Vec2) -> bool;
    pub fn Shape_ray_cast_virtual(slf: *Shape,
                                  output: *mut RayCastOutput,
                                  input: *RayCastInput,
                                  transform: *Transform) -> bool;
    pub fn Shape_compute_aabb_virtual(slf: *Shape,
                                      aabb: *mut AABB,
                                      xf: *Transform,
                                      child_id: i32);
    pub fn Shape_compute_mass_virtual(slf: *Shape,
                                      data: *mut shape::MassData,
                                      density: f32);
                                      
    pub fn ChainShape_new() -> *mut ChainShape;
    pub fn ChainShape_drop(slf: *mut ChainShape);
    pub fn ChainShape_as_shape(slf: *mut ChainShape) -> *mut Shape;
    pub fn Shape_as_chain_shape(slf: *mut Shape) -> *mut ChainShape;
    pub fn ChainShape_clear(slf: *mut ChainShape);
    pub fn ChainShape_create_loop(slf: *mut ChainShape,
                                  vertices: *Vec2,
                                  count: i32);
    pub fn ChainShape_create_chain(slf: *mut ChainShape,
                                   vertices: *Vec2,
                                   count: i32);
    pub fn ChainShape_set_prev_vertex(slf: *mut ChainShape, vertex: *Vec2);
    pub fn ChainShape_set_next_vertex(slf: *mut ChainShape, vertex: *Vec2);
    pub fn ChainShape_get_child_edge(slf: *ChainShape,
                                     edge: *mut EdgeShape,
                                     index: i32);
                                     
    pub fn EdgeShape_new() -> *mut EdgeShape;
    pub fn EdgeShape_drop(slf: *mut EdgeShape);
    pub fn EdgeShape_as_shape(slf: *mut EdgeShape) -> *mut Shape;
    pub fn Shape_as_edge_shape(slf: *mut Shape) -> *mut EdgeShape;
    pub fn EdgeShape_set(slf: *mut EdgeShape, v1: *Vec2, v2: *Vec2);

    pub fn CircleShape_new() -> *mut CircleShape;
    pub fn CircleShape_drop(slf: *mut CircleShape);
    pub fn CircleShape_as_shape(slf: *mut CircleShape) -> *mut Shape;
    pub fn Shape_as_circle_shape(slf: *mut Shape) -> *mut CircleShape;
    pub fn CircleShape_get_support(slf: *CircleShape, d: *Vec2) -> i32;
    pub fn CircleShape_get_support_vertex(slf: *CircleShape,
                                          d: *Vec2) -> *Vec2;
    pub fn CircleShape_get_vertex_count(slf: *CircleShape) -> i32;
    pub fn CircleShape_get_vertex(slf: *CircleShape, index: i32) -> *Vec2;
}
