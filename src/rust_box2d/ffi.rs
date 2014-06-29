use math::Vec2;
use math::Transform;

use dynamics::{
    BodyDef, BodyType,
    JointDef, JointType,
    DistanceJointDef, FrictionJointDef,
    GearJointDef, MotorJointDef,
    MouseJointDef, PrismaticJointDef,
    PulleyJointDef, RevoluteJointDef,
    RopeJointDef, WeldJointDef, WheelJointDef  
};
use collision::{
    ShapeType, MassData
};

pub struct World;
pub struct DestructionListener;
pub struct ContactFilter;
pub struct ContactListener;
pub struct Draw;
pub struct Body;
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
pub struct PolygonShape;
pub struct Joint;
pub struct DistanceJoint;
pub struct FrictionJoint;
pub struct GearJoint;
pub struct MotorJoint;
pub struct MouseJoint;
pub struct PrismaticJoint;
pub struct PulleyJoint;
pub struct RevoluteJoint;
pub struct RopeJoint;
pub struct WeldJoint;
pub struct WheelJoint;

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
    pub fn World_create_body(slf: *mut World, def: *BodyDef) -> *mut Body;
    pub fn World_destroy_body(slf: *mut World, body: *mut Body);
    pub fn World_create_joint(slf: *mut World, def: *JointDef) -> *mut Joint;
    pub fn World_destroy_joint(slf: *mut World, joint: *mut Joint);
    pub fn World_step(slf: *mut World,
                      time_step: f32,
                      velocity_iterations: i32,
                      position_iterations: i32);
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
    
    //pub fn BodyDef_default() -> BodyDef;
    
    pub fn Shape_drop_virtual(slf: *mut Shape);
    pub fn Shape_clone_virtual(slf: *Shape,
                               alloc: *mut BlockAllocator) -> *mut Shape;
    pub fn Shape_get_type(slf: *Shape) -> ShapeType;
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
                                      data: *mut MassData,
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
    
    pub fn PolygonShape_new() -> *mut PolygonShape;
    pub fn PolygonShape_drop(slf: *mut PolygonShape);
    pub fn PolygonShape_as_shape(slf: *mut PolygonShape) -> *mut Shape;
    pub fn Shape_as_polygon_shape(slf: *mut Shape) -> *mut PolygonShape;
    pub fn PolygonShape_set(slf: *mut PolygonShape, points: *Vec2, count: i32);
    pub fn PolygonShape_set_as_box(slf: *mut PolygonShape, hw: f32, hh: f32);
    pub fn PolygonShape_set_as_oriented_box(slf: *mut PolygonShape,
                                            hw: f32, hh: f32,
                                            center: *Vec2,
                                            angle: f32);
    pub fn PolygonShape_get_vertex_count(slf: *PolygonShape) -> i32;
    pub fn PolygonShape_get_vertex(slf: *PolygonShape, index: i32) -> *Vec2;
    pub fn PolygonShape_validate(slf: *PolygonShape) -> bool;
    
    pub fn JointDef_default() -> JointDef;
    pub fn Joint_get_type(slf: *Joint) -> JointType;
    pub fn Joint_get_body_a(slf: *mut Joint) -> *mut Body;
    pub fn Joint_get_body_b(slf: *mut Joint) -> *mut Body;
    pub fn Joint_get_anchor_a_virtual(slf: *Joint) -> Vec2;
    pub fn Joint_get_anchor_b_virtual(slf: *Joint) -> Vec2;
    pub fn Joint_get_reaction_force_virtual(slf: *Joint) -> Vec2;
    pub fn Joint_get_reaction_torque_virtual(slf: *Joint) -> Vec2;
    pub fn Joint_get_next(slf: *mut Joint) -> *mut Joint;
    pub fn Joint_get_next_const(slf: *Joint) -> *Joint;
    pub fn Joint_get_user_data(slf: *Joint) -> ffi::UserData;
    pub fn Joint_set_user_data(slf: *mut Joint, data: ffi::UserData);
    pub fn Joint_is_active(slf: *Joint) -> bool;
    pub fn Joint_dump_virtual(slf: *mut Joint);
    pub fn Joint_shift_origin_virtual(slf: *mut Joint, origin: *Vec2);
    
    pub fn DistanceJointDef_default() -> DistanceJointDef;
    pub fn DistanceJointDef_initialize(slf: *mut DistanceJointDef,
                                       body_a: *mut Body,
                                       body_b: *mut Body,
                                       anchor_a: *Vec2,
                                       anchor_b: *Vec2);
    pub fn DistanceJoint_as_joint(slf: *mut DistanceJoint) -> *mut Joint;
    pub fn Joint_as_distance_joint(slf: *mut Joint) -> *mut DistanceJoint;
    pub fn DistanceJoint_get_local_anchor_a(slf: *DistanceJoint) -> *Vec2;
    pub fn DistanceJoint_get_local_anchor_b(slf: *DistanceJoint) -> *Vec2;
    pub fn DistanceJoint_set_length(slf: *mut DistanceJoint, length: f32);
    pub fn DistanceJoint_get_length(slf: *DistanceJoint) -> f32;
    pub fn DistanceJoint_set_frequency(slf: *mut DistanceJoint, hz: f32);
    pub fn DistanceJoint_get_frequency(slf: *DistanceJoint) -> f32;
    pub fn DistanceJoint_set_damping_ratio(slf: *mut DistanceJoint, ratio: f32);
    pub fn DistanceJoint_get_damping_ratio(slf: *DistanceJoint) -> f32;
    
    pub fn FrictionJointDef_default() -> FrictionJointDef;
    pub fn FrictionJointDef_initialize(slf: *mut FrictionJointDef,
                                       body_a: *mut Body,
                                       body_b: *mut Body,
                                       anchor: *Vec2);
    pub fn FrictionJoint_as_joint(slf: *mut FrictionJoint) -> *mut Joint;
    pub fn Joint_as_friction_joint(slf: *mut Joint) -> *mut FrictionJoint;
    pub fn FrictionJoint_get_local_anchor_a(slf: *FrictionJoint) -> *Vec2;
    pub fn FrictionJoint_get_local_anchor_b(slf: *FrictionJoint) -> *Vec2;
    pub fn FrictionJoint_set_max_force(slf: *mut FrictionJoint, force: f32);
    pub fn FrictionJoint_get_max_force(slf: *FrictionJoint) -> f32;
    pub fn FrictionJoint_set_max_torque(slf: *mut FrictionJoint, torque: f32);
    pub fn FrictionJoint_get_max_torque(slf: *FrictionJoint) -> f32;
    
    pub fn GearJointDef_default() -> GearJointDef;
    pub fn GearJoint_as_joint(slf: *mut GearJoint) -> *mut Joint;
    pub fn Joint_as_gear_joint(slf: *mut Joint) -> *mut GearJoint;
    pub fn GearJoint_get_joint_1(slf: *mut GearJoint) -> *Joint;
    pub fn GearJoint_get_joint_2(slf: *mut GearJoint) -> *Joint;
    pub fn GearJoint_set_ratio(slf: *mut GearJoint, ratio: f32);
    pub fn GearJoint_get_ratio(slf: *GearJoint) -> f32;
    
    pub fn MotorJointDef_default() -> MotorJointDef;
    pub fn MotorJointDef_initialize(slf: *mut MotorJointDef,
                                    body_a: *mut Body,
                                    body_b: *mut Body);
    pub fn MotorJoint_as_joint(slf: *mut MotorJoint) -> *mut Joint;
    pub fn Joint_as_motor_joint(slf: *mut Joint) -> *mut MotorJoint;
    pub fn MotorJoint_set_linear_offset(slf: *mut MotorJoint, offset: *Vec2);
    pub fn MotorJoint_get_linear_offset(slf: *MotorJoint) -> *Vec2;
    pub fn MotorJoint_set_angular_offset(slf: *mut MotorJoint, offset: f32);
    pub fn MotorJoint_get_angular_offset(slf: *MotorJoint) -> f32;
    pub fn MotorJoint_set_max_force(slf: *mut MotorJoint, force: f32);
    pub fn MotorJoint_get_max_force(slf: *MotorJoint) -> f32;
    pub fn MotorJoint_set_max_torque(slf: *mut MotorJoint, torque: f32);
    pub fn MotorJoint_get_max_torque(slf: *MotorJoint) -> f32;
    pub fn MotorJoint_set_correction_factor(slf: *mut MotorJoint, factor: f32);
    pub fn MotorJoint_get_correction_factor(slf: *Motorjoint) -> f32;
    
    pub fn MouseJointDef_default() -> MouseJointDef;
    pub fn MouseJoint_as_joint(slf: *mut MouseJoint) -> *mut Joint;
    pub fn Joint_as_mouse_joint(slf: *mut Joint) -> *mut MouseJoint;
    pub fn MouseJoint_set_target(slf: *mut MouseJoint, target: *Vec2);
    pub fn MouseJoint_get_target(slf: *MouseJoint) -> *Vec2;
    pub fn MouseJoint_set_max_force(slf: *mut MouseJoint, force: f32);
    pub fn MouseJoint_get_max_force(slf: *MouseJoint) -> f32;
    pub fn MouseJoint_set_frequency(slf: *mut MouseJoint, hz: f32);
    pub fn MouseJoint_get_frequency(slf: *MouseJoint) -> f32;
    pub fn MouseJoint_set_damping_ratio(slf: *mut MouseJoint, ratio: f32);
    pub fn MouseJoint_get_damping_ratio(slf: *MouseJoint) -> f32; 
    
    pub fn PrismaticJointDef_default() -> JointDef;
    pub fn PrismaticJointDef_initialize(slf: *mut JointDef,
                                        body_a: *mut Body,
                                        body_b: *mut Body,
                                        anchor: *Vec2,
                                        axis: *Vec2);
    pub fn PrismaticJoint_as_joint(slf: *mut PrismaticJoint) -> *mut Joint;
    pub fn Joint_as_prismatic_joint(slf: *mut Joint) -> *mut PrismaticJoint;
    pub fn PrismaticJoint_get_local_anchor_a(slf: *PrismaticJoint) -> *Vec2;
    pub fn PrismaticJoint_get_local_anchor_b(slf: *PrismaticJoint) -> *Vec2;
    pub fn PrismaticJoint_get_local_axis_a(slf: *PrismaticJoint) -> *Vec2;
    pub fn PrismaticJoint_get_reference_angle(slf: *PrismaticJoint) -> f32;
    pub fn PrismaticJoint_get_joint_translation(slf: *PrismaticJoint) -> f32;
    pub fn PrismaticJoint_get_joint_speed(slf: *PrismaticJoint) -> f32;
    pub fn PrismaticJoint_is_limit_enabled(slf: *PrismaticJoint) -> bool;
    pub fn PrismaticJoint_enable_limit(slf: *mut PrismaticJoint, flag: bool);
    pub fn PrismaticJoint_get_lower_limit(slf: *PrismaticJoint) -> f32;
    pub fn PrismaticJoint_get_upper_limit(slf: *PrismaticJoint) -> f32;
    pub fn PrismaticJoint_set_limits(slf: *mut PrismaticJoint,
                                     lower: f32, upper: f32);
    pub fn PrismaticJoint_is_motor_enabled(slf: *PrismaticJoint) -> bool;
    pub fn PrismaticJoint_enable_motor(slf: *mut PrismaticJoint, flag: bool);
    pub fn PrismaticJoint_set_motor_speed(slf: *mut PrismaticJoint, speed: f32);
    pub fn PrismaticJoint_get_motor_speed(slf: *PrismaticJoint) -> f32;
    pub fn PrismaticJoint_set_max_motor_force(slf: *mut PrismaticJoint,
                                              force: f32);
    pub fn PrismaticJoint_get_max_motor_force(slf: *PrismaticJoint) -> f32;
    pub fn PrismaticJoint_get_motor_force(slf: *PrismaticJoint,
                                          inv_dt: f32) -> f32;
                                         
    pub fn PulleyJointDef_default() -> PulleyJointDef;
    pub fn PulleyJointDef_initialize(slf: *mut PulleyJointDef,
                                     body_a: *mut Body,
                                     body_b: *mut Body,
                                     ground_anchor_a: *Vec2,
                                     ground_anchor_b: *Vec2,
                                     anchor_a: *Vec2,
                                     anchor_b: *Vec2,
                                     ratio: f32);
    pub fn PulleyJoint_as_joint(slf: *mut PulleyJoint) -> *mut Joint;
    pub fn Joint_as_pulley_joint(slf: *mut Joint) -> *mut PulleyJoint;
    pub fn PulleyJoint_get_ground_anchor_a(slf: *PulleyJoint) -> Vec2;
    pub fn PulleyJoint_get_ground_anchor_b(slf: *PulleyJoint) -> Vec2;
    pub fn PulleyJoint_get_length_a(slf: *PulleyJoint) -> f32;
    pub fn PulleyJoint_get_length_b(slf: *PulleyJoint) -> f32;
    pub fn PulleyJoint_get_ratio(slf: *PulleyJoint) -> f32;
    pub fn PulleyJoint_get_current_length_a(slf: *PulleyJoint) -> f32;
    pub fn PulleyJoint_get_current_length_b(slf: *PulleyJoint) -> f32;
    
    pub fn RevoluteJointDef_default() -> RevoluteJointDef;
    pub fn RevoluteJointDef_initialize(slf: *mut RevoluteJointDef,
                                       body_a: *mut Body,
                                       body_b: *mut Body,
                                       anchor: *Vec2);
    pub fn RevoluteJoint_as_joint(slf: *mut RevoluteJoint) -> *mut Joint;
    pub fn Joint_as_revolute_joint(slf: *mut Joint) -> *mut RevoluteJoint;
    pub fn RevoluteJoint_get_local_anchor_a(slf: *RevoluteJoint) -> *Vec2;
    pub fn RevoluteJoint_get_local_anchor_b(slf: *RevoluteJoint) -> *Vec2;
    pub fn RevoluteJoint_get_reference_angle(slf: *RevoluteJoint) -> f32;
    pub fn RevoluteJoint_get_joint_angle(slf: *RevoluteJoint) -> f32;
    pub fn RevoluteJoint_get_joint_speed(slf: *RevoluteJoint) -> f32;
    pub fn RevoluteJoint_is_limit_enabled(slf: *RevoluteJoint) -> bool;
    pub fn RevoluteJoint_enable_limit(slf: *mut RevoluteJoint, flag: bool);
    pub fn RevoluteJoint_get_lower_limit(slf: *RevoluteJoint) -> f32;
    pub fn RevoluteJoint_get_upper_limit(slf: *RevoluteJoint) -> f32;
    pub fn RevoluteJoint_set_limits(slf: *mut RevoluteJoint,
                                    lower: f32, upper: f32);
    pub fn RevoluteJoint_is_motor_enabled(slf: *RevoluteJoint) -> bool;
    pub fn RevoluteJoint_enable_motor(slf: *mut RevoluteJoint, flag: bool);
    pub fn RevoluteJoint_set_motor_speed(slf: *mut RevoluteJoint, speed: f32);
    pub fn RevoluteJoint_get_motor_speed(slf: *RevoluteJoint) -> f32;
    pub fn RevoluteJoint_set_max_motor_torque(slf: *mut RevoluteJoint,
                                              torque: f32);
    pub fn RevoluteJoint_get_max_motor_torque(slf: *RevoluteJoint) -> f32;
    pub fn RevoluteJoint_get_motor_torque(slf: *RevoluteJoint) -> f32;
    
    pub fn RopeJointDef_default() -> RopeJointDef;
    pub fn RopeJoint_as_joint(slf: *mut RopeJoint) -> *mut Joint;
    pub fn Joint_as_rope_joint(slf: *mut Joint) -> *mut RopeJoint;
    pub fn RopeJoint_get_local_anchor_a(slf: *RopeJoint) -> *Vec2;
    pub fn RopeJoint_get_local_anchor_b(slf: *RopeJoint) -> *Vec2;
    pub fn RopeJoint_set_max_length(slf: *mut RopeJoint, length: f32);
    pub fn RopeJoint_get_max_length(slf: *RopeJoint) -> f32;
    pub fn RopeJoint_get_limit_state(slf: *RopeJoint) -> LimitState;
    
    pub fn WeldJointDef_default() -> WeldJointDef;
    pub fn WeldJointDef_initialize(slf: *mut WeldJointDef,
                                   body_a: *mut Body,
                                   body_b: *mut Body,
                                   anchor: *Vec2);
    pub fn WeldJoint_as_joint(slf: *mut WeldJoint) -> *mut Joint;
    pub fn Joint_as_weld_joint(slf: *mut Joint) -> *mut WeldJoint;
    pub fn WeldJoint_get_local_anchor_a(slf: *WeldJoint) -> *Vec2;
    pub fn WeldJoint_get_local_anchor_b(slf: *WeldJoint) -> *Vec2;
    pub fn WeldJoint_get_reference_angle(slf: *WeldJoint) -> f32;
    pub fn WeldJoint_set_frequency(slf: *mut WeldJoint, frequency: f32);
    pub fn WeldJoint_get_frequency(slf: *WeldJoint) -> f32;
    pub fn WeldJoint_set_damping_ratio(slf: *mut WeldJoint, ratio: f32);
    pub fn WeldJoint_get_damping_ratio(slf: *WeldJoint) -> f32;

    pub fn WheelJointDef_default() -> WheelJointDef;
    pub fn WheelJointDef_initialize(slf: *mut WheelJointDef,
                                    body_a: *mut Body,
                                    body_b: *mut Body,
                                    anchor: *Vec2,
                                    axis: *Vec2);
    pub fn WheelJoint_as_joint(slf: *mut WheelJoint) -> *mut Joint;
    pub fn Joint_as_wheel_joint(slf: *mut Joint) -> *mut WheelJoint;
    pub fn WheelJoint_get_local_anchor_a(slf: *WheelJoint) -> *Vec2;
    pub fn WheelJoint_get_local_anchor_b(slf: *WheelJoint) -> *Vec2;
    pub fn WheelJoint_get_local_axis_a(slf: *WheelJoint) -> *Vec2;
    pub fn WheelJoint_get_joint_translation(slf: *WheelJoint) -> f32;
    pub fn WheelJoint_get_joint_speed(slf: *WheelJoint) -> f32;
    pub fn WheelJoint_is_motor_enabled(slf: *WheelJoint) -> bool;
    pub fn WheelJoint_enable_motor(slf: *mut WheelJoint, flag: bool);
    pub fn WheelJoint_set_motor_speed(slf: *mut WheelJoint, speed: f32);
    pub fn WheelJoint_get_motor_speed(slf: *WheelJoint) -> f32;
    pub fn WheelJoint_set_max_motor_torque(slf: *mut WheelJoint, torque: f32);
    pub fn WheelJoint_get_max_motor_torque(slf: *WheelJoint) -> f32;
    pub fn WheelJoint_get_motor_torque(slf: *WheelJoint) -> f32;
    pub fn WheelJoint_set_spring_frequency(slf: *mut WheelJoint, frequency: f32);
    pub fn WheelJoint_get_spring_frequency(slf: *WheelJoint) -> f32;
    pub fn WheelJoint_set_damping_ratio(slf: *mut WheelJoint, ratio: f32);
    pub fn WheelJoint_get_damping_ratio(slf: *WheelJoint) -> f32;
}
