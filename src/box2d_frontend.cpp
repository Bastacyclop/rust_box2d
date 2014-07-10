#include <Box2D/Box2D.h>

extern "C" {

typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef float f32;
typedef double f64;
typedef i32 RustObject;

#include "dynamics/body.cpp"
#include "dynamics/fixture.cpp"
#include "dynamics/world.cpp"
#include "dynamics/world_callbacks.cpp"
#include "dynamics/draw.cpp"

#include "collision/shapes/shape.cpp"
#include "collision/shapes/chain_shape.cpp"
#include "collision/shapes/circle_shape.cpp"
#include "collision/shapes/edge_shape.cpp"
#include "collision/shapes/polygon_shape.cpp"

#include "dynamics/joints/joint.cpp"
#include "dynamics/joints/distance_joint.cpp"
#include "dynamics/joints/friction_joint.cpp"
#include "dynamics/joints/gear_joint.cpp"
#include "dynamics/joints/motor_joint.cpp"
#include "dynamics/joints/mouse_joint.cpp"
#include "dynamics/joints/prismatic_joint.cpp"
#include "dynamics/joints/pulley_joint.cpp"
#include "dynamics/joints/revolute_joint.cpp"
#include "dynamics/joints/rope_joint.cpp"
#include "dynamics/joints/weld_joint.cpp"
#include "dynamics/joints/wheel_joint.cpp"

}
