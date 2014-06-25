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

u32 SIZEOF_BODY = sizeof(b2Body);

#include "c_box2d/dynamics/body.cpp"
#include "c_box2d/dynamics/fixture.cpp"
#include "c_box2d/dynamics/world.cpp"

#include "c_box2d/collision/shapes/shape.cpp"
#include "c_box2d/collision/shapes/chain_shape.cpp"
#include "c_box2d/collision/shapes/circle_shape.cpp"
#include "c_box2d/collision/shapes/edge_shape.cpp"
#include "c_box2d/collision/shapes/polygon_shape.cpp"

#include "c_box2d/dynamics/joints/joint.cpp"
#include "c_box2d/dynamics/joints/distance_joint.cpp"
#include "c_box2d/dynamics/joints/friction_joint.cpp"
#include "c_box2d/dynamics/joints/gear_joint.cpp"
#include "c_box2d/dynamics/joints/motor_joint.cpp"
#include "c_box2d/dynamics/joints/mouse_joint.cpp"
#include "c_box2d/dynamics/joints/prismatic_joint.cpp"
#include "c_box2d/dynamics/joints/pulley_joint.cpp"
#include "c_box2d/dynamics/joints/revolute_joint.cpp"
#include "c_box2d/dynamics/joints/rope_joint.cpp"
#include "c_box2d/dynamics/joints/weld_joint.cpp"
#include "c_box2d/dynamics/joints/wheel_joint.cpp"

}
