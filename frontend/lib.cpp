#include <Box2D/Box2D.h>
#include <stdint.h>

extern "C" {

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef float f32;
typedef double f64;
typedef void* RustObject;
struct RustFatObject {
    void* raw1;
    void* raw2;
};

#include "common/draw.cpp"

#include "dynamics/body.cpp"
#include "dynamics/fixture.cpp"
#include "dynamics/world.cpp"
#include "dynamics/world_callbacks.cpp"

#include "collision/collision.cpp"
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
