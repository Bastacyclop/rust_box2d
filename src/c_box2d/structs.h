#ifndef C_BOX2D_STRUCTS_H
#define C_BOX2D_STRUCTS_H

#define def_struct(S) typedef struct S S

#ifdef __cplusplus
extern "C" {
#endif

typedef signed char i8;
typedef signed short i16;
typedef signed int i32;
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef float f32;
typedef double f64;
#ifndef __cplusplus
    typedef enum { false, true } bool;
#endif

def_struct(b2Vec2);
def_struct(b2Transform);

def_struct(b2Body);
def_struct(b2BodyDef);
def_struct(b2Fixture);
def_struct(b2FixtureDef);
def_struct(b2World);

def_struct(b2Joint);
def_struct(b2JointDef);
def_struct(b2DistanceJoint);
def_struct(b2FrictionJoint);
def_struct(b2GearJoint);
def_struct(b2MotorJoint);
def_struct(b2MouseJoint);
def_struct(b2PrismaticJoint);
def_struct(b2PulleyJoint);
def_struct(b2RevoluteJoint);
def_struct(b2RopeJoint);
def_struct(b2WeldJoint);
def_struct(b2WheelJoint);

def_struct(b2Shape);
def_struct(b2ChainShape);
def_struct(b2CircleShape);
def_struct(b2EdgeShape);
def_struct(b2PolygonShape);

def_struct(b2Contact);
def_struct(b2ContactEdge);
def_struct(b2JointEdge);
def_struct(b2Controller);

#ifdef __cplusplus
} // extern C
#endif

#endif
