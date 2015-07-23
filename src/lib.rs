#![feature(associated_consts, ptr_as_ref)]

#[link(name = "Box2D")] extern {}
#[link(name = "stdc++")] extern {}

extern crate libc;
extern crate vec_map;
#[macro_use] extern crate bitflags;

mod ffi;
#[doc(hidden)] #[macro_use] pub mod wrap;
#[doc(hidden)] pub mod handle;

pub mod common;
pub mod collision;
pub mod dynamics;

pub mod b2 {
    pub use common::{
        Color, DrawFlags, Draw,
        DRAW_AABB, DRAW_CENTER_OF_MASS, DRAW_JOINT, DRAW_PAIR, DRAW_SHAPE
    };
    pub use common::math::{ Rot, Sweep, Transform, Vec2 };
    pub use common::settings::{
        ANGULAR_SLOP, LINEAR_SLOP,
        MAX_MANIFOLD_POINTS, MAX_POLYGON_VERTICES,
        PI, POLYGON_RADIUS
    };
    pub use collision::{
        AABB, ContactFeature, ContactId, Manifold, ManifoldPoint, WorldManifold,
        RayCastInput, RayCastOutput, ContactFeatureType, ManifoldType, PointState,
        get_point_states, test_overlap, distance, time_of_impact
    };
    pub use collision::shapes::{
        MassData, ShapeType, UnknownShape, Shape,
        ChainShape, CircleShape, EdgeShape, PolygonShape
    };
    pub use dynamics::Profile;
    pub use dynamics::world::{ World, BodyHandle, JointHandle };
    pub use dynamics::world::callbacks::{
        ContactImpulse,
        ContactFilter, ContactListener, QueryCallback, RayCastCallback
    };
    pub use dynamics::body::{ Body, BodyDef, MetaBody, BodyType, FixtureHandle };
    pub use dynamics::fixture::{ Filter, Fixture, FixtureDef, MetaFixture };
    pub use dynamics::joints::{
        DistanceJoint, DistanceJointDef, FrictionJoint, FrictionJointDef,
        GearJoint, GearJointDef, JointDefBase, JointEdge, MetaJoint,
        MotorJoint, MotorJointDef, MouseJoint, MouseJointDef,
        PrismaticJoint, PrismaticJointDef, PulleyJoint, PulleyJointDef,
        RevoluteJoint, RevoluteJointDef, RopeJoint, RopeJointDef,
        WeldJoint, WeldJointDef, WheelJoint, WheelJointDef,
        JointType, LimitState, UnknownJoint, Joint, JointDef
    };
}
