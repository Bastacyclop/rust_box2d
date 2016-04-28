//! Bod2D for Rust
//!
//! You won't find a lot of information about Box2D itself here,
//! look at [the official website](http://box2d.org/) instead.
//!
//! # World
//!
//! ````
//! use wrapped2d::b2;
//!
//! let gravity = b2::Vec2 { x: 0., y: -10. };
//! let world = b2::World::new(&gravity);
//! ````
//!
//! # Handles
//!
//! Bodies, fixtures and joints are accessed through handles and
//! their borrowing is dynamically checked by `RefCell`s.
//!
//! ````
//! # use wrapped2d::b2;
//! # let gravity = b2::Vec2 { x: 0., y: -10. };
//! # let mut world = b2::World::new(&gravity);
//! let mut def = b2::BodyDef::new();
//! def.body_type = b2::BodyType::Dynamic;
//! def.position = b2::Vec2 { x: 10., y: 10. };
//!
//! let handle = world.create_body(&def);
//! let mut body = world.get_body_mut(handle);
//!
//! let mut shape = b2::PolygonShape::new();
//! shape.set_as_box(0.5, 0.5);
//!
//! let handle = body.create_fast_fixture(&shape, 2.);
//! let fixture = body.get_fixture(handle);
//! ````

#![feature(associated_consts)]

#[link(name = "Box2D")]
extern "C" {
}
#[link(name = "stdc++")]
extern "C" {
}

extern crate libc;
extern crate vec_map;
#[macro_use]
extern crate bitflags;
#[cfg(feature = "nalgebra")]
extern crate nalgebra;
#[cfg(feature = "cgmath")]
extern crate cgmath;

mod ffi;
#[doc(hidden)] #[macro_use]pub mod wrap;
#[doc(hidden)]pub mod handle;

pub mod common;
pub mod collision;
pub mod dynamics;

pub mod b2 {
    pub use common::{Color, DrawFlags, Draw, DRAW_AABB, DRAW_CENTER_OF_MASS, DRAW_JOINT,
                     DRAW_PAIR, DRAW_SHAPE};
    pub use common::math::{Rot, Sweep, Transform, Vec2};
    pub use common::math::{cross_vv, cross_vs, cross_sv};
    pub use common::settings::{ANGULAR_SLOP, LINEAR_SLOP, MAX_MANIFOLD_POINTS,
                               MAX_POLYGON_VERTICES, PI, POLYGON_RADIUS};
    pub use collision::{AABB, ContactFeature, ContactId, Manifold, ManifoldPoint, WorldManifold,
                        RayCastInput, RayCastOutput, ContactFeatureType, ManifoldType, PointState,
                        get_point_states, test_overlap, distance, time_of_impact};
    pub use collision::shapes::{MassData, ShapeType, UnknownShape, Shape, ChainShape, CircleShape,
                                EdgeShape, PolygonShape};
    pub use dynamics::Profile;
    pub use dynamics::world::{World, BodyHandle, JointHandle};
    pub use dynamics::world::callbacks::{ContactImpulse, ContactFilter, ContactListener,
                                         QueryCallback, RayCastCallback};
    pub use dynamics::body::{Body, BodyDef, MetaBody, BodyType, FixtureHandle};
    pub use dynamics::fixture::{Filter, Fixture, FixtureDef, MetaFixture};
    pub use dynamics::joints::{DistanceJoint, DistanceJointDef, FrictionJoint, FrictionJointDef,
                               GearJoint, GearJointDef, JointEdge, MetaJoint, MotorJoint,
                               MotorJointDef, MouseJoint, MouseJointDef, PrismaticJoint,
                               PrismaticJointDef, PulleyJoint, PulleyJointDef, RevoluteJoint,
                               RevoluteJointDef, RopeJoint, RopeJointDef, WeldJoint, WeldJointDef,
                               WheelJoint, WheelJointDef, JointType, LimitState, UnknownJoint,
                               Joint, JointDef};
}
