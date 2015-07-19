#![feature(associated_consts, ptr_as_ref, log_syntax, trace_macros)]

#[link(name = "Box2D")] extern {}
#[link(name = "stdc++")] extern {}

extern crate libc;
extern crate vec_map;
#[macro_use] extern crate bitflags;

mod ffi;
#[macro_use] pub mod wrap;
pub mod handle;

pub mod collision;
pub mod common;
pub mod dynamics;

pub use common::math;
pub use common::settings;

pub mod b2 {
    pub use common::*;
    pub use dynamics::*;
    pub use collision::*;
    pub use math::*;
    pub use settings::*;
}
