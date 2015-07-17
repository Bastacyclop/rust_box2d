#![feature(libc, associated_consts, vecmap, ptr_as_ref)]

#[link(name = "Box2D")] extern {}
#[link(name = "stdc++")] extern {}

extern crate libc;
#[macro_use] extern crate bitflags;

#[macro_use] pub mod wrap;
pub mod handle;
mod ffi;
pub mod dynamics;
pub mod common;
pub mod collision;

pub use common::math;
pub use common::settings;

pub mod b2 {
    pub use common::*;
    pub use dynamics::*;
    pub use collision::*;
    pub use math::*;
    pub use settings::*;
}
