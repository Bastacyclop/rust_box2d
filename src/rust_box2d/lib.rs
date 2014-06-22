#![crate_id = "rust_box2d#2.3.1"]
#![crate_type = "lib"]
#![license = "GPLv3"]

pub use common::math;

mod ffi;

pub mod dynamics;
pub mod common;
