#![crate_id = "rust_box2d#2.3.1"]
#![crate_type = "lib"]
#![license = "GPLv3"]

#![feature(macro_rules)]

pub use common::math;

pub mod ffi;

#[macro_export]
macro_rules! c_enum(
    ([$name:ident] $($element:ident = $value:expr),+) => (
        pub type $name = i32;
        $(
            pub static $element: $name = $value as $name;
        )+
    );
)

pub mod dynamics;
pub mod common;
pub mod collision;
