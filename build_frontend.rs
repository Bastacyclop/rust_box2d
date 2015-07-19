extern crate gcc;

use std::env;

fn main() {
    let mut config = gcc::Config::new();
    let config = config
        .cpp(true)
        .file("frontend/lib.cpp")
        .include("src");

    config.compile("libbox2d_frontend.a");
    let config = match env::var("BOX2D_INCLUDE_PATH") {
        Ok(path) => config.include(path),
        Err(_) => config
    };

    config.compile("libbox2d_frontend.a");
}
