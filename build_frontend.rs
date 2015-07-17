extern crate gcc;

use std::env;

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    let mut config = gcc::Config::new();
    let config = config
        .cpp(true)
        .flag("-c")
        .flag("-std=c++11")
        //.flag("-fPIC")
        .flag("-Wall")
        .file("src/box2d_frontend.cpp")
        .include("src");

    let config = match env::var("BOX2D_INCLUDE_PATH") {
        Ok(path) => config.include(path),
        Err(_) => config
    };

    config.compile("libbox2d_frontend.a");

    println!("cargo:rustc-link-search=native={}", out_dir);
    println!("cargo:rustc-link-lib=static=box2d_frontend");
}
