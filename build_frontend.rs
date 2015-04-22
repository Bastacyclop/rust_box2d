use std::env;
use std::process::Command;
use std::path::Path;

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    Command::new("g++").args(&["-c", "src/box2d_frontend.cpp", "-o",
                             &format!("{}/box2d_frontend.o", out_dir),
                             "-std=c++11", "-Isrc", "-fPIC", "-Wall"])
                       .status().unwrap();
    Command::new("ar").args(&["rcs", "libbox2d_frontend.a", "box2d_frontend.o"])
                      .current_dir(&Path::new(&out_dir))
                      .status().unwrap();

    println!("cargo:rustc-link-search=native={}", out_dir);
    println!("cargo:rustc-link-lib=static=box2d_frontend");
}
