#!/bin/sh
mkdir -p target
g++ -c src/box2d_frontend.cpp -o target/box2d_frontend.o -std=c++11 -Isrc -fPIC -Wall
ar rcs target/libbox2d_frontend.a target/box2d_frontend.o
rustc src/rust_box2d.rs --crate-type dylib --out-dir target -L target
rustc src/bin/example.rs -o target/example -L target -C link_args="-lBox2D -lstdc++"

