#!/bin/sh
mkdir -p lib
mkdir -p obj
mkdir -p bin
g++ -c src/c_box2d/c_box2d.cpp -o obj/c_box2d.o -std=c++11 -Isrc -fPIC -Wall
ar rcs lib/libc_box2d.a obj/c_box2d.o
rustc src/rust_box2d/lib.rs --crate-type dylib --out-dir lib -L lib
rustc src/example.rs -o bin/example -L lib -C link_args="-lBox2D -lstdc++"

