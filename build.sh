#!/bin/sh
mkdir -p lib
mkdir -p obj
mkdir -p bin
gcc -c -o obj/c_box2d.o src/c_box2d/c_box2d.cpp -Isrc -g
ar rcs lib/libc_box2d.a obj/c_box2d.o
rustc src/rust_box2d/lib.rs --out-dir lib -L lib
