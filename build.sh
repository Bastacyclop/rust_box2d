#!/bin/sh
mkdir -p lib
mkdir -p obj
mkdir -p bin
gcc -c -o obj/c_box2d.o src/c_box2d/c_box2d.cpp -Isrc -g
ar rcs lib/libcbox2d.a obj/c_box2d.o
