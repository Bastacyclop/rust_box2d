## rust_box2d [![Build Status](https://github.com/Bastacyclop/rust_box2d/actions/workflows/build.yml/badge.svg?branch=master)](https://travis-ci.org/Bastacyclop/rust_box2d) [![Crates.io](https://img.shields.io/crates/v/wrapped2d.svg?style=flat-square)](https://crates.io/crates/wrapped2d) [![License](https://img.shields.io/crates/l/wrapped2d.svg?style=flat-square)](/LICENSE)

A library wrapping around the Box2D physics engine. It aims to provide an idiomatic interface.

> Box2D is a 2D rigid body simulation library for games. Programmers can use it in their games to make objects move in realistic ways and make the game world more interactive. From the game engine's point of view, a physics engine is just a system for procedural animation.

You won't find a lot of information about Box2D itself here, look at [the official website](http://box2d.org/)
instead.

## [Documentation](https://bastacyclop.github.io/rust_box2d/wrapped2d/)

You can look at the [testbed](testbed) for examples.

## Dependencies

You will need [CMake](https://cmake.org/) installed in order to build Box2D.

Alternatively, you can supply your own installation of Box2D ([version 2.3.1](https://github.com/erincatto/Box2D/releases/tag/v2.3.1)) by pointing the `BOX2D_LIB_DIR` environment variable to the directory containing the compiled Box2D library. For example:

~~~~sh
BOX2D_LIB_DIR="path\to\Box2D\lib" cargo build
~~~~
