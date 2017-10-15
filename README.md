## rust_box2d [![Build Status](https://img.shields.io/travis/Bastacyclop/rust_box2d/master.svg?style=flat-square)](https://travis-ci.org/Bastacyclop/rust_box2d) [![Crates.io](https://img.shields.io/crates/v/wrapped2d.svg?style=flat-square)](https://crates.io/crates/wrapped2d) [![License](https://img.shields.io/crates/l/wrapped2d.svg?style=flat-square)](/LICENSE)

A library wrapping around the Box2D physics engine. It aims to provide an idiomatic interface.

> Box2D is a 2D rigid body simulation library for games. Programmers can use it in their games to make objects move in realistic ways and make the game world more interactive. From the game engine's point of view, a physics engine is just a system for procedural animation.

You won't find a lot of information about Box2D itself here, look at [the official website](http://box2d.org/)
instead.

## [Documentation](https://bastacyclop.github.io/rust_box2d/wrapped2d/)

You can look at the [testbed](testbed) for examples.

## Dependencies

You will need the native Box2D library (this is based on [version 2.3.1](https://github.com/erincatto/Box2D/releases/tag/v2.3.1)).  
For example, you can install it on your system with your package manager or [directly from source](.travis.yml).

If necessary, you can specify the Box2D header files location when compiling:

~~~~sh
BOX2D_INCLUDE_PATH="path/to/Box2D/Box2D" cargo build
~~~~
