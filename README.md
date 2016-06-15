## rust_box2d [![Build Status](https://travis-ci.org/Bastacyclop/rust_box2d.svg?branch=master)](https://travis-ci.org/Bastacyclop/rust_box2d)

A library wrapping around the Box2D physics engine. It aims to provide an idiomatic interface.

> Box2D is a 2D rigid body simulation library for games. Programmers can use it in their games to make objects move in realistic ways and make the game world more interactive. From the game engine's point of view, a physics engine is just a system for procedural animation.

You won't find a lot of information about Box2D itself here, look at [the official website](http://box2d.org/)
instead.

## [Documentation](https://bastacyclop.github.io/rust_box2d/wrapped2d/)

You can look at the [testbed](testbed) for examples.

## Dependencies

You will need the native Box2D library (this is based on [version 2.3.1](https://github.com/erincatto/Box2D/releases/tag/v2.3.1)). And also Piston [dependencies](https://github.com/PistonDevelopers/Piston-Tutorials/tree/master/getting-started) for the testbed.

If necessary, you can specify the Box2D header files location when compiling:

~~~~sh
BOX2D_INCLUDE_PATH="path/to/Box2D/Box2D" cargo build
~~~~
