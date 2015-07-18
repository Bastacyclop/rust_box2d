## rust_box2d [![Build Status](https://travis-ci.org/Bastacyclop/rust_box2d.svg?branch=master)](https://travis-ci.org/Bastacyclop/rust_box2d)

Box2D for Rust.

You won't find a lot of information about Box2D here, look at [the official website](http://box2d.org/)
instead.

## Dependencies

You will need the native Box2D library. And also Piston [dependencies](https://github.com/PistonDevelopers/Piston-Tutorials/tree/master/getting-started) for the examples.

If necessary, you can specify the Box2D header files location when compiling:

~~~~sh
BOX2D_INCLUDE_PATH="path/to/Box2D/Box2D" cargo run --example simple
~~~~

### TODO

- make joint defs usable
- fix multiple definitions error (with `cargo test`)
- access multiple handles with only one borrow
- test everything and track bugs
