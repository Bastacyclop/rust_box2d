## Rust_Box2D

Box2D for Rust.

# Using Rust_Box2D

You won't find a lot of information about Box2D here, look at [the official website](http://box2d.org/)
instead. If you know how to use Box2D, you should know how to use this binding.
They are differences however, so you should look at the examples and the documentation before using something.

__Rust_Box2D is not thread-safe__.

# Installation

Simply add a dependency in your Cargo.toml:

    [dependencies.box2d]

    git = "https://gitlab.com/Bastacyclop/rust_box2d.git"

Or build it from source:

0. get the source:

        $ git clone "https://gitlab.com/Bastacyclop/rust_box2d.git"

0. simply use cargo.

## Todo

- Documentation
- Replace `MaybeOwned` enum with some compile-time trick
- Check/improve safety
- Check `...Def` pointers and lifetimes
- explicit World/Local ?
- Contacts

### Authors

Thomas Koehler - main developer - <basta.t.k+git@gmail.com>

### License

Rust_Box2D is under the GPLv3 license.

### Rust_Box2D is using :

- Box2D ([website](http://box2d.org/))
