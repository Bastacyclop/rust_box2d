## Rust_Box2D

Box2D for Rust.

# Installation

Simply add a dependency in your Cargo.toml:

    [dependencies.rust_box2d]
    
    git = "https://gitlab.com/Bastacyclop/rust_box2d.git"
    
Or build it from source:

0. get the source:

        $ git clone "https://gitlab.com/Bastacyclop/rust_box2d.git"
        
0. build the library:

        $ cargo build
        
0. build the documentation:

        $ rustdoc src/rust_box2d.rs
        
0. build the examples:

        $ cargo test

## Todo

- Documentation
- More Safety
- Unit system ?
- World/Local ?
- Contacts ?
- UserData ?
- Jacobian ?
- JointEdge
- ContactEdge

### Authors

Thomas Koehler - main developer - <basta.t.k+git@gmail.com>

### License

Rust_Box2D is under the GPLv3 license.

### Rust_Box2D is using :

- Box2D ([website](http://box2d.org/))
