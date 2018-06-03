extern crate cc;
extern crate cmake;

fn main() {
    println!("cargo:rustc-link-lib=static=Box2D");
    if let Some(path) = std::env::var("BOX2D_LIB_DIR").ok() {
        println!("cargo:rustc-link-search=native={}", path);
    } else {
        let box2d_install_prefix = cmake::Config::new("Box2D/Box2D")
            .define("BOX2D_BUILD_STATIC", "ON")
            .define("BOX2D_INSTALL", "ON")
            .define("BOX2D_BUILD_SHARED", "OFF")
            .define("BOX2D_BUILD_EXAMPLES", "OFF")
            .define("BOX2D_INSTALL_DOC", "OFF")
            .build();
        println!("cargo:rustc-link-search=native={}/lib", box2d_install_prefix.display());
    };

    cc::Build::new()
        .cpp(true)
        .include("Box2D/Box2D")
        .file("frontend/lib.cpp")
        .compile("libbox2d_frontend.a");
}
