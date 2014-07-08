extern crate b2 = "rust_box2d";

use b2::math::Vec2;
use b2::dynamics::{
    World, BodyDef, FixtureDef
};
use b2::collision::{
    PolygonShape
};

fn main () {
    println!("Rust Box2D example");

    let time_step = 1./60.;
    let velocity_iterations = 6;
    let position_iterations = 2;

    let gravity = Vec2 { x: 0., y: -10. };
    let mut world = World::new(&gravity);

    assert_eq!(world.body_count(), 0);
    assert_eq!(world.gravity(), gravity);

    let mut ground_body_def = BodyDef::new();
        ground_body_def.position = Vec2 { x: 0., y: -10. };
    let mut ground_body = world.create_body(&ground_body_def);
    
    assert_eq!(ground_body.position(), Vec2 { x: 0., y: -10. });
    assert_eq!(world.body_count(), 1);
    
    let mut ground_box = PolygonShape::new();
        ground_box.set_as_box(50., 10.);
    ground_body.create_fast_fixture(&ground_box, 0.);

    let mut body_def = BodyDef::new();
        body_def.body_type = b2::dynamics::DYNAMIC_BODY;
        body_def.position = Vec2 { x: 0., y: 4. };
    let mut body = world.create_body(&body_def);

    assert_eq!(body.body_type(), b2::dynamics::DYNAMIC_BODY);
    assert_eq!(body.position(), Vec2 { x: 0., y: 4. });
    assert_eq!(world.body_count(), 2);
    
    let mut body_box = PolygonShape::new();
        body_box.set_as_box(1., 1.);
    let mut fixture_def = FixtureDef::new(&body_box);
        fixture_def.density = 1.;
        fixture_def.friction = 0.3;
    body.create_fixture(&fixture_def);
    
    for _ in range::<uint>(0, 60) {
        world.step(time_step, velocity_iterations, position_iterations);
        let pos = body.position();
        let angle = body.angle();
        println!("({}, {}) {}", pos.x, pos.y, angle);
    }

    world.destroy_body(body); // Unecessary here
    assert_eq!(world.body_count(), 1);
}
