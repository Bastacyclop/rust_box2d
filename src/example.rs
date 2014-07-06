extern crate b2 = "rust_box2d";

use b2::collision::PolygonShape;

fn main () {
    println!("Rust Box2D example");

    let time_step = 1. / 60.;
    let velocity_iterations = 6;
    let position_iterations = 2;

    let gravity = b2::math::Vec2 { x: 0., y: -10. };
    let mut world = b2::dynamics::World::new(gravity);

    assert_eq!(world.get_body_count(), 0);
    assert_eq!(world.get_gravity(), gravity);

    let mut shape = PolygonShape::new();
        shape.set_as_box(2., 3.);

    let mut body_def = b2::dynamics::BodyDef::new();
        body_def.body_type = b2::dynamics::DYNAMIC;
        body_def.position = b2::math::Vec2 { x: 0., y: 4. };
        body_def.linear_velocity = b2::math::Vec2 { x: 1. , y: 4. };

    let mut body = world.create_body(&body_def);

    let mut fixture_def = b2::dynamics::FixtureDef::new(&shape);
        fixture_def.density = 1.;
        fixture_def.friction = 0.3;
    
    body.create_fixture(&fixture_def);

    assert_eq!(world.get_body_count(), 1);

    for _ in range::<uint>(0, 60) {
        world.step(time_step, velocity_iterations, position_iterations);
        println!("body: {}", body.get_position());
    }

    world.destroy_body(body);
    assert_eq!(world.get_body_count(), 0);
}
