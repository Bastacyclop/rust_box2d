extern crate b2 = "rust_box2d";

fn main () {
    println!("Rust Box2D example");

    let time_step = 1.0 / 60.0;
    let velocity_iterations = 6;
    let position_iterations = 2;

    let gravity = b2::math::Vec2 { x:0.0, y:-10.0 };
    let mut world = b2::dynamics::World::new(gravity);

    assert_eq!(world.get_body_count(), 0);
    assert_eq!(world.get_gravity(), gravity);

    /*
    let shape = shapes::PolygonShape::box_shape(2.0, 3.0);

    let mut body_def = dynamics::BodyDef::default();
    body_def.body_type = dynamics::DYNAMIC_BODY;
    body_def.position = math::Vec2 {x:0.0, y:4.0};
    body_def.linear_velocity = math::Vec2 {x:1.0, y:4.0};

    let mut b1 = world.create_body(&body_def);

    let mut fixture_def = dynamics::FixtureDef::default(&shape as &shapes::Shape);
    fixture_def.density = 1.0;
    fixture_def.friction = 0.3;
    b1.create_fixture(&fixture_def);

    assert_eq!(world.get_body_count(), 1);

    for _ in range(0, 60) {
        world.step(time_step, velocity_iterations, position_iterations);
        let pos = b1.get_position();
        println!("b1: [x:{}, y:{}]", pos.x, pos.y);
    }

    world.destroy_body(&mut b1);
    assert_eq!(world.get_body_count(), 0);
    */
}
