extern crate wrapped2d;

use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;

#[test]
fn fall() {
    let time_step = 1./60.;
    let velocity_iterations = 6;
    let position_iterations = 2;

    let gravity = b2::Vec2 { x: 0., y: -10. };
    let mut world = b2::World::<NoUserData>::new(&gravity);

    assert_eq!(world.body_count(), 0);
    assert_eq!(world.gravity(), gravity);

    let mut ground_body_def = b2::BodyDef::new();
    ground_body_def.position = b2::Vec2 { x: 0., y: -10. };
    let ground_handle = world.create_body(&ground_body_def);
    {
        let mut ground = world.body_mut(ground_handle);

        assert_eq!(ground.position(), &b2::Vec2 { x: 0., y: -10. });
        assert_eq!(world.body_count(), 1);

        let mut ground_box = b2::PolygonShape::new();
        ground_box.set_as_box(50., 10.);
        ground.create_fast_fixture(&ground_box, 0.);
    }

    let mut body_def = b2::BodyDef::new();
    body_def.body_type = b2::BodyType::Dynamic;
    body_def.position = b2::Vec2 { x: 0., y: 4. };
    let body = world.create_body(&body_def);

    {
        let body = world.body(body);
        assert_eq!(body.body_type(), b2::BodyType::Dynamic);
        assert_eq!(body.position(), &b2::Vec2 { x: 0., y: 4. });
    }
    assert_eq!(world.body_count(), 2);

    let mut body_box = b2::PolygonShape::new();
    body_box.set_as_box(1., 1.);
    let mut fixture_def = b2::FixtureDef::new();
    fixture_def.density = 1.;
    fixture_def.friction = 0.3;
    world.body_mut(body).create_fixture(&body_box, &mut fixture_def);

    println!("(x, y) angle");
    for _ in 0..60 {
        world.step(time_step, velocity_iterations, position_iterations);
        let body = world.body(body);
        let pos = body.position();
        let angle = body.angle();
        println!("({}, {}) {}", pos.x, pos.y, angle);
    }

    world.destroy_body(body);
    assert_eq!(world.body_count(), 1);
}
