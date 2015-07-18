extern crate piston;
extern crate box2d;
extern crate testbed;

use piston::input::{ Input, Button, Key };
use box2d::b2;

fn main() {
    let mut body_def = b2::BodyDef::new();
    body_def.body_type = b2::BodyType::Dynamic;
    body_def.position = b2::Vec2 { x: -20., y: 20. };

    let mut cube_shape = b2::PolygonShape::new();
    cube_shape.set_as_box(1., 1.);

    let mut circle_shape = b2::CircleShape::new();
    circle_shape.set_radius(1.);

    let mut fixture_def = b2::FixtureDef::new();
    fixture_def.density = 1.;
    fixture_def.restitution = 0.2;
    fixture_def.friction = 0.3;

    let mut world = b2::World::new(&b2::Vec2 { x: 0., y: -10. });

    let mut ground_body_def = b2::BodyDef::new();
    ground_body_def.body_type = b2::BodyType::Static;
    ground_body_def.position = b2::Vec2 { x: 0., y: -10. };

    let ground_body = world.create_body(&ground_body_def);

    let mut ground_box = b2::PolygonShape::new();
    ground_box.set_as_box(20., 1.);
    world.get_body_mut(ground_body).unwrap()
         .create_fast_fixture(&ground_box, 0.);

    let camera = testbed::Camera {
        position: [0., -5.],
        scale: [1./20., 1./20.]
    };

    testbed::run(
        "Simple", 400, 400,
        world, camera,
        |world, _, input| {
            let mut create_body = |shape| {
                body_def.position.x += 0.5;
                if body_def.position.x > 20. {
                    body_def.position.x = -20.;
                }
                let body = world.create_body(&body_def);
                world.get_body_mut(body).unwrap()
                     .create_fixture(shape, fixture_def.clone());
            };
            match input {
                Input::Press(Button::Keyboard(Key::A)) => create_body(&cube_shape),
                Input::Press(Button::Keyboard(Key::Z)) => create_body(&circle_shape),
                _ => ()
            }
        }
    );
}
