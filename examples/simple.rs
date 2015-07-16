extern crate piston;
extern crate box2d;
extern crate testbed;

use piston::input::{ Input, Button, Key };
use box2d::b2;

fn main() {
    let mut cube_def = b2::BodyDef::new();
    cube_def.body_type = b2::BodyType::Dynamic;
    cube_def.position = b2::Vec2 { x: -20., y: 20. };

    let mut cube_shape = b2::PolygonShape::new();
    cube_shape.set_as_box(1., 1.);

    let mut circle_shape = b2::CircleShape::new();
    circle_shape.set_radius(1.);

    let mut cube_fixture_def = b2::FixtureDef::new();
    cube_fixture_def.density = 1.;
    cube_fixture_def.restitution = 0.2;
    cube_fixture_def.friction = 0.3;

    let mut world = b2::World::new(&b2::Vec2 { x: 0., y: -10. });

    let mut ground_body_def = b2::BodyDef::new();
    ground_body_def.body_type = b2::BodyType::Static;
    ground_body_def.position = b2::Vec2 { x: 0., y: -10. };

    let mut ground_body = world.create_body(&ground_body_def);

    let mut ground_box = b2::PolygonShape::new();
    ground_box.set_as_box(20., 1.);
    ground_body.create_fast_fixture(&ground_box, 0.);

    let camera = testbed::Camera {
        position: [0., -5.],
        scale: [1./20., 1./20.]
    };

    testbed::run(
        "Simple", 400, 400,
        world, camera,
        |world, _, input| {
            match input {
                Input::Press(Button::Keyboard(Key::A)) => {
                    cube_def.position.x += 0.5;
                    if cube_def.position.x > 20. {
                        cube_def.position.x = -20.;
                    }
                    let mut cube = world.create_body(&cube_def);
                    cube.create_fixture(&cube_shape, &mut cube_fixture_def);
                }
                Input::Press(Button::Keyboard(Key::Z)) => {
                    cube_def.position.x += 0.5;
                    if cube_def.position.x > 20. {
                        cube_def.position.x = -20.;
                    }
                    let mut circle = world.create_body(&cube_def);
                    circle.create_fixture(&circle_shape, &mut cube_fixture_def);
                }
                _ => ()
            }
        }
    );
}
