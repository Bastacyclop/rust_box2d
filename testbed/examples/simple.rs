extern crate piston;
extern crate wrapped2d;
extern crate testbed;

use piston::input::{ Input, Button, Key };
use wrapped2d::b2;
use testbed::World;

fn main() {
    let mut world = World::new(&b2::Vec2 { x: 0., y: -10. });

    let mut b_def = b2::BodyDef::new();
    b_def.body_type = b2::BodyType::Static;
    b_def.position = b2::Vec2 { x: 0., y: -10. };

    let mut ground_box = b2::PolygonShape::new();
    ground_box.set_as_box(20., 1.);

    let ground_handle = world.create_body(&b_def);
    world.get_body_mut(ground_handle)
         .create_fast_fixture(&ground_box, 0.);


    let mut b_def = b2::BodyDef::new();
    b_def.body_type = b2::BodyType::Dynamic;
    b_def.position = b2::Vec2 { x: -20., y: 20. };

    let mut cube_shape = b2::PolygonShape::new();
    cube_shape.set_as_box(1., 1.);

    let mut circle_shape = b2::CircleShape::new();
    circle_shape.set_radius(1.);

    let mut f_def = b2::FixtureDef::new();
    f_def.density = 1.;
    f_def.restitution = 0.2;
    f_def.friction = 0.3;


    let camera = testbed::Camera {
        position: [0., 5.],
        size: [40., 40.]
    };

    let draw_flags = b2::DRAW_SHAPE |
                     b2::DRAW_AABB |
                     b2::DRAW_JOINT |
                     b2::DRAW_PAIR |
                     b2::DRAW_CENTER_OF_MASS;

    testbed::run(
        "Simple", 400, 400,
        world, camera, draw_flags,
        |world, _, input| {
            let mut create_body = |shape| {
                b_def.position.x += 0.5;
                if b_def.position.x > 20. {
                    b_def.position.x = -20.;
                }
                let handle = world.create_body(&b_def);
                world.get_body_mut(handle)
                     .create_fixture(shape, &mut f_def);
            };
            match input {
                Input::Press(Button::Keyboard(Key::A)) => create_body(&cube_shape),
                Input::Press(Button::Keyboard(Key::Z)) => create_body(&circle_shape),
                _ => ()
            }
        }
    );
}
