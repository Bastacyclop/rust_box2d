extern crate piston;
extern crate wrapped2d;
extern crate testbed;

use piston::input::{ Input, Button, Key };
use wrapped2d::b2;
use testbed::World;

fn main() {
    let mut world = World::new(&b2::Vec2 { x: 0., y: -10. });

    let ground;
    {
        let mut def = b2::BodyDef::new();
        def.body_type = b2::BodyType::Static;
        def.position = b2::Vec2 { x: 0., y: 0. };

        let mut edge = b2::EdgeShape::new();
        edge.set(&b2::Vec2 { x: -20., y: 0. },  &b2::Vec2 { x: 20., y: 0. });

        ground = world.create_body(&def);
        world.body_mut(ground).create_fast_fixture(&edge, 0.);
    }

    let body;
    {
        let mut def = b2::BodyDef::new();
        def.body_type = b2::BodyType::Dynamic;
        def.position = b2::Vec2 { x: -5., y: 20. };

        let shape = b2::PolygonShape::new_box(1., 1.);

        body = world.create_body(&def);
        world.body_mut(body).create_fast_fixture(&shape, 0.3);
    }

    let camera = testbed::Camera {
        position: [0., 20.],
        size: [40., 40.]
    };

    let draw_flags = b2::DRAW_SHAPE |
                     b2::DRAW_AABB |
                     b2::DRAW_JOINT |
                     b2::DRAW_PAIR |
                     b2::DRAW_CENTER_OF_MASS;

    testbed::run(
        "Jump", 400, 400,
        world, camera, draw_flags,
        |world, _, input| {
            match input {
                Input::Press(Button::Keyboard(Key::Space)) => {
                    let mut body = world.body_mut(body);
                    let impulse = b2::Vec2 { x: 0., y: body.mass() * 10. };
                    let point = *body.world_center();
                    body.apply_linear_impulse(&impulse, &point, true);
                }
                _ => ()
            }
        }
    );
}
