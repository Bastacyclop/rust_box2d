extern crate piston_window;
extern crate wrapped2d;
extern crate testbed;

use piston_window::*;
use piston_window::Button::*;
use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;

type World = b2::World<NoUserData>;

fn main() {
    let mut world = World::new(&b2::Vec2 { x: 0., y: -10. });

    let b_def = b2::BodyDef {
        body_type: b2::BodyType::Static,
        position: b2::Vec2 { x: 0., y: -10. },
        .. b2::BodyDef::new()
    };

    let ground_box = b2::PolygonShape::new_box(20., 1.);

    let ground_handle = world.create_body(&b_def);
    world.body_mut(ground_handle).create_fast_fixture(&ground_box, 0.);


    let mut b_def = b2::BodyDef {
        body_type: b2::BodyType::Dynamic,
        position: b2::Vec2 { x: -20., y: 20. },
        .. b2::BodyDef::new()
    };

    let cube_shape = b2::PolygonShape::new_box(1., 1.);

    let mut circle_shape = b2::CircleShape::new();
    circle_shape.set_radius(1.);

    let mut f_def = b2::FixtureDef {
        density: 1.,
        restitution: 0.2,
        friction: 0.3,
        .. b2::FixtureDef::new()
    };

    let process_input = |input: &Input, data: &mut testbed::Data<NoUserData>| {
        let mut create_body = |shape| {
            b_def.position.x += 0.5;
            if b_def.position.x > 20. {
                b_def.position.x = -20.;
            }
            let handle = data.world.create_body(&b_def);
            data.world.body_mut(handle).create_fixture(shape, &mut f_def);
        };

        match input.press_args() {
            Some(Keyboard(Key::A)) =>  {
                create_body(&cube_shape);
            },
            Some(Keyboard(Key::Z))=>  {
                create_body(&circle_shape);
            },
            _ => ()
        }
    };

    let data = testbed::Data {
        world: world,
        camera: testbed::Camera {
            position: [0., 5.],
            size: [40., 40.]
        },
        draw_flags: b2::DrawFlags::DRAW_SHAPE |
                    b2::DrawFlags::DRAW_AABB |
                    b2::DrawFlags::DRAW_JOINT |
                    b2::DrawFlags::DRAW_PAIR |
                    b2::DrawFlags::DRAW_CENTER_OF_MASS
    };

    testbed::run(process_input, data, "Simple", 400, 400);
}
