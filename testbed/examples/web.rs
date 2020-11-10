extern crate piston_window;
extern crate testbed;
extern crate wrapped2d;

use piston_window::Button::*;
use piston_window::*;
use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;

type World = b2::World<NoUserData>;

fn main() {
    let mut world = World::new(&b2::Vec2 { x: 0., y: -10. });
    let ground = create_ground(&mut world);
    let mut bodies = create_bodies(&mut world);
    create_joints(&mut world, ground, &bodies);

    let process_input = |input: &Input, data: &mut testbed::Data<NoUserData>| {
        if let Input::Button(ButtonArgs {
            state,
            button,
            scancode: _,
        }) = input
        {
            match (state, button) {
                (ButtonState::Press, Keyboard(Key::B)) => {
                    while let Some(body) = bodies.pop() {
                        data.world.destroy_body(body);
                    }
                }
                (ButtonState::Press, Keyboard(Key::J)) => {
                    while let Some((joint, _)) = data.world.joints().next() {
                        data.world.destroy_joint(joint);
                    }
                }
                _ => {}
            };
        }
    };

    let data = testbed::Data {
        world,
        camera: testbed::Camera {
            position: [0., 10.],
            size: [40., 40.],
        },
        draw_flags: b2::DrawFlags::DRAW_SHAPE
            | b2::DrawFlags::DRAW_AABB
            | b2::DrawFlags::DRAW_JOINT
            | b2::DrawFlags::DRAW_PAIR
            | b2::DrawFlags::DRAW_CENTER_OF_MASS,
    };

    testbed::run(process_input, data, "Web", 400, 400);
}

fn create_ground(world: &mut World) -> b2::BodyHandle {
    let bd = b2::BodyDef {
        body_type: b2::BodyType::Static,
        ..b2::BodyDef::new()
    };

    let ground = world.create_body(&bd);

    let mut shape = b2::EdgeShape::new();
    shape.set(&b2::Vec2 { x: -40., y: 0. }, &b2::Vec2 { x: 40., y: 0. });
    world.body_mut(ground).create_fast_fixture(&shape, 0.1);

    ground
}

fn create_bodies(world: &mut World) -> Vec<b2::BodyHandle> {
    let shape = b2::PolygonShape::new_box(0.5, 0.5);

    let mut bodies = Vec::new();

    for &(x, y) in &[(-5., 5.), (5., 5.), (5., 15.), (-5., 15.)] {
        let bd = b2::BodyDef {
            body_type: b2::BodyType::Dynamic,
            position: b2::Vec2 { x, y },
            ..b2::BodyDef::new()
        };

        let body = world.create_body(&bd);
        world.body_mut(body).create_fast_fixture(&shape, 5.);
        bodies.push(body);
    }

    bodies
}

fn create_joints(world: &mut World, ground: b2::BodyHandle, bodies: &[b2::BodyHandle]) {
    let mut create_joint = |body_a, body_b, local_anchor_a, local_anchor_b| {
        let p1 = world.body(body_a).world_point(&local_anchor_a);
        let p2 = world.body(body_b).world_point(&local_anchor_b);

        let jd = b2::DistanceJointDef {
            frequency: 2.,
            damping_ratio: 0.,
            local_anchor_a,
            local_anchor_b,
            length: (p2 - p1).norm(),
            ..b2::DistanceJointDef::new(body_a, body_b)
        };

        world.create_joint(&jd)
    };

    for (i, &(ax, ay, bx, by)) in [
        (-10., 0., -0.5, -0.5),
        (10., 0., 0.5, -0.5),
        (10., 20., 0.5, 0.5),
        (-10., 20., -0.5, 0.5),
    ]
    .iter()
    .enumerate()
    {
        create_joint(
            ground,
            bodies[i],
            b2::Vec2 { x: ax, y: ay },
            b2::Vec2 { x: bx, y: by },
        );
    }

    for (i, &(ax, ay, bx, by)) in [
        (0.5, 0., -0.5, -0.),
        (0., 0.5, 0., -0.5),
        (-0.5, 0., 0.5, 0.),
        (0., -0.5, 0., 0.5),
    ]
    .iter()
    .enumerate()
    {
        create_joint(
            bodies[i],
            bodies[(i + 1) % bodies.len()],
            b2::Vec2 { x: ax, y: ay },
            b2::Vec2 { x: bx, y: by },
        );
    }
}
