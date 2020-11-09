extern crate piston_window;
extern crate testbed;
extern crate wrapped2d;

use piston_window::Button::*;
use piston_window::*;
use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;

type World = b2::World<NoUserData>;

fn main() {
    let mut world = World::new(&b2::Vec2 { x: 0., y: 0. });

    let ground = create_ground(&mut world);
    let body = create_body(&mut world);
    create_cubes(&mut world, ground);

    let process_input = |input: &Input, data: &mut testbed::Data<NoUserData>| match input {
        Input::Button(ButtonArgs {
            state,
            button,
            scancode: _,
        }) => {
            match (state, button) {
                (ButtonState::Press, Keyboard(Key::Z)) => {
                    let mut body = data.world.body_mut(body);
                    let f = body.world_vector(&b2::Vec2 { x: 0., y: -200. });
                    let p = body.world_point(&b2::Vec2 { x: 0., y: 2. });
                    body.apply_force(&f, &p, true);
                }
                (ButtonState::Press, Keyboard(Key::Q)) => {
                    data.world.body_mut(body).apply_torque(50., true);
                }
                (ButtonState::Press, Keyboard(Key::D)) => {
                    data.world.body_mut(body).apply_torque(-50., true);
                }
                _ => {}
            };
        }
        _ => {}
    };

    let data = testbed::Data {
        world,
        camera: testbed::Camera {
            position: [0., 0.],
            size: [40., 40.],
        },
        draw_flags: b2::DrawFlags::DRAW_SHAPE
            | b2::DrawFlags::DRAW_JOINT
            | b2::DrawFlags::DRAW_PAIR
            | b2::DrawFlags::DRAW_CENTER_OF_MASS,
    };

    testbed::run(process_input, data, "Apply Force", 400, 400);
}

fn create_ground(world: &mut World) -> b2::BodyHandle {
    let def = b2::BodyDef {
        position: b2::Vec2 { x: 0., y: 0. },
        ..b2::BodyDef::new()
    };

    let handle = world.create_body(&def);
    let mut ground = world.body_mut(handle);

    let mut edge = b2::EdgeShape::new();
    let top_right = b2::Vec2 { x: 20., y: 20. };
    let top_left = b2::Vec2 { x: -20., y: 20. };
    let bot_left = b2::Vec2 { x: -20., y: -20. };
    let bot_right = b2::Vec2 { x: 20., y: -20. };

    let mut def = b2::FixtureDef {
        density: 0.,
        restitution: 0.4,
        ..b2::FixtureDef::new()
    };

    let mut create_edge = |p1, p2| {
        edge.set(p1, p2);
        ground.create_fixture(&edge, &mut def);
    };
    create_edge(&top_right, &top_left);
    create_edge(&top_left, &bot_left);
    create_edge(&bot_left, &bot_right);
    create_edge(&bot_right, &top_right);

    handle
}

fn create_body(world: &mut World) -> b2::BodyHandle {
    let def = b2::BodyDef {
        body_type: b2::BodyType::Dynamic,
        angular_damping: 2.,
        linear_damping: 0.5,
        position: b2::Vec2 { x: 0., y: -18. },
        angle: b2::PI,
        allow_sleep: false,
        ..b2::BodyDef::new()
    };

    let handle = world.create_body(&def);
    let mut body = world.body_mut(handle);

    let mut f_def = b2::FixtureDef::new();

    let mut create_fixture = |transform: &b2::Transform, density| {
        let vertices = [
            transform * b2::Vec2 { x: -1., y: 0. },
            transform * b2::Vec2 { x: 1., y: 0. },
            transform * b2::Vec2 { x: 0., y: 0.5 },
        ];

        let polygon = b2::PolygonShape::new_with(&vertices);

        f_def.density = density;
        body.create_fixture(&polygon, &mut f_def);
    };

    let angle = 0.3524 * b2::PI;
    let rot = b2::Rot::from_angle(angle);
    let transform = b2::Transform {
        pos: rot.x_axis(),
        rot,
    };
    create_fixture(&transform, 4.);

    let angle = -angle;
    let rot = b2::Rot::from_angle(angle);
    let transform = b2::Transform {
        pos: -rot.x_axis(),
        rot,
    };
    create_fixture(&transform, 2.);

    handle
}

fn create_cubes(world: &mut World, ground: b2::BodyHandle) {
    let shape = b2::PolygonShape::new_box(0.5, 0.5);

    let mut f_def = b2::FixtureDef {
        density: 1.,
        friction: 0.3,
        ..b2::FixtureDef::new()
    };

    let mut b_def = b2::BodyDef {
        body_type: b2::BodyType::Dynamic,
        ..b2::BodyDef::new()
    };

    for i in 0..10 {
        b_def.position = b2::Vec2 {
            x: 0.,
            y: 15. - 1.54 * i as f32,
        };

        let handle = world.create_body(&b_def);
        let inertia;
        let mass;
        let radius;
        {
            let mut body = world.body_mut(handle);

            body.create_fixture(&shape, &mut f_def);

            inertia = body.inertia();
            mass = body.mass();
            radius = (2. * inertia / mass).sqrt();
        }
        let gravity = 10.;
        let j_def = b2::FrictionJointDef {
            collide_connected: true,
            local_anchor_a: b2::Vec2 { x: 0., y: 0. },
            local_anchor_b: b2::Vec2 { x: 0., y: 0. },
            max_force: mass * gravity,
            max_torque: mass * radius * gravity,
            ..b2::FrictionJointDef::new(ground, handle)
        };

        world.create_joint(&j_def);
    }
}
