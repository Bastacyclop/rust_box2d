extern crate piston;
extern crate wrapped2d;
extern crate testbed;

use piston::input::{ Input, Button, Key };
use wrapped2d::b2;

fn main() {
    let mut world = b2::World::new(&b2::Vec2 { x: 0., y: 0. });

    let ground = create_ground(&mut world);
    let body = create_body(&mut world);
    create_cubes(&mut world, ground);

    let camera = testbed::Camera {
        position: [0., 0.],
        size: [40., 40.]
    };

    let draw_flags = b2::DRAW_SHAPE |
                     b2::DRAW_JOINT |
                     b2::DRAW_PAIR |
                     b2::DRAW_CENTER_OF_MASS;

    testbed::run(
        "Apply Force", 400, 400,
        world, camera, draw_flags,
        |world, _, input| {
            match input {
                Input::Press(Button::Keyboard(Key::Z)) => {
                    let mut body = world.get_body_mut(body);
                    let f = body.world_vector(&b2::Vec2 { x: 0., y: -200. });
                    let p = body.world_point(&b2::Vec2 { x: 0., y: 2. });
                    body.apply_force(&f, &p, true);
                },
                Input::Press(Button::Keyboard(Key::Q)) => {
                    world.get_body_mut(body)
                        .apply_torque(50., true);
                },
                Input::Press(Button::Keyboard(Key::D)) => {
                    world.get_body_mut(body)
                        .apply_torque(-50., true);
                },
                _ => ()
            }
        }
    );
}

fn create_ground(world: &mut b2::World) -> b2::BodyHandle {
    let mut def = b2::BodyDef::new();
    def.position = b2::Vec2 { x: 0., y: 0. };

    let handle = world.create_body(&def);
    let mut ground = world.get_body_mut(handle);

    let mut edge = b2::EdgeShape::new();
    let top_right = b2::Vec2 { x: 20., y: 20. };
    let top_left = b2::Vec2 { x: -20., y: 20. };
    let bot_left = b2::Vec2 { x: -20., y: -20. };
    let bot_right = b2::Vec2 { x: 20., y: -20. };

    let mut def = b2::FixtureDef::new();
    def.density = 0.;
    def.restitution = 0.4;

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

fn create_body(world: &mut b2::World) -> b2::BodyHandle {
    let mut def = b2::BodyDef::new();
    def.body_type = b2::BodyType::Dynamic;
    def.angular_damping = 2.;
    def.linear_damping = 0.5;
    def.position = b2::Vec2 { x: 0., y: -18. };
    def.angle = b2::PI;
    def.allow_sleep = false;

    let handle = world.create_body(&def);
    let mut body = world.get_body_mut(handle);

    let mut f_def = b2::FixtureDef::new();

    let mut create_fixture = |transform: &b2::Transform, density| {
        let vertices = [
            transform * b2::Vec2 { x: -1., y: 0. },
            transform * b2::Vec2 { x: 1., y: 0. },
            transform * b2::Vec2 { x: 0., y: 0.5 }
        ];

        let mut polygon = b2::PolygonShape::new();
        polygon.set(&vertices);

        f_def.density = density;
        body.create_fixture(&polygon, &mut f_def);
    };

    let angle = 0.3524 * b2::PI;
    let rot = b2::Rot::from_angle(angle);
    let transform = b2::Transform {
        pos: rot.x_axis(),
        rot: rot
    };
    create_fixture(&transform, 4.);

    let angle = -angle;
    let rot = b2::Rot::from_angle(angle);
    let transform = b2::Transform {
        pos: -rot.x_axis(),
        rot: rot
    };
    create_fixture(&transform, 2.);

    handle
}

fn create_cubes(world: &mut b2::World,
                ground: b2::BodyHandle) {
    let mut shape = b2::PolygonShape::new();
    shape.set_as_box(0.5, 0.5);

    let mut f_def = b2::FixtureDef::new();
    f_def.density = 1.;
    f_def.friction = 0.3;

    let mut b_def = b2::BodyDef::new();
    b_def.body_type = b2::BodyType::Dynamic;

    for i in 0..10 {
        b_def.position = b2::Vec2 { x: 0., y: 15. - 1.54*i as f32 };

        let handle = world.create_body(&b_def);
        let inertia;
        let mass;
        let radius;
        {
            let mut body = world.get_body_mut(handle);

            body.create_fixture(&shape, &mut f_def);

            inertia = body.inertia();
            mass = body.mass();
            radius = (2.*inertia / mass).sqrt();

        }
        let gravity = 10.;
        let mut j_def = b2::FrictionJointDef::new(ground, handle);
        j_def.collide_connected = true;
        j_def.local_anchor_a = b2::Vec2 { x: 0., y: 0. };
        j_def.local_anchor_b = b2::Vec2 { x: 0., y: 0. };
        j_def.max_force = mass * gravity;
        j_def.max_torque = mass * radius * gravity;

        world.create_joint(&j_def);
    }
}
