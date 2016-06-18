extern crate piston;
extern crate wrapped2d;
extern crate testbed;

use wrapped2d::b2;
use testbed::World;

fn main() {
    let mut world = World::new(&b2::Vec2 { x: 0., y: -10. });

    let ground = create_ground(&mut world);
    let crank = create_crank(&mut world, ground);
    let connecting_rod = create_connecting_rod(&mut world, crank);
    create_piston(&mut world, ground, connecting_rod);

    let camera = testbed::Camera {
        position: [0., 20.],
        size: [40., 40.]
    };

    let draw_flags = b2::DRAW_SHAPE |
                     b2::DRAW_JOINT |
                     b2::DRAW_PAIR |
                     b2::DRAW_CENTER_OF_MASS;

    testbed::run(
        "Basic Slider Crank", 400, 400,
        world, camera, draw_flags,
        |_, _, _| { }
    );
}

fn create_ground(world: &mut World) -> b2::BodyHandle {
    let mut def = b2::BodyDef::new();
    def.position = b2::Vec2 { x: 0., y: 17. };

    world.create_body(&def)
}

fn create_crank(world: &mut World, ground: b2::BodyHandle) -> b2::BodyHandle {
    let mut shape = b2::PolygonShape::new();
    shape.set_as_box(4., 1.);

    let mut b_def = b2::BodyDef::new();
    b_def.body_type = b2::BodyType::Dynamic;
    b_def.position = b2::Vec2 { x: -8., y: 20. };

    let handle = world.create_body(&b_def);
    world.get_body_mut(handle).create_fast_fixture(&shape, 2.);

    let mut j_def = b2::RevoluteJointDef::new(ground, handle);
    j_def.init(world, ground, handle, &b2::Vec2 { x: -12., y: 20. });
    world.create_joint(&j_def);

    handle
}

fn create_connecting_rod(world: &mut World,
                         crank: b2::BodyHandle) -> b2::BodyHandle {
    let mut shape = b2::PolygonShape::new();
    shape.set_as_box(8., 1.);

    let mut b_def = b2::BodyDef::new();
    b_def.body_type = b2::BodyType::Dynamic;
    b_def.position = b2::Vec2 { x: 4., y: 20. };

    let handle = world.create_body(&b_def);
    world.get_body_mut(handle).create_fast_fixture(&shape, 2.);

    let mut j_def = b2::RevoluteJointDef::new(crank, handle);
    j_def.init(world, crank, handle, &b2::Vec2 { x: -4., y: 20. });
    world.create_joint(&j_def);

    handle
}

fn create_piston(world: &mut World,
                 ground: b2::BodyHandle,
                 connecting_rod: b2::BodyHandle) {
    let mut shape = b2::PolygonShape::new();
    shape.set_as_box(3., 3.);

    let mut b_def = b2::BodyDef::new();
    b_def.body_type = b2::BodyType::Dynamic;
    b_def.fixed_rotation = true;
    b_def.position = b2::Vec2 { x: 12., y: 20. };

    let handle = world.create_body(&b_def);
    world.get_body_mut(handle).create_fast_fixture(&shape, 2.);

    let mut j_def = b2::RevoluteJointDef::new(connecting_rod, handle);
    j_def.init(world, connecting_rod, handle, &b2::Vec2 { x: 12., y: 20. });
    world.create_joint(&j_def);

    let mut j_def = b2::PrismaticJointDef::new(ground, handle);
    j_def.init(world, ground, handle,
               &b2::Vec2 { x: 12., y: 17. },
               &b2::Vec2 { x: 1., y: 0. });
    world.create_joint(&j_def);
}
