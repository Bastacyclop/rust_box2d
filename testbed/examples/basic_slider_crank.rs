extern crate wrapped2d;
extern crate testbed;

use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;

type World = b2::World<NoUserData>;

fn main() {
    let mut world = World::new(&b2::Vec2 { x: 0., y: -10. });

    let ground = create_ground(&mut world);
    let crank = create_crank(&mut world, ground);
    let connecting_rod = create_connecting_rod(&mut world, crank);
    create_piston(&mut world, ground, connecting_rod);

    let data = testbed::Data {
        world: world,
        camera: testbed::Camera {
            position: [0., 20.],
            size: [40., 40.]
        },
        draw_flags: b2::DrawFlags::DRAW_SHAPE |
                    b2::DrawFlags::DRAW_JOINT |
                    b2::DrawFlags::DRAW_PAIR |
                    b2::DrawFlags::DRAW_CENTER_OF_MASS
    };

    testbed::run((), data, "Basic Slider Crank", 400, 400);
}

fn create_ground(world: &mut World) -> b2::BodyHandle {
    let def = b2::BodyDef {
        position: b2::Vec2 { x: 0., y: 17. },
        .. b2::BodyDef::new()
    };

    world.create_body(&def)
}

fn create_crank(world: &mut World, ground: b2::BodyHandle) -> b2::BodyHandle {
    let shape = b2::PolygonShape::new_box(4., 1.);

    let b_def = b2::BodyDef {
        body_type: b2::BodyType::Dynamic,
        position: b2::Vec2 { x: -8., y: 20. },
        .. b2::BodyDef::new()
    };

    let handle = world.create_body(&b_def);
    world.body_mut(handle).create_fast_fixture(&shape, 2.);

    let mut j_def = b2::RevoluteJointDef::new(ground, handle);
    j_def.init(world, ground, handle, &b2::Vec2 { x: -12., y: 20. });
    world.create_joint(&j_def);

    handle
}

fn create_connecting_rod(world: &mut World,
                         crank: b2::BodyHandle) -> b2::BodyHandle {
    let shape = b2::PolygonShape::new_box(8., 1.);

    let b_def = b2::BodyDef {
        body_type: b2::BodyType::Dynamic,
        position: b2::Vec2 { x: 4., y: 20. },
        .. b2::BodyDef::new()
    };

    let handle = world.create_body(&b_def);
    world.body_mut(handle).create_fast_fixture(&shape, 2.);

    let mut j_def = b2::RevoluteJointDef::new(crank, handle);
    j_def.init(world, crank, handle, &b2::Vec2 { x: -4., y: 20. });
    world.create_joint(&j_def);

    handle
}

fn create_piston(world: &mut World,
                 ground: b2::BodyHandle,
                 connecting_rod: b2::BodyHandle) {
    let shape = b2::PolygonShape::new_box(3., 3.);

    let b_def = b2::BodyDef {
        body_type: b2::BodyType::Dynamic,
        fixed_rotation: true,
        position: b2::Vec2 { x: 12., y: 20. },
        .. b2::BodyDef::new()
    };

    let handle = world.create_body(&b_def);
    world.body_mut(handle).create_fast_fixture(&shape, 2.);

    let mut j_def = b2::RevoluteJointDef::new(connecting_rod, handle);
    j_def.init(world, connecting_rod, handle, &b2::Vec2 { x: 12., y: 20. });
    world.create_joint(&j_def);

    let mut j_def = b2::PrismaticJointDef::new(ground, handle);
    j_def.init(world, ground, handle,
               &b2::Vec2 { x: 12., y: 17. },
               &b2::Vec2 { x: 1., y: 0. });
    world.create_joint(&j_def);
}
