extern crate piston;
extern crate wrapped2d;
extern crate testbed;

use std::rc::Rc;
use std::cell::Cell;
use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;
use wrapped2d::dynamics::world::callbacks::ContactAccess;

type UserData = NoUserData;
type World = b2::World<NoUserData>;

fn main() {
    let (test, world) = Test::init();

    let data = testbed::Data {
        world: world,
        camera: testbed::Camera {
            position: [0., 10.],
            size: [40., 40.]
        },
        draw_flags: b2::DRAW_SHAPE |
                    b2::DRAW_AABB |
                    b2::DRAW_JOINT |
                    b2::DRAW_PAIR |
                    b2::DRAW_CENTER_OF_MASS
    };

    testbed::run(test, data, "Web", 400, 400);
}

fn create_ground(world: &mut World) -> b2::BodyHandle {
    let bd = b2::BodyDef {
        body_type: b2::BodyType::Static,
        .. b2::BodyDef::new()
    };

    let ground = world.create_body(&bd);

    let mut shape = b2::EdgeShape::new();
    shape.set(&b2::Vec2 { x: -40., y: 0. }, &b2::Vec2 { x: 40., y: 0. });
    world.body_mut(ground).create_fast_fixture(&shape, 0.);

    ground
}

struct Test {
    body: b2::BodyHandle,
    piece2: b2::FixtureHandle,
    shape2: b2::PolygonShape,
    velocity: b2::Vec2,
    angular_velocity: f32,
    should_break: Rc<Cell<bool>>,
    broke: bool
}

impl Test {
    fn init() -> (Self, World) {
        let mut world = World::new(&b2::Vec2 { x: 0., y: -10. });
        create_ground(&mut world);
        
        let bd = b2::BodyDef {
            body_type: b2::BodyType::Dynamic,
            position: b2::Vec2 { x: 0., y: 40. },
            angle: 0.25 * b2::PI,
            .. b2::BodyDef::new()
        };

        let body = world.create_body(&bd);

        let shape1 = b2::PolygonShape::new_oriented_box(0.5, 0.5, &b2::Vec2 { x: -0.5, y: 0. }, 0.);
        let shape2 = b2::PolygonShape::new_oriented_box(0.5, 0.5, &b2::Vec2 { x: 0.5, y: 0. }, 0.);
        let (_, piece2) = {
            let mut body = world.body_mut(body);
            (body.create_fast_fixture(&shape1, 1.), body.create_fast_fixture(&shape2, 1.))
        };

        let should_break = Rc::new(Cell::new(false));
        world.set_contact_listener(Box::new(ContactListener { should_break: should_break.clone() }));

        (Test {
            body: body,
            piece2: piece2, shape2: shape2,
            velocity: b2::Vec2 { x: 0., y: 0. },
            angular_velocity: 0.,
            should_break: should_break,
            broke: false
        }, world)
    }

    fn break_it(&mut self, world: &mut World) {
        let bd = {
            let body = world.body(self.body);

            b2::BodyDef {
                body_type: b2::BodyType::Dynamic,
                position: *body.position(),
                angle: body.angle(),
                .. b2::BodyDef::new()
            }            
        };

        let other = world.create_body(&bd);

        let mut body = world.body_mut(self.body);
        let center = *body.world_center();
        body.destroy_fixture(self.piece2);

        let mut other = world.body_mut(other);
        self.piece2 = other.create_fast_fixture(&self.shape2, 1.);

		// compute consistent velocities for new bodies based on cached velocities
        let center1 = *body.world_center();
        let center2 = *other.world_center();

        let velocity1 = self.velocity + b2::cross_sv(self.angular_velocity, center1 - center);
        let velocity2 = self.velocity + b2::cross_sv(self.angular_velocity, center2 - center);

        body.set_angular_velocity(self.angular_velocity);
        body.set_linear_velocity(&velocity1);

        other.set_angular_velocity(self.angular_velocity);
        other.set_linear_velocity(&velocity2);

        self.broke = true;
    }
}

impl testbed::Test<UserData> for Test {
    fn step(&mut self, data: &mut testbed::Data<UserData>) {
        if !self.broke {
            if self.should_break.get() {
                self.break_it(&mut data.world);
            } else {
                // cache velocities to improve movement on breakage
                let body = data.world.body(self.body);
                self.velocity = *body.linear_velocity();
                self.angular_velocity = body.angular_velocity();
            }
        }

        testbed::step(data);
    }
}

struct ContactListener {
    should_break: Rc<Cell<bool>>
}

impl b2::ContactListener<UserData> for ContactListener {
    fn post_solve(&mut self, ca: ContactAccess<UserData>, impulse: &b2::ContactImpulse) {
        if !self.should_break.get() {
            let count = ca.contact.manifold().count as usize;
            
            let mut max_impulse = 0f32;
            for i in 0..count {
                max_impulse = max_impulse.max(impulse.normal_impulses[i]);
            }

            if max_impulse > 40. {
                self.should_break.set(true);
            }
        }
    }
}