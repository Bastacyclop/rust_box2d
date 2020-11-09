extern crate gfx;
extern crate graphics;
extern crate piston;
extern crate piston_window;
extern crate wrapped2d;

pub mod camera;
pub use camera::Camera;
mod debug_draw;
use debug_draw::debug_draw;

use piston::event_loop::EventLoop;
use piston::input::{Key, MouseButton};
use piston::window::WindowSettings;
use piston_window::*;
use wrapped2d::b2;
use wrapped2d::user_data::UserDataTypes;

pub const UPDATES_PER_SECOND: u64 = 60;
pub const VELOCITY_ITERATIONS: i32 = 8;
pub const POSITION_ITERATIONS: i32 = 3;

pub trait Test<U: UserDataTypes> {
    fn process_input(&mut self, _input: &Input, _data: &mut Data<U>) {}

    fn step(&mut self, data: &mut Data<U>, dt: f32) {
        step(data, dt);
    }
}

pub fn step<U: UserDataTypes>(data: &mut Data<U>, dt: f32) {
    data.world
        .step(dt, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
}

impl<U: UserDataTypes> Test<U> for () {}

impl<F, U: UserDataTypes> Test<U> for F
where
    F: FnMut(&Input, &mut Data<U>),
{
    fn process_input(&mut self, i: &Input, d: &mut Data<U>) {
        self(i, d);
    }
}

pub struct Data<U: UserDataTypes> {
    pub world: b2::World<U>,
    pub camera: Camera,
    pub draw_flags: b2::DrawFlags,
}

pub fn run<T, U>(mut test: T, mut data: Data<U>, name: &str, mut width: u32, mut height: u32)
where
    T: Test<U>,
    U: UserDataTypes,
    U::BodyData: Default,
    U::JointData: Default,
{
    let opengl = OpenGL::V3_2;

    let mut window: PistonWindow = WindowSettings::new(format!("{} Test", name), [width, height])
        .maybe_graphics_api(Some(opengl))
        .exit_on_esc(true)
        .samples(4)
        .build()
        .unwrap();
    window.set_max_fps(UPDATES_PER_SECOND);
    window.set_ups(UPDATES_PER_SECOND);

    let mut running = false;
    let mut mouse_position = b2::Vec2 { x: 0., y: 0. };
    let mut grabbing = None;

    let dummy = data.world.create_body(&b2::BodyDef::new());

    while let Some(evnt) = window.next() {
        let transform = data.camera.transform_world_to_gl();
        let window_to_gl = |w, h, x, y| {
            let scale_x = 2. / w as f64;
            let scale_y = 2. / h as f64;
            (x * scale_x - 1., -1. * (y * scale_y - 1.))
        };
        let window_to_world = |w, h, camera: &Camera, x, y| {
            let (x, y) = window_to_gl(w, h, x, y);
            camera.gl_to_world(x, y)
        };

        match &evnt {
            Event::Input(inpt, _) => {
                match inpt {
                    Input::Button(button) => match (button.state, button.button) {
                        (ButtonState::Press, Button::Keyboard(Key::Return)) => running = !running,
                        (ButtonState::Press, Button::Mouse(MouseButton::Left)) => {
                            if grabbing.is_none() {
                                grabbing = try_grab(&mut data.world, mouse_position, dummy);
                            }
                        }
                        (ButtonState::Release, Button::Mouse(MouseButton::Left)) => {
                            ungrab(&mut data.world, &mut grabbing);
                        }
                        _ => {}
                    },
                    Input::Move(mouse_move) => {
                        if let Motion::MouseCursor([x, y]) = mouse_move {
                            mouse_position = window_to_world(width, height, &data.camera, x, y);
                            update_grab(&mut data.world, mouse_position, grabbing);
                        }
                    }
                    Input::Resize(resize) => {
                        width = resize.draw_size[0];
                        height = resize.draw_size[1];
                    }
                    _ => {}
                }
                test.process_input(&inpt, &mut data);
            }
            Event::Loop(Loop::Update(UpdateArgs { dt })) => {
                if running {
                    test.step(&mut data, *dt as f32);
                }
            }
            _ => {}
        }

        window.draw_2d(&evnt, |c, g, _dev| {
            graphics::clear([0.3, 0.3, 0.3, 1.0], g);
            debug_draw(&mut data.world, data.draw_flags, transform, c, g);
        });
    }
}

fn try_grab<U>(
    world: &mut b2::World<U>,
    p: b2::Vec2,
    dummy: b2::BodyHandle,
) -> Option<b2::JointHandle>
where
    U: UserDataTypes,
    U::BodyData: Default,
    U::JointData: Default,
{
    match query_point(world, p) {
        None => None,
        Some(body_h) => {
            let mass;
            let center;
            {
                let mut body = world.body_mut(body_h);
                mass = body.mass();
                center = *body.world_center();
                body.set_awake(true);
            }

            let mut j_def = b2::MouseJointDef::new(dummy, body_h);
            j_def.target = center;
            j_def.max_force = 500. * mass;
            Some(world.create_joint(&j_def))
        }
    }
}

fn query_point<U>(world: &b2::World<U>, p: b2::Vec2) -> Option<b2::BodyHandle>
where
    U: UserDataTypes,
    U::BodyData: Default,
    U::JointData: Default,
{
    let d = b2::Vec2 { x: 0.001, y: 0.001 };
    let aabb = b2::AABB {
        lower: p - d,
        upper: p + d,
    };

    let mut result = None;
    {
        let mut callback = |body_h: b2::BodyHandle, fixture_h: b2::FixtureHandle| {
            let body = world.body(body_h);
            let fixture = body.fixture(fixture_h);

            if body.body_type() != b2::BodyType::Static && fixture.test_point(&p) {
                result = Some(body_h);
                false
            } else {
                true
            }
        };
        world.query_aabb(&mut callback, &aabb);
    }
    result
}

fn ungrab<U>(world: &mut b2::World<U>, grabbing: &mut Option<b2::JointHandle>)
where
    U: UserDataTypes,
    U::BodyData: Default,
    U::JointData: Default,
{
    if let Some(j) = grabbing.take() {
        world.destroy_joint(j)
    }
}

fn update_grab<U>(world: &b2::World<U>, target: b2::Vec2, grabbing: Option<b2::JointHandle>)
where
    U: UserDataTypes,
    U::BodyData: Default,
    U::JointData: Default,
{
    grabbing.map(|j| {
        let mut j = world.joint_mut(j);
        match **j {
            b2::UnknownJoint::Mouse(ref mut j) => {
                j.set_target(&target);
            }
            _ => panic!("expected mouse joint"),
        }
    });
}
