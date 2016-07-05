extern crate piston;
extern crate graphics;
extern crate gfx_graphics;
extern crate gfx;
extern crate gfx_device_gl;
extern crate glutin_window;
extern crate wrapped2d;

pub mod camera;
pub use camera::Camera;
mod debug_draw;
use debug_draw::debug_draw;

use piston::input::{Event, Input, Button, Motion, Key, MouseButton};
use piston::window::{Window, OpenGLWindow, WindowSettings};
use piston::event_loop::*;
use gfx::traits::*;
use glutin_window::*;
use wrapped2d::b2;
use wrapped2d::user_data::UserDataTypes;

type GfxResources = gfx_device_gl::Resources;
type GfxCommandBuffer = gfx_device_gl::command::CommandBuffer;
type Gfx2d = gfx_graphics::Gfx2d<GfxResources>;
type GfxGraphics<'a> = gfx_graphics::GfxGraphics<'a, GfxResources, GfxCommandBuffer>;

pub const UPDATES_PER_SECOND: u64 = 60;
pub const TIME_STEP: f32 = 1.0 / UPDATES_PER_SECOND as f32;
pub const VELOCITY_ITERATIONS: i32 = 8;
pub const POSITION_ITERATIONS: i32 = 3;

pub trait Test<U: UserDataTypes> {
    fn process_input(&mut self, Input, &mut Data<U>) {}

    fn step(&mut self, data: &mut Data<U>) {
        step(data);
    }
}

pub fn step<U: UserDataTypes>(data: &mut Data<U>) {
    data.world.step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
}

impl<U: UserDataTypes> Test<U> for () {}

impl<F, U: UserDataTypes> Test<U> for F
    where F: FnMut(Input, &mut Data<U>)
{
    fn process_input(&mut self, i: Input, d: &mut Data<U>) {
        self(i, d);
    }
}

pub struct Data<U: UserDataTypes> {
    pub world: b2::World<U>,
    pub camera: Camera,
    pub draw_flags: b2::DrawFlags
}

pub fn run<T, U>(mut test: T, mut data: Data<U>, name: &str, mut width: u32, mut height: u32)
    where T: Test<U>, U: UserDataTypes, U::BodyData: Default, U::JointData: Default
{
    let opengl = OpenGL::V3_2;

    let mut window: GlutinWindow = WindowSettings::new(format!("{} Test", name),
                                                       [width, height])
                                       .opengl(opengl)
                                       .exit_on_esc(true)
                                       .samples(4)
                                       .build()
                                       .unwrap();
    let mut events = window.events();
    events.set_max_fps(60);
    events.set_ups(UPDATES_PER_SECOND);

    let (mut device, mut factory) = gfx_device_gl::create(|s| {
        window.get_proc_address(s) as *const _
    });

    let draw_size = window.draw_size();
    let aa = 4u8;
    let dim = (draw_size.width as u16,
               draw_size.height as u16,
               1,
               aa.into());
    let (output_color, output_stencil) = gfx_device_gl::create_main_targets(dim);
    let mut gfx2d = Gfx2d::new(opengl, &mut factory);
    let mut encoder = factory.create_encoder();

    let mut running = false;
    let mut mouse_position = b2::Vec2 { x: 0., y: 0. };
    let mut grabbing = None;

    let dummy = data.world.create_body(&b2::BodyDef::new());

    while let Some(event) = events.next(&mut window) {
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

        match event {
            Event::Input(i) => {
                match i {
                    Input::Press(Button::Keyboard(Key::Return)) => running = !running,
                    Input::Press(Button::Mouse(MouseButton::Left)) => {
                        match grabbing {
                            None => grabbing = try_grab(&mut data.world, mouse_position, dummy),
                            Some(_) => {}
                        }
                    }
                    Input::Release(Button::Mouse(MouseButton::Left)) => {
                        ungrab(&mut data.world, &mut grabbing)
                    }
                    Input::Move(Motion::MouseCursor(x, y)) => {
                        mouse_position = window_to_world(width, height, &data.camera, x, y);
                        update_grab(&mut data.world, mouse_position, grabbing);
                    }
                    Input::Resize(w, h) => {
                        width = w;
                        height = h;
                    }
                    _ => (),
                }
                test.process_input(i, &mut data);
            }
            Event::Update(_) => {
                if running { test.step(&mut data); }
            }
            Event::Render(args) => {
                encoder.reset();
                gfx2d.draw(&mut encoder,
                           &output_color,
                           &output_stencil,
                           args.viewport(),
                           |c, g| {
                               graphics::clear([0.3, 0.3, 0.3, 1.0], g);
                               debug_draw(&mut data.world, data.draw_flags, transform, c, g);
                           });
                device.submit(encoder.as_buffer());
            }
            _ => {}
        }
    }
}

fn try_grab<U>(world: &mut b2::World<U>, p: b2::Vec2, dummy: b2::BodyHandle) -> Option<b2::JointHandle>
    where U: UserDataTypes, U::BodyData: Default, U::JointData: Default
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
    where U: UserDataTypes, U::BodyData: Default, U::JointData: Default
{
    let d = b2::Vec2 {
        x: 0.001,
        y: 0.001,
    };
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
    where U: UserDataTypes, U::BodyData: Default, U::JointData: Default
{
    grabbing.take().map(|j| world.destroy_joint(j));
}

fn update_grab<U>(world: &b2::World<U>, target: b2::Vec2, grabbing: Option<b2::JointHandle>)
    where U: UserDataTypes, U::BodyData: Default, U::JointData: Default
{
    grabbing.map(|j| {
        let mut j = world.joint_mut(j);
        match **j {
            b2::UnknownJoint::Mouse(ref mut j) => {
                j.set_target(&target);
            }
            _ => panic!("expected mouse joint")
        }
    });
}
