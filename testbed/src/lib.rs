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
use piston::event_loop::Events;
use gfx::traits::*;
use glutin_window::*;
use wrapped2d::b2;
use wrapped2d::user_data::NoUserData;

type GfxResources = gfx_device_gl::Resources;
type GfxCommandBuffer = gfx_device_gl::command::CommandBuffer;
type Gfx2d = gfx_graphics::Gfx2d<GfxResources>;
type GfxGraphics<'a> = gfx_graphics::GfxGraphics<'a, GfxResources, GfxCommandBuffer>;

pub type World = b2::World<NoUserData>;

pub fn run<F>(name: &str,
              mut width: u32,
              mut height: u32,
              mut world: World,
              mut camera: Camera,
              draw_flags: b2::DrawFlags,
              mut process_input: F)
    where F: FnMut(&mut World, &mut Camera, Input)
{

    let opengl = OpenGL::V3_2;

    let mut window: GlutinWindow = WindowSettings::new(format!("{} Testbed", name),
                                                       [width, height])
                                       .opengl(opengl)
                                       .exit_on_esc(true)
                                       .samples(4)
                                       .build()
                                       .unwrap();
    let mut events = window.events();

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

    let transform = camera.transform_world_to_gl();
    let window_to_gl = |w, h, x, y| {
        let scale_x = 2. / w as f64;
        let scale_y = 2. / h as f64;
        (x * scale_x - 1., -1. * (y * scale_y - 1.))
    };
    let window_to_world = |w, h, camera: &Camera, x, y| {
        let (x, y) = window_to_gl(w, h, x, y);
        camera.gl_to_world(x, y)
    };

    let time_step = 1. / 60.;
    let velocity_iterations = 8;
    let position_iterations = 3;

    let mut running = false;
    let mut mouse_position = b2::Vec2 { x: 0., y: 0. };
    let mut grabbing = None;
    let mut accumulator = 0.;

    let dummy = world.create_body(&b2::BodyDef::new());

    while let Some(event) = events.next(&mut window) {
        match event {
            Event::Input(i) => {
                match i {
                    Input::Press(Button::Keyboard(Key::Return)) => running = !running,
                    Input::Press(Button::Mouse(MouseButton::Left)) => {
                        match grabbing {
                            None => grabbing = try_grab(&mut world, mouse_position, dummy),
                            Some(_) => {}
                        }
                    }
                    Input::Release(Button::Mouse(MouseButton::Left)) => {
                        ungrab(&mut world, &mut grabbing)
                    }
                    Input::Move(Motion::MouseCursor(x, y)) => {
                        mouse_position = window_to_world(width, height, &camera, x, y);
                        update_grab(&mut world, mouse_position, grabbing);
                    }
                    Input::Resize(w, h) => {
                        width = w;
                        height = h;
                    }
                    _ => (),
                }
                process_input(&mut world, &mut camera, i);
            }
            Event::Update(args) => {
                accumulator += args.dt as f32;
                while accumulator >= time_step {
                    if running {
                        world.step(time_step, velocity_iterations, position_iterations);
                    }
                    accumulator -= time_step;
                }
            }
            Event::Render(args) => {
                encoder.reset();
                gfx2d.draw(&mut encoder,
                           &output_color,
                           &output_stencil,
                           args.viewport(),
                           |c, g| {
                               graphics::clear([0.3, 0.3, 0.3, 1.0], g);
                               debug_draw(&mut world, draw_flags, transform, c, g);
                           });
                device.submit(encoder.as_buffer());
            }
            _ => {}
        }
    }
}

fn try_grab(world: &mut World, p: b2::Vec2, dummy: b2::BodyHandle) -> Option<b2::JointHandle> {
    match query_point(world, p) {
        None => None,
        Some(body_h) => {
            let mass;
            let center;
            {
                let mut body = world.get_body_mut(body_h);
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

fn query_point(world: &World, p: b2::Vec2) -> Option<b2::BodyHandle> {
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
            let body = world.get_body(body_h);
            let fixture = body.get_fixture(fixture_h);

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

fn ungrab(world: &mut World, grabbing: &mut Option<b2::JointHandle>) {
    grabbing.take().map(|j| world.destroy_joint(j));
}

fn update_grab(world: &World, target: b2::Vec2, grabbing: Option<b2::JointHandle>) {
    grabbing.map(|j| {
        let mut j = world.get_joint_mut(j);
        match **j {
            b2::UnknownJoint::Mouse(ref mut j) => {
                j.set_target(&target);
            }
            _ => {
                panic!("expected mouse joint");
            }
        }
    });
}
