extern crate piston;
extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate box2d;

pub mod camera;
pub use camera::Camera;
mod draw_session;
use draw_session::DrawSession;

use piston::event::{ Event, Events };
use piston::input::Input;
use piston::window::WindowSettings;
use glutin_window::GlutinWindow;
use opengl_graphics::{ GlGraphics, OpenGL };
use box2d::b2;

pub fn run<F>(name: &str, width: u32, height: u32,
              mut world: b2::World, mut camera: Camera,
              mut process_input: F)
    where F: FnMut(&mut b2::World, &mut Camera, Input) {

    let gl_version = OpenGL::V3_2;

    let window: GlutinWindow =
        WindowSettings::new(format!("{} Testbed", name), [width, height])
            .exit_on_esc(true)
            .opengl(gl_version)
            .samples(4)
            .into();

    let mut gl = GlGraphics::new(gl_version);

    let time_step = 1./60.;
    let velocity_iterations = 6;
    let position_iterations = 2;

    let draw_flags = b2::DRAW_SHAPE |
                     b2::DRAW_AABB |
                     b2::DRAW_JOINT |
                     b2::DRAW_PAIR |
                     b2::DRAW_CENTER_OF_MASS;

    let mut accumulator = 0.;
    for event in window.events() {
        match event {
            Event::Input(i) => {
                process_input(&mut world, &mut camera, i);
            }
            Event::Update(args) => {
                accumulator += args.dt as f32;
                while accumulator >= time_step {
                    world.step(time_step,
                               velocity_iterations,
                               position_iterations);
                    accumulator -= time_step;
                }
            }
            Event::Render(args) => {
                graphics::clear([0.3, 0.3, 0.3, 1.0], &mut gl);
                DrawSession::new(&mut gl, args, camera.transform(),
                                 &mut world, draw_flags);
            }
            _ => {}
        }
    }
}
