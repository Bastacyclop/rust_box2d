extern crate piston;
extern crate glutin_window;
extern crate graphics;
extern crate opengl_graphics;
extern crate box2d;

pub mod camera;
pub use camera::Camera;
mod draw_session;
use draw_session::DrawSession;

use piston::event_loop::*;
use piston::input::{ Event, Input, Button, Motion, Key };
use piston::window::WindowSettings;
use glutin_window::*;
use opengl_graphics::GlGraphics;
use box2d::b2;

pub fn run<F>(name: &str, mut width: u32, mut height: u32,
              mut world: b2::World, mut camera: Camera, draw_flags: b2::DrawFlags,
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

    let transform = camera.transform_world_to_gl();
    let window_to_gl = |w, h, x, y| {
        let scale_x = 2. / w as f64;
        let scale_y = 2. / h as f64;
        (x*scale_x - 1., -1.*(y*scale_y - 1.))
    };
    let window_to_world = |w, h, camera: &Camera, x, y| {
        let (x, y) = window_to_gl(w, h, x, y);
        camera.gl_to_world(x, y)
    };

    let time_step = 1./60.;
    let velocity_iterations = 8;
    let position_iterations = 3;

    let mut running = false;
    let mut mouse_position = b2::Vec2 { x: 0., y: 0. };

    let mut accumulator = 0.;
    for event in window.events() {
        match event {
            Event::Input(i) => {
                match i {
                    Input::Press(Button::Keyboard(Key::Return)) =>
                        running = !running,
                    Input::Move(Motion::MouseCursor(x, y)) => {
                        mouse_position = window_to_world(width, height,
                                                         &camera, x, y);
                    }
                    Input::Resize(w, h) => {
                        width = w;
                        height = h;
                    }
                    _ => ()
                }
                process_input(&mut world, &mut camera, i);
            }
            Event::Update(args) => {
                accumulator += args.dt as f32;
                while accumulator >= time_step {
                    if running {
                        world.step(time_step,
                                   velocity_iterations,
                                   position_iterations);
                    }
                    accumulator -= time_step;
                }
            }
            Event::Render(args) => {
                graphics::clear([0.3, 0.3, 0.3, 1.0], &mut gl);
                DrawSession::new(&mut gl, args, transform,
                                 &mut world, draw_flags);
            }
            _ => {}
        }
    }
}

/*
fn try_grab(world: &mut b2::World, p: b2::Vec2,
            dummy: b2::BodyHandle) -> Option<b2::JointHandle> {
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

            let mut j_def = b2::MouseJointDef::new();
            j_def.base.body_a = Some(dummy);
            j_def.base.body_b = Some(body_h);
            j_def.target = center;
            j_def.max_force = 500.*mass;
            Some(world.create_joint(&j_def))
        }
    }
}

fn query_point(world: &b2::World, p: b2::Vec2) -> Option<b2::BodyHandle> {
    let d = b2::Vec2 { x: 0.001, y: 0.001 };
    let aabb = b2::AABB { lower: p - d, upper: p + d  };

    let mut result = None;
    {
        let mut callback = |body_h: b2::BodyHandle,
                            fixture_h: b2::FixtureHandle| {
            let body = world.get_body(body_h);
            let fixture = body.get_fixture(fixture_h);

            if body.body_type() != b2::BodyType::Static &&
               fixture.test_point(&p) {

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

fn ungrab(world: &mut b2::World, grabbing: &mut Option<b2::JointHandle>) {
    grabbing.take().map(|j| world.destroy_joint(j));
}

fn update_grab(world: &b2::World, target: b2::Vec2,
               grabbing: Option<b2::JointHandle>) {
    grabbing.map(|j| {
        let mut j = world.get_joint_mut(j);
        match **j {
            b2::UnknownJoint::Mouse(ref mut j) => {
                j.set_target(&target);
            }
            _ => { panic!("expected mouse joint"); }
        }
    });
}
*/
