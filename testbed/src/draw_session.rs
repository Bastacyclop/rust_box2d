use piston::event::RenderArgs;
use graphics::{ Context, Line, Ellipse, Polygon };
use opengl_graphics::GlGraphics;
use box2d::b2;

pub struct DrawSession<'a> {
    context: Context,
    gl: &'a mut GlGraphics,
    transform: [[f64; 3]; 2]
}

impl<'a> DrawSession<'a> {
    pub fn new(gl: &mut GlGraphics, args: RenderArgs, transform: [[f64; 3]; 2],
               world: &mut b2::World, flags: b2::DrawFlags) {

        gl.draw(args.viewport(), |c, gl| {
            let mut session = DrawSession {
                context: c,
                gl: gl,
                transform: transform
            };
            world.draw_debug_data(&mut session, flags);
        })
    }
}

impl<'a> b2::Draw for DrawSession<'a> {
    fn draw_polygon(&mut self, vertices: &[b2::Vec2], color: &b2::Color) {
        let count = vertices.len();
        for i in 0..count {
            let a = &vertices[i];
            let b = &vertices[(i + 1) % count];
            self.draw_segment(a, b, color);
        }
    }

    fn draw_solid_polygon(&mut self, vertices: &[b2::Vec2], color: &b2::Color) {
        Polygon::new(convert_color(color))
            .draw(&polygon(vertices), &self.context.draw_state, self.transform, self.gl);
    }

    fn draw_circle(&mut self, o: &b2::Vec2, r: f32, color: &b2::Color) {
        Ellipse::new_border(convert_color(color), 0.25)
            .draw(circle_rect(o, r), &self.context.draw_state, self.transform, self.gl);
    }

    fn draw_solid_circle(&mut self, o: &b2::Vec2, r: f32,
                                    axis: &b2::Vec2, color: &b2::Color) {
        Ellipse::new(convert_color(color))
            .draw(circle_rect(o, r), &self.context.draw_state, self.transform, self.gl);
        self.draw_segment(o, &(o + axis*r), &BLUE);
    }

    fn draw_segment(&mut self, p1: &b2::Vec2, p2: &b2::Vec2, color: &b2::Color) {
        let line = [p1.x as f64, p1.y as f64,
                    p2.x as f64, p2.y as f64];
        Line::new(convert_color(color), 0.05)
            .draw(line, &self.context.draw_state, self.transform, self.gl);
    }

    fn draw_transform(&mut self, xf: &b2::Transform) {
        const AXIS_SCALE: f32 = 1.2;

        let p1 = xf.pos;
        let p2 = p1 + xf.rot.x_axis()*AXIS_SCALE;
        let p3 = p1 + xf.rot.y_axis()*AXIS_SCALE;

        self.draw_segment(&p1, &p2, &RED);
        self.draw_segment(&p1, &p3, &GREEN);
    }
}

fn polygon(vertices: &[b2::Vec2]) -> Vec<[f64; 2]> {
    vertices.iter().map(|v| [v.x as f64, v.y as f64]).collect()
}

fn circle_rect(o: &b2::Vec2, r: f32) -> [f64; 4] {
    let r = r as f64;
    let d = 2.*r;
    [o.x as f64 - r, o.y as f64 - r, d, d]
}

fn convert_color(c: &b2::Color) -> [f32; 4] {
    [c.r, c.g, c.b, c.a]
}

const RED: b2::Color = b2::Color { r: 1., g: 0., b: 0., a: 1. };
const GREEN: b2::Color = b2::Color { r: 0., g: 1., b: 0., a: 1. };
const BLUE: b2::Color = b2::Color { r: 0., g: 0., b: 1., a: 1. };
