use glam::Vec2;

use crate::sim::{
    body::Body,
    forces::Gravity,
    world::World,
};
use super::{disk, rod_between};

// A Galton board: a hopper of balls drains through a funnel onto rows of
// pegs; at each peg a ball goes left or right more or less at random, and
// the bins at the bottom fill into a bell curve (a binomial distribution).
// Everything collides: fixed pegs, walls and bin dividers, and the balls
// with each other, which the contact broad phase keeps cheap.

const BALL_R: f32 = 0.03;
const BALLS: usize = 200;
const PEG_R: f32 = 0.03;
const PITCH: f32 = 0.19;       // peg spacing across
const ROW_H: f32 = 0.165;      // peg spacing down
const ROWS: usize = 11;
const TOP: f32 = 1.75;         // first peg row
const HALF_W: f32 = 1.42;      // inner half width of the board
const BIN_TOP: f32 = -0.25;
const FLOOR: f32 = -1.85;
const WALL: f32 = 0.04;

fn scenery(b: Body) -> Body {
    let mut b = b.fixed().colliding();
    b.friction = 0.15;
    b.restitution = 0.35;
    b
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let wall_w = |w: &mut World, a: Vec2, b: Vec2, hw: f32| {
        let (body, _) = rod_between(a, b, 5.0, hw);
        w.add_body(scenery(body));
    };
    let wall = |w: &mut World, a: Vec2, b: Vec2| wall_w(w, a, b, WALL);

    // Board sides, floor, hopper and funnel.
    wall(&mut w, Vec2::new(-HALF_W - WALL, FLOOR), Vec2::new(-HALF_W - WALL, TOP + 0.35));
    wall(&mut w, Vec2::new(HALF_W + WALL, FLOOR), Vec2::new(HALF_W + WALL, TOP + 0.35));
    wall(&mut w, Vec2::new(-HALF_W - WALL, FLOOR - WALL), Vec2::new(HALF_W + WALL, FLOOR - WALL));
    let mouth = 0.15;
    let funnel_y = TOP + 0.3;
    wall(&mut w, Vec2::new(-1.35, 3.55), Vec2::new(-mouth - WALL, funnel_y));
    wall(&mut w, Vec2::new(1.35, 3.55), Vec2::new(mouth + WALL, funnel_y));

    // Pegs: rows offset by half a pitch, clear of the side walls.
    for r in 0..ROWS {
        let y = TOP - r as f32 * ROW_H;
        let shift = if r % 2 == 0 { 0.0 } else { 0.5 * PITCH };
        let mut x = -((HALF_W / PITCH).floor()) * PITCH + shift;
        while x < HALF_W - 0.06 {
            if x > -HALF_W + 0.06 {
                w.add_body(scenery(disk(Vec2::new(x, y), PEG_R, 1.0)));
            }
            x += PITCH;
        }
    }

    // Bin dividers under the gaps of the last row.
    let last_shift = if (ROWS - 1).is_multiple_of(2) { 0.5 * PITCH } else { 0.0 };
    let mut x = -((HALF_W / PITCH).floor()) * PITCH + last_shift;
    while x < HALF_W {
        if x > -HALF_W + 0.05 {
            wall_w(&mut w, Vec2::new(x, FLOOR), Vec2::new(x, BIN_TOP), 0.012);
        }
        x += PITCH;
    }

    // The balls, packed in the hopper above the funnel.
    let mut n = 0;
    let gap = 2.0 * BALL_R + 0.012;
    let mut row = 0;
    let mut y = funnel_y + 0.25;
    while n < BALLS && y < 4.6 {
        // Inside the funnel's slope (and the hopper walls above it).
        let slope_half = mouth + (y - funnel_y) * (1.35 - mouth) / (3.55 - funnel_y);
        let half = slope_half.min(1.3) - BALL_R - 0.05;
        let shift = if row % 2 == 0 { 0.0 } else { 0.5 * gap };
        let mut x = -half + shift;
        while x <= half && n < BALLS {
            let mut b = disk(Vec2::new(x, y), BALL_R, 0.03).colliding();
            b.friction = 0.15;
            b.restitution = 0.35;
            w.add_body(b);
            n += 1;
            x += gap;
        }
        y += gap * 0.87;
        row += 1;
    }
    // Hopper walls above the funnel top.
    wall(&mut w, Vec2::new(-1.35, 3.55), Vec2::new(-1.35, 4.8));
    wall(&mut w, Vec2::new(1.35, 3.55), Vec2::new(1.35, 4.8));
    w
}
