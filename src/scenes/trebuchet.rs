use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{PinJoint, Rope},
    forces::Gravity,
    world::World,
};
use super::{disk, rod_between};

// A counterweight trebuchet. The hinged counterweight drops and swings the
// long arm up and over; the sling (a rope) drags the stone along the
// trough, lifts it and whips it round. The sling is a breakable rope: once
// its pull passes RELEASE newtons, late in the whip, it lets go and the
// stone flies some twenty metres, high over the field, into the wall of
// blocks downrange.

const PIVOT: Vec2 = Vec2::new(0.0, 2.2);
const SHORT: f32 = 0.9;
const LONG: f32 = 2.9;
const COCKED: f32 = 3.75;        // long arm's starting angle (rad), down and back
const WEIGHT: f32 = 32.0;
const STONE: f32 = 2.0;
const SLING: f32 = 2.4;
const RELEASE: f32 = 200.0;      // N
const GROUND: f32 = 0.0;
const WALL_X: f32 = 21.0;

fn solid(mut b: Body, friction: f32, bounce: f32) -> Body {
    b.collide = true;
    b.friction = friction;
    b.restitution = bounce;
    b
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let (ground, _) = rod_between(Vec2::new(-6.0, GROUND - 0.06), Vec2::new(27.0, GROUND - 0.06), 50.0, 0.06);
    w.add_body(solid(ground.fixed(), 0.6, 0.2));
    let (stop, _) = rod_between(Vec2::new(27.0, GROUND), Vec2::new(27.0, GROUND + 1.5), 10.0, 0.08);
    w.add_body(solid(stop.fixed(), 0.6, 0.2));
    // A-frame legs (scenery).
    for dx in [-1.1, 1.1] {
        let (leg, _) = rod_between(Vec2::new(dx, GROUND), PIVOT, 10.0, 0.07);
        w.add_body(leg.fixed());
    }

    // The arm, cocked with its long end down behind the frame.
    let u = Vec2::new(COCKED.cos(), COCKED.sin());
    let long_end = PIVOT + u * LONG;
    let short_end = PIVOT - u * SHORT;
    let (arm, hl) = rod_between(short_end, long_end, 2.0, 0.08);
    let arm = w.add_body(arm);
    let pivot_local = Vec2::new(-hl + SHORT, 0.0);
    w.add_constraint(crate::sim::constraints::PinWorld::new(arm, pivot_local, PIVOT));

    // Hinged counterweight hanging from the short end.
    let hanger_len = 0.7;
    let (hanger, hh) = rod_between(short_end, short_end - Vec2::Y * hanger_len, 2.0, 0.05);
    let hanger = w.add_body(hanger);
    w.add_constraint(PinJoint::new(arm, Vec2::new(-hl, 0.0), hanger, Vec2::new(-hh, 0.0)));
    let box_at = short_end - Vec2::Y * (hanger_len + 0.3);
    let m = WEIGHT;
    let weight = w.add_body(Body::new(box_at, 0.0, m, rod_inertia(m, 0.35) + m * 0.3 * 0.3 / 3.0,
        BodyShape::Rod { half_len: 0.35, half_width: 0.3 }));
    w.add_constraint(crate::sim::constraints::WeldJoint::new(hanger, weight, short_end - Vec2::Y * hanger_len, &w.bodies));

    // The stone lies in the trough under the frame, the sling taut to it.
    let stone_r = 0.13;
    let stone_at = Vec2::new(long_end.x + (SLING * SLING - (long_end.y - stone_r - GROUND).powi(2)).max(0.0).sqrt(), GROUND + stone_r);
    let stone = w.add_body(solid(disk(stone_at, stone_r, STONE), 0.3, 0.3));
    w.add_constraint(Rope::new(arm, Vec2::new(hl, 0.0), stone, Vec2::ZERO, &w.bodies));
    w.break_last_at(RELEASE);
    let t = w.tracers.len();
    w.add_tracer(stone, Vec2::ZERO);
    w.tracers[t].seconds = 4.0;

    // The wall: blocks laid in a stretcher bond.
    let (bw, bh) = (0.5, 0.24);
    for row in 0..10 {
        let shift = if row % 2 == 0 { 0.0 } else { 0.5 * bw };
        for col in 0..4 {
            let x = WALL_X + shift + col as f32 * (bw + 0.004);
            let y = GROUND + 0.5 * bh + row as f32 * (bh + 0.002);
            let block = Body::new(Vec2::new(x, y), 0.0, 1.2, rod_inertia(1.2, 0.5 * bw - 0.5 * bh) + 1.2 * bh * bh / 12.0,
                BodyShape::Rod { half_len: 0.5 * (bw - bh), half_width: 0.5 * bh });
            w.add_body(solid(block, 0.7, 0.1));
        }
    }
    w
}
