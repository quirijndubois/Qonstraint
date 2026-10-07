use glam::Vec2;

use crate::sim::{
    constraints::PinJoint,
    forces::{Gravity, TorsionSpring},
    world::World,
};
use super::{anchor, disk, rod_between};

// James Watt's parallel motion (1784): two equal arms pivoting from
// opposite sides, joined by a short coupler. The coupler's midpoint moves
// in a nearly straight line over a good stretch, curling into a figure
// eight at the ends (the red trace). Torsion springs at the two pivots
// make it bounce up and down; they start wound against each other.

const ARM: f32 = 1.7;
const HALF_COUPLER: f32 = 0.55;
const SPRING_K: f32 = 26.0;
const WIND: f32 = 0.55; // rest angle offset of the springs (rad)

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let top = Vec2::new(0.0, HALF_COUPLER);
    let bottom = Vec2::new(0.0, -HALF_COUPLER);
    let left_pivot = top - Vec2::X * ARM;
    let right_pivot = bottom + Vec2::X * ARM;

    let lp = w.add_body(anchor(left_pivot));
    let rp = w.add_body(anchor(right_pivot));
    let (left_body, hl) = rod_between(left_pivot, top, 1.0, 0.06);
    let left = w.add_body(left_body);
    let (right_body, hr) = rod_between(right_pivot, bottom, 1.0, 0.06);
    let right = w.add_body(right_body);
    let (coupler_body, hc) = rod_between(top, bottom, 0.6, 0.06);
    let coupler = w.add_body(coupler_body);
    let pen = w.add_body(disk(Vec2::ZERO, 0.14, 1.5));

    w.add_constraint(PinJoint::new(left, Vec2::new(-hl, 0.0), lp, Vec2::ZERO));
    w.add_constraint(PinJoint::new(right, Vec2::new(-hr, 0.0), rp, Vec2::ZERO));
    w.add_constraint(PinJoint::new(left, Vec2::new(hl, 0.0), coupler, Vec2::new(-hc, 0.0)));
    w.add_constraint(PinJoint::new(right, Vec2::new(hr, 0.0), coupler, Vec2::new(hc, 0.0)));
    w.add_constraint(PinJoint::new(coupler, Vec2::ZERO, pen, Vec2::ZERO));

    // Both springs want the midpoint up: the left arm turned
    // counter-clockwise, the right one (pointing left) clockwise.
    let mut ls = TorsionSpring::new(left, lp, Vec2::new(-hl, 0.0), SPRING_K, 0.02, &w.bodies);
    ls.rest += WIND;
    w.add_force(ls);
    let mut rs = TorsionSpring::new(right, rp, Vec2::new(-hr, 0.0), SPRING_K, 0.02, &w.bodies);
    rs.rest -= WIND;
    w.add_force(rs);

    let t = w.tracers.len();
    w.add_tracer(pen, Vec2::ZERO);
    w.tracers[t].seconds = 6.0;
    w
}
