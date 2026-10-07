use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, RollingOnRod},
    forces::Gravity,
    world::World,
};
use super::{disk, from_down, rod_between};

// A wheel rolls along the underside of a fixed overhead rail, with a
// two-link pendulum hanging from its axle: an overhead crane with a swinging
// load. The wheel's position never enters the equations of motion, so its
// conjugate momentum is conserved; released from rest it shuttles back and
// forth instead of drifting off, while the double pendulum below goes
// chaotic and keeps shoving it around.

const RAIL_Y:  f32 = 2.6;
const RAIL_HL: f32 = 3.2;
const RAIL_HW: f32 = 0.08;
const WHEEL_R: f32 = 0.3;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let (rail_body, _) = rod_between(
        Vec2::new(-RAIL_HL, RAIL_Y), Vec2::new(RAIL_HL, RAIL_Y), 4.0, RAIL_HW,
    );
    let rail = w.add_body(rail_body.fixed());

    // Wheel hanging under the rail.
    let axle = Vec2::new(0.0, RAIL_Y - RAIL_HW - WHEEL_R);
    let wheel = w.add_body(disk(axle, WHEEL_R, 1.5));
    w.add_constraint(RollingOnRod::new(wheel, rail, &w.bodies).expect("disk on rod"));

    // Two-link load, released swung out to one side.
    let elbow = axle + from_down(1.25) * 1.1;
    let hook  = elbow + from_down(1.9) * 0.9;
    let (upper_body, hl1) = rod_between(axle, elbow, 0.8, 0.06);
    let upper = w.add_body(upper_body);
    let (lower_body, hl2) = rod_between(elbow, hook, 0.6, 0.055);
    let lower = w.add_body(lower_body);
    let load = w.add_body(disk(hook, 0.22, 1.2));

    w.add_constraint(PinJoint::new(wheel, Vec2::ZERO, upper, Vec2::new(-hl1, 0.0)));
    w.add_constraint(PinJoint::new(upper, Vec2::new(hl1, 0.0), lower, Vec2::new(-hl2, 0.0)));
    w.add_constraint(PinJoint::new(lower, Vec2::new(hl2, 0.0), load, Vec2::ZERO));

    w.add_tracer(load, Vec2::ZERO);
    w
}
