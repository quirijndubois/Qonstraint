use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, PinWorld},
    forces::Gravity,
    world::World,
};
use super::{disk, from_down, rod_between};

// Two rigid links and a heavy bob, released from high up. Sensitive
// dependence on initial conditions: no two runs of the trace look alike
// once you nudge it with the mouse.

const PIVOT: Vec2 = Vec2::new(0.0, 2.2);
const LINK:  f32  = 1.4;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let elbow = PIVOT + from_down(2.1) * LINK;
    let hand  = elbow + from_down(2.75) * LINK;

    let (upper_body, hl) = rod_between(PIVOT, elbow, 1.0, 0.065);
    let upper = w.add_body(upper_body);
    let (lower_body, _) = rod_between(elbow, hand, 1.0, 0.065);
    let lower = w.add_body(lower_body);
    let bob = w.add_body(disk(hand, 0.24, 1.5));

    w.add_constraint(PinWorld::new(upper, Vec2::new(-hl, 0.0), PIVOT));
    w.add_constraint(PinJoint::new(upper, Vec2::new(hl, 0.0), lower, Vec2::new(-hl, 0.0)));
    w.add_constraint(PinJoint::new(lower, Vec2::new(hl, 0.0), bob, Vec2::ZERO));

    w.add_tracer(bob, Vec2::ZERO);
    w
}
