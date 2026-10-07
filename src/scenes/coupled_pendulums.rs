use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, PinWorld},
    forces::{Gravity, SpringDamper},
    world::World,
};
use super::{disk, from_down, rod_between};

// Two identical pendulums joined by a weak spring. Only the left one is
// released; the swing drains into the right one and back again (beats at
// the difference of the two normal-mode frequencies). Calm and periodic.

const SPACING: f32 = 2.0;
const PIVOT_Y: f32 = 2.8;
const LINK:    f32 = 1.9;
const SPRING_AT: f32 = 0.8; // distance down each rod where the spring attaches

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let mut rods = [0usize; 2];
    let mut hl = 0.0;
    for (i, (x, phi)) in [(-SPACING * 0.5, 0.45f32), (SPACING * 0.5, 0.0)].into_iter().enumerate() {
        let pivot = Vec2::new(x, PIVOT_Y);
        let end = pivot + from_down(phi) * LINK;
        let (rod_body, h) = rod_between(pivot, end, 0.6, 0.06);
        hl = h;
        let rod = w.add_body(rod_body);
        let bob = w.add_body(disk(end, 0.3, 2.0));
        w.add_constraint(PinWorld::new(rod, Vec2::new(-hl, 0.0), pivot));
        w.add_constraint(PinJoint::new(rod, Vec2::new(hl, 0.0), bob, Vec2::ZERO));
        rods[i] = rod;
    }

    // The released pendulum starts stretched against the spring; rest length
    // is the pivot spacing, so the spring is relaxed when both hang straight.
    let at = Vec2::new(-hl + SPRING_AT, 0.0);
    w.add_force(SpringDamper::new(rods[0], at, rods[1], at, SPACING, 6.0, 0.0));
    w
}
