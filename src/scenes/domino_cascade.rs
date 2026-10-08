use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{DistanceConstraint, WeldJoint},
    forces::Gravity,
    world::World,
};
use super::{disk, from_down, rod_between};

// Domino amplification: each domino is GROWTH times the size of the last,
// so a tap from a small pendulum topples a domino a hundred times heavier
// at the end of the line. A domino is an upright bar welded onto a flat
// foot (a capsule standing on its round end would roll over by itself);
// the foot gives it a flat base it can stand on and tip over.

const N: usize = 10;
const GROWTH: f32 = 1.32;
const H0: f32 = 0.16;
const FLOOR_Y: f32 = -1.0;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let (floor, _) = rod_between(Vec2::new(-3.6, FLOOR_Y - 0.05), Vec2::new(4.0, FLOOR_Y - 0.05), 10.0, 0.05);
    let mut floor = floor.fixed().colliding();
    floor.friction = 0.7;
    floor.restitution = 0.05;
    w.add_body(floor);

    let mut x = -3.0;
    for k in 0..N {
        let h = H0 * GROWTH.powi(k as i32);
        let t = 0.19 * h; // thickness
        let mass = 4.0 * h * t;
        // Upright bar: a capsule from just above the foot to the top.
        let hw = 0.5 * t;
        let foot_h = 0.012 * h.max(0.5);
        let base = FLOOR_Y + 2.0 * foot_h;
        let bar = Body::new(
            Vec2::new(x, base + 0.5 * h), std::f32::consts::FRAC_PI_2, mass,
            rod_inertia(mass, 0.5 * h) + mass * hw * hw / 3.0,
            BodyShape::Rod { half_len: 0.5 * h - hw, half_width: hw },
        );
        let mut bar = bar.colliding();
        bar.friction = 0.5;
        bar.restitution = 0.05;
        let bar = w.add_body(bar);
        // (Welded to the bar, the foot's own inertia barely matters; keep it
        // above the editor's floor for tiny parts.)
        let foot = Body::new(
            Vec2::new(x, FLOOR_Y + foot_h), 0.0, 0.2 * mass, rod_inertia(0.2 * mass, 0.5 * t).max(2e-6),
            BodyShape::Rod { half_len: 0.5 * t, half_width: foot_h },
        );
        let mut foot = foot.colliding();
        foot.friction = 0.7;
        foot.restitution = 0.05;
        let foot = w.add_body(foot);
        w.add_constraint(WeldJoint::new(bar, foot, Vec2::new(x, FLOOR_Y + foot_h), &w.bodies));
        // Next one a bit more than half this one's height away.
        x += 0.58 * h + 0.5 * t + 0.5 * t * GROWTH;
    }

    // The trigger: a small pendulum swinging in from the left.
    let pivot = Vec2::new(-3.0 - 0.06, FLOOR_Y + H0 + 0.75);
    let top = w.add_body(super::anchor(pivot));
    let len = 0.72;
    let bob = w.add_body(disk(pivot + from_down(-1.1) * len, 0.05, 0.04).colliding());
    w.add_constraint(DistanceConstraint::new(top, Vec2::ZERO, bob, Vec2::ZERO, len));
    w
}
