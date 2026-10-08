use std::f32::consts::TAU;
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::DistanceConstraint,
    forces::Gravity,
    world::World,
};
use super::{disk, from_down};

// Fifteen pendulums on one bar, lengths tuned so that in PERIOD seconds
// the longest swings FIRST times and each next one swings once more. All
// start together; they drift out of step into travelling waves, snakes and
// apparent chaos, then line up again after PERIOD seconds. Each bob is a
// disk on a massless distance link (an ideal pendulum).

const N: usize = 15;
const PERIOD: f32 = 30.0;
const FIRST: f32 = 15.0;
const G: f32 = 9.81;
const BAR_Y: f32 = 1.3;
const SPACING: f32 = 0.3;
const BOB_R: f32 = 0.075;
const START: f32 = 0.16; // release angle (rad)

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(G));

    let half = 0.5 * (N - 1) as f32 * SPACING + 0.25;
    let bar = w.add_body(Body::new(
        Vec2::new(0.0, BAR_Y), 0.0, 5.0, rod_inertia(5.0, half),
        BodyShape::Rod { half_len: half, half_width: 0.05 },
    ).fixed());

    for k in 0..N {
        // Swings per PERIOD = FIRST + k  →  T = PERIOD / (FIRST + k).
        let period = PERIOD / (FIRST + k as f32);
        let len = G * (period / TAU).powi(2);
        let x = (k as f32 - 0.5 * (N - 1) as f32) * SPACING;
        let top = Vec2::new(x, BAR_Y);
        let bob = w.add_body(disk(top + from_down(START) * len, BOB_R, 1.0));
        w.add_constraint(DistanceConstraint::new(bar, Vec2::new(x, 0.0), bob, Vec2::ZERO, len));
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    /// After one full PERIOD every pendulum is back near its release angle
    /// (in step again), while halfway they are spread out.
    #[test]
    fn realigns_after_period() {
        let mut w = build();
        let dt = 1.0 / 480.0;
        let angle = |w: &World, k: usize| {
            let b = &w.bodies[k + 1];
            let x = (k as f32 - 0.5 * (N - 1) as f32) * SPACING;
            let d = b.pos32() - Vec2::new(x, BAR_Y);
            d.x.atan2(-d.y)
        };
        let steps = (PERIOD / dt) as usize;
        let mut spread_mid = 0.0f32;
        for s in 0..steps {
            w.step(dt);
            if s == steps / 2 {
                spread_mid = (0..N).map(|k| angle(&w, k)).fold(0.0, |m, a| m.max((a - START).abs()));
            }
        }
        // Finite amplitude lengthens every period alike (by ~θ²/16), so the
        // set comes back in step slightly late: compare with each other.
        let a: Vec<f32> = (0..N).map(|k| angle(&w, k)).collect();
        let spread_end = a.iter().fold(0.0f32, |m, &x| m.max((x - a[0]).abs()));
        eprintln!("spread halfway {spread_mid:.3}, after one period {spread_end:.3}");
        assert!(spread_mid > 0.2);
        assert!(spread_end < 0.06);
    }
}
