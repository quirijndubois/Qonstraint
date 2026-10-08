use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{PinJoint, SliderJoint, WeldJoint},
    forces::{spring::SpringDamper, Gravity},
    world::World,
};
use super::{disk, from_down, rod_between};

// Barton's pendulums: a heavy driver and a row of light pendulums of
// different lengths hang from one beam that can slide sideways on a rail,
// held by springs. The driver's swing shakes the beam; each light
// pendulum is forced at the driver's frequency, and only the one whose
// length matches (the fourth) resonates and swings up, while the shorter
// and longer ones barely move. Every pendulum is a rod with its bob welded
// on. Nothing is damped, so the resonant one hands its swing back too.

const BEAM_Y: f32 = 1.4;
const DRIVER_LEN: f32 = 1.0;
/// Light pendulum lengths; the `MATCHED` one is retuned to the driver.
const LENGTHS: [f32; 7] = [0.45, 0.6, 0.8, 1.05, 1.35, 1.65, 1.95];
/// Each centring spring's stiffness.
const SPRING_K: f32 = 400.0;
const MATCHED: usize = 3;
const ROD_MASS: f32 = 0.03;
const DRIVER: (f32, f32) = (0.17, 6.0);  // bob radius, mass
const LIGHT: (f32, f32) = (0.07, 0.12);
const SPACING: f32 = 0.5;
const DRIVER_X: f32 = -2.0;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    // Rail and the sliding beam, centred by two springs.
    let (rail_body, _) = rod_between(Vec2::new(-2.8, BEAM_Y + 0.35), Vec2::new(2.8, BEAM_Y + 0.35), 5.0, 0.06);
    let rail = w.add_body(rail_body.fixed());
    let half = 2.4;
    let beam = w.add_body(Body::new(
        Vec2::new(0.0, BEAM_Y), 0.0, 1.5, rod_inertia(1.5, half),
        BodyShape::Rod { half_len: half, half_width: 0.05 },
    ));
    // Hung from the rail by two carriages, locked level.
    for x in [-1.2, 1.2] {
        let mut s = SliderJoint::new(beam, Vec2::new(x, 0.35), rail, &w.bodies).expect("rod rail");
        s.lock_rotation = true;
        w.add_constraint(s);
    }
    let post = |w: &mut World, x: f32| {
        w.add_body(super::anchor(Vec2::new(x, BEAM_Y)))
    };
    let (left, right) = (post(&mut w, -3.3), post(&mut w, 3.3));
    w.add_force(SpringDamper::new(left, Vec2::ZERO, beam, Vec2::new(-half, 0.0), 0.9, SPRING_K, 0.0));
    w.add_force(SpringDamper::new(right, Vec2::ZERO, beam, Vec2::new(half, 0.0), 0.9, SPRING_K, 0.0));

    let hang = |w: &mut World, x: f32, len: f32, bob_r: f32, bob_m: f32, angle: f32| {
        let top = Vec2::new(x, BEAM_Y);
        let tip = top + from_down(angle) * len;
        let (rod_body, hl) = rod_between(top, tip, ROD_MASS, 0.022);
        let rod = w.add_body(rod_body);
        w.add_constraint(PinJoint::new(beam, Vec2::new(x, 0.0), rod, Vec2::new(-hl, 0.0)));
        let bob = w.add_body(disk(tip, bob_r, bob_m));
        w.add_constraint(WeldJoint::new(rod, bob, tip, &w.bodies));
        bob
    };

    let driver = hang(&mut w, DRIVER_X, DRIVER_LEN, DRIVER.0, DRIVER.1, 0.35);
    w.add_tracer(driver, Vec2::ZERO);
    // The beam gives a little under the driver's swing, which slows it as
    // if it were longer by m·g / k (the springs in parallel).
    let target = equivalent_length(DRIVER_LEN, DRIVER) + DRIVER.1 * 9.81 / (2.0 * SPRING_K);
    for (k, &len) in LENGTHS.iter().enumerate() {
        let x = DRIVER_X + SPACING * (k as f32 + 1.0) + 0.05;
        let len = if k == MATCHED { matching_length(target) } else { len };
        let bob = hang(&mut w, x, len, LIGHT.0, LIGHT.1, 0.0);
        if k == MATCHED {
            let t = w.tracers.len();
            w.add_tracer(bob, Vec2::ZERO);
            w.tracers[t].seconds = 2.0;
        }
    }
    w
}

/// Length of the simple pendulum that swings like a rod of length `len`
/// with a welded disk bob (radius, mass) at its end: I / (m·d).
fn equivalent_length(len: f32, (r, m): (f32, f32)) -> f32 {
    let inertia = ROD_MASS * len * len / 3.0 + m * len * len + 0.5 * m * r * r;
    inertia / (ROD_MASS * len * 0.5 + m * len)
}

/// The light pendulum's length that swings like `target`.
fn matching_length(target: f32) -> f32 {
    let (mut lo, mut hi) = (0.5 * target, 2.0 * target);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if equivalent_length(mid, LIGHT) < target { lo = mid } else { hi = mid }
    }
    0.5 * (lo + hi)
}
