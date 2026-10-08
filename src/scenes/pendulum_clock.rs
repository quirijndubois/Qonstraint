use std::f32::consts::{FRAC_PI_4, TAU};
use glam::Vec2;

use crate::sim::{
    body::Body,
    constraints::{GearJoint, GearKind, PinWorld, Rope, WeldJoint},
    forces::Gravity,
    world::World,
};
use super::{disk, rod_between};

// A weight-driven pendulum clock with an anchor escapement, every part a
// plain editor part. The escape wheel is a hub with eighteen spike teeth
// welded on; the anchor is two arms welded to the pendulum rod, each
// ending in a pallet, a short bar set slantwise so its face is an
// inclined plane. As the pendulum swings the pallets dip into the teeth in
// turn: a tooth tip sliding along a pallet face pushes it out (the impulse
// that keeps the pendulum going), slips off its end, and the wheel jumps
// until a tooth lands on the other pallet, half a tooth later; one tooth
// per swing. The power comes from an endless rope over a drum (a
// heavy weight on one side, a light one on the other, Huygens' trick)
// belted to the escape wheel.

const E: Vec2 = Vec2::new(0.0, 0.6);      // escape wheel axle
const TEETH: usize = 18;
const ROOT_R: f32 = 0.36;
const TIP_R: f32 = 0.5;
const TOOTH_HW: f32 = 0.014;
const PALLET_HW: f32 = 0.016;
/// Half the radial depth of a pallet face: the lift it gives.
const LIFT: f32 = 0.07;
const PIN_AT: f32 = 0.57;                  // pallets' distance from the axle, anchor level
const PENDULUM: f32 = 1.6;
const START: f32 = 0.15;                  // pendulum released from (rad)
const FRICTION: f32 = 0.0;
const BOUNCE: f32 = 0.6;
const DRUM: Vec2 = Vec2::new(1.25, 0.6);

/// The two pallets (end points) for an anchor turned by `swing` about
/// `pivot`. Each sits on the tangent from the pivot to the wheel, PIN_AT
/// out, slanted at 45° to the rim with its trailing end deeper in, so a
/// tooth tip sliding along its face works it outwards over 2·LIFT: the
/// impulse. With the pallets this far out, a swing of about ±0.06 rad lets
/// one go while the other is already deep enough to catch the next tooth.
fn pallets(pivot: Vec2, swing: f32) -> [(Vec2, Vec2); 2] {
    let rot = |p: Vec2| {
        let (s, c) = swing.sin_cos();
        let d = p - pivot;
        pivot + Vec2::new(c * d.x - s * d.y, s * d.x + c * d.y)
    };
    [-3.0 * FRAC_PI_4, -FRAC_PI_4].map(|a| {
        let out = Vec2::new(a.cos(), a.sin());
        let ahead = Vec2::new(a.sin(), -a.cos()); // clockwise travel
        let along = (-ahead - out) * LIFT; // from the trailing end to the leading one
        let c = E + out * PIN_AT;
        (rot(c - along), rot(c + along))
    })
}

/// A tooth's two ends for a wheel at angle `turn`.
fn tooth(k: usize, turn: f32) -> (Vec2, Vec2) {
    let a = turn + TAU * k as f32 / TEETH as f32;
    let u = Vec2::new(a.cos(), a.sin());
    (E + u * ROOT_R, E + u * TIP_R)
}

fn seg_seg(a: (Vec2, Vec2), b: (Vec2, Vec2)) -> f32 {
    [seg_dist(a.0, b.0, b.1), seg_dist(a.1, b.0, b.1), seg_dist(b.0, a.0, a.1), seg_dist(b.1, a.0, a.1)]
        .into_iter().fold(f32::INFINITY, f32::min)
}

fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (a + ab * t - p).length()
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    // Anchor pivot below the wheel, on the line through the pins' tangents.
    let pivot = E - Vec2::Y * PIN_AT * std::f32::consts::SQRT_2;
    let at = pallets(pivot, START);

    // Turn the wheel so neither pallet starts inside a tooth.
    let clearance = |turn: f32| (0..TEETH).flat_map(|k| {
        let t = tooth(k, turn);
        at.map(|p| seg_seg(p, t))
    }).fold(f32::INFINITY, f32::min);
    let turn = (0..400).map(|i| i as f32 * TAU / TEETH as f32 / 400.0)
        .max_by(|&x, &y| clearance(x).total_cmp(&clearance(y))).unwrap();

    let hard = |mut b: Body| { b.collide = true; b.friction = FRICTION; b.restitution = BOUNCE; b };

    let hub = w.add_body(disk(E, ROOT_R, 0.3));
    w.add_constraint(PinWorld::new(hub, Vec2::ZERO, E));
    for k in 0..TEETH {
        let (a, b) = tooth(k, turn);
        let (t, _) = rod_between(a, b, 0.02, TOOTH_HW);
        let t = w.add_body(hard(t));
        w.add_constraint(WeldJoint::new(hub, t, a, &w.bodies));
    }

    // Pendulum and anchor, one welded piece hanging from the pivot.
    let down = Vec2::new(START.sin(), -START.cos());
    let (rod, hl) = rod_between(pivot, pivot + down * PENDULUM, 0.4, 0.03);
    let rod = w.add_body(rod);
    w.add_constraint(PinWorld::new(rod, Vec2::new(-hl, 0.0), pivot));
    let bob_at = pivot + down * PENDULUM;
    let bob = w.add_body(disk(bob_at, 0.17, 3.0));
    w.add_constraint(WeldJoint::new(rod, bob, bob_at, &w.bodies));
    for (a, b) in at {
        let mid = 0.5 * (a + b);
        let (arm, _) = rod_between(pivot, mid, 0.08, 0.025);
        let arm = w.add_body(arm);
        w.add_constraint(WeldJoint::new(rod, arm, pivot, &w.bodies));
        let (pallet, _) = rod_between(a, b, 0.03, PALLET_HW);
        let pallet = w.add_body(hard(pallet));
        w.add_constraint(WeldJoint::new(arm, pallet, mid, &w.bodies));
    }
    let t = w.tracers.len();
    w.add_tracer(bob, Vec2::ZERO);
    w.tracers[t].seconds = 2.0;

    // Weight drive: drum belted to the escape wheel, endless rope over it.
    // A small pinion on the escape arbor takes the belt, so the weights
    // fall slowly (rope speed = pinion radius × wheel speed).
    let pinion = w.add_body(disk(E, 0.08, 0.1));
    w.add_constraint(WeldJoint::new(hub, pinion, E, &w.bodies));
    let drum = w.add_body(disk(DRUM, 0.24, 0.5));
    w.add_constraint(PinWorld::new(drum, Vec2::ZERO, DRUM));
    w.add_constraint(GearJoint::new(drum, pinion, GearKind::Belt, &w.bodies).expect("disks"));
    let heavy = w.add_body(disk(DRUM + Vec2::new(0.24, -0.9), 0.13, 1.6));
    let light = w.add_body(disk(DRUM + Vec2::new(-0.24, -1.3), 0.09, 0.6));
    let rope = Rope::new(light, Vec2::ZERO, heavy, Vec2::ZERO, &w.bodies).over(drum, &w.bodies).expect("rope over drum");
    w.add_constraint(rope);
    w
}
