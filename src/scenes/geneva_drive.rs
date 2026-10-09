use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};
use glam::Vec2;

use crate::sim::{
    body::Body,
    constraints::{GearJoint, GearKind, PinJoint, PinWorld, WeldJoint},
    forces::{spring::SpringDamper, Gravity, Motor},
    world::World,
};
use super::{anchor, disk, rod_between};

// Intermittent motion from steady rotation, two ways, off one motor.
// Geneva drive: the drive crank's pin enters a slot of the four-slot wheel
// square on, turns it a quarter turn and leaves square on; between slots
// the wheel stands still. The slots are pairs of bars welded to the
// wheel's hub, the pin a small disk welded to the crank, and they push on
// each other through contact. Below, a belt turns an eccentric cam (a disk
// welded off-centre on its shaft) that rocks a sprung lever through a
// roller riding on it.

const D: Vec2 = Vec2::new(-1.0, 0.4);      // drive axle
const C: f32 = 1.4;                         // centre distance
const PIN_R: f32 = 0.07;
const WALL_HW: f32 = 0.03;
const FLARE_LEN: f32 = 0.22;               // mouth guides: reach outwards
const FLARE_W: f32 = 0.13;                  // and how far they spread
const CAM: Vec2 = Vec2::new(-1.0, -1.9);    // cam shaft

fn hard(mut b: Body) -> Body {
    b.collide = true;
    b.friction = 0.05;
    b.restitution = 0.0;
    b
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    // Geneva geometry for four slots: crank a = c·sin 45°, slot mouth at
    // the same radius on the wheel, pin deepest at c − a.
    let g = D + Vec2::new(C, 0.0);
    let a = C * FRAC_PI_4.sin();

    // Drive: hub disk with a crank arm and the pin at its end, starting
    // just before the pin meets the first slot.
    let start = -FRAC_PI_4 - 0.35;
    // Heavy, as a flywheel: a light drive is slowed by each index, then
    // races ahead and slams the pin across the slot, knocking the wheel back.
    let hub = w.add_body(disk(D, 0.3, 16.0));
    w.add_constraint(PinWorld::new(hub, Vec2::ZERO, D));
    w.add_force(Motor::new(hub, 6.0, 2.0));
    let tip = D + Vec2::new(start.cos(), start.sin()) * a;
    let (arm, _) = rod_between(D, tip, 0.4, 0.05);
    let arm = w.add_body(arm);
    w.add_constraint(WeldJoint::new(hub, arm, D, &w.bodies));
    let pin = w.add_body(hard(disk(tip, PIN_R, 0.1)));
    w.add_constraint(WeldJoint::new(arm, pin, tip, &w.bodies));

    // The wheel: a hub and four slots, at 45° off the line of centres so
    // the one at −135° (from the wheel) takes the pin square on.
    // At least the inertia of the slots welded on it, or XPBD's sweep diverges.
    let wheel = w.add_body(disk(g, 0.42, 10.0));
    w.add_constraint(PinWorld::new(wheel, Vec2::ZERO, g));
    w.add_force(Motor::new(wheel, 0.0, 2.5)); // bearing friction holds it between moves
    // The walls end exactly where the pin leaves (radius a): any further
    // and the outgoing pin drags the wheel back off its index.
    let (r_in, r_out) = (C - a - PIN_R - 0.04, a);
    let off = PIN_R + 0.012 + WALL_HW;
    for k in 0..4 {
        let phi = FRAC_PI_4 + k as f32 * FRAC_PI_2;
        let u = Vec2::new(phi.cos(), phi.sin());
        for side in [-1.0, 1.0] {
            let n = u.perp() * side * off;
            let (bar, _) = rod_between(g + u * r_in + n, g + u * r_out + n, 0.3, WALL_HW);
            let bar = w.add_body(hard(bar));
            w.add_constraint(WeldJoint::new(wheel, bar, g + u * r_in + n, &w.bodies));
        }
        // Flared guides at the mouth: a wheel that has crept off its
        // index position is steered back as the pin comes in.
        for side in [-1.0, 1.0] {
            let n = u.perp() * side;
            let (from, to) = (g + u * r_out + n * off, g + u * (r_out + FLARE_LEN) + n * (off + FLARE_W));
            let (guide, _) = rod_between(from, to, 0.1, WALL_HW);
            let guide = w.add_body(hard(guide));
            w.add_constraint(WeldJoint::new(wheel, guide, from, &w.bodies));
        }
        // The slot's closed end.
        let (end, _) = rod_between(g + u * r_in + u.perp() * off, g + u * r_in - u.perp() * off, 0.2, WALL_HW);
        let end = w.add_body(hard(end));
        w.add_constraint(WeldJoint::new(wheel, end, g + u * r_in, &w.bodies));
    }
    let t = w.tracers.len();
    w.add_tracer(wheel, Vec2::new(FRAC_PI_4.cos(), FRAC_PI_4.sin()) * r_out);
    w.tracers[t].seconds = 1.5;

    // Cam: a belt down to a shaft carrying an eccentric disk.
    let shaft = w.add_body(disk(CAM, 0.3, 1.0));
    w.add_constraint(PinWorld::new(shaft, Vec2::ZERO, CAM));
    w.add_constraint(GearJoint::new(hub, shaft, GearKind::Belt, &w.bodies).expect("disks"));
    let cam_c = CAM + Vec2::new(0.0, 0.17);
    let cam = w.add_body(hard(disk(cam_c, 0.42, 1.5)));
    w.add_constraint(WeldJoint::new(shaft, cam, CAM, &w.bodies));

    // Rocker with a roller on the cam, pulled down onto it by a spring.
    let roller_r = 0.09;
    let roller_at = Vec2::new(CAM.x, cam_c.y + 0.42 + roller_r);
    let pivot = Vec2::new(0.7, roller_at.y);
    let (rocker, hl) = rod_between(pivot, roller_at, 1.0, 0.05);
    let rocker = w.add_body(rocker);
    w.add_constraint(PinWorld::new(rocker, Vec2::new(-hl, 0.0), pivot));
    let roller = w.add_body(hard(disk(roller_at, roller_r, 0.2)));
    w.add_constraint(PinJoint::new(rocker, Vec2::new(hl, 0.0), roller, Vec2::ZERO));
    let spring_top = Vec2::new(-0.2, roller_at.y);
    let low = w.add_body(anchor(spring_top - Vec2::Y * 0.9));
    let on_rocker = crate::editor::world_to_local(&w.bodies[rocker], spring_top);
    w.add_force(SpringDamper::new(low, Vec2::ZERO, rocker, on_rocker, 0.7, 40.0, 0.5));
    let t = w.tracers.len();
    w.add_tracer(rocker, Vec2::new(hl, 0.0));
    w.tracers[t].seconds = 1.0;
    w
}
