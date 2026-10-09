use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, TAU};
use glam::Vec2;

use crate::sim::{
    body::Body,
    constraints::{GearJoint, GearKind, PinJoint, PinWorld, Rope, WeldJoint},
    forces::{Gravity, TorsionSpring},
    world::World,
};
use super::{anchor, circle_intersections, disk, rod_between};

// A weight-driven pendulum clock with an anchor escapement, every part a
// plain editor part. The escape wheel is a hub with eighteen spike teeth
// welded on; the anchor is two arms welded to the pendulum rod, each
// ending in a pallet, a short bar set slantwise so its face is an
// inclined plane. As the pendulum swings the pallets dip into the teeth in
// turn: a tooth tip sliding along a pallet face pushes it out (the impulse
// that keeps the pendulum going), slips off its end, and the wheel jumps
// until a tooth lands on the other pallet, half a tooth later; one tooth
// per swing. The power comes from a rope over a barrel (a heavy weight
// on one side, a light one on the other, Huygens' trick) whose great
// wheel is belted to the escape wheel.
//
// The great wheel's arbor also runs the rest of the clock, through two
// more pulleys. One belts a seconds arbor (one turn a minute, its hand on a
// small subdial) from which two meshes, 8:1 and 7.5:1, bring the minute
// arbor round once an hour at the dial centre; on that a cannon pinion
// drives the motion work, 3:1 then 4:1 back to the hour wheel turning on
// the same centre, so the hour hand goes round once in twelve. The other
// pulley belts a pin wheel, also one turn a minute: its pin pushes a
// hanging hammer aside and slips off at the top of each minute, and the
// hammer swings back onto a bell hung on a torsion spring (a passing
// strike).

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
const DRUM: Vec2 = Vec2::new(1.25, 0.6);    // great wheel and barrel
/// Seconds arbor speed over great wheel speed, so the seconds arbor (and the
/// pin wheel) turn once a minute; set from the measured escape rate.
const MINUTE_RATIO: f32 = 4.578;
const GREAT_R: f32 = 0.5;
const BARREL_R: f32 = 0.13;
const FLOOR_Y: f32 = -1.95;
const DIAL: Vec2 = Vec2::new(0.0, 2.5);     // minute and hour arbors
const DIAL_R: f32 = 0.95;
const SECONDS: Vec2 = Vec2::new(0.0, 1.95); // seconds arbor, subdial
const PIN_WHEEL: Vec2 = Vec2::new(1.7, 1.6);
const PIN_R: f32 = 0.15;
const HAMMER_PIVOT: Vec2 = Vec2::new(1.7, 2.6);
/// The time the clock starts at: 10:08:45.
const START_TIME: [f32; 3] = [10.0, 8.0, 45.0];

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

    // Weight drive: a small barrel carries the rope, a great wheel on its
    // arbor takes the belt from a pinion on the escape arbor, 6.25:1, so
    // the weights fall slowly (about a quarter of a centimetre a second)
    // and the clock runs a quarter of an hour before the heavy weight
    // settles on the floor and it stops, run down.
    let pinion = w.add_body(disk(E, 0.08, 0.1));
    w.add_constraint(WeldJoint::new(hub, pinion, E, &w.bodies));
    let great = arbor(&mut w, DRUM, GREAT_R, 1.0);
    w.add_constraint(GearJoint::new(great, pinion, GearKind::Belt, &w.bodies).expect("disks"));
    let heavy = w.add_body(disk(DRUM + Vec2::new(BARREL_R, -GREAT_R - 0.15), 0.13, 4.6).colliding());
    let light = w.add_body(disk(Vec2::new(DRUM.x - BARREL_R, FLOOR_Y + 0.1), 0.09, 0.75));
    let floor = rod_between(Vec2::new(DRUM.x - 0.05, FLOOR_Y), Vec2::new(DRUM.x + 0.5, FLOOR_Y), 1.0, 0.03).0;
    w.add_body(floor.fixed().colliding());

    // Going train. Everything below turns clockwise, like the great wheel: belts
    // keep the sense, each mesh reverses it.
    let seconds_pulley = wheel_on(&mut w, great, 0.4, 0.1);
    let seconds = arbor(&mut w, SECONDS, 0.4 / MINUTE_RATIO, 0.2);
    mesh(&mut w, seconds_pulley, seconds, GearKind::Belt);
    let (p1, w1, p2, w2) = (0.035, 0.28, 0.035, 0.2625); // 8:1, 7.5:1
    let (third_at, _) = circle_intersections(SECONDS, p1 + w1, DIAL, p2 + w2).expect("third wheel fits");
    let s_pinion = wheel_on(&mut w, seconds, p1, 0.02);
    let third = arbor(&mut w, third_at, w1, 0.3);
    mesh(&mut w, s_pinion, third, GearKind::Mesh);
    let t_pinion = wheel_on(&mut w, third, p2, 0.02);
    let minute = arbor(&mut w, DIAL, w2, 0.3);
    mesh(&mut w, t_pinion, minute, GearKind::Mesh);
    // Motion work: cannon pinion 3:1 to the minute wheel, its pinion 4:1
    // to the hour wheel, back on the dial centre.
    let (c, m, mp, h) = (0.05, 0.15, 0.04, 0.16);
    let cannon = wheel_on(&mut w, minute, c, 0.02);
    let mw_at = DIAL + Vec2::new(30f32.to_radians().cos(), 30f32.to_radians().sin()) * (c + m);
    let minute_wheel = arbor(&mut w, mw_at, m, 0.1);
    mesh(&mut w, cannon, minute_wheel, GearKind::Mesh);
    let mw_pinion = wheel_on(&mut w, minute_wheel, mp, 0.02);
    let hour = arbor(&mut w, DIAL, h, 0.2);
    mesh(&mut w, mw_pinion, hour, GearKind::Mesh);

    // Dial: hour ticks, longer at the quarters, and a seconds ring.
    for k in 0..12 {
        let u = dir(FRAC_PI_2 - TAU * k as f32 / 12.0);
        let inner = if k % 3 == 0 { DIAL_R - 0.16 } else { DIAL_R - 0.09 };
        let hw = if k % 3 == 0 { 0.025 } else { 0.015 };
        w.add_body(rod_between(DIAL + u * inner, DIAL + u * DIAL_R, 0.1, hw).0.fixed().colliding());
    }
    for k in 0..12 {
        let u = dir(FRAC_PI_2 - TAU * k as f32 / 12.0);
        let r = if k % 3 == 0 { 0.016 } else { 0.01 };
        w.add_body(disk(SECONDS + u * 0.2, r, 0.1).fixed().colliding());
    }

    // Hands, at clock angles (clockwise from twelve). Each has a short
    // tail past its arbor.
    let [hh, mm, ss] = START_TIME;
    let hand = |w: &mut World, at: Vec2, on: usize, turns: f32, len: f32, tail: f32, hw: f32| {
        let u = dir(FRAC_PI_2 - TAU * turns);
        let (body, _) = rod_between(at - u * tail, at + u * len, 0.03, hw);
        let b = w.add_body(body);
        w.add_constraint(WeldJoint::new(on, b, at, &w.bodies));
    };
    hand(&mut w, DIAL, hour, (hh + mm / 60.0 + ss / 3600.0) / 12.0, 0.5, 0.1, 0.035);
    hand(&mut w, DIAL, minute, (mm + ss / 60.0) / 60.0, 0.8, 0.14, 0.022);
    hand(&mut w, SECONDS, seconds, ss / 60.0, 0.17, 0.05, 0.008);

    // Strike: a pin wheel belted from the great wheel at the seconds arbor's
    // speed, its pin set to let the hammer go as the minute turns.
    let strike_pulley = wheel_on(&mut w, great, 0.3, 0.1);
    let pin_wheel = arbor(&mut w, PIN_WHEEL, 0.3 / MINUTE_RATIO, 0.2);
    mesh(&mut w, strike_pulley, pin_wheel, GearKind::Belt);
    let barrel = wheel_on(&mut w, great, BARREL_R, 2.0);
    let rope = Rope::new(light, Vec2::ZERO, heavy, Vec2::ZERO, &w.bodies).over(barrel, &w.bodies).expect("rope over barrel");
    w.add_constraint(rope);

    let pin_at = PIN_WHEEL + dir(PIN_PHASE - TAU * ss / 60.0) * PIN_R;
    let (arm, _) = rod_between(PIN_WHEEL, pin_at, 0.02, 0.02);
    let arm = w.add_body(arm);
    w.add_constraint(WeldJoint::new(pin_wheel, arm, PIN_WHEEL, &w.bodies));
    let pin = w.add_body(disk(pin_at, 0.025, 0.02).colliding());
    w.add_constraint(WeldJoint::new(arm, pin, pin_at, &w.bodies));

    // Hammer: hangs from its pivot, its lower end in the pin's path, the
    // head just clear of the bell.
    let tail_end = Vec2::new(HAMMER_PIVOT.x, PIN_WHEEL.y + 0.08);
    let (stem, hl) = rod_between(HAMMER_PIVOT, tail_end, 0.05, 0.02);
    let stem = w.add_body(stem.colliding());
    w.add_constraint(PinWorld::new(stem, Vec2::new(-hl, 0.0), HAMMER_PIVOT));
    let head_r = 0.07;
    let head_at = Vec2::new(HAMMER_PIVOT.x, 1.95);
    let head = w.add_body(disk(head_at, head_r, 0.2).colliding());
    w.add_constraint(WeldJoint::new(stem, head, head_at, &w.bodies));

    // Bell, hung from its crown on a stiff torsion spring so it rings.
    let bell_r = 0.16;
    let bell_at = head_at - Vec2::X * (head_r + 0.02 + bell_r);
    let crown = bell_at + Vec2::Y * bell_r;
    let mut bell = disk(bell_at, bell_r, 0.2).colliding();
    bell.restitution = 0.7;
    let bell = w.add_body(bell);
    let hanger = w.add_body(anchor(crown));
    w.add_constraint(PinJoint::new(hanger, Vec2::ZERO, bell, Vec2::Y * bell_r));
    w.add_force(TorsionSpring::new(bell, hanger, Vec2::Y * bell_r, 3.0, 0.005, &w.bodies));
    w
}

/// The pin wheel's angle (pin direction) at the top of a minute: the pin
/// is just slipping off the hammer then. Found by running the clock.
const PIN_PHASE: f32 = 0.25;

fn dir(a: f32) -> Vec2 { Vec2::new(a.cos(), a.sin()) }

/// A disk turning on a fixed axle at `at`.
fn arbor(w: &mut World, at: Vec2, r: f32, mass: f32) -> usize {
    let b = w.add_body(disk(at, r, mass));
    w.add_constraint(PinWorld::new(b, Vec2::ZERO, at));
    b
}

/// A disk welded concentric on `on` (a pinion or a second pulley).
fn wheel_on(w: &mut World, on: usize, r: f32, mass: f32) -> usize {
    let at = w.bodies[on].pos32();
    let b = w.add_body(disk(at, r, mass));
    w.add_constraint(WeldJoint::new(on, b, at, &w.bodies));
    b
}

fn mesh(w: &mut World, a: usize, b: usize, kind: GearKind) {
    w.add_constraint(GearJoint::new(a, b, kind, &w.bodies).expect("disks"));
}
