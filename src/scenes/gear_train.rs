use glam::Vec2;

use crate::sim::{
    constraints::{GearJoint, GearKind, PinJoint, PinWorld},
    forces::{Gravity, Motor},
    world::World,
};
use super::{circle_intersections, disk, rod_between};

// A small machine: a motor turns a pinion, which drives a gear three
// times its size (3:1 down, reversed); a belt from that gear to a pulley
// half its size speeds things up 2:1, same direction; a crank pin on the
// pulley works a four-bar crank-rocker whose coupler traces the red curve.

const PINION: Vec2 = Vec2::new(-2.35, 0.9);
const R_PINION: f32 = 0.3;
const R_GEAR: f32 = 0.9;
const PULLEY: Vec2 = Vec2::new(1.3, 0.9);
const R_PULLEY: f32 = 0.45;
const CRANK: f32 = 0.3;
const COUPLER: f32 = 1.35;
const ROCKER: f32 = 1.0;
const ROCKER_PIVOT: Vec2 = Vec2::new(2.3, -0.6);

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let gear_pos = PINION + Vec2::X * (R_PINION + R_GEAR);
    let pinion = w.add_body(disk(PINION, R_PINION, 0.6));
    let gear = w.add_body(disk(gear_pos, R_GEAR, 3.0));
    let pulley = w.add_body(disk(PULLEY, R_PULLEY, 1.0));
    for (b, at) in [(pinion, PINION), (gear, gear_pos), (pulley, PULLEY)] {
        w.add_constraint(PinWorld::new(b, Vec2::ZERO, at));
    }
    w.add_constraint(GearJoint::new(pinion, gear, GearKind::Mesh, &w.bodies).expect("disks"));
    w.add_constraint(GearJoint::new(gear, pulley, GearKind::Belt, &w.bodies).expect("disks"));
    w.add_force(Motor::new(pinion, 2.4, 0.25));

    // Four-bar: crank pin on the pulley, coupler, rocker on a floor pivot.
    let pin = PULLEY + Vec2::X * CRANK;
    let (_, knee) = circle_intersections(pin, COUPLER, ROCKER_PIVOT, ROCKER).expect("four-bar closes");
    let (coupler_body, hc) = rod_between(pin, knee, 0.4, 0.05);
    let coupler = w.add_body(coupler_body);
    let (rocker_body, hr) = rod_between(ROCKER_PIVOT, knee, 0.6, 0.06);
    let rocker = w.add_body(rocker_body);
    w.add_constraint(PinJoint::new(pulley, Vec2::X * CRANK, coupler, Vec2::new(-hc, 0.0)));
    w.add_constraint(PinJoint::new(coupler, Vec2::new(hc, 0.0), rocker, Vec2::new(hr, 0.0)));
    w.add_constraint(PinWorld::new(rocker, Vec2::new(-hr, 0.0), ROCKER_PIVOT));

    let t = w.tracers.len();
    w.add_tracer(coupler, Vec2::new(0.0, 0.25));
    w.tracers[t].seconds = 4.0;
    w
}
