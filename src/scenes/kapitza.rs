use std::f32::consts::FRAC_PI_2;
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{PinJoint, PinWorld, SliderJoint},
    forces::{Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// Kapitza's pendulum: an upside-down pendulum on a pivot shaken up and down
// fast enough stands upright, and swings about the top as if gravity had
// turned around. Here a motor spins a heavy flywheel whose crank pumps a
// carriage up and down a vertical rail (amplitude a, frequency ω); the
// pendulum pivots on the carriage. Upright is stable when a²ω² > 2·g·l.
// Turn the speed down (SPEED slider) or lower the motor torque and it
// falls over.

const WHEEL: Vec2 = Vec2::new(0.0, -1.5);
const THROW: f32 = 0.11;
const CONROD: f32 = 0.85;
const OMEGA: f32 = 75.0;
const DRAG: f32 = 1.2;
const PEND: f32 = 1.1;
const TILT: f32 = 0.25; // initial lean from upright (rad)

/// Crank pin and carriage height for crank angle `t`.
fn kinematics(t: f32) -> (Vec2, f32) {
    let pin = WHEEL + Vec2::new(t.cos(), t.sin()) * THROW;
    let y = pin.y + (CONROD * CONROD - pin.x * pin.x).sqrt();
    (pin, y)
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let t0 = 0.0f32;
    let (pin, y) = kinematics(t0);
    // Velocities that agree with the constraints, by differencing the
    // kinematics at the starting crank speed.
    let h = 1e-3;
    let (pin1, y1) = kinematics(t0 + h);
    let carriage_v = (y1 - y) / h * OMEGA;
    let rod_angle = |pin: Vec2, y: f32| (y - pin.y).atan2(-pin.x);
    let rod_w = (rod_angle(pin1, y1) - rod_angle(pin, y)) / h * OMEGA;

    let mut wheel = disk(WHEEL, 0.32, 18.0);
    wheel.angle = t0 as f64;
    wheel.ang_vel = OMEGA as f64;
    let wheel = w.add_body(wheel);
    w.add_constraint(PinWorld::new(wheel, Vec2::ZERO, WHEEL));
    w.add_force(Motor::new(wheel, OMEGA * DRAG, DRAG));

    let (rail_body, _) = rod_between(Vec2::new(0.0, y - 0.45), Vec2::new(0.0, y + 0.45), 2.0, 0.05);
    let rail = w.add_body(rail_body.fixed());

    let carriage_pos = Vec2::new(0.0, y);
    let mut carriage = Body::new(carriage_pos, FRAC_PI_2, 0.25, rod_inertia(0.25, 0.1),
        BodyShape::Rod { half_len: 0.1, half_width: 0.07 });
    carriage.vel.y = carriage_v as f64;
    let carriage = w.add_body(carriage);
    let mut slider = SliderJoint::new(carriage, Vec2::ZERO, rail, &w.bodies).expect("rail");
    slider.lock_rotation = true;
    w.add_constraint(slider);

    let (mut rod_body, hc) = rod_between(pin, carriage_pos, 0.15, 0.04);
    let rc = rod_body.pos32();
    rod_body.ang_vel = rod_w as f64;
    // Centre velocity of the conrod: its carriage end moves with the
    // carriage, the rest follows from its spin.
    let end_v = Vec2::new(0.0, carriage_v);
    let r_end = carriage_pos - rc;
    let v = end_v - rod_w * r_end.perp();
    rod_body.vel = v.as_dvec2();
    let conrod = w.add_body(rod_body);
    w.add_constraint(PinJoint::new(wheel, Vec2::new(THROW, 0.0), conrod, Vec2::new(-hc, 0.0)));
    w.add_constraint(PinJoint::new(conrod, Vec2::new(hc, 0.0), carriage, Vec2::ZERO));

    // The pendulum stands on the carriage's top.
    let pivot = carriage_pos + Vec2::Y * 0.1;
    let tip = pivot + Vec2::new(TILT.sin(), TILT.cos()) * PEND;
    let (mut pend_body, hp) = rod_between(pivot, tip, 0.12, 0.04);
    pend_body.vel.y = carriage_v as f64;
    let pend = w.add_body(pend_body);
    w.add_constraint(PinJoint::new(carriage, Vec2::new(0.1, 0.0), pend, Vec2::new(-hp, 0.0)));
    let mut bob = disk(tip, 0.12, 0.25);
    bob.vel.y = carriage_v as f64;
    let bob = w.add_body(bob);
    w.add_constraint(PinJoint::new(pend, Vec2::new(hp, 0.0), bob, Vec2::ZERO));

    w.add_tracer(bob, Vec2::ZERO);
    w
}
