use std::f32::consts::FRAC_PI_2;
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{cylinder::{GasMode, Stroke}, Cylinder, PinJoint, WeldJoint},
    forces::{Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// A steam locomotive pulling a tender and two coaches. Three coupled
// driving wheels run on the rail through friction; each coupling rod pins
// all three crank pins, closed loops the solver holds even though they
// are over-constrained, and the far side's rod runs a quarter turn ahead. A cylinder welded to the frame (firing at every top
// dead centre, like a single-acting engine) pushes the piston and main
// rod against the middle driver's crank pin. Boiler, smokestack and cab
// are welded on; the cars hang on pinned couplers and their wheel bearings
// drag. The camera rides along.

const RAIL_TOP: f32 = 0.0;
const DRIVER_R: f32 = 0.45;
const THROW: f32 = 0.24;
const MAIN_ROD: f32 = 1.6;
const BORE: f32 = 0.2;
const CROWN: f32 = 0.11;
const CR: f32 = 6.0;
const START: f32 = -0.8;    // crank angle just past top dead centre, rolling forwards

fn wheel(w: &mut World, c: Vec2, r: f32, mass: f32) -> usize {
    let mut b = disk(c, r, mass).colliding();
    b.friction = 0.9;
    b.restitution = 0.05;
    w.add_body(b)
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let (rail, _) = rod_between(Vec2::new(-30.0, RAIL_TOP - 0.06), Vec2::new(900.0, RAIL_TOP - 0.06), 500.0, 0.06);
    let mut rail = rail.fixed().colliding();
    rail.friction = 0.9;
    rail.restitution = 0.05;
    w.add_body(rail);

    // Locomotive frame, its axles at the drivers' height.
    let axle_y = RAIL_TOP + DRIVER_R;
    let frame_y = axle_y + 0.45;
    let (frame, _) = rod_between(Vec2::new(-1.9, frame_y), Vec2::new(3.0, frame_y), 14.0, 0.09);
    let frame = w.add_body(frame);
    let fc = w.bodies[frame].pos32();
    let at_frame = |p: Vec2| p - fc;

    // Boiler, smokestack, cab: welded on.
    let deco = |w: &mut World, a: Vec2, b: Vec2, hw: f32, mass: f32| {
        let (body, _) = rod_between(a, b, mass, hw);
        let id = w.add_body(body);
        w.add_constraint(WeldJoint::new(frame, id, 0.5 * (a + b), &w.bodies));
    };
    deco(&mut w, Vec2::new(-0.6, frame_y + 0.42), Vec2::new(2.6, frame_y + 0.42), 0.32, 10.0);
    deco(&mut w, Vec2::new(2.25, frame_y + 0.7), Vec2::new(2.25, frame_y + 1.15), 0.09, 1.0);
    deco(&mut w, Vec2::new(-1.4, frame_y + 0.1), Vec2::new(-1.4, frame_y + 1.25), 0.38, 4.0);

    // Driving wheels with their crank pins at START, coupled.
    let xs = [-0.95, 0.1, 1.15];
    let pin_local = Vec2::new(THROW, 0.0);
    let mut drivers = Vec::new();
    for &x in &xs {
        let c = Vec2::new(x, axle_y);
        let id = wheel(&mut w, c, DRIVER_R, 6.0);
        w.bodies[id].angle = START as f64;
        w.add_constraint(PinJoint::new(frame, at_frame(c), id, Vec2::ZERO));
        drivers.push(id);
    }
    let pin = |x: f32| Vec2::new(x, axle_y) + Vec2::new(START.cos(), START.sin()) * THROW;
    // Coupling rods: this side's, and the far side's a quarter turn ahead.
    // One alone is a parallelogram linkage that can flip at its dead
    // centres; two quartered ones carry each other through, as on a real
    // engine.
    for quarter in [0.0, FRAC_PI_2] {
        let a = START + quarter;
        let at = |x: f32| Vec2::new(x, axle_y) + Vec2::new(a.cos(), a.sin()) * THROW;
        let (coupling, hl) = rod_between(at(xs[0]), at(xs[2]), 1.5, 0.045);
        let coupling = w.add_body(coupling);
        let local = Vec2::new(quarter.cos(), quarter.sin()) * THROW;
        for (k, &d) in drivers.iter().enumerate() {
            let along = -hl + (xs[k] - xs[0]);
            w.add_constraint(PinJoint::new(d, local, coupling, Vec2::new(along, 0.0)));
        }
    }
    // Leading and trailing carrying wheels.
    for (x, r) in [(2.55, 0.26), (-1.8, 0.3)] {
        let c = Vec2::new(x, RAIL_TOP + r);
        let id = wheel(&mut w, c, r, 2.0);
        w.add_constraint(PinJoint::new(frame, at_frame(c), id, Vec2::ZERO));
    }

    // Cylinder ahead of the drivers, level with the axles; main rod to
    // the middle driver's crank pin.
    let mid = Vec2::new(xs[1], axle_y);
    let crank_pin = pin(xs[1]);
    let piston_x = |theta: f32| mid.x + THROW * theta.cos() + (MAIN_ROD * MAIN_ROD - (THROW * theta.sin()).powi(2)).sqrt();
    let tdc = piston_x(0.0);
    let stroke = 2.0 * THROW;
    let head = tdc + CROWN + stroke / (CR - 1.0);
    let base = tdc - stroke - CROWN - 0.04;
    let (barrel, _) = rod_between(Vec2::new(base, axle_y), Vec2::new(head, axle_y), 4.0, BORE);
    let barrel = w.add_body(barrel);
    w.add_constraint(WeldJoint::new(frame, barrel, Vec2::new(0.5 * (base + head), axle_y), &w.bodies));
    let wrist = Vec2::new(piston_x(START), axle_y);
    let half = BORE - 0.012 - CROWN;
    let piston = w.add_body(Body::new(wrist, FRAC_PI_2, 0.6, rod_inertia(0.6, half),
        BodyShape::Rod { half_len: half, half_width: CROWN }));
    let (rod, rhl) = rod_between(crank_pin, wrist, 1.2, 0.05);
    let rod = w.add_body(rod);
    w.add_constraint(PinJoint::new(drivers[1], pin_local, rod, Vec2::new(-rhl, 0.0)));
    w.add_constraint(PinJoint::new(rod, Vec2::new(rhl, 0.0), piston, Vec2::ZERO));
    let mut cyl = Cylinder::new(piston, Vec2::ZERO, barrel, &w.bodies).expect("rod barrel");
    cyl.crown = CROWN;
    cyl.mode = GasMode::TwoStroke;
    cyl.throttle = 1.0;
    cyl.begin(Stroke::Power, &w.bodies);
    w.add_constraint(cyl);

    // Tender and two coaches, each a frame on two wheels with dragging
    // bearings, hung on pinned couplers.
    let mut ahead = (frame, at_frame(Vec2::new(-1.9, frame_y)));
    let mut tail = -1.9 - 0.25;
    for (len, mass, box_h) in [(2.2, 8.0, 0.5), (3.4, 6.0, 0.9), (3.4, 6.0, 0.9)] {
        let (x1, x0) = (tail, tail - len);
        let (car, chl) = rod_between(Vec2::new(x0, frame_y), Vec2::new(x1, frame_y), mass, 0.08);
        let car = w.add_body(car);
        let cc = w.bodies[car].pos32();
        // Body shell on the frame.
        let (shell, _) = rod_between(Vec2::new(x0 + 0.25, frame_y + 0.1 + 0.5 * box_h), Vec2::new(x1 - 0.25, frame_y + 0.1 + 0.5 * box_h), 0.5 * mass, 0.5 * box_h);
        let shell = w.add_body(shell);
        w.add_constraint(WeldJoint::new(car, shell, Vec2::new(0.5 * (x0 + x1), frame_y + 0.1), &w.bodies));
        for x in [x0 + 0.55, x1 - 0.55] {
            let c = Vec2::new(x, RAIL_TOP + 0.32);
            let id = wheel(&mut w, c, 0.32, 1.5);
            let at = Vec2::new(x, frame_y) - cc;
            w.add_constraint(PinJoint::new(car, at + Vec2::new(0.0, RAIL_TOP + 0.32 - frame_y), id, Vec2::ZERO));
            w.add_force(Motor::new(id, 0.0, 0.04).on(car));
        }
        // Coupler: a short link from the car ahead to this one.
        let (link, lhl) = rod_between(Vec2::new(x1 + 0.25, frame_y), Vec2::new(x1, frame_y), 0.3, 0.035);
        let link = w.add_body(link);
        w.add_constraint(PinJoint::new(ahead.0, ahead.1, link, Vec2::new(-lhl, 0.0)));
        w.add_constraint(PinJoint::new(link, Vec2::new(lhl, 0.0), car, Vec2::new(chl, 0.0)));
        ahead = (car, Vec2::new(-chl, 0.0));
        tail = x0 - 0.25;
    }
    w.follow = Some(frame);
    w
}
