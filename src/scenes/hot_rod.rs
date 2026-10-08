use std::f32::consts::{FRAC_PI_2, PI};
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{cylinder::{GasMode, Stroke}, Cylinder, GearJoint, GearKind, PinJoint, WeldJoint},
    forces::{spring::SpringDamper, Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// A hot rod with an exposed inline four, driving over rolling ground.
// Four 4-stroke Cylinders stand on the chassis, each with its own crank;
// the cranks mesh as a gear train, which keeps them in step (neighbours
// turn opposite ways), started in the strokes for the firing order
// 1-3-4-2 and timing themselves from there. The first crank drives the
// rear wheel by a belt. Both wheels ride on sprung swing arms, so the
// chassis pitches and rolls over the bumps. The camera follows the car.

const BORE: f32 = 0.12;
const THROW: f32 = 0.12;
const ROD: f32 = 0.42;
const CROWN: f32 = 0.06;
const CR: f32 = 6.0;
const CRANK_R: f32 = 0.3;       // crank disks mesh: spacing = 2·CRANK_R
const START: f32 = -0.35;       // cylinder 1's crank past top dead centre (clockwise: drives right)

fn ground(x: f32) -> f32 {
    0.22 * (x / 2.1).sin() + 0.1 * (x / 0.83 + 1.0).sin() - 0.12 * (x / 5.3).cos()
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    // Rolling ground as a chain of fixed bars; flat where the car starts.
    let flat = |x: f32| if x < 1.5 { 0.0 } else { ground(x) - ground(1.5) * (1.0 - ((x - 1.5) / 3.0).min(1.0)) };
    let mut x = -12.0;
    while x < 260.0 {
        let (a, b) = (Vec2::new(x, flat(x)), Vec2::new(x + 0.8, flat(x + 0.8)));
        let (seg, _) = rod_between(a, b, 10.0, 0.08);
        let mut seg = seg.fixed().colliding();
        seg.friction = 1.0;
        seg.restitution = 0.05;
        w.add_body(seg);
        x += 0.8;
    }

    // A wall behind the start.
    let (wall, _) = rod_between(Vec2::new(-5.0, 0.0), Vec2::new(-5.0, 1.6), 10.0, 0.1);
    w.add_body(wall.fixed().colliding());

    // Chassis.
    let ride = 0.75;
    let (chassis, _) = rod_between(Vec2::new(-1.9, ride), Vec2::new(1.9, ride), 15.0, 0.08);
    // Chassis and wheels in different planes: the wheels tuck under it
    // without rubbing, and both still meet the ground (plane 0).
    let mut chassis = chassis.colliding();
    chassis.plane = 2;
    let chassis = w.add_body(chassis);
    let cc = w.bodies[chassis].pos32();
    let on_chassis = |p: Vec2| p - cc;

    // The engine: four cylinders in a row over meshing cranks.
    let crank_y = ride + 0.42;
    let s_tdc = THROW + ROD;
    let stroke = 2.0 * THROW;
    let head = s_tdc + CROWN + stroke / (CR - 1.0);
    let base = s_tdc - stroke - CROWN - 0.02;
    // Crank angles from top dead centre: neighbours counter-rotate, and
    // these put pistons 1 and 4 together, 2 and 3 half a turn away.
    let phi = [START, PI - START, PI + START, -START];
    let strokes = [Stroke::Power, Stroke::Exhaust, Stroke::Compression, Stroke::Intake];
    let mut cranks = Vec::new();
    for k in 0..4 {
        let cx = (k as f32 - 1.5) * 2.0 * CRANK_R;
        let c = Vec2::new(cx, crank_y);
        let mut crank = disk(c, CRANK_R, 2.0);
        crank.angle = (FRAC_PI_2 + phi[k]) as f64;
        let crank = w.add_body(crank);
        w.add_constraint(PinJoint::new(chassis, on_chassis(c), crank, Vec2::ZERO));
        if let Some(&prev) = cranks.last() {
            w.add_constraint(GearJoint::new(prev, crank, GearKind::Mesh, &w.bodies).expect("disks"));
        }
        cranks.push(crank);

        let (barrel, _) = rod_between(c + Vec2::Y * base, c + Vec2::Y * head, 2.0, BORE);
        let barrel = w.add_body(barrel);
        w.add_constraint(WeldJoint::new(chassis, barrel, c + Vec2::Y * (0.5 * (base + head)), &w.bodies));

        let pin = c + Vec2::new(-phi[k].sin(), phi[k].cos()) * THROW;
        let wrist = c + Vec2::Y * (pin.y - c.y + (ROD * ROD - (pin.x - c.x).powi(2)).sqrt());
        let half = BORE - 0.01 - CROWN;
        let piston = w.add_body(Body::new(wrist, 0.0, 0.3, rod_inertia(0.3, half),
            BodyShape::Rod { half_len: half, half_width: CROWN }));
        let (rod, hl) = rod_between(pin, wrist, 0.25, 0.03);
        let rod = w.add_body(rod);
        w.add_constraint(PinJoint::new(crank, Vec2::new(THROW, 0.0), rod, Vec2::new(-hl, 0.0)));
        w.add_constraint(PinJoint::new(rod, Vec2::new(hl, 0.0), piston, Vec2::ZERO));
        let mut cyl = Cylinder::new(piston, Vec2::ZERO, barrel, &w.bodies).expect("rod barrel");
        cyl.crown = CROWN;
        cyl.mode = GasMode::FourStroke;
        cyl.throttle = 1.0;
        cyl.begin(strokes[k], &w.bodies);
        w.add_constraint(cyl);
    }

    // Wheels on sprung swing arms; the rear one belted to the last crank.
    let mut wheels = Vec::new();
    for (pivot_x, wheel_x, r) in [(-0.7, -1.6, 0.45), (0.9, 1.65, 0.36)] {
        let pivot = Vec2::new(pivot_x, ride);
        let hub = Vec2::new(wheel_x, r);
        let (arm, ahl) = rod_between(pivot, hub, 2.0, 0.05);
        let arm = w.add_body(arm);
        w.add_constraint(PinJoint::new(chassis, on_chassis(pivot), arm, Vec2::new(-ahl, 0.0)));
        let mut wheel = disk(hub, r, 3.0).colliding();
        wheel.friction = 1.1;
        wheel.restitution = 0.1;
        wheel.plane = 1;
        let wheel = w.add_body(wheel);
        w.add_constraint(PinJoint::new(arm, Vec2::new(ahl, 0.0), wheel, Vec2::ZERO));
        // Spring from above the hub on the chassis down to the arm's end.
        let top = Vec2::new(wheel_x, ride + 0.05);
        let at_arm = crate::editor::world_to_local(&w.bodies[arm], hub + (pivot - hub).normalize() * 0.15);
        let len = (top - (hub + (pivot - hub).normalize() * 0.15)).length();
        w.add_force(SpringDamper::new(chassis, on_chassis(top), arm, at_arm, len + 0.12, 1800.0, 60.0));
        // Rolling and bearing losses.
        w.add_force(Motor::new(wheel, 0.0, 0.3).on(arm));
        wheels.push(wheel);
    }
    w.add_constraint(GearJoint::new(cranks[0], wheels[0], GearKind::Belt, &w.bodies).expect("disks"));
    w.follow = Some(chassis);
    w
}
