use std::f32::consts::{FRAC_PI_2, PI, TAU};
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{cylinder::{GasMode, Stroke}, Cylinder, PinJoint, PinWorld},
    forces::{Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// A five-cylinder, four-stroke radial engine, assembled from editor parts:
// a flywheel disk on a world pin, five fixed barrels, five pistons and
// connecting rods all pinned to one crank pin, a 4-stroke Cylinder joining
// each piston to its barrel, and a Motor with drag only as the propeller
// load. Each cylinder times itself from its piston's dead centres; the
// scene only has to start them in the right strokes for the firing order
// 1-3-5-2-4 (a power stroke every 144°). Released from rest just past
// cylinder 1's ignition, that first power stroke starts it.

const CENTRE:   Vec2 = Vec2::new(0.0, 0.0);
const N:        usize = 5;
const THROW:    f32 = 0.30;  // crank radius (stroke = 2·THROW)
const ROD_LEN:  f32 = 1.0;
const BORE:     f32 = 0.19;  // bore radius = barrel rod half width
const CROWN:    f32 = 0.11;  // piston centre to crown
const CR:       f32 = 6.0;   // compression ratio
const FLYWHEEL: f32 = 0.46;  // crank disk radius
const LOAD:     f32 = 1.8;   // propeller drag, torque per rad/s
const START:    f32 = 0.45;  // crank angle past cylinder 1's TDC (rad)

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    // Cylinder k points at angle α_k (cylinder 1 straight up, counter-clockwise).
    let alpha = |k: usize| FRAC_PI_2 + TAU * k as f32 / N as f32;
    let stroke = 2.0 * THROW;
    let s_tdc = THROW + ROD_LEN;
    let s_head = s_tdc + CROWN + stroke / (CR - 1.0);
    let s_base = s_tdc - stroke - CROWN - 0.02;

    let theta = alpha(0) + START;
    let mut crank_body = disk(CENTRE, FLYWHEEL, 14.0);
    crank_body.angle = theta as f64;
    let crank = w.add_body(crank_body);
    w.add_constraint(PinWorld::new(crank, Vec2::ZERO, CENTRE));
    w.add_force(Motor::new(crank, 0.0, LOAD));
    let pin_local = Vec2::new(THROW, 0.0);
    let pin = CENTRE + Vec2::new(theta.cos(), theta.sin()) * THROW;

    for k in 0..N {
        let a = alpha(k);
        let u = Vec2::new(a.cos(), a.sin());

        let (barrel_body, _) = rod_between(CENTRE + u * s_base, CENTRE + u * s_head, 4.0, BORE);
        let barrel = w.add_body(barrel_body.fixed());

        // Piston centre on the axis at distance ROD_LEN from the crank pin.
        let d = pin - CENTRE;
        let (along, across) = (d.dot(u), d.dot(u.perp()));
        let wrist = CENTRE + u * (along + (ROD_LEN * ROD_LEN - across * across).sqrt());

        // The piston is a short rod lying across the bore, its rounded
        // ends inside the drawn block.
        let half = BORE - 0.012 - CROWN;
        let piston = w.add_body(Body::new(
            wrist, a + FRAC_PI_2, 0.5, rod_inertia(0.5, half),
            BodyShape::Rod { half_len: half, half_width: CROWN },
        ));

        let (rod_body, hl) = rod_between(pin, wrist, 0.35, 0.05);
        let rod = w.add_body(rod_body);
        w.add_constraint(PinJoint::new(crank, pin_local, rod, Vec2::new(-hl, 0.0)));
        w.add_constraint(PinJoint::new(rod, Vec2::new(hl, 0.0), piston, Vec2::ZERO));

        // Firing order 1-3-5-2-4: cylinder k fires at its TDC (crank at α_k)
        // on the lap that spaces the firings 144° apart. Its stroke now
        // follows from how far the crank has turned since.
        let fire = a + if k % 2 == 0 { 0.0 } else { TAU };
        let phase = (theta - fire).rem_euclid(2.0 * TAU);
        let current = [Stroke::Power, Stroke::Exhaust, Stroke::Intake, Stroke::Compression][(phase / PI) as usize % 4];
        let mut cyl = Cylinder::new(piston, Vec2::ZERO, barrel, &w.bodies).expect("rod barrel");
        cyl.crown = CROWN;
        cyl.mode = GasMode::FourStroke;
        cyl.begin(current, &w.bodies);
        w.add_constraint(cyl);

        if k == 0 {
            w.add_tracer(rod, Vec2::new(-hl * 0.2, 0.0));
        }
    }
    w
}
