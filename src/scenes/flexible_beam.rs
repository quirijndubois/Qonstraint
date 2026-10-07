use glam::Vec2;

use crate::sim::{
    constraints::PinJoint,
    forces::{Gravity, TorsionSpring},
    world::World,
};
use super::{anchor, disk, rod_between};

// A cantilever made of rigid segments hinged together, each hinge sprung
// by a torsion spring: a discrete flexible beam. Released straight out
// from the wall it sags under its own weight and the tip mass, ringing in
// its first bending mode with a little of the higher ones on top.

const ROOT: Vec2 = Vec2::new(-2.2, 1.0);
const SEGMENTS: usize = 8;
const SEG_LEN: f32 = 0.36;
const SEG_MASS: f32 = 0.12;
const TIP_MASS: f32 = 0.7;
/// Hinge stiffness at the root; it tapers towards the tip, where the
/// bending moment is smaller.
const K_ROOT: f32 = 75.0;
const DAMPING: f32 = 0.02;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));
    let wall = w.add_body(anchor(ROOT));

    let mut prev = (wall, Vec2::ZERO);
    let mut p = ROOT;
    for k in 0..SEGMENTS {
        let q = p + Vec2::X * SEG_LEN;
        let (seg_body, hl) = rod_between(p, q, SEG_MASS, 0.05 - 0.003 * k as f32);
        let seg = w.add_body(seg_body);
        w.add_constraint(PinJoint::new(prev.0, prev.1, seg, Vec2::new(-hl, 0.0)));
        let taper = 1.0 - 0.75 * k as f32 / SEGMENTS as f32;
        w.add_force(TorsionSpring::new(seg, prev.0, Vec2::new(-hl, 0.0), K_ROOT * taper, DAMPING, &w.bodies));
        prev = (seg, Vec2::new(hl, 0.0));
        p = q;
    }
    let tip = w.add_body(disk(p, 0.15, TIP_MASS));
    w.add_constraint(PinJoint::new(prev.0, prev.1, tip, Vec2::ZERO));
    w.add_tracer(tip, Vec2::ZERO);
    w
}
