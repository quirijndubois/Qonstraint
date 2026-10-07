use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, PinWorld},
    forces::Gravity,
    world::World,
};
use super::{circle_intersections, disk, rod_between};

// Peaucellier–Lipkin linkage, the first exact straight-line mechanism
// (1864). Two long links from the fixed pivot O hold the far corners A, B
// of a rhombus A-C-B-P; then O, C and P stay in line with OC·OP = L² − s²,
// so P is C inverted in a circle. C swings on a crank whose circle passes
// through O, and inversion maps a circle through the centre to a straight
// line: P runs on an exact straight line (the red trace). The crank here
// is a pendulum, so gravity keeps it swinging.

const O: Vec2 = Vec2::new(0.0, 1.7);
/// Crank radius; its pivot Q sits that far below O.
const CRANK: f32 = 0.6;
const LONG: f32 = 1.8;
const SIDE: f32 = 1.0;
const SWING: f32 = 1.15; // start angle of the crank from straight down (rad)

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let q = O - Vec2::Y * CRANK;
    let c = q + Vec2::new(SWING.sin(), -SWING.cos()) * CRANK;
    let (a, b) = circle_intersections(O, LONG, c, SIDE).expect("rhombus closes");
    let p = a + b - c;

    let (crank_body, hc) = rod_between(q, c, 2.5, 0.07);
    let crank = w.add_body(crank_body);
    let (oa_body, hoa) = rod_between(O, a, 0.5, 0.05);
    let oa = w.add_body(oa_body);
    let (ob_body, hob) = rod_between(O, b, 0.5, 0.05);
    let ob = w.add_body(ob_body);
    let (ac_body, hac) = rod_between(a, c, 0.3, 0.045);
    let ac = w.add_body(ac_body);
    let (bc_body, hbc) = rod_between(b, c, 0.3, 0.045);
    let bc = w.add_body(bc_body);
    let (ap_body, hap) = rod_between(a, p, 0.3, 0.045);
    let ap = w.add_body(ap_body);
    let (bp_body, hbp) = rod_between(b, p, 0.3, 0.045);
    let bp = w.add_body(bp_body);
    let pen = w.add_body(disk(p, 0.12, 0.4));

    let s = |hl: f32| Vec2::new(-hl, 0.0);
    let e = |hl: f32| Vec2::new(hl, 0.0);
    w.add_constraint(PinWorld::new(crank, s(hc), q));
    w.add_constraint(PinWorld::new(oa, s(hoa), O));
    w.add_constraint(PinWorld::new(ob, s(hob), O));
    // C
    w.add_constraint(PinJoint::new(crank, e(hc), ac, e(hac)));
    w.add_constraint(PinJoint::new(crank, e(hc), bc, e(hbc)));
    // A and B
    w.add_constraint(PinJoint::new(oa, e(hoa), ac, s(hac)));
    w.add_constraint(PinJoint::new(oa, e(hoa), ap, s(hap)));
    w.add_constraint(PinJoint::new(ob, e(hob), bc, s(hbc)));
    w.add_constraint(PinJoint::new(ob, e(hob), bp, s(hbp)));
    // P
    w.add_constraint(PinJoint::new(ap, e(hap), bp, e(hbp)));
    w.add_constraint(PinJoint::new(ap, e(hap), pen, Vec2::ZERO));

    let t = w.tracers.len();
    w.add_tracer(pen, Vec2::ZERO);
    w.tracers[t].seconds = 6.0;
    w
}
