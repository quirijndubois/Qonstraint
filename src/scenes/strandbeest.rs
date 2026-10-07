use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, PinWorld},
    forces::{Gravity, Motor},
    world::World,
};
use super::{circle_intersections, disk, rod_between};

// Theo Jansen's Strandbeest leg, a pair of them mirrored on one crank.
// Eleven bar lengths (Jansen's "holy numbers") turn the crank's circle into
// a foot path with a long flat bottom, which is what lets the beasts walk
// on sand without bobbing. Each leg: crank pin → two links → a rigid
// triangle on the fixed pivot → a rigid foot triangle. Triangles are three
// pinned rods, so everything here is a plain editor part. A motor (torque
// with drag) drives the crank; the red traces are the feet.

/// Jansen's lengths in his units; `S` scales them to the scene.
const S: f32 = 1.0 / 38.0;
const A: f32 = 38.0;
const B: f32 = 41.5;
const C: f32 = 39.3;
const D: f32 = 40.1;
const E: f32 = 55.8;
const F: f32 = 39.4;
const G: f32 = 36.7;
const H: f32 = 65.7;
const I: f32 = 49.0;
const J: f32 = 50.0;
const K: f32 = 61.9;
const L: f32 = 7.8;
const M: f32 = 15.0;

const CENTRE: Vec2 = Vec2::new(0.0, 0.6);
const LINK_MASS: f32 = 0.15;
const LINK_HW: f32 = 0.035;

/// Joint positions of one leg (in Jansen units, crank centre at the
/// origin, fixed pivot to the left) for crank angle `theta`.
pub(crate) struct Leg { pub a: Vec2, pub p: Vec2, pub c: Vec2, pub d: Vec2, pub e: Vec2, pub f: Vec2, pub g: Vec2 }

pub(crate) fn leg(theta: f32) -> Leg {
    let a = Vec2::new(theta.cos(), theta.sin()) * M;
    let p = Vec2::new(-A, -L);
    let pick = |c1, r1, c2, r2, better: fn(Vec2, Vec2) -> bool| {
        let (x, y) = circle_intersections(c1, r1, c2, r2).expect("Jansen linkage closes");
        if better(x, y) { x } else { y }
    };
    let c = pick(a, J, p, B, |x, y| x.y > y.y);
    let d = pick(a, K, p, C, |x, y| x.y < y.y);
    let e = pick(p, D, c, E, |x, y| x.x < y.x);
    let f = pick(e, F, d, G, |x, y| x.x < y.x);
    let g = pick(d, I, f, H, |x, y| x.y < y.y);
    Leg { a, p, c, d, e, f, g }
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let theta = 0.4f32;
    let mut crank_body = disk(CENTRE, M * S * 1.35, 3.0);
    crank_body.angle = theta as f64;
    let crank = w.add_body(crank_body);
    w.add_constraint(PinWorld::new(crank, Vec2::ZERO, CENTRE));
    w.add_force(Motor::new(crank, 9.0, 3.5));
    let pin_local = Vec2::new(M * S, 0.0);

    // The mirrored leg sees the same crank pin from the other side: its own
    // crank angle is π − θ, and every point is reflected in x.
    for mirror in [1.0f32, -1.0] {
        let lg = leg(if mirror > 0.0 { theta } else { std::f32::consts::PI - theta });
        let at = |v: Vec2| CENTRE + Vec2::new(v.x * mirror, v.y) * S;
        let rod = |w: &mut World, from: Vec2, to: Vec2| {
            let (b, hl) = rod_between(at(from), at(to), LINK_MASS, LINK_HW);
            (w.add_body(b), hl)
        };
        let (j, hj) = rod(&mut w, lg.a, lg.c);
        let (k, hk) = rod(&mut w, lg.a, lg.d);
        let (b, hb) = rod(&mut w, lg.p, lg.c);   // triangle P-C-E
        let (e, he) = rod(&mut w, lg.c, lg.e);
        let (d, hd) = rod(&mut w, lg.e, lg.p);
        let (c, hc) = rod(&mut w, lg.p, lg.d);
        let (f, hf) = rod(&mut w, lg.e, lg.f);
        let (g, hg) = rod(&mut w, lg.d, lg.f);   // foot triangle D-F-G
        let (h, hh) = rod(&mut w, lg.f, lg.g);
        let (i, hi) = rod(&mut w, lg.g, lg.d);
        let start = |hl: f32| Vec2::new(-hl, 0.0);
        let end = |hl: f32| Vec2::new(hl, 0.0);

        // Crank pin
        w.add_constraint(PinJoint::new(crank, pin_local, j, start(hj)));
        w.add_constraint(PinJoint::new(crank, pin_local, k, start(hk)));
        // Fixed pivot P: the triangle and link c
        w.add_constraint(PinWorld::new(b, start(hb), at(lg.p)));
        w.add_constraint(PinWorld::new(c, start(hc), at(lg.p)));
        w.add_constraint(PinJoint::new(b, start(hb), d, end(hd)));
        // C: link j onto the triangle, triangle closes
        w.add_constraint(PinJoint::new(b, end(hb), j, end(hj)));
        w.add_constraint(PinJoint::new(b, end(hb), e, start(he)));
        // E
        w.add_constraint(PinJoint::new(e, end(he), d, start(hd)));
        w.add_constraint(PinJoint::new(d, start(hd), f, start(hf)));
        // D: links k and c, foot triangle
        w.add_constraint(PinJoint::new(c, end(hc), k, end(hk)));
        w.add_constraint(PinJoint::new(c, end(hc), g, start(hg)));
        w.add_constraint(PinJoint::new(c, end(hc), i, end(hi)));
        // F and G
        w.add_constraint(PinJoint::new(g, end(hg), f, end(hf)));
        w.add_constraint(PinJoint::new(g, end(hg), h, start(hh)));
        w.add_constraint(PinJoint::new(h, end(hh), i, start(hi)));

        let tracer = w.tracers.len();
        w.add_tracer(h, end(hh));
        w.tracers[tracer].seconds = 3.0;
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The foot path has its famous flat bottom: over a good part of the
    /// crank turn the foot stays within a small height band near its lowest.
    #[test]
    fn foot_path_has_flat_bottom() {
        let n = 360;
        let ys: Vec<f32> = (0..n).map(|k| leg(k as f32 / n as f32 * std::f32::consts::TAU).g.y).collect();
        let lo = ys.iter().cloned().fold(f32::INFINITY, f32::min);
        let hi = ys.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let flat = ys.iter().filter(|&&y| y < lo + 0.04 * (hi - lo)).count();
        eprintln!("foot y {lo:.1}..{hi:.1}, flat for {flat}/{n} of the turn");
        assert!(flat as f32 > 0.25 * n as f32);
    }
}
