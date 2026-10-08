//! Collisions with friction between bodies that have `collide` set.
//!
//! Contacts are one-sided, which the equality solvers can't express, so
//! like the slider end stops they are stiff damped penalty forces, tuned
//! to the effective mass at each contact and capped in ω·dt so both
//! integrators stay stable at any step size. Restitution sets the damping
//! ratio. Friction is Coulomb with a stick spring: each contact keeps the
//! tangential displacement since it started sticking, pulled back by a
//! spring that gives way (slips) at μ·normal force, so a block on a slope
//! below the friction angle stays put instead of creeping.
//!
//! Shapes: disks are circles, rods capsules (radius = half width); anchors
//! (points) don't collide. Bodies joined by any constraint never collide
//! with each other: joints overlap by design.
//!
//! Belts with `GearJoint::collide` are surfaces too (a conveyor): each
//! straight run a thin capsule moving with the pulley rims, the wrapped
//! arcs circles round their pulleys. A run has no body of its own, so its
//! side of a contact (`BeltSide`) takes its velocity from, and passes its
//! reaction to, the two pulleys at the run's tangent points, weighted by
//! where along the run the contact is.

use glam::DVec2;
use crate::sim::body::{Body, BodyShape};
use crate::sim::slot::Slot;
use crate::sim::constraint::Constraint;
use crate::sim::constraints::gear::{belt_strands, GearJoint, GearKind, BELT_WIDTH};

/// Natural frequency of the contact springs (rad/s) and its cap in ω·dt,
/// as for the slider end stops.
const OMEGA: f64 = 200.0;
const MAX_OMEGA_DT: f64 = 0.5;

#[derive(Clone, Copy, Debug)]
pub struct ContactPoint {
    /// Bodies, normal pointing from `i` to `j`.
    pub i: usize,
    pub j: usize,
    /// Which feature pair (several per rod–rod pair).
    pub feature: u8,
    pub n: DVec2,
    pub pen: f64,
    pub p: DVec2,
    /// Last normal and friction force (on `j`), filled after each step.
    pub f_n: f64,
    pub f_t: f64,
    /// Side `i` is a belt run, not body `i` (which is then pulley `a`).
    pub belt: Option<BeltSide>,
}

/// A belt run from tangent point `pa` on pulley `a` to `pb` on `b`; the
/// contact is a fraction `t` along it.
#[derive(Clone, Copy, Debug)]
pub struct BeltSide { pub a: usize, pub b: usize, pub t: f64, pub pa: DVec2, pub pb: DVec2 }

#[derive(Clone, Copy, Debug)]
struct Stick { i: usize, j: usize, feature: u8, s: f64 }

#[derive(Default)]
pub struct Contacts {
    /// Constrained pairs (i < j), which never collide.
    excluded: Vec<(usize, usize)>,
    colliders: Vec<usize>,
    /// Colliding belts' pulley pairs.
    belts: Vec<(usize, usize)>,
    sticks: Vec<Stick>,
    scratch: Vec<ContactPoint>,
    /// Contacts at the end of the last step, for drawing.
    pub last: Vec<ContactPoint>,
}

/// Closest point to `p` on segment a–b, and its parameter 0..1.
fn closest_on_segment(a: DVec2, b: DVec2, p: DVec2) -> (DVec2, f64) {
    let ab = b - a;
    let l2 = ab.length_squared();
    let t = if l2 > 1e-18 { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (a + ab * t, t)
}

/// Closest points between segments p1–q1 and p2–q2 (Ericson, RTCD 5.1.9).
fn closest_segments(p1: DVec2, q1: DVec2, p2: DVec2, q2: DVec2) -> (DVec2, f64, DVec2, f64) {
    let (d1, d2, r) = (q1 - p1, q2 - p2, p1 - p2);
    let (a, e, f) = (d1.length_squared(), d2.length_squared(), d2.dot(r));
    let (s, t);
    if a <= 1e-18 && e <= 1e-18 {
        return (p1, 0.0, p2, 0.0);
    }
    if a <= 1e-18 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= 1e-18 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s0 = if denom > 1e-18 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 { t0 = 0.0; s0 = (-c / a).clamp(0.0, 1.0); }
            else if t0 > 1.0 { t0 = 1.0; s0 = ((b - c) / a).clamp(0.0, 1.0); }
            s = s0;
            t = t0;
        }
    }
    (p1 + d1 * s, s, p2 + d2 * t, t)
}

/// A rod's end points and capsule radius.
fn segment(b: &Body) -> Option<(DVec2, DVec2, f64)> {
    let BodyShape::Rod { half_len, half_width } = b.shape else { return None };
    let r = b.rot();
    let u = DVec2::new(r.c, r.s) * half_len as f64;
    Some((b.pos - u, b.pos + u, half_width as f64))
}

fn bound_radius(b: &Body) -> f64 {
    match b.shape {
        BodyShape::Disk { radius } => radius as f64,
        BodyShape::Rod { half_len, half_width } => (half_len + half_width) as f64,
        BodyShape::Point => 0.0,
    }
}

/// A contact if `from` (on i's core: centre or segment point) and `to`
/// (on j's), with radii `ri`, `rj`, overlap.
#[allow(clippy::too_many_arguments)]
fn push_contact(out: &mut Vec<ContactPoint>, i: usize, j: usize, feature: u8, from: DVec2, to: DVec2, ri: f64, rj: f64, fallback: DVec2) {
    let d = to - from;
    let dist = d.length();
    let pen = ri + rj - dist;
    if pen <= 0.0 { return; }
    let n = if dist > 1e-9 { d / dist } else { fallback };
    out.push(ContactPoint { i, j, feature, n, pen, p: from + n * (ri - 0.5 * pen), f_n: 0.0, f_t: 0.0, belt: None });
}

/// Contacts between the capsule `a`–`b` of radius `h` (side i, index `i`)
/// and body `j`; features from `base`.
fn collide_capsule(bodies: &[Body], i: usize, j: usize, base: u8, (a, b, h): (DVec2, DVec2, f64), out: &mut Vec<ContactPoint>) {
    let bj = &bodies[j];
    let m = (b - a).perp().normalize_or(DVec2::Y);
    match bj.shape {
        BodyShape::Disk { radius } => {
            let (q, _) = closest_on_segment(a, b, bj.pos);
            push_contact(out, i, j, base, q, bj.pos, h, radius as f64, m);
        }
        BodyShape::Rod { .. } => {
            let (a2, b2, h2) = segment(bj).unwrap();
            let before = out.len();
            for (f, e) in [(0u8, a2), (1, b2)] {
                let (q, _) = closest_on_segment(a, b, e);
                push_contact(out, i, j, base + f, q, e, h, h2, m);
            }
            if out.len() == before {
                let (c1, _, c2, _) = closest_segments(a, b, a2, b2);
                push_contact(out, i, j, base + 2, c1, c2, h, h2, m);
            }
        }
        BodyShape::Point => {}
    }
}

/// Contacts between body `k` and the belt round pulleys `a`, `b`: its
/// two runs, and the arcs as circles round pulleys that don't collide
/// themselves.
fn collide_belt(bodies: &[Body], a: usize, b: usize, k: usize, out: &mut Vec<ContactPoint>) {
    let Some(runs) = belt_strands(&bodies[a], &bodies[b]) else { return };
    for (r, &(pa, pb)) in runs.iter().enumerate() {
        let before = out.len();
        collide_capsule(bodies, a, k, 16 + 4 * r as u8, (pa, pb, 0.5 * BELT_WIDTH), out);
        let ab = pb - pa;
        for c in &mut out[before..] {
            let t = ((c.p - pa).dot(ab) / ab.length_squared().max(1e-18)).clamp(0.0, 1.0);
            c.belt = Some(BeltSide { a, b, t, pa, pb });
        }
    }
    for p in [a, b] {
        let pulley = &bodies[p];
        let (true, Some(r)) = (!pulley.collide, disk_radius(pulley)) else { continue };
        let ring = (pulley.pos, pulley.pos, r + BELT_WIDTH);
        collide_capsule(bodies, p, k, 24, ring, out);
    }
}

fn disk_radius(b: &Body) -> Option<f64> {
    match b.shape { BodyShape::Disk { radius } => Some(radius as f64), _ => None }
}

/// Push contacts between bodies `i` and `j` onto `out`.
fn collide_pair(bodies: &[Body], i: usize, j: usize, out: &mut Vec<ContactPoint>) {
    let (bi, bj) = (&bodies[i], &bodies[j]);
    let push = |out: &mut Vec<ContactPoint>, feature, from, to, ri, rj, fallback| push_contact(out, i, j, feature, from, to, ri, rj, fallback);
    match (&bi.shape, &bj.shape) {
        (BodyShape::Disk { radius: ri }, BodyShape::Disk { radius: rj }) => {
            push(out, 0, bi.pos, bj.pos, *ri as f64, *rj as f64, DVec2::Y);
        }
        (BodyShape::Rod { .. }, BodyShape::Disk { radius }) => {
            let (a, b, hw) = segment(bi).unwrap();
            let (q, _) = closest_on_segment(a, b, bj.pos);
            push(out, 0, q, bj.pos, hw, *radius as f64, (b - a).perp().normalize_or(DVec2::Y));
        }
        (BodyShape::Disk { radius }, BodyShape::Rod { .. }) => {
            let (a, b, hw) = segment(bj).unwrap();
            let (q, _) = closest_on_segment(a, b, bi.pos);
            push(out, 0, bi.pos, q, *radius as f64, hw, -(b - a).perp().normalize_or(DVec2::Y));
        }
        (BodyShape::Rod { .. }, BodyShape::Rod { .. }) => {
            let (a1, b1, h1) = segment(bi).unwrap();
            let (a2, b2, h2) = segment(bj).unwrap();
            let before = out.len();
            // Each end against the other rod: two contacts when lying flat.
            let m2 = (b2 - a2).perp().normalize_or(DVec2::Y);
            let m1 = (b1 - a1).perp().normalize_or(DVec2::Y);
            for (f, e) in [(0u8, a1), (1, b1)] {
                let (q, _) = closest_on_segment(a2, b2, e);
                push(out, f, e, q, h1, h2, -m2);
            }
            for (f, e) in [(2u8, a2), (3, b2)] {
                let (q, _) = closest_on_segment(a1, b1, e);
                push(out, f, q, e, h1, h2, m1);
            }
            // Crossing in the middles (no end involved).
            if out.len() == before {
                let (c1, _, c2, _) = closest_segments(a1, b1, a2, b2);
                push(out, 4, c1, c2, h1, h2, m1);
            }
        }
        _ => {}
    }
}

impl Contacts {
    /// Refresh the collider list and the constrained pairs. Once per step.
    pub fn prepare(&mut self, bodies: &[Body], constraints: &[Slot<dyn Constraint>]) {
        self.colliders.clear();
        self.colliders.extend((0..bodies.len()).filter(|&k| bodies[k].collide && !matches!(bodies[k].shape, BodyShape::Point)));
        self.belts.clear();
        self.belts.extend(constraints.iter()
            .filter(|c| c.on)
            .filter_map(|c| c.as_any().downcast_ref::<GearJoint>())
            .filter(|g| g.kind == GearKind::Belt && g.collide)
            .map(|g| (g.body_a, g.body_b)));
        self.excluded.clear();
        if !self.active() { return; }
        // A switched-off joint no longer holds its bodies apart from contact.
        for c in constraints.iter().filter(|c| c.on) {
            let idx = c.body_indices();
            for (x, &a) in idx.iter().enumerate() {
                for &b in &idx[x + 1..] {
                    self.excluded.push((a.min(b), a.max(b)));
                }
            }
        }
        self.excluded.sort_unstable();
        self.excluded.dedup();
    }

    pub fn active(&self) -> bool {
        self.colliders.len() >= 2 || (!self.belts.is_empty() && !self.colliders.is_empty())
    }

    fn excluded_pair(&self, a: usize, b: usize) -> bool {
        self.excluded.binary_search(&(a.min(b), a.max(b))).is_ok()
    }

    fn detect(&self, bodies: &[Body], out: &mut Vec<ContactPoint>) {
        out.clear();
        let cs = &self.colliders;
        for (x, &i) in cs.iter().enumerate() {
            let (bi, ri) = (&bodies[i], bound_radius(&bodies[i]));
            for &j in &cs[x + 1..] {
                let bj = &bodies[j];
                if bi.fixed && bj.fixed { continue; }
                let reach = ri + bound_radius(bj);
                if (bj.pos - bi.pos).length_squared() > reach * reach { continue; }
                if self.excluded.binary_search(&(i, j)).is_ok() { continue; }
                collide_pair(bodies, i, j, out);
            }
        }
        for &(a, b) in &self.belts {
            for &k in cs {
                if k == a || k == b || self.excluded_pair(a, k) || self.excluded_pair(b, k) { continue; }
                if bodies[k].fixed && bodies[a].fixed && bodies[b].fixed { continue; }
                collide_belt(bodies, a, b, k, out);
            }
        }
    }

    /// Inverse effective mass of side i along `d` (body, or a belt run
    /// through its two pulleys).
    fn w_i(bodies: &[Body], c: &ContactPoint, d: DVec2) -> f64 {
        let w = |b: &Body, p: DVec2| {
            let rn = (p - b.pos).perp_dot(d);
            b.inv_mass() + b.inv_inertia() * rn * rn
        };
        match c.belt {
            Some(s) => (1.0 - s.t).powi(2) * w(&bodies[s.a], s.pa) + s.t.powi(2) * w(&bodies[s.b], s.pb),
            None => w(&bodies[c.i], c.p),
        }
    }

    /// Velocity of j's contact point relative to side i's surface.
    fn rel_vel(bodies: &[Body], c: &ContactPoint) -> DVec2 {
        let bj = &bodies[c.j];
        let vi = match c.belt {
            Some(s) => {
                let (a, b) = (&bodies[s.a], &bodies[s.b]);
                a.point_vel(s.pa - a.pos) * (1.0 - s.t) + b.point_vel(s.pb - b.pos) * s.t
            }
            None => bodies[c.i].point_vel(c.p - bodies[c.i].pos),
        };
        bj.point_vel(c.p - bj.pos) - vi
    }

    /// Side i's material: a belt takes its pulleys' average friction and
    /// larger restitution.
    fn material_i(bodies: &[Body], c: &ContactPoint) -> (f32, f32) {
        match c.belt {
            Some(s) => {
                let (a, b) = (&bodies[s.a], &bodies[s.b]);
                (0.5 * (a.friction + b.friction), a.restitution.max(b.restitution))
            }
            None => (bodies[c.i].friction, bodies[c.i].restitution),
        }
    }

    /// Spring rate and damping along direction `d` at point `p`.
    fn gains(&self, bodies: &[Body], c: &ContactPoint, d: DVec2, dt: f64, zeta: f64) -> Option<(f64, f64)> {
        let bj = &bodies[c.j];
        let rn = (c.p - bj.pos).perp_dot(d);
        let w_eff = Self::w_i(bodies, c, d) + bj.inv_mass() + bj.inv_inertia() * rn * rn;
        if w_eff <= 0.0 { return None; }
        let omega = OMEGA.min(MAX_OMEGA_DT / dt);
        Some((omega * omega / w_eff, 2.0 * zeta * omega / w_eff))
    }

    /// Pair friction: geometric mean, so either body at 0 is frictionless.
    fn friction(bodies: &[Body], c: &ContactPoint) -> f64 {
        (Self::material_i(bodies, c).0.max(0.0) as f64 * bodies[c.j].friction.max(0.0) as f64).sqrt()
    }

    /// Damping ratio for the bouncier of the two bodies' restitution.
    fn zeta(bodies: &[Body], c: &ContactPoint) -> f64 {
        let e = (Self::material_i(bodies, c).1.max(bodies[c.j].restitution) as f64).clamp(1e-4, 1.0);
        if e >= 1.0 { return 0.0; }
        let l = -e.ln();
        l / (std::f64::consts::PI * std::f64::consts::PI + l * l).sqrt()
    }

    fn stick(&self, c: &ContactPoint) -> f64 {
        self.sticks.iter()
            .find(|s| s.i == c.i && s.j == c.j && s.feature == c.feature)
            .map_or(0.0, |s| s.s)
    }

    /// Normal and friction force magnitudes (on `j`) at the current state.
    fn forces(&self, bodies: &[Body], c: &ContactPoint, s: f64, dt: f64) -> (f64, f64) {
        let v = Self::rel_vel(bodies, c);
        let Some((k, damp)) = self.gains(bodies, c, c.n, dt, Self::zeta(bodies, c)) else { return (0.0, 0.0) };
        let f_n = (k * c.pen - damp * c.n.dot(v)).max(0.0);
        let t = c.n.perp();
        let mu = Self::friction(bodies, c);
        if mu <= 0.0 || f_n <= 0.0 { return (f_n, 0.0); }
        let Some((kt, ct)) = self.gains(bodies, c, t, dt, 1.0) else { return (f_n, 0.0) };
        let f_t = (-kt * s - ct * t.dot(v)).clamp(-mu * f_n, mu * f_n);
        (f_n, f_t)
    }

    /// Add contact forces to the accumulators.
    pub fn apply(&mut self, bodies: &mut [Body], dt: f64) {
        if !self.active() { return; }
        let mut pts = std::mem::take(&mut self.scratch);
        self.detect(bodies, &mut pts);
        for c in &pts {
            let (f_n, f_t) = self.forces(bodies, c, self.stick(c), dt);
            let f = c.n * f_n + c.n.perp() * f_t;
            bodies[c.j].apply_force_at_world_point(f, c.p);
            match c.belt {
                Some(s) => {
                    bodies[s.a].apply_force_at_world_point(-f * (1.0 - s.t), s.pa);
                    bodies[s.b].apply_force_at_world_point(-f * s.t, s.pb);
                }
                None => bodies[c.i].apply_force_at_world_point(-f, c.p),
            }
        }
        self.scratch = pts;
    }

    /// Advance the stick springs over the step just taken and keep the
    /// contacts for drawing.
    pub fn post_step(&mut self, bodies: &[Body], dt: f64) {
        if !self.active() {
            self.sticks.clear();
            self.last.clear();
            return;
        }
        let mut pts = std::mem::take(&mut self.last);
        self.detect(bodies, &mut pts);
        let mut next = Vec::with_capacity(pts.len());
        for c in pts.iter_mut() {
            let mu = Self::friction(bodies, c);
            let s0 = self.stick(c);
            let v = Self::rel_vel(bodies, c);
            let t = c.n.perp();
            let mut s = s0 + t.dot(v) * dt;
            let (f_n, _) = self.forces(bodies, c, s, dt);
            if let Some((kt, _)) = self.gains(bodies, c, t, dt, 1.0) {
                // Slipping: the spring can't hold more than μ·N.
                let lim = mu * f_n / kt;
                s = s.clamp(-lim, lim);
            }
            let (f_n, f_t) = self.forces(bodies, c, s, dt);
            c.f_n = f_n;
            c.f_t = f_t;
            next.push(Stick { i: c.i, j: c.j, feature: c.feature, s });
        }
        self.sticks = next;
        self.last = pts;
    }

    /// Forget stick state (after the editor or a rewind teleports bodies).
    pub fn reset(&mut self) {
        self.sticks.clear();
        self.last.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec2;
    use crate::sim::body::{disk_inertia, rod_inertia};
    use crate::sim::forces::Gravity;
    use crate::sim::world::{Integrator, World};

    const BOTH: [Integrator; 2] = [Integrator::Xpbd, Integrator::Rk4];

    fn floor(w: &mut World, angle: f32) -> usize {
        let mut b = Body::new(Vec2::ZERO, angle, 10.0, rod_inertia(10.0, 5.0), BodyShape::Rod { half_len: 5.0, half_width: 0.1 }).fixed();
        b.collide = true;
        w.add_body(b)
    }

    fn block(pos: Vec2, angle: f32) -> Body {
        let mut b = Body::new(pos, angle, 1.0, rod_inertia(1.0, 0.3), BodyShape::Rod { half_len: 0.3, half_width: 0.1 });
        b.collide = true;
        b
    }

    /// Below the friction angle a block on a slope stays put; above it, it
    /// slides with a = g(sin α − μ cos α).
    #[test]
    fn block_sticks_then_slides() { for i in BOTH { block_sticks_then_slides_with(i) } }

    fn block_sticks_then_slides_with(integ: Integrator) {
        for (alpha, mu) in [(0.3f32, 0.5f32), (0.5, 0.2)] {
            let mut w = World::new();
            w.integrator = integ;
            let f = floor(&mut w, -alpha);
            let m = Vec2::new(alpha.sin(), alpha.cos());
            let b = w.add_body(block(m * 0.2, -alpha));
            for i in [f, b] {
                w.bodies[i].friction = mu;
                w.bodies[i].restitution = 0.0;
            }
            w.add_force(Gravity::new(9.81));
            for _ in 0..500 { w.step(0.0005); } // settle
            let p0 = w.bodies[b].pos;
            let t = 0.5;
            for _ in 0..1000 { w.step(0.0005); }
            let moved = (w.bodies[b].pos - p0).length() as f32;
            let acc = 9.81 * (alpha.sin() - mu * alpha.cos());
            let v0 = 0.0f32.max(acc * 0.25);
            let expected = if acc > 0.0 { v0 * t + 0.5 * acc * t * t } else { 0.0 };
            eprintln!("{integ:?} α={alpha} μ={mu}: moved {moved:.5}, expected ≈{expected:.5}, angle {:.4}", w.bodies[b].angle);
            if acc <= 0.0 {
                assert!(moved < 2e-3, "should stick, crept {moved}");
            } else {
                assert!((moved - expected).abs() / expected < 0.05);
            }
            assert!((w.bodies[b].angle as f32 + alpha).abs() < 0.01, "stays flat on the slope");
        }
    }

    /// A disk dropped on the floor bounces to e² of its height, and comes to
    /// rest on the surface (sub-millimetre sink).
    #[test]
    fn bounce_height_follows_restitution() { for i in BOTH { bounce_height_follows_restitution_with(i) } }

    fn bounce_height_follows_restitution_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        floor(&mut w, 0.0);
        let mut d = Body::new(Vec2::new(0.0, 1.3), 0.0, 1.0, disk_inertia(1.0, 0.2), BodyShape::Disk { radius: 0.2 });
        d.collide = true;
        d.restitution = 0.7; // the floor's default is lower; the larger wins
        let d = w.add_body(d);
        w.add_force(Gravity::new(9.81));
        let (mut hit, mut peak) = (false, f64::NEG_INFINITY);
        for _ in 0..12000 {
            w.step(0.0001);
            let y = w.bodies[d].pos.y;
            if y < 0.31 { hit = true; }
            if hit && w.bodies[d].vel.y > 0.0 { peak = peak.max(y); }
            if hit && peak > 0.0 && w.bodies[d].vel.y < 0.0 && y < peak - 0.05 { break; }
        }
        let rebound = (peak - 0.3) / 1.0;
        eprintln!("{integ:?} rebound {rebound:.3} (e² = 0.49)");
        assert!((rebound - 0.49).abs() < 0.08);
        for _ in 0..60000 { w.step(0.0001); }
        let rest = w.bodies[d].pos.y;
        eprintln!("{integ:?} rests at {rest}");
        assert!((rest - 0.3).abs() < 1e-3 && w.bodies[d].vel.length() < 1e-2);
    }

    /// Joined bodies don't collide with each other.
    #[test]
    fn constrained_pairs_ignored() {
        let mut w = World::new();
        let mut a = block(Vec2::ZERO, 0.0);
        let mut b = block(Vec2::new(0.3, 0.0), 0.0);
        a.collide = true;
        b.collide = true;
        let (a, b) = (w.add_body(a), w.add_body(b));
        w.add_constraint(crate::sim::constraints::PinJoint::new(a, Vec2::new(0.3, 0.0), b, Vec2::ZERO));
        w.step(0.001);
        assert!(w.contacts.last.is_empty());
    }

    /// A block dropped on a running conveyor belt is picked up to belt
    /// speed and carried along; its drag on the belt slows the pulleys.
    #[test]
    fn conveyor_carries_a_block() { for i in BOTH { conveyor_carries_a_block_with(i) } }

    fn conveyor_carries_a_block_with(integ: Integrator) {
        use crate::sim::constraints::{GearJoint, GearKind, PinWorld};
        let mut w = World::new();
        w.integrator = integ;
        let r = 0.2f32;
        let pulley = |x: f32| Body::new(Vec2::new(x, 0.0), 0.0, 40.0, disk_inertia(40.0, r), BodyShape::Disk { radius: r });
        let a = w.add_body(pulley(-1.0));
        let b = w.add_body(pulley(1.0));
        for p in [a, b] {
            w.add_constraint(PinWorld::new(p, Vec2::ZERO, w.bodies[p].pos32()));
            w.bodies[p].ang_vel = -4.0; // clockwise: the top run moves +x
        }
        let mut belt = GearJoint::new(a, b, GearKind::Belt, &w.bodies).unwrap();
        belt.collide = true;
        w.add_constraint(belt);
        let top = r as f64 + BELT_WIDTH;
        let mut block = block(Vec2::new(-0.6, top as f32 + 0.1 + 0.005), 0.0);
        block.friction = 0.8;
        let k = w.add_body(block);
        w.add_force(Gravity::new(9.81));
        let x0 = w.bodies[k].pos.x;
        for _ in 0..4000 { w.step(0.0002); } // 0.8 s
        let belt_speed = 4.0 * (r as f64 + 0.5 * BELT_WIDTH);
        let (v, y) = (w.bodies[k].vel.x, w.bodies[k].pos.y);
        eprintln!("{integ:?} block v={v:.3} (belt {belt_speed:.3}) moved {:.3} y={y:.4} pulley ω={:.3}",
            w.bodies[k].pos.x - x0, w.bodies[a].ang_vel);
        assert!((v - belt_speed).abs() < 0.1 * belt_speed, "carried at belt speed");
        assert!((y - (top + 0.1)).abs() < 5e-3, "rests on the belt");
        assert!(w.bodies[a].ang_vel > -4.0, "the block's drag slowed the belt");
    }
}
