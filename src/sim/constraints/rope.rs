use std::any::Any;
use std::f64::consts::{PI, TAU};
use glam::{DVec2, Vec2};
use crate::sim::body::Body;
use crate::sim::constraint::{Attach, Constraint, ConstraintEval, JBlock};
use super::rolling_contact::disk_radius;

/// Slack below this (world units) switches the rope off: it can pull but
/// not push. Just inside it the rope still holds, so a hanging load sits
/// on an ordinary equality constraint instead of chattering on and off.
pub const SLACK_TOL: f64 = 1e-3;

/// A rope of fixed length from a point on `body_a` to a point on `body_b`,
/// optionally running over a pulley (any disk). One-sided: it holds the
/// ends at most `length` apart along its path, and goes slack (inactive)
/// when they come closer.
///
/// Over a pulley the path is: straight strand from a to its tangent point,
/// around the rim, straight strand to b. The length's gradient with respect
/// to an end is just the unit strand direction there (the moving tangent
/// point's contributions cancel), and with respect to the pulley centre
/// minus their sum. Row 1 adds no-slip on the pulley rim when `grip` is set
/// and the pulley is free, so the rope turns it (a heavy pulley slows an
/// Atwood machine); on a fixed pulley the rope simply slides.
pub struct Rope {
    pub body_a: usize,
    pub local_a: Vec2,
    pub body_b: usize,
    pub local_b: Vec2,
    pub pulley: Option<usize>,
    /// Which way the rope wraps the pulley going from a to b: +1
    /// counter-clockwise, −1 clockwise.
    pub wrap: f64,
    pub length: f32,
    pub grip: bool,
    /// No-slip offset (`rebase`).
    k: f64,
}

/// One strand: from the pulley's tangent point `t` to the end point.
struct Strand {
    end: Attach,
    t: DVec2,
    /// Unit direction tangent point → end.
    u: DVec2,
    len: f64,
    /// Polar angle of the tangent point on the pulley.
    phi: f64,
}

/// What `draw_world` needs: the polyline of the rope.
pub struct RopePath {
    pub a: Vec2,
    pub b: Vec2,
    /// Pulley centre, radius and the arc the rope lies on (start, sweep).
    pub over: Option<(Vec2, f32, f32, f32)>,
    pub ta: Vec2,
    pub tb: Vec2,
    pub slack: bool,
}

fn wrap_pi(a: f64) -> f64 {
    a - TAU * ((a + PI) / TAU).floor()
}

/// Tangent point from `p` to the circle (c, r), on side `side` (±1, the
/// sign of the rotation from c→p to c→t). None if p is inside the circle.
fn tangent(c: DVec2, r: f64, p: DVec2, side: f64) -> Option<(DVec2, f64)> {
    let d = p - c;
    let dist = d.length();
    if dist <= r * (1.0 + 1e-9) { return None; }
    let beta = (r / dist).acos() * side;
    let a = d.y.atan2(d.x) + beta;
    Some((c + r * DVec2::new(a.cos(), a.sin()), a))
}

impl Rope {
    /// A taut rope between two attachment points (body-local).
    pub fn new(body_a: usize, local_a: Vec2, body_b: usize, local_b: Vec2, bodies: &[Body]) -> Self {
        let mut r = Self { body_a, local_a, body_b, local_b, pulley: None, wrap: 1.0, length: 0.0, grip: true, k: 0.0 };
        r.length = r.current_length(bodies) as f32;
        r
    }

    /// Route the rope over `pulley` (a disk), on the side that bends it
    /// round the pulley rather than through it, taut at the current pose.
    /// `None` if `pulley` isn't a usable disk.
    pub fn over(mut self, pulley: usize, bodies: &[Body]) -> Option<Self> {
        if pulley == self.body_a || pulley == self.body_b { return None; }
        let r = disk_radius(&bodies[pulley])?;
        let c = bodies[pulley].pos;
        let pa = bodies[self.body_a].world_point_d(self.local_a.as_dvec2());
        let pb = bodies[self.body_b].world_point_d(self.local_b.as_dvec2());
        // The rope lies on the far side of the pulley from the a–b chord.
        let m = (pb - pa).perp();
        let far = if m.dot(c - (pa + pb) * 0.5) >= 0.0 { 1.0 } else { -1.0 };
        let mut best = None;
        for wrap in [1.0, -1.0] {
            let Some((ta, _)) = tangent(c, r, pa, wrap) else { continue };
            let score = far * m.dot(ta - c);
            if best.is_none_or(|(s, _)| score > s) { best = Some((score, wrap)); }
        }
        self.wrap = best?.1;
        self.pulley = Some(pulley);
        self.length = self.current_length(bodies) as f32;
        self.rebase(bodies);
        Some(self)
    }

    fn strands(&self, bodies: &[Body], p: usize) -> Option<(Strand, Strand, f64)> {
        let r = disk_radius(&bodies[p])?;
        let c = bodies[p].pos;
        let ea = Attach::new(&bodies[self.body_a], self.local_a);
        let eb = Attach::new(&bodies[self.body_b], self.local_b);
        let (ta, pha) = tangent(c, r, ea.p, self.wrap)?;
        let (tb, phb) = tangent(c, r, eb.p, -self.wrap)?;
        let mk = |end: Attach, t: DVec2, phi: f64| {
            let s = end.p - t;
            let len = s.length();
            Strand { end, t, u: if len > 1e-12 { s / len } else { DVec2::ZERO }, len, phi }
        };
        Some((mk(ea, ta, pha), mk(eb, tb, phb), r))
    }

    /// Arc the rope covers on the pulley, from a's tangent point to b's.
    fn arc(&self, pha: f64, phb: f64) -> f64 {
        (self.wrap * (phb - pha)).rem_euclid(TAU)
    }

    /// Length of the rope's path at the current pose.
    pub fn current_length(&self, bodies: &[Body]) -> f64 {
        let pa = bodies[self.body_a].world_point_d(self.local_a.as_dvec2());
        let pb = bodies[self.body_b].world_point_d(self.local_b.as_dvec2());
        match self.pulley.and_then(|p| self.strands(bodies, p)) {
            Some((sa, sb, r)) => sa.len + sb.len + r * self.arc(sa.phi, sb.phi),
            None => (pb - pa).length(),
        }
    }

    pub fn path(&self, bodies: &[Body]) -> RopePath {
        let pa = bodies[self.body_a].world_point(self.local_a);
        let pb = bodies[self.body_b].world_point(self.local_b);
        let slack = self.current_length(bodies) < self.length as f64 - SLACK_TOL;
        match self.pulley.and_then(|p| self.strands(bodies, p).map(|s| (p, s))) {
            Some((p, (sa, sb, r))) => {
                let sweep = self.wrap * self.arc(sa.phi, sb.phi);
                RopePath {
                    a: pa, b: pb, ta: sa.t.as_vec2(), tb: sb.t.as_vec2(), slack,
                    over: Some((bodies[p].pos32(), r as f32, sa.phi as f32, sweep as f32)),
                }
            }
            None => RopePath { a: pa, b: pb, ta: pa, tb: pb, over: None, slack },
        }
    }

    /// Rows active right now: the rope is taut and the pulley can turn.
    fn gripping(&self, bodies: &[Body]) -> bool {
        self.grip && self.pulley.is_some_and(|p| !bodies[p].fixed)
    }

    /// No-slip coordinate: rope paid out on a's side relative to the rim.
    /// s_a = −wrap makes its gradient w.r.t. the tangent angle vanish.
    fn no_slip(&self, sa: &Strand, r: f64, theta_p: f64) -> f64 {
        let s = -self.wrap;
        s * r * wrap_pi((sa.len - self.k) / (s * r) + sa.phi - theta_p)
    }
}

impl Constraint for Rope {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { if self.pulley.is_some() { 2 } else { 1 } }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let n = self.dim();
        out.c[..n].fill(0.0);
        out.c_dot[..n].fill(0.0);
        out.bias[..n].fill(0.0);
        out.n_blocks = 0;
        let (a, b) = (&bodies[self.body_a], &bodies[self.body_b]);
        let rest = self.length as f64;

        let Some(p) = self.pulley else {
            let ea = Attach::new(a, self.local_a);
            let eb = Attach::new(b, self.local_b);
            let d = eb.p - ea.p;
            let len = d.length();
            let c = len - rest;
            if c < -SLACK_TOL || len < 1e-9 { return; }
            let u = d / len;
            out.c[0] = c;
            out.blocks[0] = JBlock { body: self.body_a, ..Default::default() };
            out.blocks[0].j[0] = [-u.x, -u.y, -u.dot(ea.dp_dtheta())];
            out.blocks[1] = JBlock { body: self.body_b, ..Default::default() };
            out.blocks[1].j[0] = [u.x, u.y, u.dot(eb.dp_dtheta())];
            out.n_blocks = 2;
            if vel {
                let w = eb.vel(b) - ea.vel(a);
                let uw = u.dot(w);
                out.c_dot[0] = uw;
                out.bias[0] = (w.length_squared() - uw * uw) / len
                    + u.dot(eb.centripetal(b) - ea.centripetal(a));
            }
            return;
        };

        let Some((sa, sb, r)) = self.strands(bodies, p) else { return };
        let c = sa.len + sb.len + r * self.arc(sa.phi, sb.phi) - rest;
        if c < -SLACK_TOL || sa.len < 1e-9 || sb.len < 1e-9 { return; }
        let pul = &bodies[p];
        let grip = self.gripping(bodies);

        out.c[0] = c;
        let uc = -(sa.u + sb.u);
        out.blocks[0] = JBlock { body: self.body_a, ..Default::default() };
        out.blocks[0].j[0] = [sa.u.x, sa.u.y, sa.u.dot(sa.end.dp_dtheta())];
        out.blocks[1] = JBlock { body: self.body_b, ..Default::default() };
        out.blocks[1].j[0] = [sb.u.x, sb.u.y, sb.u.dot(sb.end.dp_dtheta())];
        out.blocks[2] = JBlock { body: p, ..Default::default() };
        out.blocks[2].j[0] = [uc.x, uc.y, 0.0];
        out.n_blocks = 3;
        let s = -self.wrap;
        if grip {
            out.c[1] = self.no_slip(&sa, r, pul.angle);
            out.blocks[0].j[1] = out.blocks[0].j[0];
            out.blocks[2].j[1] = [-sa.u.x, -sa.u.y, -s * r];
        }

        if vel {
            // Rate of a strand's length and its J̇q̇ (the strand turning, as
            // in a distance constraint, plus the end point's centripetal term).
            let strand = |st: &Strand, body: &Body| {
                let w = st.end.vel(body) - pul.vel;
                let uw = st.u.dot(w);
                (uw, (w.length_squared() - uw * uw) / st.len + st.u.dot(st.end.centripetal(body)))
            };
            let (ra, ba) = strand(&sa, a);
            let (rb, bb) = strand(&sb, b);
            out.c_dot[0] = ra + rb;
            out.bias[0] = ba + bb;
            if grip {
                out.c_dot[1] = ra - s * r * pul.ang_vel;
                out.bias[1] = ba;
            }
        }
    }

    fn rebase(&mut self, bodies: &[Body]) {
        let Some(p) = self.pulley else { return };
        let Some((sa, _, r)) = self.strands(bodies, p) else { return };
        let s = -self.wrap;
        self.k = sa.len + s * r * (sa.phi - bodies[p].angle);
    }

    fn body_indices(&self) -> Vec<usize> {
        let mut v = vec![self.body_a, self.body_b];
        v.extend(self.pulley);
        v
    }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body_a == old { self.body_a = new; }
        if self.body_b == old { self.body_b = new; }
        if self.pulley == Some(old) { self.pulley = Some(new); }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::body::{disk_inertia, BodyShape};
    use crate::sim::constraints::PinWorld;
    use crate::sim::forces::Gravity;
    use crate::sim::world::{Integrator, World};

    const BOTH: [Integrator; 2] = [Integrator::Xpbd, Integrator::Rk4];

    fn disk(pos: Vec2, r: f32, m: f32) -> Body {
        Body::new(pos, 0.0, m, disk_inertia(m, r), BodyShape::Disk { radius: r })
    }

    /// The Jacobian matches finite differences of C, for both rows, with
    /// every body (ends and pulley) displaced and turned.
    #[test]
    fn jacobian_matches_finite_differences() {
        let mut bodies = vec![
            Body::new(Vec2::new(-1.3, -2.0), 0.4, 1.0, 1.0, BodyShape::Rod { half_len: 0.3, half_width: 0.05 }),
            disk(Vec2::new(0.9, -1.5), 0.2, 1.0),
            disk(Vec2::new(0.1, 0.2), 0.4, 1.0),
        ];
        let rope = Rope::new(0, Vec2::new(0.25, 0.0), 1, Vec2::new(0.0, 0.1), &bodies).over(2, &bodies).unwrap();
        // Stretch it a little so it is taut.
        let mut rope = rope;
        rope.length -= 0.01;
        let mut e = ConstraintEval::default();
        rope.evaluate(&bodies, false, &mut e);
        assert_eq!(e.n_blocks, 3);
        let h = 1e-6;
        for blk in e.blocks().to_vec() {
            for col in 0..3 {
                let bump = |bodies: &mut Vec<Body>, d: f64| {
                    let b = &mut bodies[blk.body];
                    match col { 0 => b.pos.x += d, 1 => b.pos.y += d, _ => b.angle += d }
                };
                bump(&mut bodies, h);
                let mut ep = ConstraintEval::default();
                rope.evaluate(&bodies, false, &mut ep);
                bump(&mut bodies, -2.0 * h);
                let mut em = ConstraintEval::default();
                rope.evaluate(&bodies, false, &mut em);
                bump(&mut bodies, h);
                for row in 0..2 {
                    let fd = (ep.c[row] - em.c[row]) / (2.0 * h);
                    assert!((fd - blk.j[row][col]).abs() < 1e-5,
                        "body {} row {row} col {col}: fd {fd} vs J {}", blk.body, blk.j[row][col]);
                }
            }
        }
    }

    /// Atwood machine with a heavy pulley: a = (m1 − m2)·g / (m1 + m2 + I/r²),
    /// and the pulley turns with the rope.
    #[test]
    fn atwood_with_massive_pulley() { for i in BOTH { atwood_with_massive_pulley_with(i) } }

    fn atwood_with_massive_pulley_with(integ: Integrator) {
        let (m1, m2, mp, r) = (3.0f32, 1.0f32, 2.0f32, 0.3f32);
        let mut w = World::new();
        w.integrator = integ;
        let pul = w.add_body(disk(Vec2::ZERO, r, mp));
        w.add_constraint(PinWorld::new(pul, Vec2::ZERO, Vec2::ZERO));
        let a = w.add_body(disk(Vec2::new(-r, -2.0), 0.1, m1));
        let b = w.add_body(disk(Vec2::new(r, -2.0), 0.1, m2));
        w.add_constraint(Rope::new(a, Vec2::ZERO, b, Vec2::ZERO, &w.bodies).over(pul, &w.bodies).unwrap());
        w.add_force(Gravity::new(9.81));
        let t = 0.5;
        let n = 2000;
        for _ in 0..n { w.step(t / n as f32); }
        let drop = -2.0 - w.bodies[a].pos.y as f32;
        let i_over_r2 = disk_inertia(mp, r) / (r * r);
        let acc = (m1 - m2) * 9.81 / (m1 + m2 + i_over_r2);
        let expected = 0.5 * acc * t * t;
        let turned = w.bodies[pul].angle as f32;
        eprintln!("{integ:?} drop {drop:.5} expected {expected:.5}, pulley turned {turned:.4} (expect {:.4}) b rose {:.5}",
            drop / r, w.bodies[b].pos.y + 2.0);
        assert!((drop - expected).abs() / expected < 0.01);
        assert!((w.bodies[b].pos.y as f32 + 2.0 - drop).abs() < 1e-3, "rope length kept");
        // Heavy side on the left going down turns the pulley counter-clockwise.
        assert!((turned - drop / r).abs() < 1e-2);
    }

    /// Thrown upward, an end goes slack and flies free; the rope never pushes.
    #[test]
    fn goes_slack_and_never_pushes() { for i in BOTH { goes_slack_and_never_pushes_with(i) } }

    fn goes_slack_and_never_pushes_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let anchor = w.add_body(Body::new(Vec2::ZERO, 0.0, 1.0, 1.0, BodyShape::Point).fixed());
        let bob = w.add_body(disk(Vec2::new(0.0, -1.0), 0.1, 1.0));
        w.add_constraint(Rope::new(anchor, Vec2::ZERO, bob, Vec2::ZERO, &w.bodies));
        w.add_force(Gravity::new(9.81));
        w.bodies[bob].vel.y = 3.0;
        let mut peak = f64::NEG_INFINITY;
        for _ in 0..300 { w.step(0.001); peak = peak.max(w.bodies[bob].pos.y); }
        // Free flight: y = −1 + 3t − g t²/2, peak at t = 0.306 s.
        let free_peak = -1.0 + 9.0 / (2.0 * 9.81);
        eprintln!("{integ:?} peak {peak:.5} free {free_peak:.5}");
        assert!((peak - free_peak).abs() < 2e-3);
        for _ in 0..3000 { w.step(0.001); }
        let len = (w.bodies[bob].pos).length();
        eprintln!("{integ:?} length after falling back {len}");
        assert!(len < 1.0 + 2e-3, "rope caught it");
    }
}
