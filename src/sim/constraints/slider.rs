use std::any::Any;
use glam::{DVec2, Vec2};
use crate::sim::body::{Body, BodyShape};
use crate::sim::constraint::{Attach, Constraint, ConstraintEval};

/// Half length of the carriage drawn around the rail (world units).
pub const COLLAR_HALF: f32 = 0.16;
/// Width of the end stops drawn on the rail.
pub const STOP_W: f32 = 0.05;
/// Natural frequency of the end-stop bumpers (rad/s): stiff enough to look
/// rigid (sub-millimetre squash under gravity).
const STOP_OMEGA: f64 = 150.0;
/// Upper bound on ω·dt for the bumpers. They are explicit forces, and both
/// integrators blow up once a step spans too much of a bumper oscillation
/// (symplectic Euler with this damping past ω·dt ≈ 1.1), so at few steps per
/// frame they soften instead.
const STOP_MAX_OMEGA_DT: f64 = 0.5;
const STOP_ZETA: f64 = 0.6;

/// A point on `rider` slides along the centre line of the rod `rail` (a
/// carriage on a rail). One equation, perpendicular offset m·d = 0, with
/// rotation free; `lock_rotation` adds θ_rider − θ_rail = const, turning it
/// into a prismatic joint. Travel stops where the carriage meets the rail's
/// end stops, through stiff damped bumpers (`apply_forces`), so it can't
/// run off the end.
pub struct SliderJoint {
    pub rider: usize,
    /// Attachment point on the rider, which sits on the rail's centre line.
    pub local: Vec2,
    pub rail: usize,
    pub lock_rotation: bool,
    angle0: f64,
}

/// Rail frame and the rider point, shared by the equations and the stops.
pub(crate) struct OnRail {
    /// Rail axis and normal.
    pub u: DVec2,
    pub m: DVec2,
    /// Rider point relative to the rail centre.
    pub d: DVec2,
    pub pt: Attach,
}

impl OnRail {
    pub fn new(bodies: &[Body], rider: usize, local: Vec2, rail: usize) -> Self {
        let rail_b = &bodies[rail];
        let r = rail_b.rot();
        let u = DVec2::new(r.c, r.s);
        let pt = Attach::new(&bodies[rider], local);
        Self { u, m: u.perp(), d: pt.p - rail_b.pos, pt }
    }

    /// Position of the rider point along the rail, from its centre.
    pub fn along(&self) -> f64 { self.u.dot(self.d) }

    /// Rate of `along`.
    pub fn along_rate(&self, bodies: &[Body], rider: usize, rail: usize) -> f64 {
        let (b, r) = (&bodies[rider], &bodies[rail]);
        self.u.dot(self.pt.vel(b) - r.vel) + r.ang_vel * self.m.dot(self.d)
    }

    /// Jacobians of `along` for rider and rail (∂u/∂θ_rail = m).
    pub fn along_jacobians(&self) -> ([f64; 3], [f64; 3]) {
        (
            [self.u.x, self.u.y, self.u.dot(self.pt.dp_dtheta())],
            [-self.u.x, -self.u.y, self.m.dot(self.d)],
        )
    }
}

/// Rows for "rider point on the rail's centre line" (row 0) and, if
/// `lock` is `Some(θ0)`, "rider angle − rail angle = θ0" (row 1).
pub(crate) fn eval_on_rail(
    bodies: &[Body], rider: usize, local: Vec2, rail: usize, lock: Option<f64>,
    vel: bool, out: &mut ConstraintEval,
) {
    let f = OnRail::new(bodies, rider, local, rail);
    let (b, rb) = (&bodies[rider], &bodies[rail]);
    let (u, m, d) = (f.u, f.m, f.d);

    out.c[0] = m.dot(d);
    out.blocks[0].body = rider;
    out.blocks[0].j[0] = [m.x, m.y, m.dot(f.pt.dp_dtheta())];
    // ∂m/∂θ_rail = −u
    out.blocks[1].body = rail;
    out.blocks[1].j[0] = [-m.x, -m.y, -u.dot(d)];
    out.n_blocks = 2;

    if let Some(a0) = lock {
        out.c[1] = b.angle - rb.angle - a0;
        out.blocks[0].j[1] = [0.0, 0.0, 1.0];
        out.blocks[1].j[1] = [0.0, 0.0, -1.0];
    }

    if vel {
        let dd = f.pt.vel(b) - rb.vel;
        let wr = rb.ang_vel;
        out.c_dot[0] = m.dot(dd) - wr * u.dot(d);
        // ṁ = −ω_r·u, u̇ = ω_r·m, rider point accelerates −ω_b²·r at q̈ = 0
        out.bias[0] = -2.0 * wr * u.dot(dd)
            - b.ang_vel * b.ang_vel * m.dot(f.pt.r)
            - wr * wr * m.dot(d);
        if lock.is_some() {
            out.c_dot[1] = b.ang_vel - rb.ang_vel;
            out.bias[1] = 0.0;
        }
    }
}

/// End stops: stiff damped bumpers keeping the rider point's position
/// along the rail within `lo..=hi`. They only ever push.
pub(crate) fn rail_stops(bodies: &mut [Body], rider: usize, local: Vec2, rail: usize, lo: f64, hi: f64, dt: f64) {
    let f = OnRail::new(bodies, rider, local, rail);
    let s = f.along();
    let pen = if s > hi { s - hi } else if s < lo { s - lo } else { return };

    let (jb, jr) = f.along_jacobians();
    let (b, rb) = (&bodies[rider], &bodies[rail]);
    let w = |body: &Body, j: [f64; 3]| body.inv_mass() * (j[0] * j[0] + j[1] * j[1]) + body.inv_inertia() * j[2] * j[2];
    let w_eff = w(b, jb) + w(rb, jr);
    if w_eff <= 0.0 { return; }
    let s_dot = f.along_rate(bodies, rider, rail);

    // Spring and damper tuned to the effective mass along the rail.
    let omega = STOP_OMEGA.min(STOP_MAX_OMEGA_DT / dt);
    let k = omega * omega / w_eff;
    let c = 2.0 * STOP_ZETA * omega / w_eff;
    let mut force = -k * pen - c * s_dot;
    if pen > 0.0 { force = force.min(0.0) } else { force = force.max(0.0) }

    for (idx, j) in [(rider, jb), (rail, jr)] {
        let body = &mut bodies[idx];
        if body.fixed { continue; }
        body.force_accum.x += force * j[0];
        body.force_accum.y += force * j[1];
        body.torque_accum += force * j[2];
    }
}

impl SliderJoint {
    /// `None` unless `rail` is a rod distinct from `rider`.
    pub fn new(rider: usize, local: Vec2, rail: usize, bodies: &[Body]) -> Option<Self> {
        if rider == rail { return None; }
        let BodyShape::Rod { .. } = bodies[rail].shape else { return None };
        let mut s = Self { rider, local, rail, lock_rotation: false, angle0: 0.0 };
        s.rebase(bodies);
        Some(s)
    }

    /// How far the attachment point may travel from the rail's centre.
    pub fn travel(rail: &Body) -> f32 {
        match rail.shape {
            BodyShape::Rod { half_len, .. } => (half_len - STOP_W - COLLAR_HALF).max(0.0),
            _ => 0.0,
        }
    }
}

impl Constraint for SliderJoint {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { if self.lock_rotation { 2 } else { 1 } }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let lock = self.lock_rotation.then_some(self.angle0);
        eval_on_rail(bodies, self.rider, self.local, self.rail, lock, vel, out);
    }

    fn apply_forces(&self, bodies: &mut [Body], dt: f64) {
        let lim = Self::travel(&bodies[self.rail]) as f64;
        rail_stops(bodies, self.rider, self.local, self.rail, -lim, lim, dt);
    }

    fn rebase(&mut self, bodies: &[Body]) {
        self.angle0 = bodies[self.rider].angle - bodies[self.rail].angle;
    }

    fn body_indices(&self) -> Vec<usize> { vec![self.rider, self.rail] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.rider == old { self.rider = new; }
        if self.rail == old { self.rail = new; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::body::rod_inertia;
    use crate::sim::forces::Gravity;
    use crate::sim::world::{Integrator, World};

    const BOTH: [Integrator; 2] = [Integrator::Xpbd, Integrator::Rk4];

    fn rod(pos: Vec2, angle: f32, hl: f32) -> Body {
        Body::new(pos, angle, 1.0, rod_inertia(1.0, hl), BodyShape::Rod { half_len: hl, half_width: 0.06 })
    }

    /// A carriage on a tilted fixed rail slides down (frictionless: a = g·sin α),
    /// stays on the rail, then stops at the end stop instead of running off.
    #[test]
    fn slides_down_and_stops_at_end() { for i in BOTH { slides_down_and_stops_at_end_with(i) } }

    fn slides_down_and_stops_at_end_with(integ: Integrator) {
        let alpha = 0.4f32;
        let mut w = World::new();
        w.integrator = integ;
        let rail = w.add_body(rod(Vec2::ZERO, -alpha, 2.0).fixed());
        let cart = w.add_body(rod(Vec2::ZERO, 0.3, 0.2));
        w.add_constraint(SliderJoint::new(cart, Vec2::ZERO, rail, &w.bodies).unwrap());
        w.add_force(Gravity::new(9.81));

        let t = 0.3;
        for _ in 0..3000 { w.step(t / 3000.0); }
        let travelled = w.bodies[cart].pos32().length();
        let expected = 0.5 * 9.81 * alpha.sin() * t * t;
        eprintln!("{integ:?} travelled={travelled} expected={expected}");
        assert!((travelled - expected).abs() / expected < 0.01);

        for _ in 0..30000 { w.step(0.0001); }
        let u = Vec2::new(alpha.cos(), -alpha.sin());
        let s = w.bodies[cart].pos32().dot(u);
        let lim = SliderJoint::travel(&w.bodies[rail]);
        let c = w.constraints[0].eval_c(&w.bodies);
        eprintln!("{integ:?} s={s} lim={lim} C={c:?} v={:?}", w.bodies[cart].vel);
        assert!(c[0].abs() < 1e-4, "left the rail: {c:?}");
        assert!((s - lim).abs() < 0.01, "should rest at the end stop");
        assert!(w.bodies[cart].vel.length() < 0.05, "should come to rest");
    }

    /// A long rod hanging through a fixed collar ends up resting on its end
    /// stop. At one step per frame that used to blow up under XPBD, because
    /// the stiff explicit bumper outran the step.
    #[test]
    fn end_stop_stable_at_large_steps() { for i in BOTH { end_stop_stable_at_large_steps_with(i) } }

    fn end_stop_stable_at_large_steps_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let collar = w.add_body(rod(Vec2::new(-0.5, 0.0), 0.0, 0.5).fixed());
        let a = 1.2f32;
        let rail = w.add_body(rod(-0.8 * Vec2::new(a.cos(), a.sin()), a, 2.0));
        w.add_constraint(SliderJoint::new(collar, Vec2::new(0.5, 0.0), rail, &w.bodies).unwrap());
        w.add_force(Gravity::new(9.81));

        let mut max_v = 0.0f64;
        for _ in 0..600 {
            w.step(1.0 / 60.0);
            max_v = max_v.max(w.bodies[rail].vel.length());
        }
        let c = w.constraints[0].eval_c(&w.bodies);
        eprintln!("{integ:?} max v={max_v} C={c:?}");
        assert!(max_v < 10.0 && c[0].abs() < 1e-3, "unstable: v={max_v} C={c:?}");
    }

    /// With rotation locked the carriage keeps its angle relative to a
    /// swinging rail.
    #[test]
    fn locked_rotation_follows_rail() { for i in BOTH { locked_rotation_follows_rail_with(i) } }

    fn locked_rotation_follows_rail_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let rail = w.add_body(rod(Vec2::new(1.0, 0.0), 0.0, 1.5));
        w.add_constraint(crate::sim::constraints::PinWorld::new(rail, Vec2::new(-1.0, 0.0), Vec2::ZERO));
        let cart = w.add_body(rod(Vec2::new(1.5, 0.0), 0.5, 0.2));
        let mut sj = SliderJoint::new(cart, Vec2::ZERO, rail, &w.bodies).unwrap();
        sj.lock_rotation = true;
        w.add_constraint(sj);
        w.add_force(Gravity::new(9.81));

        for _ in 0..5000 { w.step(0.0002); }
        let rel = w.bodies[cart].angle - w.bodies[rail].angle;
        let c = w.constraints[1].eval_c(&w.bodies);
        eprintln!("{integ:?} rel={rel} C={c:?} rail θ={}", w.bodies[rail].angle);
        assert!(w.bodies[rail].angle.abs() > 0.5, "rail should swing");
        assert!((rel - 0.5).abs() < 1e-3 && c[0].abs() < 1e-3);
    }
}

