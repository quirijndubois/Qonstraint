use std::any::Any;
use std::f64::consts::{PI, TAU};
use crate::sim::body::Body;
use crate::sim::constraint::{Constraint, ConstraintEval};
use super::rolling_contact::disk_radius;

/// How two wheels' rotations are tied together.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GearKind {
    /// Meshing teeth (or a crossed belt): the wheels turn opposite ways.
    Mesh,
    /// An open belt or chain (or an internal gear): they turn the same way.
    Belt,
}

impl GearKind {
    fn sign(self) -> f64 { match self { GearKind::Mesh => 1.0, GearKind::Belt => -1.0 } }
}

/// Ties the rotation of two disks: pitch circles roll on each other (mesh)
/// or are wrapped by a belt, with no slip. One equation,
///
///   r_a·(θ_a − φ) + s·r_b·(θ_b − φ) − k = 0,   s = +1 mesh, −1 belt
///
/// with φ the direction of the line of centres. Measuring rotation against
/// that line, not the world, is what makes it right when the axles move
/// (a planet gear on a carrier turns as it orbits). Only rotation is
/// constrained: the axles are held by whatever else (pins, a carrier). For
/// coaxial wheels φ is undefined and the world frame is used instead.
/// Radii come from the disks each evaluation, so resizing is picked up;
/// `k` holds the phase at creation (re-captured by `rebase`), so rotation
/// is locked at position level and can't drift.
pub struct GearJoint {
    pub body_a: usize,
    pub body_b: usize,
    pub kind: GearKind,
    k: f64,
}

/// Below this centre distance the line of centres is ignored.
const COAXIAL: f64 = 1e-6;

/// Wrap an angle to (-π, π].
fn wrap_pi(a: f64) -> f64 {
    a - TAU * ((a + PI) / TAU).floor()
}

impl GearJoint {
    /// `None` unless both bodies are (distinct) disks.
    pub fn new(body_a: usize, body_b: usize, kind: GearKind, bodies: &[Body]) -> Option<Self> {
        if body_a == body_b { return None; }
        disk_radius(&bodies[body_a])?;
        disk_radius(&bodies[body_b])?;
        let mut g = Self { body_a, body_b, kind, k: 0.0 };
        g.rebase(bodies);
        Some(g)
    }

    fn radii(&self, bodies: &[Body]) -> (f64, f64) {
        (
            disk_radius(&bodies[self.body_a]).unwrap_or(0.0),
            disk_radius(&bodies[self.body_b]).unwrap_or(0.0),
        )
    }

    /// r_a·θ_a + s·r_b·θ_b
    fn turned(&self, bodies: &[Body], ra: f64, rb: f64) -> f64 {
        ra * bodies[self.body_a].angle + self.kind.sign() * rb * bodies[self.body_b].angle
    }

    /// Angular velocity ratio ω_b / ω_a with the axles held still.
    pub fn ratio(&self, bodies: &[Body]) -> f64 {
        let (ra, rb) = self.radii(bodies);
        -self.kind.sign() * ra / rb.max(1e-9)
    }
}

impl Constraint for GearJoint {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 1 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let (ra, rb) = self.radii(bodies);
        let s = self.kind.sign();
        let kk = ra + s * rb; // coefficient of φ
        let (a, b) = (&bodies[self.body_a], &bodies[self.body_b]);
        let d = b.pos - a.pos;
        let len = d.length();
        let rel = self.turned(bodies, ra, rb) - self.k;

        out.blocks[0].body = self.body_a;
        out.blocks[1].body = self.body_b;
        out.n_blocks = 2;
        out.bias[0] = 0.0;

        if len < COAXIAL || kk.abs() < 1e-12 {
            // Coaxial (or equal-radius belt): rotation only, world frame.
            out.c[0] = rel;
            out.blocks[0].j[0] = [0.0, 0.0, ra];
            out.blocks[1].j[0] = [0.0, 0.0, s * rb];
            if vel { out.c_dot[0] = ra * a.ang_vel + s * rb * b.ang_vel; }
            return;
        }

        // φ wraps every orbit: compare it with the angle the turning implies.
        let phi = d.y.atan2(d.x);
        out.c[0] = kk * wrap_pi(rel / kk - phi);
        let n = d / len;
        let t = n.perp();
        let g = kk / len; // ∂(kφ)/∂p_b = k·t/len
        out.blocks[0].j[0] = [g * t.x, g * t.y, ra];
        out.blocks[1].j[0] = [-g * t.x, -g * t.y, s * rb];

        if vel {
            let v = b.vel - a.vel;
            let (nv, tv) = (n.dot(v), t.dot(v));
            out.c_dot[0] = ra * a.ang_vel + s * rb * b.ang_vel - kk * tv / len;
            // −k·φ̈ at q̈ = 0, as in RollingContact's no-slip row.
            out.bias[0] = 2.0 * kk * nv * tv / (len * len);
        }
    }

    fn rebase(&mut self, bodies: &[Body]) {
        let (ra, rb) = self.radii(bodies);
        let kk = ra + self.kind.sign() * rb;
        let d = bodies[self.body_b].pos - bodies[self.body_a].pos;
        let phi = if d.length() < COAXIAL || kk.abs() < 1e-12 { 0.0 } else { d.y.atan2(d.x) };
        self.k = self.turned(bodies, ra, rb) - kk * phi;
    }

    fn body_indices(&self) -> Vec<usize> { vec![self.body_a, self.body_b] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body_a == old { self.body_a = new; }
        if self.body_b == old { self.body_b = new; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec2;
    use crate::sim::body::{disk_inertia, BodyShape};
    use crate::sim::constraints::{PinJoint, PinWorld};
    use crate::sim::body::rod_inertia;
    use crate::sim::forces::Motor;
    use crate::sim::world::{Integrator, World};

    const BOTH: [Integrator; 2] = [Integrator::Xpbd, Integrator::Rk4];

    fn disk(pos: Vec2, r: f32) -> Body {
        Body::new(pos, 0.0, 1.0, disk_inertia(1.0, r), BodyShape::Disk { radius: r })
    }

    /// A driven pinion turns a gear three times its size a third as fast,
    /// the other way; a belt turns its pulley the same way.
    #[test]
    fn mesh_and_belt_ratios() { for i in BOTH { mesh_and_belt_ratios_with(i) } }

    fn mesh_and_belt_ratios_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let p = w.add_body(disk(Vec2::ZERO, 0.2));
        let g = w.add_body(disk(Vec2::new(0.8, 0.0), 0.6));
        let q = w.add_body(disk(Vec2::new(0.8, 2.0), 0.3));
        for (b, at) in [(p, Vec2::ZERO), (g, Vec2::new(0.8, 0.0)), (q, Vec2::new(0.8, 2.0))] {
            w.add_constraint(PinWorld::new(b, Vec2::ZERO, at));
        }
        w.add_constraint(GearJoint::new(p, g, GearKind::Mesh, &w.bodies).unwrap());
        w.add_constraint(GearJoint::new(g, q, GearKind::Belt, &w.bodies).unwrap());
        w.add_force(Motor::new(p, 1.0, 0.0));
        for _ in 0..2000 { w.step(0.001); }
        let (wp, wg, wq) = (w.bodies[p].ang_vel, w.bodies[g].ang_vel, w.bodies[q].ang_vel);
        let (tp, tg, tq) = (w.bodies[p].angle, w.bodies[g].angle, w.bodies[q].angle);
        eprintln!("{integ:?} ω = {wp:.4} {wg:.4} {wq:.4}  θ = {tp:.4} {tg:.4} {tq:.4}");
        assert!(wp > 1.0, "motor should spin the pinion up");
        assert!((wg + wp / 3.0).abs() < 1e-3 * wp);
        assert!((wq - 2.0 * wg).abs() < 1e-3 * wp);
        assert!((tg + tp / 3.0).abs() < 1e-4 && (tq - 2.0 * tg).abs() < 1e-4, "locked at position level");
    }

    /// A planet meshing a fixed sun, its axle on a carrier: turning the
    /// carrier once turns the planet (1 + r_sun/r_planet) times.
    #[test]
    fn planet_on_carrier() { for i in BOTH { planet_on_carrier_with(i) } }

    fn planet_on_carrier_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let sun = w.add_body(disk(Vec2::ZERO, 0.5).fixed());
        let carrier = w.add_body(Body::new(
            Vec2::new(0.4, 0.0), 0.0, 1.0, rod_inertia(1.0, 0.4),
            BodyShape::Rod { half_len: 0.4, half_width: 0.05 },
        ));
        w.add_constraint(PinWorld::new(carrier, Vec2::new(-0.4, 0.0), Vec2::ZERO));
        let planet = w.add_body(disk(Vec2::new(0.8, 0.0), 0.3));
        w.add_constraint(PinJoint::new(carrier, Vec2::new(0.4, 0.0), planet, Vec2::ZERO));
        w.add_constraint(GearJoint::new(sun, planet, GearKind::Mesh, &w.bodies).unwrap());
        w.add_force(Motor::new(carrier, 2.0, 0.0));
        for _ in 0..3000 { w.step(0.001); }
        let (tc, tp) = (w.bodies[carrier].angle, w.bodies[planet].angle);
        eprintln!("{integ:?} carrier {tc:.4} planet {tp:.4} ratio {:.4}", tp / tc);
        assert!(tc > 1.0);
        assert!((tp / tc - (1.0 + 0.5 / 0.3)).abs() < 1e-3);
    }
}
