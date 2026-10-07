use std::any::Any;
use std::f64::consts::{PI, TAU};
use crate::sim::body::{Body, BodyShape};
use crate::sim::constraint::{Constraint, ConstraintEval};

/// Rolling contact between two disks (external contact). Two equations:
///
///   row 0, contact:  |p_b - p_a| - (r_a + r_b) = 0
///   row 1, no-slip:  r_a·θ_a + r_b·θ_b - (r_a + r_b)·φ - k = 0
///
/// where φ is the direction angle of p_b - p_a. Its time derivative is the
/// usual velocity no-slip condition, but holding it at position level means
/// rotation is locked to the rolled distance and cannot drift. `k` is the
/// offset captured at creation (and by `rebase`). Radii are read from the
/// bodies each evaluation, so editing a disk's radius is picked up.
pub struct RollingContact {
    pub body_a: usize,
    pub body_b: usize,
    k: f64,
}

impl RollingContact {
    pub fn new(body_a: usize, body_b: usize, bodies: &[Body]) -> Option<Self> {
        disk_radius(&bodies[body_a])?;
        disk_radius(&bodies[body_b])?;
        let mut c = Self { body_a, body_b, k: 0.0 };
        c.rebase(bodies);
        Some(c)
    }

    fn radii(&self, bodies: &[Body]) -> (f64, f64) {
        (
            disk_radius(&bodies[self.body_a]).unwrap_or(0.0),
            disk_radius(&bodies[self.body_b]).unwrap_or(0.0),
        )
    }

    fn rolled(&self, bodies: &[Body], ra: f64, rb: f64) -> f64 {
        ra * bodies[self.body_a].angle + rb * bodies[self.body_b].angle
    }
}

pub(crate) fn disk_radius(body: &Body) -> Option<f64> {
    match body.shape {
        BodyShape::Disk { radius } => Some(radius as f64),
        _ => None,
    }
}

/// Wrap an angle to (-π, π].
fn wrap_pi(a: f64) -> f64 {
    a - TAU * ((a + PI) / TAU).floor()
}

impl Constraint for RollingContact {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 2 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let (ra, rb) = self.radii(bodies);
        let l = ra + rb;
        let (a, b) = (&bodies[self.body_a], &bodies[self.body_b]);
        let d = b.pos - a.pos;
        let len = d.length();
        let phi = d.y.atan2(d.x);
        // φ wraps every orbit; compare it with the angle implied by the
        // rolled amount and wrap the difference so C stays small.
        let expected_phi = (self.rolled(bodies, ra, rb) - self.k) / l;
        out.c[0] = len - l;
        out.c[1] = l * wrap_pi(expected_phi - phi);
        out.c_dot[..2].fill(0.0);
        out.bias[..2].fill(0.0);
        if len < 1e-9 { out.n_blocks = 0; return; }
        let n = d / len;
        let t = n.perp();
        let kk = l / len; // ∂φ/∂p_b = t/len, scaled by (r_a + r_b)

        out.blocks[0].body = self.body_a;
        out.blocks[0].j[0] = [-n.x, -n.y, 0.0];
        out.blocks[0].j[1] = [kk * t.x, kk * t.y, ra];
        out.blocks[1].body = self.body_b;
        out.blocks[1].j[0] = [n.x, n.y, 0.0];
        out.blocks[1].j[1] = [-kk * t.x, -kk * t.y, rb];
        out.n_blocks = 2;

        if vel {
            let v = b.vel - a.vel;
            let (nv, tv) = (n.dot(v), t.dot(v));
            out.c_dot[0] = nv;
            out.c_dot[1] = ra * a.ang_vel + rb * b.ang_vel - l * tv / len;
            // contact: ṅ·v_rel = (t̂·v)²/L
            // no-slip: -(r_a+r_b)·d/dt(t̂/L)·v_rel = 2(r_a+r_b)(n̂·v)(t̂·v)/L²
            out.bias[0] = tv * tv / len;
            out.bias[1] = 2.0 * l * nv * tv / (len * len);
        }
    }

    fn rebase(&mut self, bodies: &[Body]) {
        let (ra, rb) = self.radii(bodies);
        let d = bodies[self.body_b].pos - bodies[self.body_a].pos;
        self.k = self.rolled(bodies, ra, rb) - (ra + rb) * d.y.atan2(d.x);
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
    use crate::sim::body::disk_inertia;
    use crate::sim::world::{Integrator, World};

    const BOTH: [Integrator; 2] = [Integrator::Xpbd, Integrator::Rk4];

    fn disk(pos: Vec2, r: f32) -> Body {
        Body::new(pos, 0.0, 1.0, disk_inertia(1.0, r), BodyShape::Disk { radius: r })
    }

    /// A spinning disk on a fixed disk must roll around it, not spin in place.
    #[test]
    fn spin_converts_to_rolling_on_fixed_disk() { for i in BOTH { spin_converts_to_rolling_on_fixed_disk_with(i) } }

    fn spin_converts_to_rolling_on_fixed_disk_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let a = w.add_body(disk(Vec2::ZERO, 0.5).fixed());
        let b = w.add_body(disk(Vec2::new(0.0, 1.0), 0.5));
        w.add_constraint(RollingContact::new(a, b, &w.bodies).unwrap());
        w.bodies[b].ang_vel = 5.0;

        for _ in 0..1000 { w.step(0.001); }

        let bb = &w.bodies[b];
        let moved = (bb.pos32() - Vec2::new(0.0, 1.0)).length();
        let c = w.constraints[0].eval_c(&w.bodies);
        let slip = w.constraints[0].eval_c_dot(&w.bodies)[1];
        eprintln!("{integ:?} pos={:?} angle={} ω={} v={:?} moved={moved} C={c:?} slip_vel={slip}", bb.pos, bb.angle, bb.ang_vel, bb.vel);
        // Starts heavily violated (spin with no motion); Baumgarte settles it.
        assert!(c[0].abs() < 1e-3 && c[1].abs() < 1e-2, "constraint not held: {c:?}");
        assert!(slip.abs() < 5e-3, "no-slip violated: {slip}");
        assert!(moved > 0.05, "disk spun in place");
    }

    /// Rotation stays locked to position: after orbiting, the disk's angle is
    /// exactly what rolling the travelled arc implies (no drift).
    #[test]
    fn rotation_locked_to_rolled_distance() { for i in BOTH { rotation_locked_to_rolled_distance_with(i) } }

    fn rotation_locked_to_rolled_distance_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let a = w.add_body(disk(Vec2::ZERO, 0.5).fixed());
        let b = w.add_body(disk(Vec2::new(0.0, 1.0), 0.5));
        w.add_constraint(RollingContact::new(a, b, &w.bodies).unwrap());
        w.add_force(crate::sim::forces::Gravity::new(9.81));
        w.bodies[b].pos.x += 0.01; // nudge off the top so it rolls down

        for _ in 0..3000 { w.step(0.001); }

        // Rolling around the outside of an equal disk turns it the same way as
        // the orbit, twice per lap (coin rotation paradox): θ_b = (r_a+r_b)/r_b·Δφ
        let d = w.bodies[b].pos32();
        let dphi = d.y.atan2(d.x) - std::f32::consts::FRAC_PI_2;
        let expected = 2.0 * dphi;
        eprintln!("{integ:?} Δφ={dphi} θ={} expected≈{expected}", w.bodies[b].angle);
        assert!(dphi.abs() > 0.5, "disk should have rolled a fair way");
        assert!((w.bodies[b].angle32() - expected).abs() < 0.03);
    }
}
