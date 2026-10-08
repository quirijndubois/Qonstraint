use std::any::Any;
use glam::Vec2;
use crate::sim::body::Body;
use crate::sim::constraint::{Attach, Constraint, ConstraintEval};

/// Welds two bodies into one rigid piece: a point on A pinned to a point on
/// B (2 equations, as `PinJoint`) plus their relative angle held at what it
/// was when welded (1 equation). Builds compound parts (toothed wheels,
/// spoked hubs, brackets) from plain disks and rods.
pub struct WeldJoint {
    pub body_a: usize,
    pub local_a: Vec2,
    pub body_b: usize,
    pub local_b: Vec2,
    /// θ_b − θ_a held by the weld.
    pub angle: f64,
}

impl WeldJoint {
    /// Weld at world point `at` in the bodies' current relative pose.
    pub fn new(body_a: usize, body_b: usize, at: Vec2, bodies: &[Body]) -> Self {
        let (a, b) = (&bodies[body_a], &bodies[body_b]);
        Self {
            body_a, local_a: to_local(a, at),
            body_b, local_b: to_local(b, at),
            angle: b.angle - a.angle,
        }
    }
}

fn to_local(b: &Body, p: Vec2) -> Vec2 {
    let r = (p.as_dvec2() - b.pos).as_vec2();
    let (s, c) = b.angle32().sin_cos();
    Vec2::new(c * r.x + s * r.y, -s * r.x + c * r.y)
}

impl Constraint for WeldJoint {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 3 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let (a, b) = (&bodies[self.body_a], &bodies[self.body_b]);
        let pa = Attach::new(a, self.local_a);
        let pb = Attach::new(b, self.local_b);
        let c = pa.p - pb.p;
        out.c[..3].copy_from_slice(&[c.x, c.y, b.angle - a.angle - self.angle]);
        out.blocks[0] = pa.block(self.body_a, 1.0);
        out.blocks[0].j[2] = [0.0, 0.0, -1.0];
        out.blocks[1] = pb.block(self.body_b, -1.0);
        out.blocks[1].j[2] = [0.0, 0.0, 1.0];
        out.n_blocks = 2;
        if vel {
            let cd = pa.vel(a) - pb.vel(b);
            let bias = pa.centripetal(a) - pb.centripetal(b);
            out.c_dot[..3].copy_from_slice(&[cd.x, cd.y, b.ang_vel - a.ang_vel]);
            out.bias[..3].copy_from_slice(&[bias.x, bias.y, 0.0]);
        }
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
    use crate::sim::body::{rod_inertia, BodyShape};
    use crate::sim::constraints::PinWorld;
    use crate::sim::forces::Gravity;
    use crate::sim::world::{Integrator, World};

    /// An L of two welded rods swinging on a pivot keeps its shape and
    /// swings like the single rigid body it is (energy conserved under RK4).
    #[test]
    fn welded_l_stays_rigid() {
        for integ in [Integrator::Rk4, Integrator::Xpbd] {
            let mut w = World::new();
            w.integrator = integ;
            let rod = |pos: Vec2, angle: f32| Body::new(pos, angle, 1.0, rod_inertia(1.0, 0.5), BodyShape::Rod { half_len: 0.5, half_width: 0.05 });
            let a = w.add_body(rod(Vec2::new(0.5, 0.0), 0.0));
            let b = w.add_body(rod(Vec2::new(1.0, -0.5), -std::f32::consts::FRAC_PI_2));
            w.add_constraint(PinWorld::new(a, Vec2::new(-0.5, 0.0), Vec2::ZERO));
            w.add_constraint(WeldJoint::new(a, b, Vec2::new(1.0, 0.0), &w.bodies));
            w.add_force(Gravity::new(9.81));
            let e = |w: &World| w.kinetic_energy() + w.potential_energy(9.81);
            let e0 = e(&w);
            let mut max_err = 0.0f32;
            for _ in 0..4000 {
                w.step(0.0005);
                max_err = max_err.max(w.constraint_error());
            }
            let rel = w.bodies[b].angle - w.bodies[a].angle;
            eprintln!("{integ:?}: err {max_err:.1e}, angle {rel:.5}, E {e0:.4} → {:.4}", e(&w));
            assert!(max_err < 1e-3);
            assert!((rel + std::f64::consts::FRAC_PI_2).abs() < 1e-3);
            if integ == Integrator::Rk4 { assert!((e(&w) - e0).abs() < 1e-3); }
        }
    }
}
