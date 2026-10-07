use std::any::Any;
use glam::Vec2;
use crate::sim::body::Body;
use crate::sim::constraint::{Attach, Constraint, ConstraintEval};

/// Keeps the distance between two attachment points fixed (1 scalar equation).
pub struct DistanceConstraint {
    pub body_a: usize,
    pub local_a: Vec2,
    pub body_b: usize,
    pub local_b: Vec2,
    pub rest_len: f32,
}

impl DistanceConstraint {
    pub fn new(body_a: usize, local_a: Vec2, body_b: usize, local_b: Vec2, rest_len: f32) -> Self {
        Self { body_a, local_a, body_b, local_b, rest_len }
    }
}

impl Constraint for DistanceConstraint {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 1 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let (a, b) = (&bodies[self.body_a], &bodies[self.body_b]);
        let pa = Attach::new(a, self.local_a);
        let pb = Attach::new(b, self.local_b);
        let d = pb.p - pa.p;
        let len = d.length();
        out.c[0] = len - self.rest_len as f64;
        out.c_dot[0] = 0.0;
        out.bias[0] = 0.0;
        if len < 1e-9 { out.n_blocks = 0; return; }
        let n = d / len;

        out.blocks[0].body = self.body_a;
        out.blocks[0].j[0] = [-n.x, -n.y, -n.dot(pa.dp_dtheta())];
        out.blocks[1].body = self.body_b;
        out.blocks[1].j[0] = [n.x, n.y, n.dot(pb.dp_dtheta())];
        out.n_blocks = 2;

        if vel {
            let v_rel = pb.vel(b) - pa.vel(a);
            let nv = n.dot(v_rel);
            out.c_dot[0] = nv;
            // ṅ·v_rel plus the rotating attachment points' centripetal terms
            let n_dot_term = (v_rel.length_squared() - nv * nv) / len;
            out.bias[0] = n_dot_term + n.dot(pb.centripetal(b) - pa.centripetal(a));
        }
    }

    fn body_indices(&self) -> Vec<usize> { vec![self.body_a, self.body_b] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body_a == old { self.body_a = new; }
        if self.body_b == old { self.body_b = new; }
    }
}
