use std::any::Any;
use glam::Vec2;
use crate::sim::body::Body;
use crate::sim::constraint::{Attach, Constraint, ConstraintEval};

/// Pins a point on body A to a point on body B (2 scalar equations).
pub struct PinJoint {
    pub body_a: usize,
    pub local_a: Vec2,
    pub body_b: usize,
    pub local_b: Vec2,
}

impl PinJoint {
    pub fn new(body_a: usize, local_a: Vec2, body_b: usize, local_b: Vec2) -> Self {
        Self { body_a, local_a, body_b, local_b }
    }
}

impl Constraint for PinJoint {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 2 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let (a, b) = (&bodies[self.body_a], &bodies[self.body_b]);
        let pa = Attach::new(a, self.local_a);
        let pb = Attach::new(b, self.local_b);
        let c = pa.p - pb.p;
        out.c[..2].copy_from_slice(&[c.x, c.y]);
        out.blocks[0] = pa.block(self.body_a, 1.0);
        out.blocks[1] = pb.block(self.body_b, -1.0);
        out.n_blocks = 2;
        if vel {
            let cd = pa.vel(a) - pb.vel(b);
            let bias = pa.centripetal(a) - pb.centripetal(b);
            out.c_dot[..2].copy_from_slice(&[cd.x, cd.y]);
            out.bias[..2].copy_from_slice(&[bias.x, bias.y]);
        }
    }

    fn body_indices(&self) -> Vec<usize> { vec![self.body_a, self.body_b] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body_a == old { self.body_a = new; }
        if self.body_b == old { self.body_b = new; }
    }
}
