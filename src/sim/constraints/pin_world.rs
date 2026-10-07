use std::any::Any;
use glam::Vec2;
use crate::sim::body::Body;
use crate::sim::constraint::{Attach, Constraint, ConstraintEval};

/// Pins a point on a body to a fixed world position (2 scalar equations).
pub struct PinWorld {
    pub body: usize,
    pub local: Vec2,
    pub target: Vec2,
}

impl PinWorld {
    pub fn new(body: usize, local: Vec2, target: Vec2) -> Self {
        Self { body, local, target }
    }
}

impl Constraint for PinWorld {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 2 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        let b = &bodies[self.body];
        let p = Attach::new(b, self.local);
        let c = p.p - self.target.as_dvec2();
        out.c[..2].copy_from_slice(&[c.x, c.y]);
        out.blocks[0] = p.block(self.body, 1.0);
        out.n_blocks = 1;
        if vel {
            let v = p.vel(b);
            let bias = p.centripetal(b);
            out.c_dot[..2].copy_from_slice(&[v.x, v.y]);
            out.bias[..2].copy_from_slice(&[bias.x, bias.y]);
        }
    }

    fn body_indices(&self) -> Vec<usize> { vec![self.body] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body == old { self.body = new; }
    }
}
