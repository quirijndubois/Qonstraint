use std::any::Any;
use crate::sim::body::Body;
use crate::sim::force::Force;

/// Torque on one body: a constant drive minus viscous drag, τ − c·ω.
/// Drive alone spins things up; drag alone is a brake or a load (a
/// propeller, a dynamometer, bearing friction); both together settle at
/// ω = τ/c.
pub struct Motor {
    pub body: usize,
    pub torque: f32,
    pub drag: f32,
}

impl Motor {
    pub fn new(body: usize, torque: f32, drag: f32) -> Self { Self { body, torque, drag } }
}

impl Force for Motor {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn body_indices(&self) -> Vec<usize> { vec![self.body] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body == old { self.body = new; }
    }
    fn apply(&self, bodies: &mut [Body]) {
        let b = &mut bodies[self.body];
        if !b.fixed { b.torque_accum += self.torque as f64 - self.drag as f64 * b.ang_vel; }
    }
}
