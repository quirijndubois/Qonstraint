use std::any::Any;
use crate::sim::body::Body;
use crate::sim::force::Force;

/// Torque on one body: a constant drive minus viscous drag, τ − c·ω.
/// Drive alone spins things up; drag alone is a brake or a load (a
/// propeller, a dynamometer, bearing friction); both together settle at
/// ω = τ/c.
///
/// With a `stator` the motor is mounted on that body (a chassis, a hull):
/// it pushes back on it with −τ and the drag acts on the spin relative to
/// it, so a vehicle's drive is internal and tips the body that carries it.
/// Without one it reacts on the world.
pub struct Motor {
    pub body: usize,
    pub torque: f32,
    pub drag: f32,
    pub stator: Option<usize>,
}

impl Motor {
    pub fn new(body: usize, torque: f32, drag: f32) -> Self { Self { body, torque, drag, stator: None } }

    /// The same motor mounted on `stator`.
    pub fn on(mut self, stator: usize) -> Self {
        self.stator = Some(stator);
        self
    }
}

impl Force for Motor {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn body_indices(&self) -> Vec<usize> {
        let mut v = vec![self.body];
        v.extend(self.stator);
        v
    }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body == old { self.body = new; }
        if self.stator == Some(old) { self.stator = Some(new); }
    }
    fn apply(&self, bodies: &mut [Body]) {
        let w_stator = self.stator.map_or(0.0, |s| bodies[s].ang_vel);
        let tau = self.torque as f64 - self.drag as f64 * (bodies[self.body].ang_vel - w_stator);
        let b = &mut bodies[self.body];
        if !b.fixed { b.torque_accum += tau; }
        if let Some(s) = self.stator {
            let s = &mut bodies[s];
            if !s.fixed { s.torque_accum -= tau; }
        }
    }
}
