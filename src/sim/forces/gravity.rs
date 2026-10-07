use std::any::Any;
use crate::sim::body::Body;
use crate::sim::force::Force;

pub struct Gravity {
    pub g: f32,
}

impl Gravity {
    pub fn new(g: f32) -> Self { Self { g } }
}

impl Force for Gravity {
    fn apply(&self, bodies: &mut [Body]) {
        for body in bodies.iter_mut() {
            if !body.fixed {
                body.force_accum.y -= self.g as f64 * body.mass as f64;
            }
        }
    }
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
}
