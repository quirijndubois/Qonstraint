use std::any::Any;
use crate::sim::body::Body;

pub trait Force: Send + Sync + Any {
    fn apply(&self, bodies: &mut [Body]);
    fn potential_energy(&self, _bodies: &[Body]) -> f64 { 0.0 }
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    /// Body indices this force touches (for deletion filtering).
    fn body_indices(&self) -> Vec<usize> { vec![] }
    /// Remap a body index after deletion.
    fn remap_body(&mut self, _old: usize, _new: usize) {}
}
