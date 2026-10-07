use std::any::Any;
use glam::Vec2;
use crate::sim::body::Body;
use crate::sim::force::Force;

/// Damped spring between two attachment points on two bodies.
pub struct SpringDamper {
    pub body_a: usize,
    pub local_a: Vec2,
    pub body_b: usize,
    pub local_b: Vec2,
    pub rest_len: f32,
    pub stiffness: f32,
    pub damping: f32,
}

impl SpringDamper {
    pub fn new(
        body_a: usize, local_a: Vec2,
        body_b: usize, local_b: Vec2,
        rest_len: f32, stiffness: f32, damping: f32,
    ) -> Self {
        Self { body_a, local_a, body_b, local_b, rest_len, stiffness, damping }
    }
}

impl Force for SpringDamper {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn body_indices(&self) -> Vec<usize> { vec![self.body_a, self.body_b] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body_a == old { self.body_a = new; }
        if self.body_b == old { self.body_b = new; }
    }
    fn apply(&self, bodies: &mut [Body]) {
        let (a, b) = (&bodies[self.body_a], &bodies[self.body_b]);
        let ra = a.rot().apply(self.local_a.as_dvec2());
        let rb = b.rot().apply(self.local_b.as_dvec2());
        let (pa, pb) = (a.pos + ra, b.pos + rb);

        let delta = pb - pa;
        let len = delta.length();
        if len < 1e-9 { return; }
        let n = delta / len;

        let spring_force = self.stiffness as f64 * (len - self.rest_len as f64);
        let damp_force = self.damping as f64 * (b.point_vel(rb) - a.point_vel(ra)).dot(n);
        let force = (spring_force + damp_force) * n;

        bodies[self.body_a].apply_force_at_world_point(force, pa);
        bodies[self.body_b].apply_force_at_world_point(-force, pb);
    }

    fn potential_energy(&self, bodies: &[Body]) -> f64 {
        let pa = bodies[self.body_a].world_point_d(self.local_a.as_dvec2());
        let pb = bodies[self.body_b].world_point_d(self.local_b.as_dvec2());
        let stretch = (pb - pa).length() - self.rest_len as f64;
        0.5 * self.stiffness as f64 * stretch * stretch
    }
}
