use std::any::Any;
use std::sync::{Arc, Mutex};
use glam::Vec2;
use crate::sim::body::Body;
use crate::sim::force::Force;

pub struct MouseSpringData {
    pub active:       bool,
    pub body_idx:     usize,
    pub local_attach: Vec2,
    pub target:       Vec2,
    pub k:            f32,
    pub damping:      f32,
}

impl Default for MouseSpringData {
    fn default() -> Self {
        Self {
            active: false,
            body_idx: 0,
            local_attach: Vec2::ZERO,
            target: Vec2::ZERO,
            k: 350.0,
            damping: 20.0,
        }
    }
}

pub struct MouseSpring(pub Arc<Mutex<MouseSpringData>>);

impl Force for MouseSpring {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn apply(&self, bodies: &mut [Body]) {
        let d = self.0.lock().unwrap();
        if !d.active || d.body_idx >= bodies.len() { return; }
        let body = &mut bodies[d.body_idx];
        if body.fixed { return; }
        let r        = body.rot().apply(d.local_attach.as_dvec2());
        let world_pt = body.pos + r;
        let vel      = body.point_vel(r);
        let force    = d.k as f64 * (d.target.as_dvec2() - world_pt) - d.damping as f64 * vel;
        body.apply_force_at_world_point(force, world_pt);
    }
}
