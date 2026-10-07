use std::any::Any;
use glam::Vec2;
use crate::sim::body::Body;
use crate::sim::force::Force;

/// Damped rotational spring between two bodies: a torque
/// −k·(θ_a − θ_b − rest) − c·(ω_a − ω_b) on `body_a` and the opposite on
/// `body_b`. It acts on relative angle only, so it is usually paired with
/// a pin joint (a sprung hinge, a balance wheel's hairspring); pointing it
/// at a fixed body springs one body against the world. `local_a` is only
/// where it is drawn.
pub struct TorsionSpring {
    pub body_a: usize,
    pub body_b: usize,
    pub local_a: Vec2,
    pub rest: f32,
    pub stiffness: f32,
    pub damping: f32,
}

impl TorsionSpring {
    /// Relaxed at the bodies' current relative angle.
    pub fn new(body_a: usize, body_b: usize, local_a: Vec2, stiffness: f32, damping: f32, bodies: &[Body]) -> Self {
        let rest = (bodies[body_a].angle - bodies[body_b].angle) as f32;
        Self { body_a, body_b, local_a, rest, stiffness, damping }
    }

    /// Twist away from rest (rad).
    pub fn twist(&self, bodies: &[Body]) -> f64 {
        bodies[self.body_a].angle - bodies[self.body_b].angle - self.rest as f64
    }
}

impl Force for TorsionSpring {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn body_indices(&self) -> Vec<usize> { vec![self.body_a, self.body_b] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.body_a == old { self.body_a = new; }
        if self.body_b == old { self.body_b = new; }
    }

    fn apply(&self, bodies: &mut [Body]) {
        let rel_w = bodies[self.body_a].ang_vel - bodies[self.body_b].ang_vel;
        let tau = -(self.stiffness as f64) * self.twist(bodies) - self.damping as f64 * rel_w;
        bodies[self.body_a].torque_accum += tau;
        bodies[self.body_b].torque_accum -= tau;
    }

    fn potential_energy(&self, bodies: &[Body]) -> f64 {
        let t = self.twist(bodies);
        0.5 * self.stiffness as f64 * t * t
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::body::{disk_inertia, BodyShape};
    use crate::sim::constraints::PinWorld;
    use crate::sim::world::World;

    /// A disk on a pin with a torsion spring to the world is a harmonic
    /// oscillator with ω = √(k/I), and conserves energy without damping.
    #[test]
    fn torsion_oscillator_period() {
        let mut w = World::new();
        let ground = w.add_body(Body::new(Vec2::ZERO, 0.0, 1.0, 1.0, BodyShape::Point).fixed());
        let d = w.add_body(Body::new(Vec2::ZERO, 0.0, 2.0, disk_inertia(2.0, 0.5), BodyShape::Disk { radius: 0.5 }));
        w.add_constraint(PinWorld::new(d, Vec2::ZERO, Vec2::ZERO));
        let k = 4.0;
        w.add_force(TorsionSpring::new(d, ground, Vec2::ZERO, k, 0.0, &w.bodies));
        w.bodies[d].angle = 0.5;
        let e0 = w.potential_energy(0.0);
        let omega = (k / disk_inertia(2.0, 0.5)).sqrt() as f64;
        let period = std::f64::consts::TAU / omega;
        let n = 10000;
        for _ in 0..n { w.step((period / n as f64) as f32); }
        let e1 = w.kinetic_energy() + w.potential_energy(0.0);
        eprintln!("θ after one period {} (start 0.5), E {e0} → {e1}", w.bodies[d].angle);
        assert!((w.bodies[d].angle - 0.5).abs() < 1e-3);
        assert!((e1 - e0).abs() < 1e-4 * e0);
    }
}
