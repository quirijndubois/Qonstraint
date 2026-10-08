use std::any::Any;
use glam::DVec2;
use crate::sim::body::{Body, BodyShape};
use crate::sim::constraint::{Constraint, ConstraintEval};
use super::rolling_contact::disk_radius;

/// A disk rolling without slip along one face of a rod. The rod may itself
/// move and rotate (free, pinned or fixed). Two equations, in the rod frame
/// (u = rod axis, m = rod normal, d = p_disk - p_rod, s = ±1 for the side):
///
///   row 0, contact:  s·(m·d) - (r + w) = 0          (w = rod half-width)
///   row 1, no-slip:  u·d + s·r·(θ_disk - θ_rod) - k = 0
///
/// Row 1 says the distance travelled along the rod equals the arc rolled
/// relative to the rod, so the disk's rotation is locked to its position.
/// Once the contact point rolls past an end of the rod (|u·d| > half
/// length) the constraint lets go and the disk falls; it takes hold again
/// (re-capturing `k`) if the disk comes back onto the same face within the
/// rod's length. Checked in `post_step`, so it never changes mid-step.
pub struct RollingOnRod {
    pub disk: usize,
    pub rod:  usize,
    side: f64,
    k:    f64,
    /// The disk is on the rod (false once it has rolled off an end).
    pub on: bool,
}

struct Frame { u: DVec2, m: DVec2, d: DVec2, r: f64, w: f64, half_len: f64 }

impl RollingOnRod {
    /// `None` unless `disk` is a disk and `rod` a rod. The side is taken from
    /// where the disk currently is relative to the rod.
    pub fn new(disk: usize, rod: usize, bodies: &[Body]) -> Option<Self> {
        disk_radius(&bodies[disk])?;
        let BodyShape::Rod { .. } = bodies[rod].shape else { return None };
        let mut c = Self { disk, rod, side: 1.0, k: 0.0, on: true };
        let f = c.frame(bodies);
        c.side = if f.m.dot(f.d) < 0.0 { -1.0 } else { 1.0 };
        c.rebase(bodies);
        Some(c)
    }

    fn frame(&self, bodies: &[Body]) -> Frame {
        let (disk, rod) = (&bodies[self.disk], &bodies[self.rod]);
        let (s, c) = rod.angle.sin_cos();
        let u = DVec2::new(c, s);
        let (half_len, w) = match rod.shape {
            BodyShape::Rod { half_len, half_width } => (half_len as f64, half_width as f64),
            _ => (0.0, 0.0),
        };
        Frame { u, m: u.perp(), d: disk.pos - rod.pos, r: disk_radius(disk).unwrap_or(0.0), w, half_len }
    }

    /// The contact point lies within the rod's length.
    fn within(f: &Frame) -> bool {
        f.u.dot(f.d).abs() <= f.half_len
    }

    /// The disk touches (or presses into) its face from its own side.
    fn touching(&self, f: &Frame) -> bool {
        let gap = self.side * f.m.dot(f.d) - (f.r + f.w);
        Self::within(f) && gap <= 0.0 && gap > -f.r
    }

    fn capture_k(&mut self, bodies: &[Body], f: &Frame) {
        let rel_angle = bodies[self.disk].angle - bodies[self.rod].angle;
        self.k = f.u.dot(f.d) + self.side * f.r * rel_angle;
    }
}

impl Constraint for RollingOnRod {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 2 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        if !self.on {
            out.n_blocks = 0;
            return;
        }
        let f = self.frame(bodies);
        let s = self.side;
        let (u, m, d) = (f.u, f.m, f.d);
        let (disk, rod) = (&bodies[self.disk], &bodies[self.rod]);
        let (ud, md) = (u.dot(d), m.dot(d));
        out.c[0] = s * md - (f.r + f.w);
        out.c[1] = ud + s * f.r * (disk.angle - rod.angle) - self.k;

        out.blocks[0].body = self.disk;
        out.blocks[0].j[0] = [s * m.x, s * m.y, 0.0];
        out.blocks[0].j[1] = [u.x, u.y, s * f.r];
        // ∂u/∂θ = m, ∂m/∂θ = -u
        out.blocks[1].body = self.rod;
        out.blocks[1].j[0] = [-s * m.x, -s * m.y, -s * ud];
        out.blocks[1].j[1] = [-u.x, -u.y, md - s * f.r];
        out.n_blocks = 2;

        if vel {
            let v = disk.vel - rod.vel;
            let wr = rod.ang_vel;
            let (uv, mv) = (u.dot(v), m.dot(v));
            out.c_dot[0] = s * mv - s * wr * ud;
            out.c_dot[1] = uv + s * f.r * disk.ang_vel + wr * (md - s * f.r);
            // From u̇ = ω_r·m, ṁ = -ω_r·u, ḋ = v_rel
            out.bias[0] = -2.0 * s * wr * uv - s * wr * wr * md;
            out.bias[1] = 2.0 * wr * mv - wr * wr * ud;
        }
    }

    fn rebase(&mut self, bodies: &[Body]) {
        let f = self.frame(bodies);
        // Editor moves put the disk wherever it was dropped: on the rod if
        // that's within its length (it is pulled onto the face), else off.
        self.on = Self::within(&f);
        self.capture_k(bodies, &f);
    }

    fn post_step(&mut self, bodies: &[Body]) {
        let f = self.frame(bodies);
        if self.on {
            if !Self::within(&f) { self.on = false; }
        } else if self.touching(&f) {
            self.on = true;
            self.capture_k(bodies, &f);
        }
    }

    fn save_state(&self, out: &mut Vec<f64>) {
        out.push(if self.on { 1.0 } else { 0.0 });
        out.push(self.k);
    }

    fn load_state(&mut self, input: &mut &[f64]) {
        let Some(([on, k], rest)) = input.split_first_chunk::<2>() else { return };
        self.on = *on != 0.0;
        self.k = *k;
        *input = rest;
    }

    fn body_indices(&self) -> Vec<usize> { vec![self.disk, self.rod] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.disk == old { self.disk = new; }
        if self.rod == old { self.rod = new; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec2;
    use crate::sim::body::{disk_inertia, rod_inertia};
    use crate::sim::constraints::PinWorld;
    use crate::sim::forces::Gravity;
    use crate::sim::world::{Integrator, World};

    const BOTH: [Integrator; 2] = [Integrator::Xpbd, Integrator::Rk4];

    const R: f32 = 0.3;
    const W: f32 = 0.06;

    fn disk_at(pos: Vec2) -> Body {
        Body::new(pos, 0.0, 1.0, disk_inertia(1.0, R), BodyShape::Disk { radius: R })
    }

    fn rod(pos: Vec2, angle: f32) -> Body {
        Body::new(pos, angle, 2.0, rod_inertia(2.0, 2.0), BodyShape::Rod { half_len: 2.0, half_width: W })
    }

    /// On a fixed incline a rolling disk accelerates at (2/3)·g·sin α and its
    /// rotation exactly matches the distance travelled.
    #[test]
    fn rolls_down_fixed_incline() { for i in BOTH { rolls_down_fixed_incline_with(i) } }

    fn rolls_down_fixed_incline_with(integ: Integrator) {
        let alpha = 0.3f32;
        let mut w = World::new();
        w.integrator = integ;
        let r = w.add_body(rod(Vec2::ZERO, -alpha).fixed());
        let m = Vec2::new(alpha.sin(), alpha.cos()); // rod normal (upward)
        let d = w.add_body(disk_at(m * (R + W)));
        w.add_constraint(RollingOnRod::new(d, r, &w.bodies).unwrap());
        w.add_force(Gravity::new(9.81));

        let t_end = 0.5;
        for _ in 0..500 { w.step(t_end / 500.0); }

        let travelled = (w.bodies[d].pos32() - m * (R + W)).length();
        let expected = 0.5 * (2.0 / 3.0) * 9.81 * alpha.sin() * t_end * t_end;
        let c = w.constraints[0].eval_c(&w.bodies);
        eprintln!("{integ:?} travelled={travelled} expected={expected} C={c:?} angle={}", w.bodies[d].angle);
        assert!((travelled - expected).abs() / expected < 0.01);
        assert!(c[0].abs() < 1e-4 && c[1].abs() < 1e-4);
        // Rolling downhill to the right turns the disk clockwise by s/R.
        assert!((w.bodies[d].angle32() + travelled / R).abs() < 1e-2);
    }

    /// A disk on a free, pinned rod: rod and disk interact, constraint holds,
    /// and a disk sitting still cannot spin in place.
    #[test]
    fn holds_on_moving_rod_and_locks_rotation() { for i in BOTH { holds_on_moving_rod_and_locks_rotation_with(i) } }

    fn holds_on_moving_rod_and_locks_rotation_with(integ: Integrator) {
        let mut w = World::new();
        w.integrator = integ;
        let r = w.add_body(rod(Vec2::ZERO, 0.2));
        w.add_constraint(PinWorld::new(r, Vec2::ZERO, Vec2::ZERO));
        let m = Vec2::new(-(0.2f32).sin(), (0.2f32).cos());
        let d = w.add_body(disk_at(m * (R + W) + Vec2::new(0.8, 0.0)));
        w.add_constraint(RollingOnRod::new(d, r, &w.bodies).unwrap());
        w.add_force(Gravity::new(9.81));
        w.bodies[d].ang_vel = 4.0; // try to spin it in place

        for _ in 0..2000 { w.step(0.001); }

        let c = w.constraints[1].eval_c(&w.bodies);
        eprintln!("{integ:?} C={c:?} rod θ={} disk pos={:?}", w.bodies[r].angle, w.bodies[d].pos);
        assert!(c[0].abs() < 1e-3 && c[1].abs() < 1e-3);
        assert!(w.bodies[r].angle32().abs() > 0.05, "rod should react to the disk");
    }

    /// Rolling down a fixed incline, the disk leaves the rod at its lower end
    /// and falls freely below it; a rewind before that puts it back on.
    #[test]
    fn falls_off_the_end() { for i in BOTH { falls_off_the_end_with(i) } }

    fn falls_off_the_end_with(integ: Integrator) {
        let alpha = 0.3f32;
        let mut w = World::new();
        w.integrator = integ;
        let r = w.add_body(rod(Vec2::ZERO, -alpha).fixed());
        let m = Vec2::new(alpha.sin(), alpha.cos());
        let d = w.add_body(disk_at(m * (R + W)));
        w.add_constraint(RollingOnRod::new(d, r, &w.bodies).unwrap());
        w.add_force(Gravity::new(9.81));
        let mut saved = Vec::new();
        w.save_state(&mut saved);

        for _ in 0..4000 { w.step(0.001); }

        let on = w.constraints[0].as_any().downcast_ref::<RollingOnRod>().unwrap().on;
        let p = w.bodies[d].pos32();
        eprintln!("{integ:?} on={on} disk at {p:?}");
        assert!(!on, "should have rolled off");
        // Past the lower end, well below where the rod's face would be.
        let end_y = -2.0 * alpha.sin();
        assert!(p.y < end_y - 1.0, "should be falling, at {p:?}");

        w.load_state(&saved);
        let on = w.constraints[0].as_any().downcast_ref::<RollingOnRod>().unwrap().on;
        assert!(on, "rewind restores the contact");
    }
}
