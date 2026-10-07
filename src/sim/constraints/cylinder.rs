use std::any::Any;
use glam::Vec2;
use crate::sim::body::{Body, BodyShape};
use crate::sim::constraint::{Constraint, ConstraintEval};
use super::slider::{eval_on_rail, rail_stops, OnRail};

/// Ratio of specific heats for the gas (air–fuel mix ≈ 1.3).
const GAMMA: f64 = 1.3;
/// Pressure rise at ignition over atmospheric, at full throttle.
const FIRE_OVER_ATM: f64 = 40.0;
/// Atmospheric pressure × piston width (2D "area"), force per unit bore.
const ATM_PER_BORE: f32 = 32.0;
/// The crown may not come closer to the head than this.
const MIN_CLEARANCE: f64 = 0.02;
/// A turn (dead centre) is only taken once the gas column has reversed by
/// this much, so numerical jitter at rest can't step the cycle.
const TURN_EPS: f64 = 1e-4;

/// What the gas in a cylinder does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GasMode {
    /// Sealed gas: an adiabatic air spring, at atmospheric pressure in the
    /// position it was built (or left the editor) in.
    Sealed,
    /// Ignites at every top dead centre: compression, power, compression…
    TwoStroke,
    /// Intake, compression, power (ignition at TDC), exhaust.
    FourStroke,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stroke { Intake, Compression, Power, Exhaust }

impl Stroke {
    /// Strokes that move the piston towards the head.
    fn compressing(self) -> bool { matches!(self, Stroke::Compression | Stroke::Exhaust) }
    /// Valves shut: the gas follows an adiabat.
    fn sealed(self) -> bool { matches!(self, Stroke::Compression | Stroke::Power) }
}

/// What `draw_world` needs to show a cylinder's state.
#[derive(Clone, Copy, Debug)]
pub struct CylinderState {
    pub stroke: Stroke,
    /// Progress through the current stroke, 0..1 (by travel).
    pub progress: f32,
    /// Gauge pressure as a fraction of the full-throttle ignition rise.
    pub pressure: f32,
    /// Crown position along the barrel, from the barrel's centre.
    pub crown: f32,
}

/// A piston in a barrel, with gas between the piston crown and the head.
///
/// The barrel is any rod (fixed, pinned, free); its head is at local
/// `+half_len` and its bore radius is the rod's half width. The piston is
/// any body: its point `local` (the wrist pin) runs on the barrel axis with
/// rotation locked to the barrel, and its crown sits `crown` further
/// towards the head. Gas pressure pushes piston and head apart.
///
/// The cycle times itself from the piston's own motion: it turns at each
/// dead centre (the gas column switching between shrinking and growing),
/// igniting at TDC in the combustion modes. So it works in any mechanism,
/// with nothing to tell it the crank angle.
pub struct Cylinder {
    pub piston: usize,
    pub local: Vec2,
    pub barrel: usize,
    pub crown: f32,
    pub mode: GasMode,
    /// 0..1, scales the ignition pressure rise.
    pub throttle: f32,
    pub stroke: Stroke,
    angle0: f64,
    /// Adiabat in force: p·h^γ through (h_ref, p_ref), absolute pressure.
    h_ref: f64,
    p_ref: f64,
    /// Gas column at the last dead centre, its extreme since, and the
    /// length of the last full stroke (for drawing progress).
    h_turn: f64,
    h_ext: f64,
    last_len: f64,
}

impl Cylinder {
    /// `None` unless `barrel` is a rod distinct from `piston`. Starts sealed
    /// at the current position; set `mode` and call `begin` for an engine.
    pub fn new(piston: usize, local: Vec2, barrel: usize, bodies: &[Body]) -> Option<Self> {
        if piston == barrel { return None; }
        let BodyShape::Rod { half_len, .. } = bodies[barrel].shape else { return None };
        let crown = match bodies[piston].shape {
            BodyShape::Rod { half_width, .. } => half_width,
            BodyShape::Disk { radius } => radius,
            BodyShape::Point => 0.05,
        };
        let mut c = Self {
            piston, local, barrel, crown,
            mode: GasMode::Sealed, throttle: 1.0, stroke: Stroke::Compression,
            angle0: 0.0, h_ref: 1.0, p_ref: 0.0, h_turn: 0.0, h_ext: 0.0,
            last_len: half_len as f64,
        };
        c.rebase(bodies);
        Some(c)
    }

    pub fn bore(&self, bodies: &[Body]) -> f32 {
        match bodies[self.barrel].shape { BodyShape::Rod { half_width, .. } => half_width, _ => 0.0 }
    }

    fn half_len(&self, bodies: &[Body]) -> f64 {
        match bodies[self.barrel].shape { BodyShape::Rod { half_len, .. } => half_len as f64, _ => 0.0 }
    }

    fn atm(&self, bodies: &[Body]) -> f64 {
        (ATM_PER_BORE * 2.0 * self.bore(bodies)) as f64
    }

    /// Gas column height (head to crown) and its rate.
    fn column(&self, bodies: &[Body]) -> (f64, f64) {
        let f = OnRail::new(bodies, self.piston, self.local, self.barrel);
        let h = self.half_len(bodies) - f.along() - self.crown as f64;
        (h, -f.along_rate(bodies, self.piston, self.barrel))
    }

    /// Enter `stroke` at the current position: seal or ignite as it implies.
    pub fn begin(&mut self, stroke: Stroke, bodies: &[Body]) {
        let (h, _) = self.column(bodies);
        let atm = self.atm(bodies);
        self.stroke = stroke;
        self.h_turn = h;
        self.h_ext = h;
        self.h_ref = h.max(MIN_CLEARANCE);
        self.p_ref = match stroke {
            Stroke::Power => atm * (1.0 + FIRE_OVER_ATM * self.throttle.clamp(0.0, 1.0) as f64),
            _ => atm,
        };
    }

    /// Absolute pressure for a gas column `h`.
    fn pressure_abs(&self, h: f64, atm: f64) -> f64 {
        if self.mode == GasMode::Sealed || self.stroke.sealed() {
            self.p_ref * (self.h_ref / h.max(0.2 * MIN_CLEARANCE)).powf(GAMMA)
        } else {
            atm
        }
    }

    pub fn state(&self, bodies: &[Body]) -> CylinderState {
        let (h, _) = self.column(bodies);
        let atm = self.atm(bodies);
        let gauge = self.pressure_abs(h, atm) - atm;
        CylinderState {
            stroke: self.stroke,
            progress: ((h - self.h_turn).abs() / self.last_len.max(1e-6)).min(1.0) as f32,
            pressure: (gauge / (atm * FIRE_OVER_ATM)) as f32,
            crown: (self.half_len(bodies) - h) as f32,
        }
    }

    /// The stroke after a dead centre in the current mode.
    fn next(&self) -> Stroke {
        match (self.mode, self.stroke) {
            (GasMode::FourStroke, Stroke::Intake) => Stroke::Compression,
            (GasMode::FourStroke, Stroke::Compression) => Stroke::Power,
            (GasMode::FourStroke, Stroke::Power) => Stroke::Exhaust,
            (GasMode::FourStroke, Stroke::Exhaust) => Stroke::Intake,
            (_, Stroke::Compression) => Stroke::Power,
            _ => Stroke::Compression,
        }
    }
}

impl Constraint for Cylinder {
    fn as_any(&self) -> &dyn Any { self }
    fn as_any_mut(&mut self) -> &mut dyn Any { self }
    fn dim(&self) -> usize { 2 }

    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval) {
        eval_on_rail(bodies, self.piston, self.local, self.barrel, Some(self.angle0), vel, out);
    }

    fn apply_forces(&self, bodies: &mut [Body], dt: f64) {
        let hl = self.half_len(bodies);
        let crown = self.crown as f64;
        // Keep the piston in the barrel: crown below the head, wrist above the open end.
        rail_stops(bodies, self.piston, self.local, self.barrel, -hl, hl - crown - MIN_CLEARANCE, dt);

        let (h, _) = self.column(bodies);
        let atm = self.atm(bodies);
        let p = self.pressure_abs(h, atm) - atm;
        if p.abs() < 1e-12 { return; }
        // Pushes the piston away from the head and the head away from the piston.
        let f = OnRail::new(bodies, self.piston, self.local, self.barrel);
        let head = bodies[self.barrel].pos + f.u * hl;
        bodies[self.piston].apply_force_at_world_point(-p * f.u, f.pt.p);
        bodies[self.barrel].apply_force_at_world_point(p * f.u, head);
    }

    /// Sealed gas is conservative: the work to bring the column to h
    /// against (p − atm), V = p_ref·h_ref^γ·h^(1−γ)/(γ−1) + atm·h, taken
    /// relative to the sealing point. Combustion modes add and remove
    /// energy each cycle, so they report none.
    fn potential_energy(&self, bodies: &[Body]) -> f64 {
        if self.mode != GasMode::Sealed { return 0.0; }
        let (h, _) = self.column(bodies);
        let atm = self.atm(bodies);
        let v = |h: f64| {
            let h = h.max(0.2 * MIN_CLEARANCE);
            self.p_ref * self.h_ref.powf(GAMMA) * h.powf(1.0 - GAMMA) / (GAMMA - 1.0) + atm * h
        };
        v(h) - v(self.h_ref)
    }

    fn post_step(&mut self, bodies: &[Body]) {
        if self.mode == GasMode::Sealed { return; }
        let (h, h_dot) = self.column(bodies);
        if self.stroke.compressing() {
            self.h_ext = self.h_ext.min(h);
            if h_dot > 0.0 && h - self.h_ext > TURN_EPS {
                // Top dead centre
                self.last_len = (self.h_turn - self.h_ext).abs().max(MIN_CLEARANCE);
                let next = self.next();
                let (gauge_h, ext) = (h, self.h_ext);
                let atm = self.atm(bodies);
                let p_tdc = self.pressure_abs(ext, atm);
                self.stroke = next;
                self.h_turn = ext;
                self.h_ext = gauge_h;
                if next == Stroke::Power {
                    // Combustion at (nearly) constant volume adds pressure.
                    self.h_ref = ext.max(MIN_CLEARANCE);
                    self.p_ref = p_tdc + atm * FIRE_OVER_ATM * self.throttle.clamp(0.0, 1.0) as f64;
                }
            }
        } else {
            self.h_ext = self.h_ext.max(h);
            if h_dot < 0.0 && self.h_ext - h > TURN_EPS {
                // Bottom dead centre
                self.last_len = (self.h_ext - self.h_turn).abs().max(MIN_CLEARANCE);
                let ext = self.h_ext;
                self.stroke = self.next();
                self.h_turn = ext;
                self.h_ext = h;
                if self.stroke == Stroke::Compression {
                    // Fresh charge sealed at atmospheric.
                    self.h_ref = ext;
                    self.p_ref = self.atm(bodies);
                }
            }
        }
    }

    fn rebase(&mut self, bodies: &[Body]) {
        self.angle0 = bodies[self.piston].angle - bodies[self.barrel].angle;
        // The editor may have moved the piston: restart the current stroke
        // here (a sealed cylinder now rests at this position).
        self.begin(self.stroke, bodies);
    }

    fn body_indices(&self) -> Vec<usize> { vec![self.piston, self.barrel] }
    fn remap_body(&mut self, old: usize, new: usize) {
        if self.piston == old { self.piston = new; }
        if self.barrel == old { self.barrel = new; }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::body::rod_inertia;
    use crate::sim::world::{Integrator, World};

    fn rod(pos: Vec2, angle: f32, hl: f32, hw: f32) -> Body {
        Body::new(pos, angle, 1.0, rod_inertia(1.0, hl), BodyShape::Rod { half_len: hl, half_width: hw })
    }

    /// A sealed cylinder is an air spring: shoved and released, the piston
    /// oscillates about where it was sealed (compression pushes it back,
    /// the partial vacuum pulls it back), stays in the barrel, and with RK4
    /// passes its rest point at the speed it started with.
    #[test]
    fn sealed_is_an_air_spring() {
        for integ in [Integrator::Rk4, Integrator::Xpbd] {
            let mut w = World::new();
            w.integrator = integ;
            let barrel = w.add_body(rod(Vec2::ZERO, 0.0, 1.0, 0.2).fixed());
            let piston = w.add_body(rod(Vec2::ZERO, 1.57, 0.1, 0.1));
            w.add_constraint(Cylinder::new(piston, Vec2::ZERO, barrel, &w.bodies).unwrap());
            w.bodies[piston].vel.x = 3.0; // shove it towards the head
            let (mut lo, mut hi, mut pass_v) = (0.0f64, 0.0f64, 0.0f64);
            for _ in 0..20000 {
                let x0 = w.bodies[piston].pos.x;
                w.step(0.0001);
                let x1 = w.bodies[piston].pos.x;
                lo = lo.min(x1);
                hi = hi.max(x1);
                if x0 < 0.0 && x1 >= 0.0 { pass_v = w.bodies[piston].vel.x; }
            }
            eprintln!("{integ:?} x in [{lo:.3}, {hi:.3}], speed through rest {pass_v:.3}");
            assert!(hi > 0.2 && hi < 0.8, "compresses but stays off the head");
            assert!(lo < -0.1 && lo > -1.0, "vacuum pulls it back before the open end");
            if integ == Integrator::Rk4 {
                assert!((pass_v - 3.0).abs() < 0.01, "air spring should be lossless");
            }
        }
    }
}
