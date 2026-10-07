//! Tools that watch the simulation rather than take part in it: the rewind
//! timeline, butterfly-effect mode (a perturbed shadow world and its
//! divergence), the phase-space / Poincaré plot, and the editor's undo
//! history.

use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};
use std::sync::{Arc, Mutex};

use crate::scene_file::{clone_world, SceneFile};
use crate::sim::{
    body::BodyShape,
    forces::{MouseSpring, MouseSpringData},
    world::World,
};

/// Wrap an angle to (-π, π].
pub fn wrap_pi(a: f64) -> f64 {
    a - TAU * ((a + PI) / TAU).floor()
}

/// Bodies worth plotting or perturbing: free, and not anchors.
pub fn free_bodies(world: &World) -> Vec<usize> {
    (0..world.bodies.len())
        .filter(|&i| !world.bodies[i].fixed && !matches!(world.bodies[i].shape, BodyShape::Point))
        .collect()
}

// ── Rewind ────────────────────────────────────────────────────────────────────

/// How much history the timeline keeps.
pub const TIMELINE_SECONDS: f64 = 30.0;

/// Ring buffer of world states, one per rendered frame while simulating.
#[derive(Default)]
pub struct Timeline {
    /// (sim time, state) oldest first.
    frames: VecDeque<(f64, Vec<f64>)>,
    /// Bodies and constraints the states were taken from.
    shape: (usize, usize),
    /// Sim time of the newest state.
    pub time: f64,
    /// While scrubbing: the frame on show.
    pub cursor: Option<usize>,
    spare: Vec<Vec<f64>>,
}

impl Timeline {
    pub fn clear(&mut self) {
        while let Some((_, v)) = self.frames.pop_front() { self.spare.push(v); }
        self.cursor = None;
        self.time = 0.0;
    }

    pub fn len(&self) -> usize { self.frames.len() }

    /// Time of frame `i` relative to the newest.
    pub fn age(&self, i: usize) -> f64 {
        self.frames.get(i).map_or(0.0, |(t, _)| self.time - t)
    }

    pub fn span(&self) -> f64 {
        self.frames.front().map_or(0.0, |(t, _)| self.time - t)
    }

    /// Record the current state, `dt` sim seconds after the last one.
    /// Resuming after a scrub first drops the frames after the cursor.
    pub fn record(&mut self, world: &World, dt: f64) {
        let shape = (world.bodies.len(), world.constraints.len());
        if shape != self.shape {
            self.clear();
            self.shape = shape;
        }
        if let Some(c) = self.cursor.take() {
            while self.frames.len() > c + 1 {
                if let Some((_, v)) = self.frames.pop_back() { self.spare.push(v); }
            }
            self.time = self.frames.back().map_or(0.0, |(t, _)| *t);
        }
        self.time += dt;
        while self.span() > TIMELINE_SECONDS {
            if let Some((_, v)) = self.frames.pop_front() { self.spare.push(v); }
        }
        let mut v = self.spare.pop().unwrap_or_default();
        world.save_state(&mut v);
        self.frames.push_back((self.time, v));
    }

    /// Show frame `i`: puts that state into the world.
    pub fn seek(&mut self, world: &mut World, i: usize) -> bool {
        if (world.bodies.len(), world.constraints.len()) != self.shape { return false; }
        let Some((_, v)) = self.frames.get(i) else { return false };
        world.load_state(v);
        self.cursor = Some(i);
        true
    }
}

// ── Butterfly effect ──────────────────────────────────────────────────────────

/// Velocity nudge given to the shadow world.
pub const PERTURBATION: f64 = 1e-9;
/// Divergence (phase-space distance) where the exponential phase is over.
const SATURATED: f64 = 1e-2;
const DIVERGENCE_LEN: usize = 600;

/// A copy of the world, nudged by `PERTURBATION`, stepped in lockstep with
/// the real one. Their distance grows like e^(λt) in a chaotic system; the
/// slope of its log over time estimates the largest Lyapunov exponent λ.
pub struct Butterfly {
    pub ghost: World,
    /// (time since seeding, log10 distance), oldest first.
    pub history: VecDeque<(f64, f64)>,
    pub time: f64,
    pub lyapunov: Option<f64>,
    fit: (f64, f64, f64, f64, usize),
}

impl Butterfly {
    pub fn seed(world: &World, mouse: &Arc<Mutex<MouseSpringData>>) -> Self {
        let mut ghost = clone_world(world);
        ghost.add_force(MouseSpring(mouse.clone()));
        for i in free_bodies(&ghost) {
            let b = &mut ghost.bodies[i];
            b.vel.x += PERTURBATION;
            b.ang_vel += PERTURBATION;
        }
        Self { ghost, history: VecDeque::new(), time: 0.0, lyapunov: None, fit: (0.0, 0.0, 0.0, 0.0, 0) }
    }

    /// Phase-space distance to the real world: positions, angles and
    /// velocities of the free bodies.
    pub fn distance(&self, world: &World) -> f64 {
        let mut d2 = 0.0;
        for (a, b) in world.bodies.iter().zip(&self.ghost.bodies).filter(|(a, _)| !a.fixed) {
            d2 += (a.pos - b.pos).length_squared() + (a.angle - b.angle).powi(2)
                + (a.vel - b.vel).length_squared() + (a.ang_vel - b.ang_vel).powi(2);
        }
        d2.sqrt()
    }

    /// Log a sample after `dt` more sim seconds.
    pub fn sample(&mut self, world: &World, dt: f64) {
        self.time += dt;
        let d = self.distance(world).max(1e-300);
        if self.history.len() == DIVERGENCE_LEN { self.history.pop_front(); }
        self.history.push_back((self.time, d.log10()));
        // Least-squares slope of ln d over the exponential phase: past the
        // first instants (where the nudge sorts itself out) and before the
        // copies are macroscopically apart.
        if d < SATURATED && self.time > 0.25 {
            let (t, y) = (self.time, d.ln());
            let f = &mut self.fit;
            f.0 += t; f.1 += y; f.2 += t * t; f.3 += t * y; f.4 += 1;
            let n = f.4 as f64;
            let den = n * f.2 - f.0 * f.0;
            if f.4 > 30 && den > 1e-12 {
                self.lyapunov = Some((n * f.3 - f.0 * f.1) / den);
            }
        }
    }

    pub fn saturated(&self) -> bool {
        self.history.back().is_some_and(|&(_, l)| l > SATURATED.log10())
    }
}

// ── Phase space ───────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlotMode {
    /// Recent trajectory of (θ, ω).
    Phase,
    /// (θ, ω) whenever the section body's angle crosses 0 going forward.
    Poincare,
}

const TRAIL: usize = 2400;
const MAX_DOTS: usize = 40_000;

pub struct PhasePlot {
    pub body: Option<usize>,
    /// Body whose upward zero crossing triggers a Poincaré dot.
    pub section: Option<usize>,
    pub mode: PlotMode,
    pub trail: VecDeque<(f32, f32)>,
    pub dots: Vec<(f32, f32)>,
    /// Largest |ω| seen, for the vertical scale.
    pub w_max: f32,
    last_section: Option<f64>,
}

impl Default for PhasePlot {
    fn default() -> Self {
        Self {
            body: None, section: None, mode: PlotMode::Phase,
            trail: VecDeque::new(), dots: Vec::new(), w_max: 1.0, last_section: None,
        }
    }
}

impl PhasePlot {
    pub fn clear(&mut self) {
        self.trail.clear();
        self.dots.clear();
        self.w_max = 1.0;
        self.last_section = None;
    }

    /// Pick sensible bodies if the current ones aren't valid.
    pub fn ensure_bodies(&mut self, world: &World) {
        let free = free_bodies(world);
        let valid = |i: Option<usize>| i.is_some_and(|i| free.contains(&i));
        if !valid(self.body) {
            self.body = free.last().copied();
            self.clear();
        }
        if !valid(self.section) {
            self.section = free.first().copied();
            self.last_section = None;
        }
    }

    /// After every physics step: look for a section crossing.
    pub fn after_step(&mut self, world: &World) {
        if self.mode != PlotMode::Poincare { return; }
        let (Some(b), Some(s)) = (self.body, self.section) else { return };
        let (Some(bb), Some(sb)) = (world.bodies.get(b), world.bodies.get(s)) else { return };
        let a = wrap_pi(sb.angle);
        if let Some(prev) = self.last_section {
            // Forward through 0 (not the ±π wrap).
            if prev < 0.0 && a >= 0.0 && a - prev < 1.0 && sb.ang_vel > 0.0 && self.dots.len() < MAX_DOTS {
                let (t, w) = (wrap_pi(bb.angle) as f32, bb.ang_vel as f32);
                self.w_max = self.w_max.max(w.abs());
                self.dots.push((t, w));
            }
        }
        self.last_section = Some(a);
    }

    /// Once per frame: extend the phase trail.
    pub fn after_frame(&mut self, world: &World) {
        let Some(b) = self.body.and_then(|i| world.bodies.get(i)) else { return };
        let (t, w) = (wrap_pi(b.angle) as f32, b.ang_vel as f32);
        if self.mode == PlotMode::Phase {
            self.w_max = self.w_max.max(w.abs());
        }
        if self.trail.len() == TRAIL { self.trail.pop_front(); }
        self.trail.push_back((t, w));
    }
}

// ── Undo ──────────────────────────────────────────────────────────────────────

const UNDO_DEPTH: usize = 200;

/// Snapshots of the edited world (JSON), taken whenever it has settled
/// into a new state between edits.
#[derive(Default)]
pub struct UndoStack {
    past: Vec<String>,
    future: Vec<String>,
    current: Option<String>,
}

impl UndoStack {
    pub fn reset(&mut self, world: &World) {
        self.past.clear();
        self.future.clear();
        self.current = Some(SceneFile::from_world(world).to_json());
    }

    /// Record the world if it differs from the last snapshot.
    pub fn commit(&mut self, world: &World) {
        let now = SceneFile::from_world(world).to_json();
        if self.current.as_deref() == Some(now.as_str()) { return; }
        if let Some(prev) = self.current.replace(now) {
            self.past.push(prev);
            if self.past.len() > UNDO_DEPTH { self.past.remove(0); }
        }
        self.future.clear();
    }

    pub fn can_undo(&self) -> bool { !self.past.is_empty() }
    pub fn can_redo(&self) -> bool { !self.future.is_empty() }

    /// The world to go back to, if any.
    pub fn undo(&mut self) -> Option<World> {
        let prev = self.past.pop()?;
        if let Some(cur) = self.current.replace(prev.clone()) { self.future.push(cur); }
        SceneFile::from_json(&prev).map(|f| f.to_world())
    }

    pub fn redo(&mut self) -> Option<World> {
        let next = self.future.pop()?;
        if let Some(cur) = self.current.replace(next.clone()) { self.past.push(cur); }
        SceneFile::from_json(&next).map(|f| f.to_world())
    }
}

/// Every analysis tool's state plus the HUD toggles for them.
#[derive(Default)]
pub struct Analysis {
    pub timeline: Timeline,
    pub butterfly: Option<Butterfly>,
    pub phase: PhasePlot,
    pub show_forces: bool,
    pub show_phase: bool,
    pub show_chaos: bool,
    /// Rewind bar open (history is recorded either way).
    pub show_timeline: bool,
    /// Share panel open, the code shown in it, and what the user pasted.
    pub show_share: bool,
    pub share_code: String,
    pub paste: String,
    pub share_note: String,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenes::SCENES;

    /// Rewinding to a frame and stepping on reproduces the original run.
    #[test]
    fn rewind_replays_exactly() {
        let def = SCENES.iter().find(|d| d.name == "Radial Engine").unwrap();
        let mut w = (def.build)();
        let mut tl = Timeline::default();
        let mut states = Vec::new();
        for _ in 0..120 {
            for _ in 0..8 { w.step(1.0 / 480.0); }
            tl.record(&w, 1.0 / 60.0);
            let mut s = Vec::new();
            w.save_state(&mut s);
            states.push(s);
        }
        assert!(tl.seek(&mut w, 40));
        for _ in 0..8 { w.step(1.0 / 480.0); }
        let mut s = Vec::new();
        w.save_state(&mut s);
        let diff = s.iter().zip(&states[41]).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        assert!(diff < 1e-9, "replay differs by {diff}");
    }

    /// The double pendulum is chaotic: λ comes out clearly positive.
    #[test]
    fn double_pendulum_has_positive_lyapunov() {
        let def = SCENES.iter().find(|d| d.name == "Double Pendulum").unwrap();
        let mut w = (def.build)();
        let mouse = Arc::new(Mutex::new(MouseSpringData::default()));
        let mut bf = Butterfly::seed(&w, &mouse);
        let dt = 1.0 / 240.0;
        for _ in 0..(240 * 8) {
            w.step(dt);
            bf.ghost.step(dt);
            bf.sample(&w, dt as f64);
        }
        eprintln!("λ ≈ {:?}, last log10 d {:?}", bf.lyapunov, bf.history.back());
        assert!(bf.lyapunov.is_some_and(|l| l > 0.5));
    }
}
