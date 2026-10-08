use glam::{DVec2, Vec2};
use crate::sim::body::Body;
use crate::sim::constraint::{Constraint, ConstraintEval, Reaction};
use crate::sim::contact::Contacts;
use crate::sim::force::Force;
use crate::sim::slot::Slot;
use crate::sim::solver::WitkinSolver;
use crate::sim::xpbd::{self, XpbdScratch};

/// A body-local point whose path the app draws as a fading trail.
/// Pure presentation metadata: it has no effect on the physics.
#[derive(Clone, Copy, Debug)]
pub struct Tracer {
    pub body:    usize,
    pub local:   Vec2,
    /// How much history the trail keeps, in seconds.
    pub seconds: f32,
}

/// How `World::step` advances time.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Integrator {
    /// RK4 with Witkin constraint forces and Baumgarte feedback (`solver.rs`).
    /// Conserves energy far better at any step count; the default.
    #[default]
    Rk4,
    /// Position-based projection, one cheap sweep per step (`xpbd.rs`).
    /// Several times cheaper per step and holds constraints tighter, but
    /// slowly bleeds energy.
    Xpbd,
}

pub struct World {
    pub bodies: Vec<Body>,
    pub constraints: Vec<Slot<dyn Constraint>>,
    pub forces: Vec<Slot<dyn Force>>,
    pub tracers: Vec<Tracer>,
    pub integrator: Integrator,
    /// Collisions between bodies with `collide` set, and their material.
    pub contacts: Contacts,
    /// When set, each step records the force every constraint applies
    /// (`reactions`, indexed like `constraints`), for the force overlay.
    pub record_reactions: bool,
    pub reactions: Vec<Reaction>,
    /// Set when the configuration may violate the constraints (new world,
    /// editor changes); the next XPBD step settles positions first.
    needs_settle: bool,
    solver: WitkinSolver,
    rk4: Rk4Scratch,
    xpbd: XpbdScratch,
}

impl World {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            constraints: Vec::new(),
            forces: Vec::new(),
            tracers: Vec::new(),
            integrator: Integrator::default(),
            contacts: Contacts::default(),
            record_reactions: false,
            reactions: Vec::new(),
            needs_settle: true,
            solver: WitkinSolver::default(),
            rk4: Rk4Scratch::default(),
            xpbd: XpbdScratch::default(),
        }
    }

    pub fn add_body(&mut self, body: Body) -> usize {
        let idx = self.bodies.len();
        self.bodies.push(body);
        idx
    }

    pub fn add_constraint(&mut self, c: impl Constraint + 'static) {
        self.constraints.push(Slot::new(Box::new(c)));
    }

    pub fn add_force(&mut self, f: impl Force + 'static) {
        self.forces.push(Slot::new(Box::new(f)));
    }

    pub fn add_tracer(&mut self, body: usize, local: Vec2) {
        self.tracers.push(Tracer { body, local, seconds: 3.0 });
    }

    /// Re-capture constraint reference state after bodies were moved by hand
    /// (the editor), and settle any violation before the next step.
    pub fn rebase(&mut self) {
        for c in self.constraints.iter_mut() { c.rebase(&self.bodies); }
        self.contacts.reset();
        self.needs_settle = true;
    }

    pub fn step(&mut self, dt: f32) {
        let dt = dt as f64;
        self.contacts.prepare(&self.bodies, &self.constraints);
        match self.integrator {
            Integrator::Xpbd => {
                if std::mem::take(&mut self.needs_settle) {
                    xpbd::settle(&mut self.bodies, &self.constraints, &mut self.xpbd);
                }
                let rec = self.record_reactions.then_some(&mut self.reactions);
                xpbd::step(&mut self.bodies, &self.constraints, &self.forces, &mut self.contacts, dt, &mut self.xpbd, rec);
            }
            Integrator::Rk4 => self.rk4_step(dt),
        }
        for c in self.constraints.iter_mut().filter(|c| c.on) { c.post_step(&self.bodies); }
        self.contacts.post_step(&self.bodies, dt);
    }

    /// Dynamic state of every body (and any evolving constraint state), for
    /// rewinding. Only valid for a world with the same structure.
    pub fn save_state(&self, out: &mut Vec<f64>) {
        out.clear();
        for b in &self.bodies {
            out.extend([b.pos.x, b.pos.y, b.angle, b.vel.x, b.vel.y, b.ang_vel]);
        }
        for c in &self.constraints { c.save_state(out); }
    }

    /// Put back a state from `save_state`. Contact stick state is dropped.
    pub fn load_state(&mut self, state: &[f64]) {
        let (body_part, mut rest) = state.split_at((self.bodies.len() * 6).min(state.len()));
        for (b, v) in self.bodies.iter_mut().zip(body_part.chunks_exact(6)) {
            b.pos = DVec2::new(v[0], v[1]);
            b.angle = v[2];
            b.vel = DVec2::new(v[3], v[4]);
            b.ang_vel = v[5];
        }
        for c in self.constraints.iter_mut() { c.load_state(&mut rest); }
        self.contacts.reset();
    }

    pub fn kinetic_energy(&self) -> f32 {
        self.bodies.iter().filter(|b| !b.fixed).map(|b| {
            0.5 * b.mass as f64 * b.vel.length_squared() + 0.5 * b.inertia as f64 * b.ang_vel * b.ang_vel
        }).sum::<f64>() as f32
    }

    pub fn potential_energy(&self, g: f32) -> f32 {
        let grav: f64 = self.bodies.iter()
            .filter(|b| !b.fixed)
            .map(|b| b.mass as f64 * g as f64 * b.pos.y)
            .sum();
        let elastic: f64 = self.forces.iter().filter(|f| f.on)
            .map(|f| f.potential_energy(&self.bodies))
            .chain(self.constraints.iter().filter(|c| c.on).map(|c| c.potential_energy(&self.bodies)))
            .sum();
        (grav + elastic) as f32
    }

    pub fn constraint_error(&self) -> f32 {
        let mut e = ConstraintEval::default();
        let mut err = 0.0f64;
        for c in self.constraints.iter().filter(|c| c.on) {
            c.evaluate(&self.bodies, false, &mut e);
            err += e.c[..c.dim()].iter().map(|v| v * v).sum::<f64>();
        }
        err.sqrt() as f32
    }

    /// Classic RK4. Each stage's state is written straight into the bodies
    /// and its derivative folded into a running weighted sum, so the only
    /// storage is the start state and that sum.
    fn rk4_step(&mut self, dt: f64) {
        const STAGE_DT: [f64; 3] = [0.5, 0.5, 1.0];
        const WEIGHT:   [f64; 4] = [1.0, 2.0, 2.0, 1.0];

        let s = &mut self.rk4;
        s.s0.clear();
        s.s0.extend(self.bodies.iter().map(|b| State { pos: b.pos, angle: b.angle, vel: b.vel, ang_vel: b.ang_vel }));
        s.sum.clear();
        s.sum.resize(self.bodies.len(), State::default());

        for stage in 0..4 {
            for b in self.bodies.iter_mut() { b.clear_accumulators(); }
            for f in self.forces.iter().filter(|f| f.on) { f.apply(&mut self.bodies); }
            for c in self.constraints.iter().filter(|c| c.on) { c.apply_forces(&mut self.bodies, dt); }
            self.contacts.apply(&mut self.bodies, dt);
            self.solver.apply(&mut self.bodies, &self.constraints);
            if stage == 0 && self.record_reactions {
                self.solver.reactions(&self.constraints, &mut self.reactions);
            }

            for ((b, s0), sum) in self.bodies.iter_mut().zip(&s.s0).zip(s.sum.iter_mut()) {
                if b.fixed { continue; }
                let acc = b.force_accum * b.inv_mass();
                let ang_acc = b.torque_accum * b.inv_inertia();
                let w = WEIGHT[stage];
                sum.pos += w * b.vel;
                sum.angle += w * b.ang_vel;
                sum.vel += w * acc;
                sum.ang_vel += w * ang_acc;
                if stage < 3 {
                    let h = STAGE_DT[stage] * dt;
                    b.pos = s0.pos + h * b.vel;
                    b.angle = s0.angle + h * b.ang_vel;
                    b.vel = s0.vel + h * acc;
                    b.ang_vel = s0.ang_vel + h * ang_acc;
                }
            }
        }

        let h = dt / 6.0;
        for ((b, s0), sum) in self.bodies.iter_mut().zip(&s.s0).zip(&s.sum) {
            if b.fixed { continue; }
            b.pos = s0.pos + h * sum.pos;
            b.angle = s0.angle + h * sum.angle;
            b.vel = s0.vel + h * sum.vel;
            b.ang_vel = s0.ang_vel + h * sum.ang_vel;
        }
    }
}

/// A body's dynamic state, or (in `Rk4Scratch::sum`) its weighted derivative.
#[derive(Clone, Copy, Default)]
struct State {
    pos: DVec2,
    angle: f64,
    vel: DVec2,
    ang_vel: f64,
}

#[derive(Default)]
struct Rk4Scratch {
    s0: Vec<State>,
    sum: Vec<State>,
}
