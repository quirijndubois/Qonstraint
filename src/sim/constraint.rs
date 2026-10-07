use std::any::Any;
use glam::{DVec2, Vec2};
use crate::sim::body::Body;

/// Most scalar equations a single constraint may have.
pub const MAX_DIM: usize = 3;
/// Most bodies a single constraint may touch (a rope: two ends and a pulley).
pub const MAX_BLOCKS: usize = 3;

/// The Jacobian block of one constraint w.r.t. one body:
/// `j[row][col]` = ∂C_row/∂q_col with col ∈ {0 = x, 1 = y, 2 = θ}.
#[derive(Clone, Copy, Debug, Default)]
pub struct JBlock {
    pub body: usize,
    pub j: [[f64; 3]; MAX_DIM],
}

/// Everything the integrators need from a constraint at the current state,
/// produced by one call so shared geometry (rotations, lengths, frames) is
/// computed once. Rows `dim..MAX_DIM` are unused.
#[derive(Clone, Copy, Debug, Default)]
pub struct ConstraintEval {
    /// C(q)
    pub c: [f64; MAX_DIM],
    /// Ċ = J q̇ (only filled when velocities are requested)
    pub c_dot: [f64; MAX_DIM],
    /// J̇ q̇ (only filled when velocities are requested)
    pub bias: [f64; MAX_DIM],
    pub blocks: [JBlock; MAX_BLOCKS],
    /// Blocks in use. 0 means the constraint is degenerate right now
    /// (e.g. coincident points) and contributes nothing.
    pub n_blocks: usize,
}

/// The force (x, y) and torque a constraint applied to each body it
/// touches, i.e. Jᵀλ split by block. Recorded for the force overlay.
#[derive(Clone, Copy, Debug, Default)]
pub struct Reaction {
    pub n: usize,
    pub body: [usize; MAX_BLOCKS],
    pub f: [glam::DVec3; MAX_BLOCKS],
}

impl Reaction {
    /// Force on `body` (summed if it appears in several blocks).
    pub fn on(&self, body: usize) -> glam::DVec2 {
        (0..self.n).filter(|&k| self.body[k] == body)
            .map(|k| glam::DVec2::new(self.f[k].x, self.f[k].y)).sum()
    }
}

impl ConstraintEval {
    #[inline]
    pub fn blocks(&self) -> &[JBlock] { &self.blocks[..self.n_blocks] }
}

pub trait Constraint: Send + Sync + Any {
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    /// Number of scalar constraint equations (≤ `MAX_DIM`).
    fn dim(&self) -> usize;

    /// Fill `out` with C, the Jacobian blocks and, if `vel`, Ċ and J̇q̇.
    /// Must set `n_blocks` and every used row; may leave stale data elsewhere.
    fn evaluate(&self, bodies: &[Body], vel: bool, out: &mut ConstraintEval);

    /// C(q): constraint value vector (should be ~0).
    #[cfg(test)]
    fn eval_c(&self, bodies: &[Body]) -> Vec<f64> {
        let mut e = ConstraintEval::default();
        self.evaluate(bodies, false, &mut e);
        e.c[..self.dim()].to_vec()
    }

    /// Ċ = J q̇
    #[cfg(test)]
    fn eval_c_dot(&self, bodies: &[Body]) -> Vec<f64> {
        let mut e = ConstraintEval::default();
        self.evaluate(bodies, true, &mut e);
        e.c_dot[..self.dim()].to_vec()
    }

    /// Add any forces the constraint applies besides its multipliers, e.g.
    /// one-sided end stops (which an equality solver can't express).
    /// `dt` is the step size, so stiff forces can stay stable for it.
    fn apply_forces(&self, _bodies: &mut [Body], _dt: f64) {}

    /// Energy the constraint stores (e.g. compressed gas), for the energy
    /// readout. 0 for ideal joints.
    fn potential_energy(&self, _bodies: &[Body]) -> f64 { 0.0 }

    /// Update discrete internal state (e.g. an engine cycle) once the step
    /// is complete. Not called between RK4 stages.
    fn post_step(&mut self, _bodies: &[Body]) {}

    /// Append any discrete state that evolves while simulating (an engine
    /// cycle), so a rewind can put it back with `load_state`.
    fn save_state(&self, _out: &mut Vec<f64>) {}

    /// Read back what `save_state` wrote, advancing `input`.
    fn load_state(&mut self, _input: &mut &[f64]) {}

    /// Re-capture any reference state (e.g. rolled-distance offsets) from the
    /// current configuration. Called when leaving the editor, since editor
    /// moves teleport bodies without the physics that keeps such state valid.
    fn rebase(&mut self, _bodies: &[Body]) {}

    /// Body indices this constraint touches (for deletion filtering).
    fn body_indices(&self) -> Vec<usize>;

    /// Remap a body index: called after a body is removed so survivors shift down.
    fn remap_body(&mut self, old: usize, new: usize);
}

/// A body-attached point, with everything position constraints need from it.
#[derive(Clone, Copy, Debug)]
pub struct Attach {
    /// World position.
    pub p: DVec2,
    /// World-space offset from the body centre (R·local).
    pub r: DVec2,
}

impl Attach {
    #[inline]
    pub fn new(body: &Body, local: Vec2) -> Self {
        let r = body.rot().apply(local.as_dvec2());
        Self { p: body.pos + r, r }
    }

    /// ∂p/∂θ
    #[inline]
    pub fn dp_dtheta(&self) -> DVec2 { self.r.perp() }

    /// Point velocity v + ω × r.
    #[inline]
    pub fn vel(&self, body: &Body) -> DVec2 { body.point_vel(self.r) }

    /// J̇q̇ for this point: centripetal acceleration −ω²·r.
    #[inline]
    pub fn centripetal(&self, body: &Body) -> DVec2 { -body.ang_vel * body.ang_vel * self.r }

    /// Jacobian block of `sign·p` (2 rows: x, y).
    #[inline]
    pub fn block(&self, body: usize, sign: f64) -> JBlock {
        let d = self.dp_dtheta();
        let mut b = JBlock { body, ..Default::default() };
        b.j[0] = [sign, 0.0, sign * d.x];
        b.j[1] = [0.0, sign, sign * d.y];
        b
    }
}
