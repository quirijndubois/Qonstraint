//! XPBD-style position-based integrator with "small steps" (Macklin et al.
//! 2019): each step predicts positions from velocities and applied forces
//! (symplectic Euler), projects every constraint once (Gauss–Seidel over
//! constraints, each one's rows solved together), then derives velocities
//! from the position change.
//!
//! Configuration space is flat (x, y, θ) with a constant diagonal mass
//! matrix, so a constraint's projection is just Δq = W Jᵀ Δλ with
//! (J W Jᵀ) Δλ = −C, using the same `evaluate` as the Witkin solver. No
//! global solve, no Baumgarte constants, and constraints hold at position
//! level every step. Forces stay explicit; with many small steps that is
//! stable for any spring the editor can make.

// Small dense index math reads clearer with explicit indices.
#![allow(clippy::needless_range_loop)]

use glam::{DVec2, DVec3};
use crate::sim::body::Body;
use crate::sim::constraint::{Constraint, ConstraintEval, MAX_BLOCKS, MAX_DIM};
use crate::sim::force::Force;

/// Projection sweeps when settling a fresh configuration.
const SETTLE_SWEEPS: usize = 50;

#[derive(Default)]
pub struct XpbdScratch {
    prev: Vec<(DVec2, f64)>,
    eval: ConstraintEval,
}

pub fn step(
    bodies: &mut [Body],
    constraints: &[Box<dyn Constraint>],
    forces: &[Box<dyn Force>],
    dt: f64,
    s: &mut XpbdScratch,
) {
    // Applied forces at the start-of-step state.
    for b in bodies.iter_mut() { b.clear_accumulators(); }
    for f in forces { f.apply(bodies); }
    for c in constraints { c.apply_forces(bodies, dt); }

    s.prev.clear();
    for b in bodies.iter_mut() {
        s.prev.push((b.pos, b.angle));
        if b.fixed { continue; }
        b.vel     += dt * b.inv_mass() * b.force_accum;
        b.ang_vel += dt * b.inv_inertia() * b.torque_accum;
        b.pos     += dt * b.vel;
        b.angle   += dt * b.ang_vel;
    }

    project(bodies, constraints, &mut s.eval);

    let inv_dt = 1.0 / dt;
    for (b, &(p, a)) in bodies.iter_mut().zip(&s.prev) {
        if b.fixed { continue; }
        b.vel     = (b.pos - p) * inv_dt;
        b.ang_vel = (b.angle - a) * inv_dt;
    }
}

/// Pulls positions onto the constraints without touching velocities. Run
/// once before the first step of a new or edited configuration: a step
/// would otherwise turn any existing violation C into a velocity kick C/dt.
pub fn settle(bodies: &mut [Body], constraints: &[Box<dyn Constraint>], s: &mut XpbdScratch) {
    for _ in 0..SETTLE_SWEEPS { project(bodies, constraints, &mut s.eval); }
}

/// One Gauss–Seidel sweep: each constraint solved exactly in its linearisation.
fn project(bodies: &mut [Body], constraints: &[Box<dyn Constraint>], e: &mut ConstraintEval) {
    for c in constraints {
        c.evaluate(bodies, false, e);
        if e.n_blocks == 0 { continue; }
        let n = c.dim();

        // K = J W Jᵀ (n×n), WJᵀ columns kept for the update.
        let mut wj = [[DVec3::ZERO; MAX_DIM]; MAX_BLOCKS];
        let mut k = [[0.0f64; MAX_DIM]; MAX_DIM];
        for (bi, blk) in e.blocks().iter().enumerate() {
            let b = &bodies[blk.body];
            let w = DVec3::new(b.inv_mass(), b.inv_mass(), b.inv_inertia());
            for r in 0..n {
                wj[bi][r] = w * DVec3::from(blk.j[r]);
            }
            for r in 0..n {
                for col in 0..=r {
                    k[r][col] += wj[bi][r].dot(DVec3::from(blk.j[col]));
                }
            }
        }

        let mut dl = [0.0f64; MAX_DIM];
        for r in 0..n { dl[r] = -e.c[r]; }
        if !solve_spd(&mut k, &mut dl, n) { continue; }

        for (bi, blk) in e.blocks().iter().enumerate() {
            let mut dq = DVec3::ZERO;
            for r in 0..n { dq += dl[r] * wj[bi][r]; }
            let b = &mut bodies[blk.body];
            b.pos.x += dq.x;
            b.pos.y += dq.y;
            b.angle += dq.z;
        }
    }
}

/// Solves K x = b in place for a small SPD K (lower triangle filled) by
/// LDLᵀ. Rows whose pivot vanishes (redundant or massless) get x = 0.
/// Returns false if nothing can move.
#[inline]
fn solve_spd(k: &mut [[f64; MAX_DIM]; MAX_DIM], b: &mut [f64; MAX_DIM], n: usize) -> bool {
    let mut d = [0.0f64; MAX_DIM];
    let mut any = false;
    for i in 0..n {
        for j in 0..i {
            let mut s = k[i][j];
            for p in 0..j { s -= k[i][p] * k[j][p] * d[p]; }
            k[i][j] = if d[j] != 0.0 { s / d[j] } else { 0.0 };
        }
        let diag = k[i][i];
        let mut s = diag;
        for p in 0..i { s -= k[i][p] * k[i][p] * d[p]; }
        if diag > 0.0 && s > 1e-12 * diag { d[i] = s; any = true; } else { d[i] = 0.0; }
    }
    if !any { return false; }
    for i in 0..n {
        for p in 0..i { b[i] -= k[i][p] * b[p]; }
    }
    for i in 0..n { b[i] = if d[i] != 0.0 { b[i] / d[i] } else { 0.0 }; }
    for i in (0..n).rev() {
        for p in i + 1..n { b[i] -= k[p][i] * b[p]; }
    }
    true
}
