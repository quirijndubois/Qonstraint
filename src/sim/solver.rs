//! Witkin-style constraint forces (Witkin 1997, eq. 11):
//!
//!   JWJᵀ λ = −J̇q̇ − JWQ − ks·C − kd·Ċ,   Q̂ = Jᵀλ
//!
//! solved exactly with a sparse (envelope / skyline) LDLᵀ factorisation.
//! JWJᵀ couples two constraints only when they share a free body, so for
//! linkages it is very sparse. Constraints are ordered by reverse
//! Cuthill–McKee to keep its envelope narrow; the ordering and the storage
//! layout are rebuilt only when the constraint graph changes. All buffers
//! persist between calls, so a solve allocates nothing.

// Small dense index math reads clearer with explicit indices.
#![allow(clippy::needless_range_loop)]

use glam::DVec3;
use crate::sim::body::Body;
use crate::sim::constraint::{Constraint, ConstraintEval, Reaction};

const KS: f64 = 150.0; // position feedback (Baumgarte)
const KD: f64 = 15.0;  // velocity feedback (Baumgarte)
/// A pivot that has lost all but this fraction of its diagonal marks a row
/// that is redundant with earlier ones (e.g. an over-constrained loop). Its
/// multiplier is pinned to 0; the rows it duplicates carry the load.
const PIVOT_TOL: f64 = 1e-9;

#[derive(Default)]
pub struct WitkinSolver {
    evals: Vec<ConstraintEval>,
    /// Graph signature the current layout was built for, and a scratch copy.
    sig: Vec<usize>,
    sig_new: Vec<usize>,
    /// First row of each constraint in solve order.
    row_of: Vec<usize>,
    /// For each free body, the (constraint, block slot) pairs touching it.
    body_cons: Vec<Vec<(usize, usize)>>,
    /// Envelope layout: row i stores columns first[i]..=i at a[start[i]..].
    first: Vec<usize>,
    start: Vec<usize>,
    a: Vec<f64>,
    d: Vec<f64>,
    d_inv: Vec<f64>,
    /// Right-hand side, overwritten in place by λ (solve order).
    x: Vec<f64>,
    /// W·Q per body: (ax, ay, α) from applied forces alone.
    wq: Vec<DVec3>,
    /// Per body (1/m, 1/m, 1/I).
    w: Vec<DVec3>,
    /// RCM scratch.
    adj: Vec<Vec<usize>>,
    order: Vec<usize>,
    queue: Vec<usize>,
    seen: Vec<bool>,
}

impl WitkinSolver {
    /// Adds constraint forces to the bodies' force/torque accumulators, which
    /// must already hold the applied forces.
    pub fn apply(&mut self, bodies: &mut [Body], constraints: &[Box<dyn Constraint>]) {
        if constraints.is_empty() { return; }

        self.evals.resize(constraints.len(), ConstraintEval::default());
        for (c, e) in constraints.iter().zip(self.evals.iter_mut()) {
            c.evaluate(bodies, true, e);
        }

        self.w.clear();
        self.wq.clear();
        for b in bodies.iter() {
            let w = DVec3::new(b.inv_mass(), b.inv_mass(), b.inv_inertia());
            self.w.push(w);
            self.wq.push(w * DVec3::new(b.force_accum.x, b.force_accum.y, b.torque_accum));
        }

        self.sync_layout(bodies, constraints);
        self.assemble(constraints);
        self.factor();
        self.solve();

        // Q̂ = Jᵀλ
        for (ci, (c, e)) in constraints.iter().zip(&self.evals).enumerate() {
            let r0 = self.row_of[ci];
            for blk in e.blocks() {
                let body = &mut bodies[blk.body];
                if body.fixed { continue; }
                let mut f = DVec3::ZERO;
                for row in 0..c.dim() {
                    f += self.x[r0 + row] * DVec3::from(blk.j[row]);
                }
                body.force_accum.x += f.x;
                body.force_accum.y += f.y;
                body.torque_accum  += f.z;
            }
        }
    }

    /// Jᵀλ per constraint and block from the last `apply`.
    pub fn reactions(&self, constraints: &[Box<dyn Constraint>], out: &mut Vec<Reaction>) {
        out.clear();
        for (ci, (c, e)) in constraints.iter().zip(&self.evals).enumerate() {
            let r0 = self.row_of.get(ci).copied().unwrap_or(0);
            let mut r = Reaction { n: e.n_blocks, ..Default::default() };
            for (k, blk) in e.blocks().iter().enumerate() {
                r.body[k] = blk.body;
                for row in 0..c.dim() {
                    r.f[k] += self.x[r0 + row] * DVec3::from(blk.j[row]);
                }
            }
            out.push(r);
        }
    }

    /// Rebuilds ordering and envelope layout if the constraint graph changed.
    fn sync_layout(&mut self, bodies: &[Body], constraints: &[Box<dyn Constraint>]) {
        self.sig_new.clear();
        self.sig_new.push(bodies.len());
        for (c, e) in constraints.iter().zip(&self.evals) {
            self.sig_new.push(c.dim());
            for blk in e.blocks() {
                if !bodies[blk.body].fixed { self.sig_new.push(blk.body + 1); }
            }
            self.sig_new.push(0);
        }
        if self.sig_new == self.sig { return; }
        std::mem::swap(&mut self.sig, &mut self.sig_new);

        let nc = constraints.len();

        // Body → constraint incidence (free bodies only: fixed ones don't couple).
        self.body_cons.resize_with(bodies.len(), Vec::new);
        for l in self.body_cons.iter_mut() { l.clear(); }
        for (ci, e) in self.evals.iter().enumerate() {
            for (slot, blk) in e.blocks().iter().enumerate() {
                if !bodies[blk.body].fixed { self.body_cons[blk.body].push((ci, slot)); }
            }
        }

        // Constraint adjacency.
        self.adj.resize_with(nc, Vec::new);
        for l in self.adj.iter_mut() { l.clear(); }
        for list in &self.body_cons {
            for &(ci, _) in list {
                for &(cj, _) in list {
                    if ci != cj { self.adj[ci].push(cj); }
                }
            }
        }
        for l in self.adj.iter_mut() { l.sort_unstable(); l.dedup(); }

        // Reverse Cuthill–McKee, component by component, each started from
        // a minimum-degree node.
        self.order.clear();
        self.seen.clear();
        self.seen.resize(nc, false);
        while self.order.len() < nc {
            let start = (0..nc).filter(|&i| !self.seen[i])
                .min_by_key(|&i| self.adj[i].len()).unwrap();
            self.seen[start] = true;
            self.queue.clear();
            self.queue.push(start);
            let mut head = 0;
            while head < self.queue.len() {
                let ci = self.queue[head];
                head += 1;
                self.order.push(ci);
                let from = self.queue.len();
                for &cj in &self.adj[ci] {
                    if !self.seen[cj] { self.seen[cj] = true; self.queue.push(cj); }
                }
                let adj = &self.adj;
                self.queue[from..].sort_unstable_by_key(|&cj| adj[cj].len());
            }
        }
        self.order.reverse();

        self.row_of.resize(nc, 0);
        let mut m = 0;
        for &ci in &self.order {
            self.row_of[ci] = m;
            m += constraints[ci].dim();
        }

        // Envelope: a constraint's rows start at the earliest row of any
        // neighbour (or itself).
        self.first.resize(m, 0);
        self.start.resize(m + 1, 0);
        for ci in 0..nc {
            let r0 = self.row_of[ci];
            let lo = self.adj[ci].iter().map(|&cj| self.row_of[cj]).fold(r0, usize::min);
            for row in 0..constraints[ci].dim() { self.first[r0 + row] = lo; }
        }
        let mut s = 0;
        for i in 0..m {
            self.start[i] = s;
            s += i - self.first[i] + 1;
        }
        self.start[m] = s;
        self.a.resize(s, 0.0);
        self.d.resize(m, 0.0);
        self.d_inv.resize(m, 0.0);
        self.x.resize(m, 0.0);
    }

    #[inline]
    fn at(&self, i: usize, j: usize) -> usize { self.start[i] + j - self.first[i] }

    /// A = JWJᵀ into the envelope (lower triangle), b = RHS into `x`.
    fn assemble(&mut self, constraints: &[Box<dyn Constraint>]) {
        self.a.fill(0.0);
        for (body, list) in self.body_cons.iter().enumerate() {
            let w = self.w[body];
            for &(ci, si) in list {
                let bi = &self.evals[ci].blocks[si];
                let (ri, di) = (self.row_of[ci], constraints[ci].dim());
                for &(cj, sj) in list {
                    let (rj, dj) = (self.row_of[cj], constraints[cj].dim());
                    if rj > ri + di - 1 { continue; } // wholly upper triangle
                    let bj = &self.evals[cj].blocks[sj];
                    for x in 0..di {
                        let wx = w * DVec3::from(bi.j[x]);
                        for y in 0..dj {
                            if rj + y > ri + x { break; }
                            let idx = self.start[ri + x] + rj + y - self.first[ri + x];
                            self.a[idx] += wx.dot(DVec3::from(bj.j[y]));
                        }
                    }
                }
            }
        }

        for (ci, (c, e)) in constraints.iter().zip(&self.evals).enumerate() {
            let r0 = self.row_of[ci];
            for row in 0..c.dim() {
                let jwq: f64 = e.blocks().iter()
                    .map(|b| DVec3::from(b.j[row]).dot(self.wq[b.body]))
                    .sum();
                self.x[r0 + row] = -e.bias[row] - jwq - KS * e.c[row] - KD * e.c_dot[row];
            }
        }
    }

    /// In-place envelope LDLᵀ: the strict lower part of `a` becomes L.
    fn factor(&mut self) {
        let m = self.d.len();
        for i in 0..m {
            let (fi, si) = (self.first[i], self.start[i]);
            for j in fi..i {
                let (fj, sj) = (self.first[j], self.start[j]);
                let mut s = self.a[si + j - fi];
                for k in fi.max(fj)..j {
                    s -= self.a[si + k - fi] * self.a[sj + k - fj] * self.d[k];
                }
                self.a[si + j - fi] = s * self.d_inv[j];
            }
            let diag = self.a[si + i - fi];
            let mut s = diag;
            for k in fi..i {
                let l = self.a[si + k - fi];
                s -= l * l * self.d[k];
            }
            if diag > 0.0 && s > PIVOT_TOL * diag {
                self.d[i] = s;
                self.d_inv[i] = 1.0 / s;
            } else {
                self.d[i] = 0.0;
                self.d_inv[i] = 0.0;
            }
        }
    }

    /// x ← A⁻¹ x using the factors.
    fn solve(&mut self) {
        let m = self.d.len();
        for i in 0..m {
            let fi = self.first[i];
            let mut s = self.x[i];
            for k in fi..i { s -= self.a[self.at(i, k)] * self.x[k]; }
            self.x[i] = s;
        }
        for i in 0..m { self.x[i] *= self.d_inv[i]; }
        for i in (0..m).rev() {
            let xi = self.x[i];
            for k in self.first[i]..i {
                let l = self.a[self.at(i, k)];
                self.x[k] -= l * xi;
            }
        }
    }
}
