//! Physics benchmark: per-step cost plus accuracy at a fixed wall-time budget.
//! `cargo test --release bench_scenes -- --ignored --nocapture`

use web_time::Instant;
use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, PinWorld, RollingContact},
    forces::{Gravity, SpringDamper},
    world::World,
};
use super::{anchor, disk, from_down, rod_between, SCENES};

/// Physics share of a 60 fps frame, as in `App::update`.
const FRAME_BUDGET: f32 = 0.8 / 60.0;

pub(crate) fn energy(w: &World) -> f32 {
    let g = w.forces.iter()
        .find_map(|f| f.as_any().downcast_ref::<Gravity>().map(|g| g.g))
        .unwrap_or(0.0);
    w.kinetic_energy() + w.potential_energy(g)
}

/// Independent copies of several scenes in one world (they may overlap in
/// space: nothing collides). Gravity is kept once.
pub(crate) fn merged(builds: &[fn() -> World]) -> World {
    let mut out = World::new();
    out.add_force(Gravity::new(9.81));
    for build in builds {
        let mut w = build();
        let off = out.bodies.len();
        let n = w.bodies.len();
        // Highest index first, so a shifted index never collides with one
        // still waiting to be shifted.
        for i in (0..n).rev() {
            for c in w.constraints.iter_mut() { c.remap_body(i, i + off); }
            for f in w.forces.iter_mut() { f.remap_body(i, i + off); }
        }
        out.bodies.append(&mut w.bodies);
        out.constraints.append(&mut w.constraints);
        out.forces.extend(w.forces.into_iter().filter(|f| !f.as_any().is::<Gravity>()));
    }
    out
}

/// A 20-link chain hanging from a world pin, with a wheel pinned to two of
/// its joints, a disk rolling around each wheel, and a spring from its tip
/// to the floor: one long coupled system (banded JWJᵀ), unlike merged
/// copies (block diagonal).
pub(crate) fn chain() -> World {
    const LINKS: usize = 20;
    const LEN: f32 = 0.25;
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));
    let top = Vec2::new(0.0, 4.0);
    let mut prev: Option<(usize, f32)> = None;
    let mut p = top;
    let mut ids = Vec::new();
    for i in 0..LINKS {
        let q = p + from_down(1.2 - 0.05 * i as f32) * LEN;
        let (body, hl) = rod_between(p, q, 0.3, 0.04);
        let id = w.add_body(body);
        match prev {
            None => w.add_constraint(PinWorld::new(id, Vec2::new(-hl, 0.0), top)),
            Some((pid, phl)) => w.add_constraint(PinJoint::new(pid, Vec2::new(phl, 0.0), id, Vec2::new(-hl, 0.0))),
        }
        ids.push((id, hl));
        prev = Some((id, hl));
        p = q;
    }
    for &k in &[5usize, 12] {
        let (rod, hl) = ids[k];
        let joint = w.bodies[rod].world_point(Vec2::new(hl, 0.0));
        let wheel = w.add_body(disk(joint, 0.15, 0.5));
        w.add_constraint(PinJoint::new(rod, Vec2::new(hl, 0.0), wheel, Vec2::ZERO));
        let rider = w.add_body(disk(joint + Vec2::new(0.0, 0.25), 0.1, 0.3));
        w.add_constraint(RollingContact::new(wheel, rider, &w.bodies).unwrap());
    }
    let (tip, hl) = ids[LINKS - 1];
    let floor = w.add_body(anchor(Vec2::new(p.x, -1.0)));
    w.add_force(SpringDamper::new(tip, Vec2::new(hl, 0.0), floor, Vec2::ZERO, (p.y + 1.0) * 0.8, 40.0, 0.2));
    w
}

type Named = (&'static str, fn() -> World);

/// A scene's builder by name.
fn scene(name: &str) -> fn() -> World {
    SCENES.iter().find(|d| d.name == name).expect("scene").build
}

fn stress_worlds() -> Vec<Named> {
    vec![
        ("3x Rocker Linkage", || { let r = scene("Rocker Linkage"); merged(&[r, r, r]) }),
        ("Mix x3", || merged(&[
            scene("Rocker Linkage"), scene("Rolling Crane"), scene("Planetary Pendulum"),
            scene("Double Pendulum"), scene("Coupled Pendulums"),
        ])),
        ("Chain 20", chain),
    ]
}

use crate::sim::world::Integrator;

struct Run { us_per_step: f32, max_err: f32, drift: f32 }

/// Simulates `secs` at step `dt`, timing only the steps.
fn run(build: fn() -> World, integ: Integrator, dt: f32, secs: f32) -> Run {
    let mut w = build();
    w.integrator = integ;
    let e0 = energy(&w);
    let n = (secs / dt) as usize;
    let mut max_err = 0.0f32;
    let mut spent = 0.0f64;
    // Check error every few steps so measuring doesn't dominate.
    let chunk = 16;
    let mut done = 0;
    while done < n {
        let k = chunk.min(n - done);
        let t0 = Instant::now();
        for _ in 0..k { w.step(dt); }
        spent += t0.elapsed().as_secs_f64();
        max_err = max_err.max(w.constraint_error());
        done += k;
    }
    let ok = w.bodies.iter().all(|b| b.pos.is_finite() && b.vel.is_finite());
    let drift = if ok { energy(&w) - e0 } else { f32::NAN };
    Run { us_per_step: (spent * 1e6 / n as f64) as f32, max_err, drift }
}

/// Engine speed over time (crank is body 0), to tune the scene.
#[test]
#[ignore]
fn engine_rpm() {
    for integ in [Integrator::Rk4, Integrator::Xpbd] {
        let mut w = super::radial_engine::build();
        w.integrator = integ;
        let hz = 4000.0;
        let mut line = String::new();
        let mut max_err = 0.0f32;
        for k in 0..(12.0 * hz) as usize {
            if k % (hz as usize) == 0 {
                line += &format!(" {:.0}", w.bodies[0].ang_vel * 60.0 / std::f64::consts::TAU);
            }
            w.step(1.0 / hz);
            max_err = max_err.max(w.constraint_error());
        }
        eprintln!("{integ:?} rpm/s:{line}  max err {max_err:.1e}");
    }
}

#[test]
#[ignore]
fn bench_scenes() {
    let mut all: Vec<Named> =
        SCENES.iter().filter(|d| d.name != "Sandbox").map(|d| (d.name, d.build)).collect();
    all.extend(stress_worlds());

    eprintln!(
        "{:<25} {:>5} {:>5} | {:>8} {:>9} {:>9} | {:>6} {:>9} {:>9}",
        "world", "bod", "rows", "µs/step", "err@960", "ΔE@960", "steps", "err@budg", "ΔE@budg",
    );
    for ((name, build), integ) in all.iter().flat_map(|&e| [(e, Integrator::Rk4), (e, Integrator::Xpbd)]) {
        let name = format!("{name} {}", if integ == Integrator::Rk4 { "RK4" } else { "XPBD" });
        let w = build();
        let rows: usize = w.constraints.iter().map(|c| c.dim()).sum();
        let fixed = run(build, integ, 1.0 / 960.0, 20.0);
        // As many steps per 60 fps frame as the budget allows.
        let steps = (FRAME_BUDGET / (fixed.us_per_step * 1e-6)).clamp(1.0, 20000.0) as usize;
        let budg = run(build, integ, 1.0 / 60.0 / steps as f32, 5.0);
        eprintln!(
            "{:<25} {:>5} {:>5} | {:>8.2} {:>9.1e} {:>+9.3} | {:>6} {:>9.1e} {:>+9.3}",
            name, w.bodies.len(), rows, fixed.us_per_step, fixed.max_err, fixed.drift,
            steps, budg.max_err, budg.drift,
        );
    }
}


