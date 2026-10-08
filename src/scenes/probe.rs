//! Development probe: `PROBE="Scene Name" cargo test --release probe -- --ignored --nocapture`
//! runs one scene and prints, every half second, energy, constraint error,
//! the free bodies' extent, contacts and step cost. `PROBE_INTEG=xpbd`
//! switches integrator, `PROBE_T` sets the duration, `PROBE_BODIES=1,4`
//! adds those bodies' positions, `PROBE_EVERY` prints every that many frames.

use glam::Vec2;
use super::SCENES;
use crate::sim::{forces::Gravity, world::Integrator};

#[test]
#[ignore]
fn probe() {
    let name = std::env::var("PROBE").unwrap_or_default();
    let def = SCENES.iter().find(|d| d.name.eq_ignore_ascii_case(&name)).expect("PROBE=<scene name>");
    let mut w = (def.build)();
    if std::env::var("PROBE_INTEG").is_ok_and(|v| v == "xpbd") { w.integrator = Integrator::Xpbd; }
    w.record_reactions = true;
    let sub = if w.integrator == Integrator::Xpbd { 4 } else { 1 };
    let t_end: f32 = std::env::var("PROBE_T").ok().and_then(|v| v.parse().ok()).unwrap_or(20.0);
    let watch: Vec<usize> = std::env::var("PROBE_BODIES").ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect()).unwrap_or_default();
    let g = w.forces.iter().find_map(|f| f.as_any().downcast_ref::<Gravity>().map(|g| g.g)).unwrap_or(0.0);
    let dt = 1.0 / 240.0;
    let frames = (t_end / dt) as usize;
    let c = Vec2::from(def.camera_center);
    let half = Vec2::from(def.view_size) * 0.5;
    let f0 = w.follow.map_or(Vec2::ZERO, |i| w.bodies[i].pos32());
    eprintln!("{}: {} bodies, {} constraints, {} forces", def.name, w.bodies.len(), w.constraints.len(), w.forces.len());
    let start: Vec<Vec2> = w.bodies.iter().map(|b| b.pos32()).collect();
    let mut peak = vec![0.0f32; w.bodies.len()];
    // PROBE_DUMP=file and PROBE_AT=t1,t2,..: every body's shape and pose at
    // those times, one frame per line, for drawing offline.
    let dump_at: Vec<f32> = std::env::var("PROBE_AT").ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect()).unwrap_or_default();
    let mut dump = std::env::var("PROBE_DUMP").ok().map(|f| std::fs::File::create(f).expect("dump file"));
    let every: usize = std::env::var("PROBE_EVERY").ok().and_then(|v| v.parse().ok()).unwrap_or(120);
    let mut broken_before = 0;
    let t0 = std::time::Instant::now();
    let mut max_err = 0.0f32;
    let mut max_out = f32::NEG_INFINITY;
    for frame in 1..=frames {
        for _ in 0..sub { w.step(dt / sub as f32); }
        max_err = max_err.max(w.constraint_error());
        if let Some(f) = dump.as_mut() {
            let t = frame as f32 * dt;
            if dump_at.iter().any(|&a| (a - t).abs() < 0.5 * dt) {
                use std::io::Write;
                let mut line = format!("{t}");
                for b in &w.bodies {
                    let (kind, a, c) = match b.shape {
                        crate::sim::body::BodyShape::Disk { radius } => ("D", radius, 0.0),
                        crate::sim::body::BodyShape::Rod { half_len, half_width } => ("R", half_len, half_width),
                        crate::sim::body::BodyShape::Point => ("P", 0.0, 0.0),
                    };
                    line += &format!(";{kind},{},{},{},{a},{c}", b.pos.x, b.pos.y, b.angle);
                }
                writeln!(f, "{line}").unwrap();
            }
        }
        let broken_now = w.constraints.iter().filter(|c| c.broken).count();
        if broken_now != broken_before {
            eprint!("BREAK t={:.3} broken={broken_now}", frame as f32 * dt);
            for &i in &watch {
                let b = &w.bodies[i];
                eprint!(" [{i}: p=({:.2},{:.2}) v=({:.2},{:.2})]", b.pos.x, b.pos.y, b.vel.x, b.vel.y);
            }
            eprintln!();
            broken_before = broken_now;
        }
        for &i in &watch { peak[i] = peak[i].max((w.bodies[i].pos32() - start[i]).length()); }
        let cc = c + w.follow.map_or(Vec2::ZERO, |i| w.bodies[i].pos32()) - f0;
        for b in w.bodies.iter().filter(|b| !b.fixed) {
            max_out = max_out.max(((b.pos32() - cc).abs() - half).max_element());
        }
        if frame % every == 0 {
            let (mut lo, mut hi) = (Vec2::splat(f32::INFINITY), Vec2::splat(f32::NEG_INFINITY));
            for b in w.bodies.iter().filter(|b| !b.fixed) { lo = lo.min(b.pos32()); hi = hi.max(b.pos32()); }
            let broken = w.constraints.iter().filter(|c| c.broken).count();
            let load = w.reactions.iter().enumerate().map(|(i, r)| (r.max_force(), i)).fold((0.0, 0), |m, v| if v.0 > m.0 { v } else { m });
            let us = t0.elapsed().as_secs_f64() * 1e6 / (frame * sub) as f64;
            eprint!("t={:5.1} E={:9.3} KE={:8.3} err={:.1e} maxerr={:.1e} out={:+.2} box=({:.2},{:.2})..({:.2},{:.2}) contacts={} broken={} load {:.0}@{} {:.1}µs/step",
                frame as f32 * dt, w.kinetic_energy() + w.potential_energy(g), w.kinetic_energy(), w.constraint_error(), max_err, max_out,
                lo.x, lo.y, hi.x, hi.y, w.contacts.last.len(), broken, load.0, load.1, us);
            for &i in &watch {
                let b = &w.bodies[i];
                eprint!(" [{i}: {:.2},{:.2} a={:.2} w={:.2}]", b.pos.x, b.pos.y, b.angle, b.ang_vel);
            }
            eprintln!();
            if std::env::var("PROBE_FAST").is_ok() {
                let (v, i) = w.bodies.iter().enumerate().map(|(i, b)| (b.vel.length() + b.ang_vel.abs(), i)).fold((0.0, 0), |m, x| if x.0 > m.0 { x } else { m });
                let b = &w.bodies[i];
                eprintln!("    fastest {i}: |v|+|w| {v:.2} at ({:.2},{:.2}) {:?}", b.pos.x, b.pos.y, b.shape);
            }
            // PROBE_CONTACTS=k: the contacts on body k.
            if let Some(k) = std::env::var("PROBE_CONTACTS").ok().and_then(|v| v.parse::<usize>().ok()) {
                for c in w.contacts.last.iter().filter(|c| c.i == k || c.j == k) {
                    eprintln!("    contact {}-{} f{} pen {:.4} n ({:.2},{:.2}) f_n {:.1} f_t {:.1} belt {}", c.i, c.j, c.feature, c.pen, c.n.x, c.n.y, c.f_n, c.f_t, c.belt.is_some());
                }
            }
        }
    }
    for &i in &watch {
        let b = &w.bodies[i];
        eprintln!("body {i}: peak displacement {:.3}, ends at ({:.2},{:.2})", peak[i], b.pos.x, b.pos.y);
    }
}

/// `PROBE="Scene" cargo test --release probe_reload -- --ignored --nocapture`:
/// after 50 steps, steps the world and two reloaded copies of it side by
/// side and reports the first step where they part.
#[test]
#[ignore]
fn probe_reload() {
    use crate::scene_file::SceneFile;
    let name = std::env::var("PROBE").unwrap_or_default();
    let def = SCENES.iter().find(|d| d.name.eq_ignore_ascii_case(&name)).expect("PROBE=<scene name>");
    let mut a = (def.build)();
    for _ in 0..50 { a.step(1.0 / 240.0); }
    let reload = |w: &crate::sim::world::World| SceneFile::from_code(&SceneFile::from_world(w).to_code()).unwrap().to_world();
    let (mut b, mut c) = (reload(&a), reload(&a));
    a.contacts.reset();
    let diff = |x: &crate::sim::world::World, y: &crate::sim::world::World| x.bodies.iter().zip(&y.bodies).enumerate()
        .map(|(i, (p, q))| ((p.pos - q.pos).length() + (p.vel - q.vel).length(), i))
        .fold((0.0, 0), |m, v| if v.0 > m.0 { v } else { m });
    for s in 0..200 {
        for w in [&mut a, &mut b, &mut c] { w.step(1.0 / 240.0); }
        let (ab, bc) = (diff(&a, &b), diff(&b, &c));
        if ab.0 > 0.0 || bc.0 > 0.0 {
            eprintln!("step {s}: original vs reload {:.2e} (body {}), reload vs reload {:.2e}", ab.0, ab.1, bc.0);
            break;
        }
    }
}

