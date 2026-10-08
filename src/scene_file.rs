//! Worlds to and from text: JSON for undo snapshots and clones, and a
//! compact share code (deflated JSON, base64url) for links and the clipboard.
//!
//! Each constraint and force type has a record holding what its
//! constructor and public fields need; reference state (rolling offsets,
//! slider angles) is re-captured from the saved pose on load, which is
//! where it was taken, and an engine cylinder's cycle is saved explicitly.

use base64::Engine;
use glam::{DVec2, Vec2};
use serde::{Deserialize, Serialize};

use crate::sim::{
    body::{Body, BodyShape, DEFAULT_FRICTION, DEFAULT_RESTITUTION},
    constraint::Constraint,
    constraints::{
        cylinder::GasMode, Cylinder, DistanceConstraint, GearJoint, GearKind, PinJoint, PinWorld,
        RollingContact, RollingOnRod, Rope, SliderJoint,
    },
    forces::{Gravity, Motor, SpringDamper, TorsionSpring},
    world::{Tracer, World},
};

const VERSION: u32 = 1;
/// Prefix of a share code, so pasted text is recognisable.
pub const CODE_PREFIX: &str = "psim1:";

#[derive(Serialize, Deserialize)]
pub struct SceneFile {
    pub v: u32,
    pub bodies: Vec<BodyRec>,
    pub items: Vec<Item>,
    #[serde(default)]
    pub tracers: Vec<TracerRec>,
    /// Indices into `items` of connections switched off (`Slot::on`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub switched_off: Vec<usize>,
    /// World-wide contact material from before it was per body; only
    /// read, as the fallback for bodies that don't carry their own.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub friction: Option<f32>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub restitution: Option<f32>,
    /// Camera centre and visible world width × height, if known.
    #[serde(default)]
    pub view: Option<[f32; 4]>,
}


#[derive(Serialize, Deserialize)]
pub enum ShapeRec {
    Disk { r: f32 },
    Rod { hl: f32, hw: f32 },
    Point,
}

#[derive(Serialize, Deserialize)]
pub struct BodyRec {
    pub shape: ShapeRec,
    pub pos: [f64; 2],
    pub angle: f64,
    #[serde(default)]
    pub vel: [f64; 2],
    #[serde(default)]
    pub ang_vel: f64,
    pub mass: f32,
    pub inertia: f32,
    #[serde(default)]
    pub fixed: bool,
    #[serde(default)]
    pub collide: bool,
    #[serde(default)]
    pub friction: Option<f32>,
    #[serde(default)]
    pub restitution: Option<f32>,
}

#[derive(Serialize, Deserialize)]
pub struct TracerRec { pub body: usize, pub local: [f32; 2], pub seconds: f32 }

#[derive(Serialize, Deserialize)]
pub enum Item {
    Pin { a: usize, la: [f32; 2], b: usize, lb: [f32; 2] },
    PinWorld { body: usize, local: [f32; 2], target: [f32; 2] },
    Distance { a: usize, la: [f32; 2], b: usize, lb: [f32; 2], len: f32 },
    Rolling { a: usize, b: usize },
    /// `off`: the disk has rolled off the rod's end.
    RollingOnRod {
        disk: usize,
        rod: usize,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        off: bool,
    },
    Slider { rider: usize, local: [f32; 2], rail: usize, lock: bool },
    Cylinder {
        piston: usize, local: [f32; 2], barrel: usize, crown: f32,
        mode: u8, throttle: f32, state: Vec<f64>,
    },
    Gear {
        a: usize, b: usize, belt: bool,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        collide: bool,
    },
    Rope {
        a: usize, la: [f32; 2], b: usize, lb: [f32; 2],
        pulley: Option<usize>, wrap: f64, length: f32, grip: bool,
    },
    Gravity { g: f32 },
    Motor { body: usize, torque: f32, drag: f32 },
    Spring { a: usize, la: [f32; 2], b: usize, lb: [f32; 2], rest: f32, k: f32, c: f32 },
    Torsion { a: usize, b: usize, la: [f32; 2], rest: f32, k: f32, c: f32 },
}

fn v2(v: Vec2) -> [f32; 2] { [v.x, v.y] }
fn d2(v: DVec2) -> [f64; 2] { [v.x, v.y] }

impl SceneFile {
    pub fn from_world(w: &World) -> Self {
        let bodies = w.bodies.iter().map(|b| BodyRec {
            shape: match b.shape {
                BodyShape::Disk { radius } => ShapeRec::Disk { r: radius },
                BodyShape::Rod { half_len, half_width } => ShapeRec::Rod { hl: half_len, hw: half_width },
                BodyShape::Point => ShapeRec::Point,
            },
            pos: d2(b.pos), angle: b.angle, vel: d2(b.vel), ang_vel: b.ang_vel,
            mass: b.mass, inertia: b.inertia, fixed: b.fixed, collide: b.collide,
            friction: Some(b.friction), restitution: Some(b.restitution),
        }).collect();

        let mut items = Vec::new();
        let mut switched_off = Vec::new();
        for c in &w.constraints {
            let any = c.as_any();
            let item = if let Some(p) = any.downcast_ref::<PinJoint>() {
                Item::Pin { a: p.body_a, la: v2(p.local_a), b: p.body_b, lb: v2(p.local_b) }
            } else if let Some(p) = any.downcast_ref::<PinWorld>() {
                Item::PinWorld { body: p.body, local: v2(p.local), target: v2(p.target) }
            } else if let Some(d) = any.downcast_ref::<DistanceConstraint>() {
                Item::Distance { a: d.body_a, la: v2(d.local_a), b: d.body_b, lb: v2(d.local_b), len: d.rest_len }
            } else if let Some(r) = any.downcast_ref::<RollingContact>() {
                Item::Rolling { a: r.body_a, b: r.body_b }
            } else if let Some(r) = any.downcast_ref::<RollingOnRod>() {
                Item::RollingOnRod { disk: r.disk, rod: r.rod, off: !r.on }
            } else if let Some(s) = any.downcast_ref::<SliderJoint>() {
                Item::Slider { rider: s.rider, local: v2(s.local), rail: s.rail, lock: s.lock_rotation }
            } else if let Some(cy) = any.downcast_ref::<Cylinder>() {
                let mut state = Vec::new();
                cy.save_state(&mut state);
                let mode = match cy.mode { GasMode::Sealed => 0, GasMode::TwoStroke => 2, GasMode::FourStroke => 4 };
                Item::Cylinder {
                    piston: cy.piston, local: v2(cy.local), barrel: cy.barrel, crown: cy.crown,
                    mode, throttle: cy.throttle, state,
                }
            } else if let Some(g) = any.downcast_ref::<GearJoint>() {
                Item::Gear { a: g.body_a, b: g.body_b, belt: g.kind == GearKind::Belt, collide: g.collide }
            } else if let Some(r) = any.downcast_ref::<Rope>() {
                Item::Rope {
                    a: r.body_a, la: v2(r.local_a), b: r.body_b, lb: v2(r.local_b),
                    pulley: r.pulley, wrap: r.wrap, length: r.length, grip: r.grip,
                }
            } else {
                continue;
            };
            if !c.on { switched_off.push(items.len()); }
            items.push(item);
        }
        for f in &w.forces {
            let any = f.as_any();
            let item = if let Some(g) = any.downcast_ref::<Gravity>() {
                Item::Gravity { g: g.g }
            } else if let Some(m) = any.downcast_ref::<Motor>() {
                Item::Motor { body: m.body, torque: m.torque, drag: m.drag }
            } else if let Some(s) = any.downcast_ref::<SpringDamper>() {
                Item::Spring {
                    a: s.body_a, la: v2(s.local_a), b: s.body_b, lb: v2(s.local_b),
                    rest: s.rest_len, k: s.stiffness, c: s.damping,
                }
            } else if let Some(t) = any.downcast_ref::<TorsionSpring>() {
                Item::Torsion { a: t.body_a, b: t.body_b, la: v2(t.local_a), rest: t.rest, k: t.stiffness, c: t.damping }
            } else {
                continue; // the mouse spring belongs to the app
            };
            if !f.on { switched_off.push(items.len()); }
            items.push(item);
        }

        let tracers = w.tracers.iter()
            .map(|t| TracerRec { body: t.body, local: v2(t.local), seconds: t.seconds })
            .collect();
        Self {
            v: VERSION, bodies, items, tracers, switched_off,
            friction: None, restitution: None, view: None,
        }
    }

    /// Rebuild the world. Records that don't fit (bad indices, wrong
    /// shapes) are skipped rather than failing the whole load.
    pub fn to_world(&self) -> World {
        let mut w = World::new();
        for r in &self.bodies {
            let shape = match r.shape {
                ShapeRec::Disk { r } => BodyShape::Disk { radius: r.max(0.01) },
                ShapeRec::Rod { hl, hw } => BodyShape::Rod { half_len: hl.max(0.01), half_width: hw.max(0.005) },
                ShapeRec::Point => BodyShape::Point,
            };
            let mut b = Body::new(Vec2::ZERO, 0.0, r.mass.max(1e-3), r.inertia.max(1e-6), shape);
            b.pos = DVec2::from(r.pos);
            b.angle = r.angle;
            b.vel = DVec2::from(r.vel);
            b.ang_vel = r.ang_vel;
            b.fixed = r.fixed;
            b.collide = r.collide;
            b.friction = r.friction.or(self.friction).unwrap_or(DEFAULT_FRICTION);
            b.restitution = r.restitution.or(self.restitution).unwrap_or(DEFAULT_RESTITUTION);
            w.add_body(b);
        }
        let n = w.bodies.len();
        let l = |a: [f32; 2]| Vec2::from(a);

        for (idx, item) in self.items.iter().enumerate() {
            let before = (w.constraints.len(), w.forces.len());
            self.add_item(&mut w, item, n);
            if self.switched_off.contains(&idx) {
                if w.constraints.len() > before.0 { w.constraints.last_mut().unwrap().on = false; }
                if w.forces.len() > before.1 { w.forces.last_mut().unwrap().on = false; }
            }
        }
        for t in &self.tracers {
            if t.body < n {
                w.tracers.push(Tracer { body: t.body, local: l(t.local), seconds: t.seconds });
            }
        }
        w
    }

    /// Add one record's constraint or force to `w` (which has `n` bodies);
    /// records that don't fit are skipped.
    fn add_item(&self, w: &mut World, item: &Item, n: usize) {
        let ok = |ids: &[usize]| ids.iter().all(|&i| i < n);
        let l = |a: [f32; 2]| Vec2::from(a);
        match *item {
            Item::Pin { a, la, b, lb } if ok(&[a, b]) => w.add_constraint(PinJoint::new(a, l(la), b, l(lb))),
            Item::PinWorld { body, local, target } if ok(&[body]) => w.add_constraint(PinWorld::new(body, l(local), l(target))),
            Item::Distance { a, la, b, lb, len } if ok(&[a, b]) => w.add_constraint(DistanceConstraint::new(a, l(la), b, l(lb), len)),
            Item::Rolling { a, b } if ok(&[a, b]) => {
                if let Some(c) = RollingContact::new(a, b, &w.bodies) { w.add_constraint(c); }
            }
            Item::RollingOnRod { disk, rod, off } if ok(&[disk, rod]) => {
                if let Some(mut c) = RollingOnRod::new(disk, rod, &w.bodies) {
                    c.on &= !off;
                    w.add_constraint(c);
                }
            }
            Item::Slider { rider, local, rail, lock } if ok(&[rider, rail]) => {
                if let Some(mut s) = SliderJoint::new(rider, l(local), rail, &w.bodies) {
                    s.lock_rotation = lock;
                    w.add_constraint(s);
                }
            }
            Item::Cylinder { piston, local, barrel, crown, mode, throttle, ref state } if ok(&[piston, barrel]) => {
                if let Some(mut cy) = Cylinder::new(piston, l(local), barrel, &w.bodies) {
                    cy.crown = crown;
                    cy.mode = match mode { 2 => GasMode::TwoStroke, 4 => GasMode::FourStroke, _ => GasMode::Sealed };
                    cy.throttle = throttle;
                    cy.rebase(&w.bodies);
                    cy.load_state(&mut state.as_slice());
                    w.add_constraint(cy);
                }
            }
            Item::Gear { a, b, belt, collide } if ok(&[a, b]) => {
                let kind = if belt { GearKind::Belt } else { GearKind::Mesh };
                if let Some(mut g) = GearJoint::new(a, b, kind, &w.bodies) {
                    g.collide = collide && belt;
                    w.add_constraint(g);
                }
            }
            Item::Rope { a, la, b, lb, pulley, wrap, length, grip } if ok(&[a, b]) && pulley.is_none_or(|p| p < n) => {
                let mut r = Rope::new(a, l(la), b, l(lb), &w.bodies);
                r.pulley = pulley;
                r.wrap = wrap;
                r.length = length;
                r.grip = grip;
                r.rebase(&w.bodies);
                w.add_constraint(r);
            }
            Item::Gravity { g } => w.add_force(Gravity::new(g)),
            Item::Motor { body, torque, drag } if ok(&[body]) => w.add_force(Motor::new(body, torque, drag)),
            Item::Spring { a, la, b, lb, rest, k, c } if ok(&[a, b]) => {
                w.add_force(SpringDamper::new(a, l(la), b, l(lb), rest, k, c));
            }
            Item::Torsion { a, b, la, rest, k, c } if ok(&[a, b]) => {
                let mut t = TorsionSpring::new(a, b, l(la), k, c, &w.bodies);
                t.rest = rest;
                w.add_force(t);
            }
            _ => {}
        }
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    pub fn from_json(s: &str) -> Option<Self> {
        serde_json::from_str(s).ok()
    }

    /// Compact share code: `psim1:` + base64url(deflate(JSON)).
    pub fn to_code(&self) -> String {
        let packed = miniz_oxide::deflate::compress_to_vec(self.to_json().as_bytes(), 9);
        format!("{CODE_PREFIX}{}", base64::engine::general_purpose::URL_SAFE_NO_PAD.encode(packed))
    }

    /// Accepts a share code with or without its prefix, or a link ending in
    /// `#s=<code>`, surrounded by whitespace.
    pub fn from_code(s: &str) -> Option<Self> {
        let s = s.trim();
        let s = s.rsplit_once("#s=").map_or(s, |(_, c)| c);
        let s = s.strip_prefix(CODE_PREFIX).unwrap_or(s);
        let packed = base64::engine::general_purpose::URL_SAFE_NO_PAD.decode(s.trim_end_matches('=')).ok()?;
        let json = miniz_oxide::inflate::decompress_to_vec_with_limit(&packed, 64 << 20).ok()?;
        Self::from_json(std::str::from_utf8(&json).ok()?)
    }
}

/// A deep copy of a world through its saved form (the app's mouse spring
/// is not included).
/// The bodies `keep` with every constraint, force and trace that involves
/// only them, as a scene file (the editor clipboard). `None` if empty.
pub fn extract(w: &World, keep: &[usize]) -> Option<SceneFile> {
    if keep.is_empty() { return None; }
    let mut sub = SceneFile::from_world(w).to_world();
    let ed = crate::editor::Editor::default();
    for i in (0..sub.bodies.len()).rev() {
        if !keep.contains(&i) { ed.delete_body(i, &mut sub); }
    }
    let mut file = SceneFile::from_world(&sub);
    file.view = None;
    Some(file)
}

/// Add a clipboard scene to `w`, centred on `at`. Returns the new bodies'
/// indices. World pins move along; gravity isn't duplicated.
pub fn paste_into(w: &mut World, file: &SceneFile, at: Vec2) -> Vec<usize> {
    let mut sub = file.to_world();
    let n = sub.bodies.len();
    if n == 0 { return Vec::new(); }
    let centre = sub.bodies.iter().map(|b| b.pos).sum::<DVec2>() / n as f64;
    let shift = at.as_dvec2() - centre;
    for b in &mut sub.bodies { b.pos += shift; }
    let off = w.bodies.len();
    // Highest index first, so a shifted index never meets one still
    // waiting to be shifted.
    for i in (0..n).rev() {
        for c in sub.constraints.iter_mut() { c.remap_body(i, i + off); }
        for f in sub.forces.iter_mut() { f.remap_body(i, i + off); }
    }
    for t in &mut sub.tracers { t.body += off; }
    w.bodies.append(&mut sub.bodies);
    w.constraints.append(&mut sub.constraints);
    w.forces.extend(sub.forces.into_iter().filter(|f| !f.as_any().is::<Gravity>()));
    w.tracers.append(&mut sub.tracers);
    let ids: Vec<usize> = (off..off + n).collect();
    for &i in &ids { crate::editor::sync_world_pins(w, i); }
    ids
}

pub fn clone_world(w: &World) -> World {
    let mut c = SceneFile::from_world(w).to_world();
    c.integrator = w.integrator;
    c
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenes::SCENES;

    /// Every scene survives a round trip through a share code and then
    /// steps exactly like the original.
    #[test]
    fn round_trip_every_scene() {
        for def in SCENES {
            let mut a = (def.build)();
            for _ in 0..50 { a.step(1.0 / 240.0); }
            let code = SceneFile::from_world(&a).to_code();
            let mut b = SceneFile::from_code(&code).expect("decodes").to_world();
            assert_eq!(a.bodies.len(), b.bodies.len(), "{}", def.name);
            assert_eq!(a.constraints.len(), b.constraints.len(), "{}", def.name);
            assert_eq!(a.forces.len(), b.forces.len(), "{}", def.name);
            // Contact friction (stick springs) isn't saved, as with a rewind;
            // drop it on the original too so both start alike.
            a.contacts.reset();
            for _ in 0..200 { a.step(1.0 / 240.0); b.step(1.0 / 240.0); }
            let drift = a.bodies.iter().zip(&b.bodies)
                .map(|(x, y)| (x.pos - y.pos).length() + (x.angle - y.angle).abs())
                .fold(0.0, f64::max);
            eprintln!("{:<22} code {:>5} chars, drift after 200 steps {drift:.1e}", def.name, code.len());
            assert!(drift < 1e-6, "{} diverged after reload: {drift}", def.name);
        }
    }

    /// Copying part of a scene keeps only what links the copied bodies;
    /// pasting adds it whole (gravity not doubled), world pins moved along.
    #[test]
    fn copy_paste_part_of_a_scene() {
        use crate::sim::forces::Gravity;
        for (idx, def) in SCENES.iter().enumerate() {
            let mut w = (def.build)();
            let n = w.bodies.len();
            if n < 2 { continue; }
            // Copy everything: pasting doubles bodies and constraints.
            let all: Vec<usize> = (0..n).collect();
            let file = extract(&w, &all).unwrap();
            let (nc, nf) = (w.constraints.len(), w.forces.len());
            let gravity = w.forces.iter().filter(|f| f.as_any().is::<Gravity>()).count();
            let ids = paste_into(&mut w, &file, Vec2::new(50.0, 0.0));
            assert_eq!(ids, (n..2 * n).collect::<Vec<_>>(), "{}", def.name);
            assert_eq!(w.constraints.len(), 2 * nc, "{}", def.name);
            assert_eq!(w.forces.len(), 2 * nf - gravity, "{}", def.name);
            for c in &w.constraints {
                assert!(c.body_indices().iter().all(|&b| b < 2 * n), "{} #{idx}", def.name);
            }
            for _ in 0..200 { w.step(0.001); }
            assert!(w.constraint_error() < 1e-2, "{}: pasted copy holds together, err {}", def.name, w.constraint_error());

            // Copy one body: nothing that reaches outside comes along.
            let one = extract(&(def.build)(), &[0]).unwrap();
            assert_eq!(one.bodies.len(), 1);
        }
    }

    /// A switched-off pin lets its body fall; the switch survives a share
    /// code; switched back on, the pin holds again.
    #[test]
    fn switched_off_connections() {
        use crate::sim::body::disk_inertia;
        use crate::sim::forces::Gravity;
        let mut w = World::new();
        let d = w.add_body(Body::new(Vec2::ZERO, 0.0, 1.0, disk_inertia(1.0, 0.2), BodyShape::Disk { radius: 0.2 }));
        w.add_constraint(PinWorld::new(d, Vec2::ZERO, Vec2::ZERO));
        w.add_force(Gravity::new(9.81));
        w.constraints[0].on = false;
        let mut b = SceneFile::from_code(&SceneFile::from_world(&w).to_code()).unwrap().to_world();
        assert!(!b.constraints[0].on, "saved switched off");
        for _ in 0..100 { b.step(0.001); }
        assert!(b.bodies[d].pos.y < -0.04, "falls while off: {}", b.bodies[d].pos.y);
        assert_eq!(b.constraint_error(), 0.0, "an off constraint isn't counted");
        b.constraints[0].on = true;
        for _ in 0..2000 { b.step(0.001); }
        assert!(b.bodies[d].pos.length() < 1e-3, "pulled back once on: {:?}", b.bodies[d].pos);
    }
}
