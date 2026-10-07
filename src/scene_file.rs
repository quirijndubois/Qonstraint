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
    body::{Body, BodyShape},
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
    #[serde(default = "default_friction")]
    pub friction: f32,
    #[serde(default = "default_restitution")]
    pub restitution: f32,
    /// Camera centre and visible world width × height, if known.
    #[serde(default)]
    pub view: Option<[f32; 4]>,
}

fn default_friction() -> f32 { 0.5 }
fn default_restitution() -> f32 { 0.3 }

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
}

#[derive(Serialize, Deserialize)]
pub struct TracerRec { pub body: usize, pub local: [f32; 2], pub seconds: f32 }

#[derive(Serialize, Deserialize)]
pub enum Item {
    Pin { a: usize, la: [f32; 2], b: usize, lb: [f32; 2] },
    PinWorld { body: usize, local: [f32; 2], target: [f32; 2] },
    Distance { a: usize, la: [f32; 2], b: usize, lb: [f32; 2], len: f32 },
    Rolling { a: usize, b: usize },
    RollingOnRod { disk: usize, rod: usize },
    Slider { rider: usize, local: [f32; 2], rail: usize, lock: bool },
    Cylinder {
        piston: usize, local: [f32; 2], barrel: usize, crown: f32,
        mode: u8, throttle: f32, state: Vec<f64>,
    },
    Gear { a: usize, b: usize, belt: bool },
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
        }).collect();

        let mut items = Vec::new();
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
                Item::RollingOnRod { disk: r.disk, rod: r.rod }
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
                Item::Gear { a: g.body_a, b: g.body_b, belt: g.kind == GearKind::Belt }
            } else if let Some(r) = any.downcast_ref::<Rope>() {
                Item::Rope {
                    a: r.body_a, la: v2(r.local_a), b: r.body_b, lb: v2(r.local_b),
                    pulley: r.pulley, wrap: r.wrap, length: r.length, grip: r.grip,
                }
            } else {
                continue;
            };
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
            items.push(item);
        }

        let tracers = w.tracers.iter()
            .map(|t| TracerRec { body: t.body, local: v2(t.local), seconds: t.seconds })
            .collect();
        Self {
            v: VERSION, bodies, items, tracers,
            friction: w.contacts.friction, restitution: w.contacts.restitution, view: None,
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
            w.add_body(b);
        }
        let n = w.bodies.len();
        let ok = |ids: &[usize]| ids.iter().all(|&i| i < n);
        let l = |a: [f32; 2]| Vec2::from(a);

        for item in &self.items {
            match *item {
                Item::Pin { a, la, b, lb } if ok(&[a, b]) => w.add_constraint(PinJoint::new(a, l(la), b, l(lb))),
                Item::PinWorld { body, local, target } if ok(&[body]) => w.add_constraint(PinWorld::new(body, l(local), l(target))),
                Item::Distance { a, la, b, lb, len } if ok(&[a, b]) => w.add_constraint(DistanceConstraint::new(a, l(la), b, l(lb), len)),
                Item::Rolling { a, b } if ok(&[a, b]) => {
                    if let Some(c) = RollingContact::new(a, b, &w.bodies) { w.add_constraint(c); }
                }
                Item::RollingOnRod { disk, rod } if ok(&[disk, rod]) => {
                    if let Some(c) = RollingOnRod::new(disk, rod, &w.bodies) { w.add_constraint(c); }
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
                Item::Gear { a, b, belt } if ok(&[a, b]) => {
                    let kind = if belt { GearKind::Belt } else { GearKind::Mesh };
                    if let Some(g) = GearJoint::new(a, b, kind, &w.bodies) { w.add_constraint(g); }
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
        for t in &self.tracers {
            if t.body < n {
                w.tracers.push(Tracer { body: t.body, local: l(t.local), seconds: t.seconds });
            }
        }
        w.contacts.friction = self.friction;
        w.contacts.restitution = self.restitution;
        w
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
            for _ in 0..200 { a.step(1.0 / 240.0); b.step(1.0 / 240.0); }
            let drift = a.bodies.iter().zip(&b.bodies)
                .map(|(x, y)| (x.pos - y.pos).length() + (x.angle - y.angle).abs())
                .fold(0.0, f64::max);
            eprintln!("{:<22} code {:>5} chars, drift after 200 steps {drift:.1e}", def.name, code.len());
            assert!(drift < 1e-6, "{} diverged after reload: {drift}", def.name);
        }
    }
}
