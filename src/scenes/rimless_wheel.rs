use std::f32::consts::TAU;
use glam::Vec2;

use crate::sim::{
    body::Body,
    constraints::WeldJoint,
    forces::Gravity,
    world::World,
};
use super::{disk, rod_between};

// The rimless wheel, the simplest passive walker: a hub with ten spokes
// welded on and no rim, rolling down a gentle slope. Each step ends in an
// inelastic impact as the next spoke lands, which takes away a fixed share
// of the speed, while the slope adds energy; the two balance at a steady
// gait (a limit cycle), whatever speed it starts with. At the bottom the
// slope runs out onto the flat and the impacts stop it within a few steps.
// The camera follows the hub.

const SPOKES: usize = 10;
const LEG: f32 = 0.6;
const SLOPE: f32 = 0.14;      // rad
const RAMP: f32 = 34.0;       // length along the slope

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    // Ramp down to the right, then a long flat.
    let top = Vec2::new(-2.0, 0.0);
    let foot = top + Vec2::new(SLOPE.cos(), -SLOPE.sin()) * RAMP;
    let ground = |w: &mut World, a: Vec2, b: Vec2| {
        let (g, _) = rod_between(a, b, 50.0, 0.05);
        let mut g = g.fixed().colliding();
        g.friction = 1.2;
        g.restitution = 0.0;
        w.add_body(g);
    };
    ground(&mut w, top - Vec2::new(SLOPE.cos(), -SLOPE.sin()) * 2.0, foot);
    ground(&mut w, foot, foot + Vec2::new(14.0, 0.0));

    // Standing on two spokes straddling the slope's normal.
    let n = Vec2::new(SLOPE.sin(), SLOPE.cos());
    let half = TAU / SPOKES as f32 * 0.5;
    let hub_at = top + Vec2::new(SLOPE.cos(), -SLOPE.sin()) * 1.0 + n * (0.05 + (LEG + 0.03) * half.cos());
    // The hub outweighs its spokes in inertia: XPBD's single projection
    // sweep diverges on a light hub holding many heavier welded parts.
    let mut hub = disk(hub_at, 0.2, 8.0);
    // A gentle push down the slope (clockwise roll).
    hub.vel = (Vec2::new(SLOPE.cos(), -SLOPE.sin()) * 0.6).as_dvec2();
    let hub_v = hub.vel;
    hub.ang_vel = -0.6 / (LEG as f64);
    let spin = hub.ang_vel;
    let hub = w.add_body(hub);
    let down = -n;
    let base = down.y.atan2(down.x) + half;
    for k in 0..SPOKES {
        let a = base + TAU * k as f32 / SPOKES as f32;
        let u = Vec2::new(a.cos(), a.sin());
        let (spoke, _) = rod_between(hub_at + u * 0.1, hub_at + u * LEG, 0.08, 0.03);
        let mut spoke: Body = spoke.colliding();
        spoke.friction = 1.2;
        spoke.restitution = 0.0;
        // Rigid with the hub: v = v_hub + ω × r.
        let r = spoke.pos - hub_at.as_dvec2();
        spoke.vel = hub_v + spin * r.perp();
        spoke.ang_vel = spin;
        let s = w.add_body(spoke);
        w.add_constraint(WeldJoint::new(hub, s, hub_at, &w.bodies));
    }
    w.add_tracer(hub, Vec2::ZERO);
    w.tracers[0].seconds = 6.0;
    w.follow = Some(hub);
    w
}
