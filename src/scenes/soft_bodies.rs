use std::f32::consts::TAU;
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::PinWorld,
    forces::{spring::SpringDamper, Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// Soft bodies from springs: each blob is a ring of small colliding disks
// (its skin) tied to its neighbours, to the ones two along, and by spokes
// to a hub, all with damped springs. Six blobs drop onto two shelves into
// a tub where a slow paddle keeps kneading them; they squash, bounce, pile
// up and slide over each other.

const RING: usize = 12;
const SKIN_R: f32 = 0.05;
const HALF_W: f32 = 2.6;
const FLOOR: f32 = -1.6;

fn scenery(b: Body) -> Body {
    let mut b = b.fixed().colliding();
    b.friction = 0.6;
    b.restitution = 0.2;
    b
}

/// A blob of radius `r` at `c`: its skin disks' springs and spokes.
fn blob(w: &mut World, c: Vec2, r: f32, stiff: f32, spin: f32) {
    let hub = w.add_body(disk(c, 0.08, 0.15));
    let skin: Vec<usize> = (0..RING).map(|k| {
        let a = TAU * k as f32 / RING as f32;
        let p = c + Vec2::new(a.cos(), a.sin()) * r;
        let mut b = disk(p, SKIN_R, 0.05).colliding();
        b.friction = 0.6;
        b.restitution = 0.2;
        // Spinning as a whole: v = ω × r.
        b.vel = (Vec2::new(-a.sin(), a.cos()) * r * spin).as_dvec2();
        w.add_body(b)
    }).collect();
    let spring = |w: &mut World, a: usize, b: usize, k: f32| {
        let rest = (w.bodies[b].pos - w.bodies[a].pos).length() as f32;
        w.add_force(SpringDamper::new(a, Vec2::ZERO, b, Vec2::ZERO, rest, k, 0.04 * k.sqrt()));
    };
    for k in 0..RING {
        spring(w, skin[k], skin[(k + 1) % RING], stiff);
        spring(w, skin[k], skin[(k + 2) % RING], 0.5 * stiff);
        spring(w, hub, skin[k], 0.35 * stiff);
    }
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let wall = |w: &mut World, a: Vec2, b: Vec2| {
        let (body, _) = rod_between(a, b, 5.0, 0.06);
        w.add_body(scenery(body));
    };
    // Tub and shelves.
    wall(&mut w, Vec2::new(-HALF_W, FLOOR), Vec2::new(HALF_W, FLOOR));
    wall(&mut w, Vec2::new(-HALF_W, FLOOR), Vec2::new(-HALF_W, 3.2));
    wall(&mut w, Vec2::new(HALF_W, FLOOR), Vec2::new(HALF_W, 3.2));
    wall(&mut w, Vec2::new(-HALF_W, 1.2), Vec2::new(-0.6, 0.5));
    wall(&mut w, Vec2::new(HALF_W, 0.1), Vec2::new(0.9, -0.5));

    // A slow paddle in the bottom of the tub.
    let hub = Vec2::new(0.2, FLOOR + 0.55);
    let paddle = w.add_body(Body::new(hub, 0.4, 3.0, rod_inertia(3.0, 0.45),
        BodyShape::Rod { half_len: 0.45, half_width: 0.06 }).colliding());
    w.add_constraint(PinWorld::new(paddle, Vec2::ZERO, hub));
    w.add_force(Motor::new(paddle, 25.0, 12.0));

    for (x, y, r, k, spin) in [
        (-1.9, 2.3, 0.38, 900.0, 0.0),
        (-0.9, 2.6, 0.30, 1200.0, 2.0),
        (0.3, 2.5, 0.42, 700.0, -1.0),
        (1.5, 2.4, 0.34, 1000.0, 0.0),
        (-1.4, 3.6, 0.33, 600.0, 0.0),
        (1.0, 3.7, 0.36, 800.0, 1.5),
    ] {
        blob(&mut w, Vec2::new(x, y), r, k, spin);
    }
    w
}
