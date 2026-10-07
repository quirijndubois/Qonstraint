use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::PinWorld,
    forces::{Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// Collisions with friction: a closed box with a motor-driven paddle that
// keeps flinging a handful of disks and bars around. Every part has
// COLLIDE on; the contact material (friction, bounce) is set in the editor.

const HALF_W: f32 = 2.4;
const HALF_H: f32 = 1.7;
const WALL: f32 = 0.08;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));
    w.contacts.friction = 0.45;
    w.contacts.restitution = 0.5;

    let corners = [
        Vec2::new(-HALF_W, -HALF_H), Vec2::new(HALF_W, -HALF_H),
        Vec2::new(HALF_W, HALF_H), Vec2::new(-HALF_W, HALF_H),
    ];
    for k in 0..4 {
        let (b, _) = rod_between(corners[k], corners[(k + 1) % 4], 5.0, WALL);
        w.add_body(b.fixed().colliding());
    }
    // A shelf on the left to bounce things around.
    let (shelf, _) = rod_between(Vec2::new(-HALF_W, 0.3), Vec2::new(-1.1, -0.25), 2.0, 0.06);
    w.add_body(shelf.fixed().colliding());

    // Paddle on a pin, driven round slowly.
    let hub = Vec2::new(0.6, -0.55);
    let paddle = w.add_body(Body::new(hub, 0.3, 2.0, rod_inertia(2.0, 0.95),
        BodyShape::Rod { half_len: 0.95, half_width: 0.07 }).colliding());
    w.add_constraint(PinWorld::new(paddle, Vec2::ZERO, hub));
    w.add_force(Motor::new(paddle, 14.0, 6.0));

    for (k, (x, y, r)) in [(-1.6, 1.2, 0.22), (-0.9, 1.3, 0.3), (-0.2, 1.1, 0.18), (0.5, 1.3, 0.26), (1.3, 1.2, 0.2), (1.9, 0.9, 0.16)]
        .into_iter().enumerate()
    {
        let mut d = disk(Vec2::new(x, y), r, 0.4 + 2.0 * r);
        d.ang_vel = if k % 2 == 0 { 2.0 } else { -2.0 };
        w.add_body(d.colliding());
    }
    for (x, y, a) in [(-1.6, 0.6, 0.4), (1.6, 0.3, -0.8), (-0.4, 0.5, 1.2)] {
        let b = Body::new(Vec2::new(x, y), a, 0.5, rod_inertia(0.5, 0.3), BodyShape::Rod { half_len: 0.3, half_width: 0.06 });
        w.add_body(b.colliding());
    }
    let t = w.tracers.len();
    w.add_tracer(7, Vec2::ZERO);
    w.tracers[t].seconds = 2.0;
    w
}
