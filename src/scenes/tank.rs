use std::f32::consts::TAU;
use glam::Vec2;

use crate::sim::{
    body::Body,
    constraints::PinJoint,
    forces::{Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// A tracked vehicle. The track is a closed loop of short links pinned end
// to end, laid snugly round the drive sprocket, the idler and four road
// wheels; the sprocket (driven by a motor mounted on the hull) pulls it
// round by friction, and the bottom run grips the ground. It climbs over
// two logs. The hull sits in its own collision plane so it never
// rubs on its wheels or track. The camera follows the hull.

const LINK: f32 = 0.19;          // target link length
const LINK_HW: f32 = 0.035;
const HULL_Y: f32 = 0.95;

/// Points round the convex hull of the circles, `step` apart along it.
fn band(circles: &[(Vec2, f32)], step: f32) -> Vec<Vec2> {
    // Dense samples on every circle, then their convex hull (monotone chain).
    let mut pts: Vec<Vec2> = circles.iter().flat_map(|&(c, r)| {
        (0..180).map(move |k| {
            let a = TAU * k as f32 / 180.0;
            c + Vec2::new(a.cos(), a.sin()) * r
        })
    }).collect();
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let cross = |o: Vec2, a: Vec2, b: Vec2| (a - o).perp_dot(b - o);
    let mut hull: Vec<Vec2> = Vec::new();
    for pass in 0..2 {
        let start = hull.len();
        let iter: Box<dyn Iterator<Item = &Vec2>> = if pass == 0 { Box::new(pts.iter()) } else { Box::new(pts.iter().rev()) };
        for &p in iter {
            while hull.len() >= start + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    // Resample by arc length into whole links.
    let n = hull.len();
    let total: f32 = (0..n).map(|k| hull[k].distance(hull[(k + 1) % n])).sum();
    let count = (total / step).round() as usize;
    let each = total / count as f32;
    let mut out = Vec::with_capacity(count);
    let (mut k, mut into) = (0usize, 0.0f32);
    for i in 0..count {
        let want = i as f32 * each;
        let mut walked = into;
        while walked + hull[k].distance(hull[(k + 1) % n]) < want {
            walked += hull[k].distance(hull[(k + 1) % n]);
            k += 1;
        }
        into = walked;
        let (a, b) = (hull[k], hull[(k + 1) % n]);
        out.push(a + (b - a).normalize_or_zero() * (want - walked));
    }
    out
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    // Ground with two logs.
    let mut solid = |b: Body| {
        let mut b = b.fixed().colliding();
        b.friction = 1.0;
        b.restitution = 0.05;
        w.add_body(b);
    };
    let (g1, _) = rod_between(Vec2::new(-8.0, -0.06), Vec2::new(80.0, -0.06), 50.0, 0.06);
    solid(g1);
    solid(disk(Vec2::new(4.0, 0.06), 0.2, 10.0));
    solid(disk(Vec2::new(9.5, 0.06), 0.23, 10.0));

    // Hull, in a plane of its own.
    let (hull, _) = rod_between(Vec2::new(-1.45, HULL_Y), Vec2::new(1.45, HULL_Y), 40.0, 0.16);
    let mut hull = hull.colliding();
    hull.plane = 2;
    let hull = w.add_body(hull);
    let hc = w.bodies[hull].pos32();
    let (turret, _) = rod_between(Vec2::new(-0.6, HULL_Y + 0.32), Vec2::new(0.5, HULL_Y + 0.32), 12.0, 0.18);
    let turret = w.add_body(turret);
    w.add_constraint(crate::sim::constraints::WeldJoint::new(hull, turret, Vec2::new(0.0, HULL_Y + 0.2), &w.bodies));
    let (barrel, _) = rod_between(Vec2::new(0.45, HULL_Y + 0.38), Vec2::new(1.9, HULL_Y + 0.42), 3.0, 0.045);
    let barrel = w.add_body(barrel);
    w.add_constraint(crate::sim::constraints::WeldJoint::new(turret, barrel, Vec2::new(0.45, HULL_Y + 0.38), &w.bodies));

    // Wheels, pinned to the hull.
    let ground_gap = 2.0 * LINK_HW + 0.01;
    let mut circles = Vec::new();
    let mut wheel = |w: &mut World, c: Vec2, r: f32, mass: f32| {
        let mut b = disk(c, r, mass).colliding();
        b.friction = 1.2;
        b.restitution = 0.0;
        b.plane = 1;
        let id = w.add_body(b);
        w.add_constraint(PinJoint::new(hull, c - hc, id, Vec2::ZERO));
        circles.push((c, r + LINK_HW + 0.01));
        id
    };
    let sprocket = wheel(&mut w, Vec2::new(-1.3, 0.62), 0.3, 5.0);
    wheel(&mut w, Vec2::new(1.3, 0.62), 0.3, 4.0);
    for x in [-0.78, -0.26, 0.26, 0.78] {
        wheel(&mut w, Vec2::new(x, ground_gap + 0.22), 0.22, 3.0);
    }
    w.add_force(Motor::new(sprocket, -110.0, 22.0).on(hull));

    // The track: links pinned end to end round the wheels.
    let pts = band(&circles, LINK);
    let n = pts.len();
    let links: Vec<(usize, f32)> = (0..n).map(|k| {
        let (a, b) = (pts[k], pts[(k + 1) % n]);
        let (body, hl) = rod_between(a, b, 0.9, LINK_HW);
        let mut body = body.colliding();
        body.friction = 1.0;
        body.restitution = 0.0;
        body.plane = 1;
        (w.add_body(body), hl)
    }).collect();
    for k in 0..n {
        let (a, ha) = links[k];
        let (b, hb) = links[(k + 1) % n];
        w.add_constraint(PinJoint::new(a, Vec2::new(ha, 0.0), b, Vec2::new(-hb, 0.0)));
    }
    w.follow = Some(hull);
    w
}
