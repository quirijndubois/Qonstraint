use std::f32::consts::{FRAC_PI_2, TAU};
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{GearJoint, GearKind, PinJoint, PinWorld, SliderJoint},
    forces::{spring::SpringDamper, Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// A stamping line. A motor spins the sun gear of a planetary gearbox:
// three planets on a carrier mesh with the sun and with a fixed internal
// ring, so the carrier turns 1 + 42/18 = 3.33 times slower and stronger. A
// crank on the carrier drives the ram up and down its guide, and the
// punch rides in the ram on a spring, so it presses each block it meets
// with a bounded force instead of crushing it. Underneath, a conveyor belt
// (a colliding belt) carries blocks through the press and drops them into
// a bin.

/// Tooth counts at the drawing's tooth pitch: Z_ring = Z_sun + 2·Z_planet,
/// and (Z_sun + Z_ring) / 3 is whole, so three planets fit evenly.
const PITCH: f32 = 0.11;
const Z_SUN: f32 = 18.0;
const Z_PLANET: f32 = 12.0;
const Z_RING: f32 = 42.0;
const CENTRE: Vec2 = Vec2::new(0.0, 1.6);
const THROW: f32 = 0.38;
const CONROD: f32 = 1.45;
const BELT_Y: f32 = -1.515;
const PULLEY_R: f32 = 0.2;

fn pitch_r(z: f32) -> f32 { z * PITCH / TAU }

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let (rs, rp, rr) = (pitch_r(Z_SUN), pitch_r(Z_PLANET), pitch_r(Z_RING));
    let ring = w.add_body(disk(CENTRE, rr, 20.0).fixed());
    let carrier = w.add_body(disk(CENTRE, 0.62, 3.0));
    w.add_constraint(PinWorld::new(carrier, Vec2::ZERO, CENTRE));
    let orbit = rs + rp;
    let planets: Vec<usize> = (0..3).map(|k| {
        let a = TAU * k as f32 / 3.0;
        let p = CENTRE + Vec2::new(a.cos(), a.sin()) * orbit;
        let planet = w.add_body(disk(p, rp, 0.5));
        w.add_constraint(PinJoint::new(carrier, p - CENTRE, planet, Vec2::ZERO));
        planet
    }).collect();
    let sun = w.add_body(disk(CENTRE, rs, 0.8));
    w.add_constraint(PinWorld::new(sun, Vec2::ZERO, CENTRE));
    w.add_force(Motor::new(sun, 7.0, 0.6));
    // Sun meshes first so the drawing phases the ring from the planets.
    for &p in &planets {
        w.add_constraint(GearJoint::new(sun, p, GearKind::Mesh, &w.bodies).expect("disks"));
    }
    for &p in &planets {
        w.add_constraint(GearJoint::new(ring, p, GearKind::Belt, &w.bodies).expect("disks"));
    }

    // Crank pin between two planets, connecting rod down to the ram.
    let crank_a = FRAC_PI_2 + TAU / 6.0;
    let pin_local = Vec2::new(crank_a.cos(), crank_a.sin()) * THROW;
    let pin = CENTRE + pin_local;
    let ram_x = 0.0;
    let ram_top = Vec2::new(ram_x, pin.y - (CONROD * CONROD - (pin.x - ram_x).powi(2)).sqrt());
    let (ram_hl, ram_hw) = (0.35, 0.14);
    let ram = w.add_body(Body::new(
        ram_top - Vec2::Y * ram_hl, FRAC_PI_2, 3.0, rod_inertia(3.0, ram_hl),
        BodyShape::Rod { half_len: ram_hl, half_width: ram_hw },
    ));
    let (conrod, hl) = rod_between(pin, ram_top, 0.6, 0.05);
    let conrod = w.add_body(conrod);
    w.add_constraint(PinJoint::new(carrier, pin_local, conrod, Vec2::new(-hl, 0.0)));
    w.add_constraint(PinJoint::new(conrod, Vec2::new(hl, 0.0), ram, Vec2::new(ram_hl, 0.0)));
    let (guide, _) = rod_between(Vec2::new(ram_x, -1.0), Vec2::new(ram_x, 0.6), 5.0, 0.06);
    // Not colliding: the punch hangs from the ram along the guide's own axis.
    let guide = w.add_body(guide.fixed());
    let mut slide = SliderJoint::new(ram, Vec2::ZERO, guide, &w.bodies).expect("rod rail");
    slide.lock_rotation = true;
    w.add_constraint(slide);

    // The punch: a short bar sliding in the ram, sprung downwards.
    let (punch_hl, rider) = (0.3, 0.2);
    let ram_c = w.bodies[ram].pos32();
    let ram_travel = SliderJoint::travel(&w.bodies[ram]);
    let rider_at = ram_c - Vec2::Y * ram_travel;
    let mut punch = Body::new(
        rider_at - Vec2::Y * rider, FRAC_PI_2, 0.4, rod_inertia(0.4, punch_hl),
        BodyShape::Rod { half_len: punch_hl, half_width: 0.05 },
    ).colliding();
    punch.friction = 0.6;
    let punch = w.add_body(punch);
    let mut ride = SliderJoint::new(punch, Vec2::new(rider, 0.0), ram, &w.bodies).expect("rod rail");
    ride.lock_rotation = true;
    w.add_constraint(ride);
    w.add_force(SpringDamper::new(ram, Vec2::new(ram_hl - 0.05, 0.0), punch, Vec2::new(punch_hl, 0.0), 0.5, 150.0, 3.0));

    // Conveyor and the blocks it carries.
    let pulley = |w: &mut World, x: f32| {
        let p = w.add_body(disk(Vec2::new(x, BELT_Y), PULLEY_R, 2.0));
        w.add_constraint(PinWorld::new(p, Vec2::ZERO, Vec2::new(x, BELT_Y)));
        w.bodies[p].friction = 0.9;
        p
    };
    let (left, right) = (pulley(&mut w, -2.7), pulley(&mut w, 2.6));
    let mut belt = GearJoint::new(left, right, GearKind::Belt, &w.bodies).expect("disks");
    belt.collide = true;
    w.add_constraint(belt);
    w.add_force(Motor::new(left, -4.0, 3.3));
    let top = BELT_Y + PULLEY_R + crate::sim::constraints::gear::BELT_WIDTH as f32;
    for k in 0..9 {
        let x = -2.4 + 0.55 * k as f32;
        let mut b = Body::new(Vec2::new(x, top + 0.072), 0.0, 0.5, rod_inertia(0.5, 0.15),
            BodyShape::Rod { half_len: 0.15, half_width: 0.07 }).colliding();
        b.friction = 0.8;
        b.restitution = 0.1;
        w.add_body(b);
    }

    // Bin under the end of the belt (its near wall tucked under the pulley,
    // so nothing can wedge between them).
    for (a, b) in [
        (Vec2::new(2.35, -2.05), Vec2::new(2.35, -2.9)),
        (Vec2::new(2.35, -2.9), Vec2::new(4.1, -2.9)),
        (Vec2::new(4.1, -2.9), Vec2::new(4.1, -1.9)),
    ] {
        let (wall, _) = rod_between(a, b, 5.0, 0.05);
        let mut wall = wall.fixed().colliding();
        wall.friction = 0.6;
        wall.restitution = 0.1;
        w.add_body(wall);
    }
    w
}
