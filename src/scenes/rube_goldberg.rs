use std::f32::consts::FRAC_PI_2;
use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{GearJoint, GearKind, PinJoint, PinWorld, Rope, SliderJoint, WeldJoint},
    forces::Gravity,
    world::World,
};
use super::{disk, rod_between};

// A Rube Goldberg machine, one cause after another:
// a ball rolls down three ramps and into a row of dominoes; the last,
// biggest domino lands on the free end of a trapdoor whose latch is a
// breakable pin; the trapdoor swings away and drops the weight resting on
// it, which pulls a rope over a pulley; the rope lifts a gate and the balls
// queued behind it roll out into a basket, while the pulley turns a gear
// that waves a flag.

const LATCH_BREAK: f32 = 45.0;   // N

fn scenery(b: Body) -> Body {
    let mut b = b.fixed().colliding();
    b.friction = 0.6;
    b.restitution = 0.1;
    b
}

fn ball(at: Vec2, r: f32, mass: f32) -> Body {
    let mut b = disk(at, r, mass).colliding();
    b.friction = 0.6;
    b.restitution = 0.2;
    b
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));
    let bar = |w: &mut World, a: Vec2, b: Vec2| {
        let (body, _) = rod_between(a, b, 10.0, 0.05);
        w.add_body(scenery(body))
    };

    // 1. Ramps, zigzagging down to the domino shelf.
    bar(&mut w, Vec2::new(-6.9, 3.3), Vec2::new(-4.2, 2.6));
    bar(&mut w, Vec2::new(-6.95, 3.25), Vec2::new(-6.95, 3.7));
    bar(&mut w, Vec2::new(-3.6, 2.1), Vec2::new(-6.6, 1.4));
    bar(&mut w, Vec2::new(-3.45, 2.05), Vec2::new(-3.45, 2.6));
    bar(&mut w, Vec2::new(-6.95, 0.8), Vec2::new(-3.0, 0.0));
    bar(&mut w, Vec2::new(-7.05, 0.75), Vec2::new(-7.05, 1.9));
    bar(&mut w, Vec2::new(-3.0, 0.0), Vec2::new(-0.65, 0.0));
    let start = w.add_body(ball(Vec2::new(-6.6, 3.3 + 0.05 + 0.13), 0.13, 0.6));
    w.add_tracer(start, Vec2::ZERO);

    // 2. Dominoes, each a bar welded to a foot, growing towards the end.
    let mut x = -2.55;
    for k in 0..5 {
        let h = 0.42 * 1.15f32.powi(k);
        let t = 0.16 * h;
        let mass = 0.3 * 1.3f32.powi(k);
        let foot_h = 0.012;
        let bar_body = Body::new(Vec2::new(x, 0.05 + 2.0 * foot_h + 0.5 * h), FRAC_PI_2, mass,
            rod_inertia(mass, 0.5 * h) + mass * t * t / 12.0, BodyShape::Rod { half_len: 0.5 * (h - t), half_width: 0.5 * t });
        let mut bar_body = bar_body.colliding();
        bar_body.friction = 0.5;
        bar_body.restitution = 0.05;
        let b = w.add_body(bar_body);
        let foot = Body::new(Vec2::new(x, 0.05 + foot_h), 0.0, 0.2 * mass, rod_inertia(0.2 * mass, 0.5 * t).max(2e-6),
            BodyShape::Rod { half_len: 0.5 * t, half_width: foot_h });
        let mut foot = foot.colliding();
        foot.friction = 0.7;
        let f = w.add_body(foot);
        w.add_constraint(WeldJoint::new(b, f, Vec2::new(x, 0.05 + foot_h), &w.bodies));
        x += 0.62 * h;
    }

    // 3. Trapdoor: hinged on the right, latched on the left by a breakable
    //    pin to a post; the weight rests on it near the hinge.
    let (hinge, latch) = (Vec2::new(0.8, 0.0), Vec2::new(-0.5, 0.0));
    let (door, dhl) = rod_between(latch, hinge, 1.5, 0.05);
    let mut door = door.colliding();
    door.friction = 0.6;
    let door = w.add_body(door);
    w.add_constraint(PinWorld::new(door, Vec2::new(dhl, 0.0), hinge));
    let (post, phl) = rod_between(Vec2::new(-0.5, -1.0), latch, 5.0, 0.05);
    // (Not colliding: once the latch lets go, the door must swing past it.)
    let post = w.add_body(post.fixed());
    w.add_constraint(PinJoint::new(post, Vec2::new(phl, 0.0), door, Vec2::new(-dhl, 0.0)));
    w.break_last_at(LATCH_BREAK);
    let weight_r = 0.2;
    let weight = w.add_body(ball(Vec2::new(0.45, 0.05 + weight_r), weight_r, 4.0));

    // 4. Pulley, gate and the queued balls; a gear with a flag on the pulley.
    let pulley_c = Vec2::new(0.45 + 0.25, 3.4);
    // Pulley and gear heavy enough to hold their own against what they
    // couple (a light wheel between heavy parts trips up XPBD's sweep).
    let pulley = w.add_body(disk(pulley_c, 0.25, 2.0));
    w.add_constraint(PinWorld::new(pulley, Vec2::ZERO, pulley_c));
    let gear_c = pulley_c + Vec2::new(0.25 + 0.4, 0.0);
    let gear = w.add_body(disk(gear_c, 0.4, 3.0));
    w.add_constraint(PinWorld::new(gear, Vec2::ZERO, gear_c));
    w.add_constraint(GearJoint::new(pulley, gear, GearKind::Mesh, &w.bodies).expect("disks"));
    let (flag, _) = rod_between(gear_c, gear_c + Vec2::new(0.0, 0.9), 0.1, 0.04);
    let flag = w.add_body(flag);
    w.add_constraint(WeldJoint::new(gear, flag, gear_c, &w.bodies));

    // The gate rides a short rail; closed, its foot sits just above the
    // ramp the balls wait on.
    let gate_x = pulley_c.x + 0.25;
    let ramp_y = 1.45;                     // ramp centre line under the gate
    let up_left = Vec2::new(-2.4, 0.65).normalize();
    let ramp_end = Vec2::new(gate_x + 0.15, ramp_y);
    bar(&mut w, ramp_end, ramp_end + up_left * 2.5);
    let gate_hl = 0.33;
    let gate_c = Vec2::new(gate_x, ramp_y + 0.05 + 0.04 + 0.05 + gate_hl);
    let (rail, _) = rod_between(Vec2::new(gate_x, gate_c.y), Vec2::new(gate_x, 2.95), 5.0, 0.04);
    let rail = w.add_body(scenery(rail));
    let mut gate = Body::new(gate_c, FRAC_PI_2, 1.0, rod_inertia(1.0, gate_hl),
        BodyShape::Rod { half_len: gate_hl, half_width: 0.05 }).colliding();
    gate.friction = 0.2;
    let gate = w.add_body(gate);
    let mut slide = SliderJoint::new(gate, Vec2::new(gate_hl, 0.0), rail, &w.bodies).expect("rod rail");
    slide.lock_rotation = true;
    w.add_constraint(slide);
    let rope = Rope::new(weight, Vec2::ZERO, gate, Vec2::new(gate_hl, 0.0), &w.bodies)
        .over(pulley, &w.bodies).expect("rope over the pulley");
    w.add_constraint(rope);

    // The queue of balls, and the chute down to the basket.
    let normal = up_left.perp() * -1.0;
    let normal = if normal.y < 0.0 { -normal } else { normal };
    for k in 0..5 {
        let along = 0.4 + 0.24 * k as f32;
        w.add_body(ball(ramp_end + up_left * along + normal * (0.05 + 0.11 + 0.003), 0.11, 0.3));
    }
    let chute = ramp_end + Vec2::new(0.15, -0.35);
    bar(&mut w, chute, chute + Vec2::new(1.6, -0.85));
    let basket = chute + Vec2::new(2.0, -1.4);
    bar(&mut w, basket + Vec2::new(-0.3, 0.0), basket + Vec2::new(1.2, 0.0));
    bar(&mut w, basket + Vec2::new(1.2, 0.0), basket + Vec2::new(1.25, 0.7));
    bar(&mut w, basket + Vec2::new(-0.3, 0.0), basket + Vec2::new(-0.35, 0.35));

    // Somewhere for the weight and the door to end up.
    bar(&mut w, Vec2::new(-1.2, -3.0), Vec2::new(1.6, -3.0));
    w
}
