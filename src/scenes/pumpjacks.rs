use glam::Vec2;

use crate::sim::{
    body::{rod_inertia, Body, BodyShape},
    constraints::{GearJoint, GearKind, PinJoint, PinWorld, Rope, SliderJoint, WeldJoint},
    forces::{Gravity, Motor},
    world::World,
};
use super::{circle_intersections, disk, rod_between};

// Three oil pumpjacks of different sizes, each its own crank-rocker: a
// motor turns the crank through a belt, the pitman arm rocks the walking
// beam on its A-frame, and the horsehead (a disk welded to the beam's nose)
// winds the bridle (a rope over it) up and down, lifting the heavy sucker
// rod through the wellhead on a locked slider. A counterweight welded to
// each crank stores energy on the down stroke and gives it back lifting
// the rod, so the motors only make up the difference. Slightly different
// sizes and speeds let the three drift in and out of step.

const GROUND: f32 = -1.4;

struct Jack { x: f32, s: f32, phase: f32, torque: f32 }

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let (ground, _) = rod_between(Vec2::new(-7.0, GROUND - 0.05), Vec2::new(7.5, GROUND - 0.05), 20.0, 0.05);
    w.add_body(ground.fixed().colliding());

    for jack in [
        Jack { x: -4.2, s: 0.85, phase: 0.3, torque: 7.0 },
        Jack { x: 0.0, s: 1.15, phase: 2.6, torque: 12.5 },
        Jack { x: 4.4, s: 1.0, phase: 4.6, torque: 11.0 },
    ] {
        pumpjack(&mut w, jack);
    }
    w
}

fn scenery(b: Body) -> Body { b.fixed().colliding() }

fn pumpjack(w: &mut World, Jack { x, s, phase, torque }: Jack) {
    let pivot = Vec2::new(x, GROUND + 2.4 * s);
    let (rear_arm, front_arm) = (1.3 * s, 1.5 * s);
    let crank_c = Vec2::new(x - rear_arm, GROUND + 0.55 * s);
    let (throw, pitman_len) = (0.4 * s, 1.85 * s);
    let head_r = 0.4 * s;

    // A-frame (Samson post).
    let foot = 0.45 * s;
    let (leg, leg_hl) = rod_between(Vec2::new(x - foot, GROUND), pivot, 4.0, 0.06 * s);
    let leg = w.add_body(scenery(leg));
    let (leg2, _) = rod_between(Vec2::new(x + foot, GROUND), pivot, 4.0, 0.06 * s);
    w.add_body(scenery(leg2));

    // Close the four-bar for this crank angle: the beam's rear end is
    // `rear_arm` from the pivot and `pitman_len` from the crank pin.
    let pin = crank_c + Vec2::new(phase.cos(), phase.sin()) * throw;
    let (r1, r2) = circle_intersections(pivot, rear_arm, pin, pitman_len).expect("pumpjack closes");
    let rear = if r1.x < r2.x { r1 } else { r2 }; // behind the pivot
    let dir = (pivot - rear).normalize();
    let nose = pivot + dir * front_arm;

    let beam_mass = 2.0 * s;
    let (beam_body, beam_hl) = rod_between(rear, nose, beam_mass, 0.075 * s);
    let beam = w.add_body(beam_body);
    w.add_constraint(PinJoint::new(leg, Vec2::new(leg_hl, 0.0), beam, Vec2::new(-beam_hl + rear_arm, 0.0)));

    let head = w.add_body(disk(nose, head_r, 1.2 * s));
    w.add_constraint(WeldJoint::new(beam, head, nose, &w.bodies));

    // Crank with its counterweight, turned by a belt from the motor pulley.
    let mut crank_body = disk(crank_c, 0.5 * s, 2.0 * s);
    crank_body.angle = phase as f64;
    let crank = w.add_body(crank_body);
    w.add_constraint(PinWorld::new(crank, Vec2::ZERO, crank_c));
    let weight_at = crank_c + Vec2::new(phase.cos(), phase.sin()) * 0.22 * s;
    let weight = w.add_body(disk(weight_at, 0.24 * s, 5.0 * s));
    w.add_constraint(WeldJoint::new(crank, weight, weight_at, &w.bodies));

    let (pitman_body, pitman_hl) = rod_between(pin, rear, 0.6 * s, 0.045 * s);
    let pitman = w.add_body(pitman_body);
    w.add_constraint(PinJoint::new(crank, Vec2::new(throw, 0.0), pitman, Vec2::new(-pitman_hl, 0.0)));
    w.add_constraint(PinJoint::new(pitman, Vec2::new(pitman_hl, 0.0), beam, Vec2::new(-beam_hl, 0.0)));

    let motor_c = Vec2::new(crank_c.x + 0.95 * s, GROUND + 0.25 * s);
    let motor = w.add_body(disk(motor_c, 0.14 * s, 0.6 * s));
    w.add_constraint(PinWorld::new(motor, Vec2::ZERO, motor_c));
    w.add_force(Motor::new(motor, torque * s, 1.6 * s));
    w.add_constraint(GearJoint::new(motor, crank, GearKind::Belt, &w.bodies).expect("disks"));

    // Wellhead: the polished rod rides a fixed guide straight down from the
    // horsehead's face; the bridle runs over the horsehead to its top.
    let well_x = nose.x + head_r;
    // Hung well below the horsehead, so its whole stroke clears it.
    let rod_top = Vec2::new(well_x, nose.y - head_r - 1.3 * s);
    let rod_len = 0.9 * s;
    let rod_mass = 2.5 * s;
    let sucker = w.add_body(Body::new(
        rod_top - Vec2::Y * rod_len * 0.5, std::f32::consts::FRAC_PI_2, rod_mass,
        rod_inertia(rod_mass, rod_len * 0.5), BodyShape::Rod { half_len: rod_len * 0.5, half_width: 0.035 * s },
    ));
    let (guide, _) = rod_between(Vec2::new(well_x, GROUND - 0.9 * s), Vec2::new(well_x, GROUND + 1.6 * s), 3.0, 0.05 * s);
    let guide = w.add_body(scenery(guide));
    let mut slide = SliderJoint::new(sucker, Vec2::ZERO, guide, &w.bodies).expect("rod rail");
    slide.lock_rotation = true;
    w.add_constraint(slide);

    let top = nose + Vec2::new(-0.02 * s, head_r + 0.03 * s);
    let local_top = crate::editor::world_to_local(&w.bodies[beam], top);
    let rope = Rope::new(beam, local_top, sucker, Vec2::new(rod_len * 0.5, 0.0), &w.bodies)
        .over(head, &w.bodies)
        .expect("bridle over the horsehead");
    let mut rope = rope;
    // The bridle is fixed to the horsehead: nothing slides on its rim.
    rope.grip = false;
    w.add_constraint(rope);
}
