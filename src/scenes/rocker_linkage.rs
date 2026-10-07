use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, PinWorld, RollingOnRod},
    forces::{Gravity, SpringDamper},
    world::World,
};
use super::{anchor, disk, rod_between};

// A lever pivoted at its right end on a floor mount and held up at its left
// end by a spring. A heavy disk rolls on top of the lever; a coupler links
// the disk's centre to one end of a rocker that pivots at its middle from a
// ceiling mount, with a small disk on the rocker's far end.
//
// 15 body coordinates − 12 constraint equations = 3 DOF: lever angle,
// rocker angle, and the small disk's free spin.

const G: f32 = 9.81;

const LEVER_PIVOT: Vec2 = Vec2::new(0.0, 0.5);
const LEVER_LEN:   f32  = 4.0;
const LEVER_DROP:  f32  = 0.15; // left end sits this much lower
const LEVER_HW:    f32  = 0.07;
const LEVER_MASS:  f32  = 1.5;

const DISK_R:      f32  = 0.75;
const DISK_MASS:   f32  = 3.0;
const DISK_ALONG:  f32  = 1.6; // disk centre's distance from lever's left end

const ROCKER_PIVOT: Vec2 = Vec2::new(2.0, 2.0);
const ROCKER_ANGLE: f32  = -0.44;
const ROCKER_HL:    f32  = 1.1;

const SPRING_FLOOR_Y: f32 = -1.0;
const SPRING_K:       f32 = 60.0;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(G));

    // Lever: left end → pivot
    let left = LEVER_PIVOT - Vec2::new(LEVER_LEN, LEVER_DROP).normalize() * LEVER_LEN;
    let (lever_body, lever_hl) = rod_between(left, LEVER_PIVOT, LEVER_MASS, LEVER_HW);
    let lever_u = (LEVER_PIVOT - left).normalize();
    let lever = w.add_body(lever_body);

    // Big disk resting on the lever's top face
    let disk_pos = left + lever_u * DISK_ALONG + lever_u.perp() * (DISK_R + LEVER_HW);
    let big = w.add_body(disk(disk_pos, DISK_R, DISK_MASS));

    // Rocker, pivoting at its middle
    let ru = Vec2::new(ROCKER_ANGLE.cos(), ROCKER_ANGLE.sin());
    let apex = ROCKER_PIVOT - ru * ROCKER_HL;
    let tip  = ROCKER_PIVOT + ru * ROCKER_HL;

    // Coupler from disk centre to the rocker's apex
    let (coupler_body, coupler_hl) = rod_between(disk_pos, apex, 1.2, 0.06);
    let coupler = w.add_body(coupler_body);
    let (rocker_body, _) = rod_between(apex, tip, 0.8, 0.06);
    let rocker = w.add_body(rocker_body);
    let small = w.add_body(disk(tip, 0.32, 1.2));

    let floor = w.add_body(anchor(Vec2::new(left.x, SPRING_FLOOR_Y)));

    w.add_constraint(PinWorld::new(lever, Vec2::new(lever_hl, 0.0), LEVER_PIVOT));
    w.add_constraint(RollingOnRod::new(big, lever, &w.bodies).expect("disk on rod"));
    w.add_constraint(PinJoint::new(big, Vec2::ZERO, coupler, Vec2::new(-coupler_hl, 0.0)));
    w.add_constraint(PinJoint::new(coupler, Vec2::new(coupler_hl, 0.0), rocker, Vec2::new(-ROCKER_HL, 0.0)));
    w.add_constraint(PinWorld::new(rocker, Vec2::ZERO, ROCKER_PIVOT));
    w.add_constraint(PinJoint::new(rocker, Vec2::new(ROCKER_HL, 0.0), small, Vec2::ZERO));

    // Spring sized so it roughly carries the lever and disk at this pose:
    // torque about the pivot / lever arm = force the spring must push with.
    let torque = G * (LEVER_MASS * LEVER_LEN * 0.5 + DISK_MASS * (LEVER_PIVOT.x - disk_pos.x));
    let push = torque / LEVER_LEN;
    let len = left.y - SPRING_FLOOR_Y;
    w.add_force(SpringDamper::new(
        lever, Vec2::new(-lever_hl, 0.0), floor, Vec2::ZERO,
        len + push / SPRING_K, SPRING_K, 0.4,
    ));

    // The pose is off balance, so gravity starts it moving. Only the small
    // disk gets an initial spin: it turns freely on its pin, so that velocity
    // agrees with every constraint (a kick to the rocker would not).
    w.bodies[small].ang_vel = 4.0;
    w.add_tracer(small, Vec2::new(0.32 * 0.55, 0.0));
    w
}
