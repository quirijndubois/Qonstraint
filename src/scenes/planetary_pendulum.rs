use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, RollingContact},
    forces::Gravity,
    world::World,
};
use super::{disk, rod_between};

// A planet disk rolls without slipping around a fixed sun, while a
// pendulum hangs from a point on the planet's rim. Rolling ties the
// planet's spin to its orbit (twice per lap for equal radii); the pendulum
// couples to both, and the traced tip draws something between an
// epicycloid and chaos.

const SUN:      Vec2 = Vec2::new(0.0, 1.0);
const SUN_R:    f32  = 0.8;
const PLANET_R: f32  = 0.4;
const START:    f32  = 0.6; // planet's angle from the top of the sun (rad)
const ARM:      f32  = 1.1; // pendulum length

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let sun = w.add_body(disk(SUN, SUN_R, 5.0).fixed());
    let planet_pos = SUN + Vec2::new(START.sin(), START.cos()) * (SUN_R + PLANET_R);
    let planet = w.add_body(disk(planet_pos, PLANET_R, 1.0));
    w.add_constraint(RollingContact::new(sun, planet, &w.bodies).expect("two disks"));

    // Pendulum hangs from a point 70% out on the planet.
    let hinge_local = Vec2::new(PLANET_R * 0.7, 0.0);
    let hinge = planet_pos + hinge_local;
    let (arm_body, hl) = rod_between(hinge, hinge - Vec2::Y * ARM, 0.5, 0.055);
    let arm = w.add_body(arm_body);
    w.add_constraint(PinJoint::new(planet, hinge_local, arm, Vec2::new(-hl, 0.0)));

    w.add_tracer(arm, Vec2::new(hl, 0.0));
    w
}
