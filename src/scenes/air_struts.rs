use glam::Vec2;

use crate::sim::{
    constraints::{Cylinder, PinJoint, PinWorld},
    forces::Gravity,
    world::World,
};
use super::{disk, from_down, rod_between};

// A heavy hub stands on two splayed air struts: Cylinders in SEALED mode,
// i.e. adiabatic air springs. Each barrel stands on a floor pin by its head
// and can lean; its piston rod is pinned to the hub. A pendulum hangs from
// the hub. Released off-centre with the struts at rest length, the hub
// drops onto the air, bounces and sways, and the pendulum turns it chaotic.
// Air springs are lossless, so it never settles.

const FLOOR_Y:  f32 = -1.7;
const SPREAD:   f32 = 1.3;   // floor pins at ±SPREAD
const HUB:      Vec2 = Vec2::new(0.3, 0.7);
const BARREL:   f32 = 1.7;   // barrel length (long enough that the piston never reaches the open end)
const BORE:     f32 = 0.2;
const COLUMN:   f32 = 0.85;  // gas column at rest length
const CROWN:    f32 = 0.08;
const PEND:     f32 = 0.95;  // pendulum length

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let hub = w.add_body(disk(HUB, 0.3, 3.0));

    for side in [-1.0f32, 1.0] {
        let floor = Vec2::new(side * SPREAD, FLOOR_Y);
        let dir = (HUB - floor).normalize();
        // Barrel: head (its +x end) on the floor pin, open end towards the hub.
        let (barrel_body, bhl) = rod_between(floor + dir * BARREL, floor, 1.2, BORE);
        let barrel = w.add_body(barrel_body);
        w.add_constraint(PinWorld::new(barrel, Vec2::new(bhl, 0.0), floor));

        // Piston rod from the piston, COLUMN above the head, to the hub.
        let piston_at = floor + dir * (COLUMN + CROWN);
        let (rod_body, rhl) = rod_between(piston_at, HUB, 0.4, 0.035);
        let rod = w.add_body(rod_body);
        w.add_constraint(PinJoint::new(rod, Vec2::new(rhl, 0.0), hub, Vec2::ZERO));

        let mut cyl = Cylinder::new(rod, Vec2::new(-rhl, 0.0), barrel, &w.bodies).expect("rod barrel");
        cyl.crown = CROWN;
        w.add_constraint(cyl); // sealed: an air spring at rest here
    }

    let bob_at = HUB + from_down(0.7) * PEND;
    let (arm_body, ahl) = rod_between(HUB, bob_at, 0.3, 0.04);
    let arm = w.add_body(arm_body);
    w.add_constraint(PinJoint::new(hub, Vec2::ZERO, arm, Vec2::new(-ahl, 0.0)));
    let bob = w.add_body(disk(bob_at, 0.17, 1.0));
    w.add_constraint(PinJoint::new(arm, Vec2::new(ahl, 0.0), bob, Vec2::ZERO));

    w.add_tracer(bob, Vec2::ZERO);
    w
}
