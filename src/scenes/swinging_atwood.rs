use glam::Vec2;

use crate::sim::{
    constraints::{PinWorld, Rope},
    forces::Gravity,
    world::World,
};
use super::disk;

// Swinging Atwood's machine: one rope over a pulley, a counterweight
// hanging straight down on one side, and a light mass swinging like a
// pendulum on the other, whose rope length changes as the counterweight
// rises and falls. Its swing flings rope out against the heavier
// counterweight. With mass ratio μ = M/m = 1.8 and the small mass released
// at rest level with the pulley, the motion is chaotic: a looping path that
// never repeats, and stays clear of the pulley (most other ratios
// eventually reel the small mass in).

const PULLEY: Vec2 = Vec2::new(0.0, 1.6);
const R: f32 = 0.12;
const M_SMALL: f32 = 1.0;
const MU: f32 = 1.8;
const START: f32 = 1.2; // small mass's distance from the pulley

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));
    let pulley = w.add_body(disk(PULLEY, R, 0.2));
    w.add_constraint(PinWorld::new(pulley, Vec2::ZERO, PULLEY));

    let small = w.add_body(disk(PULLEY + Vec2::new(-START, R), 0.11, M_SMALL));
    let weight = w.add_body(disk(PULLEY + Vec2::new(R, -1.0), 0.2, M_SMALL * MU));
    let rope = Rope::new(small, Vec2::ZERO, weight, Vec2::ZERO, &w.bodies)
        .over(pulley, &w.bodies).expect("pulley");
    w.add_constraint(rope);

    let t = w.tracers.len();
    w.add_tracer(small, Vec2::ZERO);
    w.tracers[t].seconds = 8.0;
    w
}

