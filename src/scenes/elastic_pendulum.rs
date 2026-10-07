use glam::Vec2;

use crate::sim::{forces::{Gravity, SpringDamper}, world::World};
use super::{anchor, disk, from_down};

// A mass on a spring, free to swing. With the spring's bounce frequency
// close to twice the swing frequency (√(k/m) ≈ 2·√(g/L)) the two modes
// resonate and energy sloshes between bouncing and swinging, tracing a
// rosette that never quite repeats.

const ANCHOR: Vec2 = Vec2::new(0.0, 2.6);
const MASS:   f32  = 1.0;
const K:      f32  = 25.0;
const REST:   f32  = 1.5;

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let top = w.add_body(anchor(ANCHOR));
    // Start swung out and a little stretched.
    let bob = w.add_body(disk(ANCHOR + from_down(0.9) * 2.15, 0.28, MASS));
    w.add_force(SpringDamper::new(top, Vec2::ZERO, bob, Vec2::ZERO, REST, K, 0.0));

    w.add_tracer(bob, Vec2::ZERO);
    w
}
