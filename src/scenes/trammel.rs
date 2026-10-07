use glam::Vec2;

use crate::sim::{
    constraints::{PinJoint, SliderJoint},
    forces::Gravity,
    world::World,
};
use super::{disk, rod_between};

// Trammel of Archimedes: a bar slides with one point on a horizontal rail
// and another on a vertical rail. Every point of the bar moves on an
// ellipse; the weighted tip, a distance E past the second slider, traces
// one with semi-axes E (across) and L + E (down). Released near the top,
// it swings to and fro along the lower half.

const CENTRE: Vec2 = Vec2::new(0.0, 1.4);
const RAIL_HL: f32 = 1.65;
const L:       f32 = 1.2; // slider spacing along the bar
const E:       f32 = 0.6; // tip overhang past the vertical-rail slider
const START:   f32 = 0.3; // bar angle below horizontal (rad)

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let (h_body, _) = rod_between(CENTRE - Vec2::X * RAIL_HL, CENTRE + Vec2::X * RAIL_HL, 3.0, 0.07);
    let h_rail = w.add_body(h_body.fixed());
    let (v_body, _) = rod_between(CENTRE - Vec2::Y * RAIL_HL, CENTRE + Vec2::Y * RAIL_HL, 3.0, 0.07);
    let v_rail = w.add_body(v_body.fixed());

    let a = CENTRE + Vec2::X * L * START.cos();
    let b = CENTRE - Vec2::Y * L * START.sin();
    let tip = a + (b - a) * ((L + E) / L);
    let (bar_body, hl) = rod_between(a, tip, 1.0, 0.06);
    let bar = w.add_body(bar_body);
    w.add_constraint(SliderJoint::new(bar, Vec2::new(-hl, 0.0), h_rail, &w.bodies).expect("rail"));
    w.add_constraint(SliderJoint::new(bar, Vec2::new(-hl + L, 0.0), v_rail, &w.bodies).expect("rail"));

    let weight = w.add_body(disk(tip, 0.2, 1.5));
    w.add_constraint(PinJoint::new(bar, Vec2::new(hl, 0.0), weight, Vec2::ZERO));

    w.add_tracer(weight, Vec2::ZERO);
    w
}
