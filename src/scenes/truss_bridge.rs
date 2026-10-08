use glam::Vec2;

use crate::sim::{
    body::Body,
    constraints::{PinJoint, PinWorld},
    forces::{Gravity, Motor},
    world::World,
};
use super::{disk, rod_between};

// A Warren deck truss with breakable joints, and a heavy cart driving
// across. Every member is a plain rod; at each node the members are
// pinned to each other in a chain, every pin breaking at BREAK newtons.
// The cart's weight runs through the truss as tension and compression
// (FORCES shows it); as it nears midspan the chord joints there go over
// their limit, let go one after another, and the bridge folds into the
// river. Members and cart all collide, so the wreck piles up on the bed.

const PANELS: usize = 8;
const PANEL: f32 = 1.0;
const DEPTH: f32 = 0.9;
const DECK: f32 = 0.0;          // top chord height
const BREAK: f32 = 1400.0;      // N
const RIVER: f32 = -1.35;

fn part(mut b: Body, friction: f32) -> Body {
    b.collide = true;
    b.friction = friction;
    b.restitution = 0.1;
    b
}

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let half = 0.5 * PANELS as f32 * PANEL;
    // Approach roads, banks and the river bed.
    let ground = |w: &mut World, a: Vec2, b: Vec2| {
        let (g, _) = rod_between(a, b, 50.0, 0.06);
        w.add_body(part(g.fixed(), 0.9));
    };
    // Road surfaces flush with the top of the deck chord.
    let road_y = DECK + 0.045 - 0.06;
    ground(&mut w, Vec2::new(-half - 6.0, road_y), Vec2::new(-half - 0.12, road_y));
    ground(&mut w, Vec2::new(half + 0.12, road_y), Vec2::new(half + 6.0, road_y));
    ground(&mut w, Vec2::new(-half - 0.1, road_y - 0.1), Vec2::new(-half + 0.6, RIVER));
    ground(&mut w, Vec2::new(half + 0.1, road_y - 0.1), Vec2::new(half - 0.6, RIVER));
    ground(&mut w, Vec2::new(-half - 1.0, RIVER), Vec2::new(half + 1.0, RIVER));
    // A buffer at the end of the far road.
    ground(&mut w, Vec2::new(half + 2.4, road_y), Vec2::new(half + 2.4, road_y + 0.7));

    // Nodes: top chord on the deck line, bottom chord offset half a panel.
    let top: Vec<Vec2> = (0..=PANELS).map(|i| Vec2::new(-half + i as f32 * PANEL, DECK)).collect();
    let bottom: Vec<Vec2> = (0..PANELS).map(|i| Vec2::new(-half + (i as f32 + 0.5) * PANEL, DECK - DEPTH)).collect();
    let mut members: Vec<(Vec2, Vec2)> = Vec::new();
    for i in 0..PANELS {
        members.push((top[i], top[i + 1]));
        members.push((top[i], bottom[i]));
        members.push((bottom[i], top[i + 1]));
        if i + 1 < PANELS { members.push((bottom[i], bottom[i + 1])); }
    }

    // Every member a rod; collect which member ends meet at each node.
    let mut at_node: Vec<(Vec2, Vec<(usize, Vec2)>)> = Vec::new();
    for &(a, b) in &members {
        let deck = a.y == DECK && b.y == DECK;
        let (rod, hl) = rod_between(a, b, if deck { 3.0 } else { 1.0 }, if deck { 0.045 } else { 0.035 });
        let id = w.add_body(part(rod, 0.9));
        for (p, local) in [(a, Vec2::new(-hl, 0.0)), (b, Vec2::new(hl, 0.0))] {
            match at_node.iter_mut().find(|(q, _)| q.distance(p) < 1e-4) {
                Some((_, ends)) => ends.push((id, local)),
                None => at_node.push((p, vec![(id, local)])),
            }
        }
    }
    // Pin each node's members in a chain, every pin breakable; the two end
    // nodes sit on the banks.
    for (p, ends) in &at_node {
        for pair in ends.windows(2) {
            w.add_constraint(PinJoint::new(pair[0].0, pair[0].1, pair[1].0, pair[1].1));
            w.break_last_at(BREAK);
        }
        if (p.x.abs() - half).abs() < 1e-4 && p.y == DECK {
            let (id, local) = ends[0];
            w.add_constraint(PinWorld::new(id, local, *p));
        }
    }

    // The cart: a heavy flatbed on two wheels, its motor mounted on it.
    let wheel_r = 0.22;
    let start = Vec2::new(-half - 2.2, road_y + 0.06 + wheel_r);
    let chassis_hl = 0.75;
    let (bed, _) = rod_between(start + Vec2::new(-chassis_hl, 0.25), start + Vec2::new(chassis_hl, 0.25), 32.0, 0.12);
    let bed = w.add_body(part(bed, 0.6));
    let mut rear = None;
    for dx in [-0.5, 0.5] {
        let c = start + Vec2::new(dx, 0.0);
        let wheel = w.add_body(part(disk(c, wheel_r, 4.0), 1.0));
        w.add_constraint(PinJoint::new(bed, Vec2::new(dx, -0.25), wheel, Vec2::ZERO));
        rear.get_or_insert(wheel);
    }
    w.add_force(Motor::new(rear.unwrap(), -80.0, 22.0).on(bed));
    w
}
