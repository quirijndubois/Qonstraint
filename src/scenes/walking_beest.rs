use glam::Vec2;

use crate::sim::{
    body::Body,
    constraints::{GearJoint, GearKind, PinJoint},
    forces::{Gravity, Motor},
    world::World,
};
use super::{disk, rod_between, strandbeest::leg};

// The Strandbeest let loose: two sets of Jansen legs (four legs) on a free
// frame, their cranks half a turn apart and tied by a chain (a belt), the
// motor mounted on the frame so its drive is internal. Only the feet
// touch the ground, through friction, each leg in its own collision plane
// (side by side in depth, as on the real thing) so feet pass each other; the flat bottoms of the foot paths
// carry the frame along almost level while the other set lifts its feet.
// The camera follows the frame.

const S: f32 = 1.0 / 38.0;
const M: f32 = 15.0;              // crank radius (Jansen units)
const LINK_MASS: f32 = 0.4;
const LINK_HW: f32 = 0.035;
const FOOT_MASS: f32 = 1.0;
const SPACING: f32 = 1.5;         // between the two crank axles
const HEIGHT: f32 = 2.6;          // crank axles above the ground line (approx.)

pub fn build() -> World {
    let mut w = World::new();
    w.add_force(Gravity::new(9.81));

    let theta0 = 0.4f32;
    let cranks = [Vec2::new(-0.5 * SPACING, HEIGHT), Vec2::new(0.5 * SPACING, HEIGHT)];

    // Lowest foot over both sets' starting angles sets the ground.
    let angles = [theta0, theta0 + std::f32::consts::PI];
    let foot_y = |c: Vec2, th: f32| {
        [th, std::f32::consts::PI - th].iter().map(|&a| c.y + leg(a).g.y * S).fold(f32::INFINITY, f32::min)
    };
    let low = cranks.iter().zip(angles).map(|(&c, a)| foot_y(c, a)).fold(f32::INFINITY, f32::min);
    let ground_y = low - LINK_HW - 0.002;
    let (ground, _) = rod_between(Vec2::new(-20.0, ground_y - 0.06), Vec2::new(400.0, ground_y - 0.06), 100.0, 0.06);
    let mut ground = ground.fixed().colliding();
    ground.friction = 1.0;
    ground.restitution = 0.0;
    w.add_body(ground);

    let (frame, _) = rod_between(cranks[0] - Vec2::new(1.35, 0.0), cranks[1] + Vec2::new(1.35, 0.0), 6.0, 0.06);
    let frame = w.add_body(frame);
    let frame_c = w.bodies[frame].pos32();
    let on_frame = |p: Vec2| p - frame_c;

    let mut crank_ids = Vec::new();
    let mut plane = 0u8;
    for (&c, &theta) in cranks.iter().zip(&angles) {
        let mut crank_body = disk(c, M * S * 1.35, 3.0);
        crank_body.angle = theta as f64;
        let crank = w.add_body(crank_body);
        w.add_constraint(PinJoint::new(frame, on_frame(c), crank, Vec2::ZERO));
        crank_ids.push(crank);
        let pin_local = Vec2::new(M * S, 0.0);

        for mirror in [1.0f32, -1.0] {
            let lg = leg(if mirror > 0.0 { theta } else { std::f32::consts::PI - theta });
            let at = |v: Vec2| c + Vec2::new(v.x * mirror, v.y) * S;
            let rod = |w: &mut World, from: Vec2, to: Vec2| {
                let (b, hl) = rod_between(at(from), at(to), LINK_MASS, LINK_HW);
                (w.add_body(b), hl)
            };
            let (j, hj) = rod(&mut w, lg.a, lg.c);
            let (k, hk) = rod(&mut w, lg.a, lg.d);
            let (b, hb) = rod(&mut w, lg.p, lg.c);
            let (e, he) = rod(&mut w, lg.c, lg.e);
            let (d, hd) = rod(&mut w, lg.e, lg.p);
            let (cc, hc) = rod(&mut w, lg.p, lg.d);
            let (f, hf) = rod(&mut w, lg.e, lg.f);
            let (g, hg) = rod(&mut w, lg.d, lg.f);
            let (h, hh) = rod(&mut w, lg.f, lg.g);
            let (i, hi) = rod(&mut w, lg.g, lg.d);
            // The feet: the two bars meeting at G touch the ground.
            plane += 1;
            for foot in [h, i] {
                let body: &mut Body = &mut w.bodies[foot];
                // Heavier feet: contacts are as stiff as the bodies they touch.
                body.inertia *= FOOT_MASS / body.mass;
                body.mass = FOOT_MASS;
                body.collide = true;
                body.friction = 1.0;
                body.restitution = 0.0;
                body.plane = plane;
            }
            let start = |hl: f32| Vec2::new(-hl, 0.0);
            let end = |hl: f32| Vec2::new(hl, 0.0);

            w.add_constraint(PinJoint::new(crank, pin_local, j, start(hj)));
            w.add_constraint(PinJoint::new(crank, pin_local, k, start(hk)));
            // The fixed pivot P is on the frame now.
            w.add_constraint(PinJoint::new(frame, on_frame(at(lg.p)), b, start(hb)));
            w.add_constraint(PinJoint::new(frame, on_frame(at(lg.p)), cc, start(hc)));
            w.add_constraint(PinJoint::new(b, start(hb), d, end(hd)));
            w.add_constraint(PinJoint::new(b, end(hb), j, end(hj)));
            w.add_constraint(PinJoint::new(b, end(hb), e, start(he)));
            w.add_constraint(PinJoint::new(e, end(he), d, start(hd)));
            w.add_constraint(PinJoint::new(d, start(hd), f, start(hf)));
            w.add_constraint(PinJoint::new(cc, end(hc), k, end(hk)));
            w.add_constraint(PinJoint::new(cc, end(hc), g, start(hg)));
            w.add_constraint(PinJoint::new(cc, end(hc), i, end(hi)));
            w.add_constraint(PinJoint::new(g, end(hg), f, end(hf)));
            w.add_constraint(PinJoint::new(g, end(hg), h, start(hh)));
            w.add_constraint(PinJoint::new(h, end(hh), i, start(hi)));

            let t = w.tracers.len();
            w.add_tracer(h, end(hh));
            w.tracers[t].seconds = 2.0;
        }
    }
    // Chain between the cranks keeps them half a turn apart.
    w.add_constraint(GearJoint::new(crank_ids[0], crank_ids[1], GearKind::Belt, &w.bodies).expect("disks"));
    w.add_force(Motor::new(crank_ids[0], -30.0, 4.0).on(frame));
    w.follow = Some(frame);
    w
}
