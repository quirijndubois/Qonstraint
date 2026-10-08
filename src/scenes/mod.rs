use glam::Vec2;
use crate::renderer::geometry::GeometryBuilder;
use crate::sim::{
    body::{disk_inertia, rod_inertia, Body, BodyShape},
    constraints::{
        cylinder::{CylinderState, GasMode, Stroke}, slider::{COLLAR_HALF, STOP_W},
        Cylinder, DistanceConstraint, GearJoint, GearKind, PinJoint, PinWorld, Rope, SliderJoint,
    },
    forces::{spring::SpringDamper, Motor, TorsionSpring},
    world::World,
};

pub mod air_struts;
pub mod coupled_pendulums;
pub mod double_pendulum;
pub mod elastic_pendulum;
pub mod flexible_beam;
pub mod gear_train;
pub mod kapitza;
pub mod peaucellier;
pub mod planetary_pendulum;
pub mod radial_engine;
pub mod rocker_linkage;
pub mod rolling_crane;
pub mod sandbox;
pub mod strandbeest;
pub mod swinging_atwood;
pub mod trammel;
pub mod tumbler;
pub mod watt_linkage;
#[cfg(test)]
mod bench;

pub const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
/// Matches the grid shader background, so outlines and holes read as gaps.
pub const BG:    [f32; 4] = [0.018, 0.019, 0.023, 1.0];

pub struct SceneDef {
    pub name:          &'static str,
    pub description:   &'static str,
    pub build:         fn() -> World,
    pub camera_center: [f32; 2],
    /// World-space width × height that must be visible; the camera zoom is
    /// fitted to the window so the whole mechanism shows at any size.
    pub view_size:     [f32; 2],
}

pub static SCENES: &[SceneDef] = &[
    SceneDef {
        name:          "Rocker Linkage",
        description:   "Disk rolls on a sprung lever, coupled to a rocker",
        build:         rocker_linkage::build,
        camera_center: [-0.4, 0.75],
        view_size:     [9.5, 5.0],
    },
    SceneDef {
        name:          "Radial Engine",
        description:   "Five cylinders, four strokes, one crank pin: fires 1-3-5-2-4",
        build:         radial_engine::build,
        camera_center: [0.0, 0.0],
        view_size:     [4.6, 4.6],
    },
    SceneDef {
        name:          "Double Pendulum",
        description:   "Two links, one bob: textbook deterministic chaos",
        build:         double_pendulum::build,
        camera_center: [0.0, 2.0],
        view_size:     [7.0, 6.6],
    },
    SceneDef {
        name:          "Elastic Pendulum",
        description:   "Spring pendulum near 2:1 resonance, swing and bounce trade energy",
        build:         elastic_pendulum::build,
        camera_center: [0.0, 1.2],
        view_size:     [6.5, 4.8],
    },
    SceneDef {
        name:          "Coupled Pendulums",
        description:   "A weak spring passes the swing back and forth",
        build:         coupled_pendulums::build,
        camera_center: [0.0, 1.3],
        view_size:     [6.0, 4.4],
    },
    SceneDef {
        name:          "Rolling Crane",
        description:   "Wheel rolls under a rail, a double pendulum swings from its axle",
        build:         rolling_crane::build,
        camera_center: [0.0, 1.0],
        view_size:     [7.2, 4.6],
    },
    SceneDef {
        name:          "Planetary Pendulum",
        description:   "Disk rolls around a fixed disk, a pendulum swings from its rim",
        build:         planetary_pendulum::build,
        camera_center: [0.0, 1.0],
        view_size:     [6.0, 5.4],
    },
    SceneDef {
        name:          "Trammel",
        description:   "A bar slides on two crossed rails, its tip traces an ellipse",
        build:         trammel::build,
        camera_center: [0.0, 0.9],
        view_size:     [5.0, 5.2],
    },
    SceneDef {
        name:          "Air Struts",
        description:   "A hub bounces on two sealed air struts, a pendulum swings below",
        build:         air_struts::build,
        camera_center: [0.0, -0.4],
        view_size:     [5.0, 4.6],
    },
    SceneDef {
        name:          "Strandbeest",
        description:   "Jansen's walking leg, mirrored on one crank: the feet trace flat-bottomed paths",
        build:         strandbeest::build,
        camera_center: [0.0, -0.1],
        view_size:     [6.0, 4.6],
    },
    SceneDef {
        name:          "Peaucellier-Lipkin",
        description:   "A swinging crank and an inverting rhombus: P runs on an exact straight line",
        build:         peaucellier::build,
        camera_center: [0.0, 0.6],
        view_size:     [6.0, 4.4],
    },
    SceneDef {
        name:          "Watt's Linkage",
        description:   "Two arms and a coupler: the midpoint bobs on a near-straight line, figure eight at the ends",
        build:         watt_linkage::build,
        camera_center: [0.0, 0.0],
        view_size:     [5.6, 4.2],
    },
    SceneDef {
        name:          "Kapitza's Pendulum",
        description:   "Shaken fast enough from below, an upside-down pendulum stands up",
        build:         kapitza::build,
        camera_center: [0.0, -0.3],
        view_size:     [4.0, 3.8],
    },
    SceneDef {
        name:          "Gear Train",
        description:   "Motor, 3:1 gears, 1:2 belt, then a crank-rocker tracing its coupler curve",
        build:         gear_train::build,
        camera_center: [0.0, 0.4],
        view_size:     [6.8, 3.8],
    },
    SceneDef {
        name:          "Swinging Atwood",
        description:   "A rope over a pulley: counterweight on one end, a swinging mass on the other",
        build:         swinging_atwood::build,
        camera_center: [0.0, 0.5],
        view_size:     [5.0, 4.4],
    },
    SceneDef {
        name:          "Tumbler",
        description:   "Collisions with friction: a paddle stirs disks and bars in a box",
        build:         tumbler::build,
        camera_center: [0.0, 0.0],
        view_size:     [5.4, 3.9],
    },
    SceneDef {
        name:          "Flexible Beam",
        description:   "Rigid segments on torsion-sprung hinges: a cantilever that rings",
        build:         flexible_beam::build,
        camera_center: [0.0, 0.2],
        view_size:     [5.6, 3.8],
    },
    SceneDef {
        name:          "Untitled",
        description:   "Empty  ·  drag parts in from the editor panel",
        build:         sandbox::build,
        camera_center: [0.0, 0.0],
        view_size:     [10.0, 6.0],
    },
];

// ── Scene-building helpers ────────────────────────────────────────────────────

pub(crate) fn disk(pos: Vec2, radius: f32, mass: f32) -> Body {
    Body::new(pos, 0.0, mass, disk_inertia(mass, radius), BodyShape::Disk { radius })
}

/// Rod spanning world points `a` → `b`: local `(-half_len, 0)` is `a`,
/// `(half_len, 0)` is `b`. Returns the body and its half length.
pub(crate) fn rod_between(a: Vec2, b: Vec2, mass: f32, half_width: f32) -> (Body, f32) {
    let d = b - a;
    let hl = d.length() * 0.5;
    let body = Body::new(
        (a + b) * 0.5, d.y.atan2(d.x), mass, rod_inertia(mass, hl),
        BodyShape::Rod { half_len: hl, half_width },
    );
    (body, hl)
}

pub(crate) fn anchor(pos: Vec2) -> Body {
    Body::new(pos, 0.0, 1.0, 1.0, BodyShape::Point).fixed()
}

/// The two points at distance `r1` from `c1` and `r2` from `c2` (left and
/// right of the direction c1 → c2), if the circles meet.
pub(crate) fn circle_intersections(c1: Vec2, r1: f32, c2: Vec2, r2: f32) -> Option<(Vec2, Vec2)> {
    let d = c2 - c1;
    let l = d.length();
    if l < 1e-6 || l > r1 + r2 || l < (r1 - r2).abs() { return None; }
    let a = (r1 * r1 - r2 * r2 + l * l) / (2.0 * l);
    let h = (r1 * r1 - a * a).max(0.0).sqrt();
    let u = d / l;
    let m = c1 + u * a;
    Some((m + u.perp() * h, m - u.perp() * h))
}

/// Unit vector at angle `phi` measured from straight down (pendulum angle).
pub(crate) fn from_down(phi: f32) -> Vec2 {
    Vec2::new(phi.sin(), -phi.cos())
}

// ── Shared renderer ───────────────────────────────────────────────────────────
//
// Every scene draws through `draw_world`, so each element looks the same
// everywhere: flat white parts, a background-coloured outline that separates
// overlapping parts, and pin bosses at every connection point.

const OUTLINE:     f32 = 0.024;
const PIN_BOSS_R:  f32 = 0.085;
const PIN_HOLE_R:  f32 = 0.040;
const PIN_CORE_R:  f32 = 0.018;
const PED_HW:      f32 = 0.10;
const PED_H:       f32 = 0.30;
const GROUND_HW:   f32 = 0.42;
const GROUND_W:    f32 = 0.035;
const LINK_W:      f32 = 0.06;
const SPRING_W:    f32 = 0.034;
const SPRING_AMP:  f32 = 0.10;
const GROOVE_W:       f32 = 0.022;
const STOP_OVERHANG:  f32 = 0.05;
const CARRIAGE_CLEAR: f32 = 0.045;
const CARRIAGE_ROUND: f32 = 0.03;

/// `anchor_ref_y` decides mount direction: anchors above it hang from a
/// ceiling, the rest stand on the floor. The app updates it live while
/// editing and freezes it while simulating, so mounts never flip mid-run.
/// `tint` colours bodies by index (the force overlay); empty means white.
pub fn draw_world(world: &World, anchor_ref_y: Option<f32>, tint: &[[f32; 4]], geo: &mut GeometryBuilder) {
    let bodies = &world.bodies;
    let n = bodies.len();
    let fill = |i: usize| tint.get(i).copied().unwrap_or(WHITE);
    let gears = gear_phases(world);

    let ceiling = |p: Vec2| anchor_ref_y.is_some_and(|y| p.y > y);

    let mut pins: Vec<Vec2> = Vec::new();

    // 1. Springs (behind everything)
    for f in world.forces.iter().filter(|f| f.on) {
        if let Some(sd) = f.as_any().downcast_ref::<SpringDamper>() {
            if sd.body_a < n && sd.body_b < n {
                let pa = bodies[sd.body_a].world_point(sd.local_a);
                let pb = bodies[sd.body_b].world_point(sd.local_b);
                // Coil count is fixed by the rest length, so the spring
                // visibly compresses/stretches instead of gaining coils.
                let coils = (sd.rest_len / 0.34).clamp(3.0, 8.0).round() as u32;
                geo.draw_spring(pa, pb, coils, SPRING_AMP, SPRING_W, WHITE);
                pins.push(pa);
                pins.push(pb);
            }
        }
    }

    // 2. Anchors: world pins, Point bodies and any fixed body
    for c in world.constraints.iter().filter(|c| c.on) {
        if let Some(pw) = c.as_any().downcast_ref::<PinWorld>() {
            draw_pedestal(geo, pw.target, ceiling(pw.target));
            pins.push(pw.target);
        }
    }
    for b in bodies.iter().filter(|b| b.fixed || matches!(b.shape, BodyShape::Point)) {
        draw_pedestal(geo, b.pos32(), ceiling(b.pos32()));
        if matches!(b.shape, BodyShape::Point) {
            draw_pin_hole(geo, b.pos32());
        }
    }

    // 3. Engine cylinders: fins, barrel, head, gas, valves, plug
    for cy in cylinders(world) {
        draw_cylinder(geo, cy, &bodies[cy.barrel], cy.state(bodies));
    }

    // 4. Disks, toothed where they mesh
    for (i, b) in bodies.iter().enumerate() {
        if let BodyShape::Disk { radius } = b.shape {
            let color = fill(i);
            let body_r = match gears.get(i).copied().flatten() {
                Some(phase) => {
                    draw_teeth(geo, b.pos32(), radius, b.angle32() + phase, color);
                    radius - TOOTH_DED
                }
                None => radius,
            };
            geo.draw_circle(b.pos32(), body_r + OUTLINE, 64, BG);
            geo.draw_circle(b.pos32(), body_r, 64, color);
            let hub = (radius * 0.25).min(0.035);
            geo.draw_circle(b.pos32(), hub, 16, BG);
            // Off-centre dot so rotation is visible
            let (s, c) = b.angle32().sin_cos();
            let dot = b.pos32() + Vec2::new(c, s) * (radius * 0.55);
            geo.draw_circle(dot, (radius * 0.08).max(0.012), 12, BG);
        }
    }

    // Belts over their pulleys
    for c in world.constraints.iter().filter(|c| c.on) {
        if let Some(g) = c.as_any().downcast_ref::<GearJoint>() {
            if g.kind == GearKind::Belt && g.body_a < n && g.body_b < n {
                draw_belt(geo, &bodies[g.body_a], &bodies[g.body_b]);
            }
        }
    }

    // 5. Rods
    // Cylinder barrels are drawn as cylinders instead.
    let barrels: Vec<usize> = cylinders(world).map(|cy| cy.barrel).collect();
    for (i, b) in bodies.iter().enumerate() {
        if barrels.contains(&i) { continue; }
        if let BodyShape::Rod { half_len, half_width } = b.shape {
            let (s, c) = b.angle32().sin_cos();
            let d = Vec2::new(c, s) * half_len;
            let w = 2.0 * half_width;
            geo.draw_rod(b.pos32() - d, b.pos32() + d, w + 2.0 * OUTLINE, BG);
            geo.draw_rod(b.pos32() - d, b.pos32() + d, w, fill(i));
        }
    }

    // Ropes, over their pulleys
    for c in world.constraints.iter().filter(|c| c.on) {
        if let Some(r) = c.as_any().downcast_ref::<Rope>() {
            if r.body_a < n && r.body_b < n && r.pulley.is_none_or(|p| p < n) {
                draw_rope(geo, r, bodies);
                pins.push(bodies[r.body_a].world_point(r.local_a));
                pins.push(bodies[r.body_b].world_point(r.local_b));
            }
        }
    }

    // 6. Rails: a dark groove between two end stops, then the carriages
    let sliders = || world.constraints.iter().filter(|c| c.on)
        .filter_map(|c| c.as_any().downcast_ref::<SliderJoint>())
        .filter(|sj| sj.rider < n && sj.rail < n);
    for sj in sliders() {
        let rail = &bodies[sj.rail];
        let BodyShape::Rod { half_len, half_width } = rail.shape else { continue };
        let (s, c) = rail.angle32().sin_cos();
        let u = Vec2::new(c, s);
        let reach = SliderJoint::travel(rail) + COLLAR_HALF;
        let groove = (half_width * 0.45).min(GROOVE_W);
        geo.draw_line(rail.pos32() - u * reach, rail.pos32() + u * reach, groove, BG);
        for side in [-1.0, 1.0] {
            let at = rail.pos32() + u * side * (half_len - STOP_W * 0.5);
            let w = 2.0 * (half_width + STOP_OVERHANG);
            geo.draw_line(at - u * (STOP_W * 0.5 + OUTLINE), at + u * (STOP_W * 0.5 + OUTLINE), w + 2.0 * OUTLINE, BG);
            geo.draw_line(at - u * STOP_W * 0.5, at + u * STOP_W * 0.5, w, WHITE);
        }
    }
    for sj in sliders() {
        let rail = &bodies[sj.rail];
        let BodyShape::Rod { half_width, .. } = rail.shape else { continue };
        let (s, c) = rail.angle32().sin_cos();
        let u = Vec2::new(c, s);
        let p = bodies[sj.rider].world_point(sj.local);
        let hw = half_width + CARRIAGE_CLEAR;
        rounded_rect(geo, p, u, COLLAR_HALF + OUTLINE, hw + OUTLINE, CARRIAGE_ROUND + OUTLINE, BG);
        rounded_rect(geo, p, u, COLLAR_HALF, hw, CARRIAGE_ROUND, WHITE);
        // Bearing slots on the running faces
        let half = COLLAR_HALF - CARRIAGE_ROUND;
        let m = u.perp() * (half_width + CARRIAGE_CLEAR * 0.5);
        for side in [-1.0, 1.0] {
            geo.draw_line(p + m * side - u * half, p + m * side + u * half, 0.012, BG);
        }
        if sj.lock_rotation {
            // Bolted to the carriage: two bolt heads instead of a pivot.
            for side in [-1.0, 1.0] {
                let b = p + u * side * half * 0.75;
                geo.draw_circle(b, PIN_HOLE_R * 0.8, 16, BG);
                geo.draw_circle(b, PIN_CORE_R, 12, WHITE);
            }
        } else {
            pins.push(p);
        }
    }

    // 7. Pistons over their connecting rods
    for cy in cylinders(world) {
        draw_piston(geo, cy, bodies);
    }

    // 8. Massless links and joint locations
    for c in world.constraints.iter().filter(|c| c.on) {
        let c = c.as_any();
        if let Some(dc) = c.downcast_ref::<DistanceConstraint>() {
            if dc.body_a < n && dc.body_b < n {
                let pa = bodies[dc.body_a].world_point(dc.local_a);
                let pb = bodies[dc.body_b].world_point(dc.local_b);
                outlined_rod(geo, pa, pb, LINK_W);
                pins.push(pa);
                pins.push(pb);
            }
        } else if let Some(pj) = c.downcast_ref::<PinJoint>() {
            if pj.body_a < n && pj.body_b < n {
                pins.push(bodies[pj.body_a].world_point(pj.local_a));
            }
        }
    }

    // Torsion springs: a spiral that winds up with the twist
    for f in world.forces.iter().filter(|f| f.on) {
        if let Some(t) = f.as_any().downcast_ref::<TorsionSpring>() {
            if t.body_a < n && t.body_b < n {
                let p = bodies[t.body_a].world_point(t.local_a);
                draw_torsion(geo, p, bodies[t.body_a].angle32(), t.twist(bodies) as f32);
                pins.push(p);
            }
        }
    }

    // 9. Pins on top
    for p in pins {
        draw_pin(geo, p);
    }

    // 10. Switched-off connections: a faint dashed trace of what they link
    let off_constraints = world.constraints.iter().filter(|c| !c.on).map(|c| {
        let any = c.as_any();
        let end = any.downcast_ref::<PinWorld>().map(|pw| pw.target);
        (c.body_indices(), end)
    });
    let off_forces = world.forces.iter().filter(|f| !f.on).map(|f| (f.body_indices(), None));
    for (ids, end) in off_constraints.chain(off_forces) {
        draw_switched_off(geo, bodies, &ids, end);
    }

    // 11. Motors: a turning arrow round the driven body's centre
    let motors = world.forces.iter().filter(|f| f.on).filter_map(|f| f.as_any().downcast_ref::<Motor>());
    for m in motors.filter(|m| m.body < n) {
        draw_motor(geo, &bodies[m.body], m.torque);
    }
}

/// A connection that is switched off: faint dashes between its bodies'
/// centres (and to a world point `end`), or a faint ring on a lone body.
fn draw_switched_off(geo: &mut GeometryBuilder, bodies: &[Body], ids: &[usize], end: Option<Vec2>) {
    const OFF: [f32; 4] = [1.0, 1.0, 1.0, 0.3];
    let mut pts: Vec<Vec2> = ids.iter().filter_map(|&i| bodies.get(i)).map(|b| b.pos32()).collect();
    pts.extend(end);
    match pts.as_slice() {
        [] => {}
        [p] => geo.draw_dashed_ring(*p, 0.16, 12, 0.014, OFF),
        _ => for w in pts.windows(2) { geo.draw_dashed(w[0], w[1], 0.014, 0.06, 0.05, OFF); },
    }
}

/// Motor hub radius: a little larger than a pin boss, which it covers.
const MOTOR_R: f32 = 0.14;

/// A driving motor on `b`, seen end-on: a dark rotor hub with a thin white
/// rim, three white chevrons inside pointing the way the torque turns (they
/// rotate with the body), and the axle as a white dot. Drag alone (a brake
/// or load) drives nothing and isn't drawn.
fn draw_motor(geo: &mut GeometryBuilder, b: &Body, torque: f32) {
    if torque == 0.0 || matches!(b.shape, BodyShape::Point) { return; }
    let c = b.pos32();
    let dir = torque.signum();
    geo.draw_circle(c, MOTOR_R + OUTLINE, 40, BG);
    geo.draw_arc(c, MOTOR_R - 0.009, 0.0, std::f32::consts::TAU, 40, 0.018, WHITE);
    let r = MOTOR_R * 0.6;
    for k in 0..3 {
        let a = b.angle32() + k as f32 * std::f32::consts::TAU / 3.0;
        let radial = Vec2::new(a.cos(), a.sin());
        let t = radial.perp() * dir; // the way this point travels
        let p = c + radial * r;
        let (len, half) = (0.032, 0.03);
        let tip = p + t * len;
        let back = p - t * len * 0.5;
        // A chevron: two strokes meeting at the tip.
        geo.draw_line(back + radial * half, tip, 0.016, WHITE);
        geo.draw_line(back - radial * half, tip, 0.016, WHITE);
    }
    geo.draw_circle(c, 0.024, 16, WHITE);
}

// ── Gears, belts, ropes, torsion springs ─────────────────────────────────────

/// Tooth pitch along the rim, addendum and dedendum.
const TOOTH_PITCH: f32 = 0.11;
const TOOTH_ADD:   f32 = 0.035;
const TOOTH_DED:   f32 = 0.035;
const BELT_W:      f32 = crate::sim::constraints::gear::BELT_WIDTH as f32;
const ROPE_W:      f32 = 0.022;

fn tooth_count(radius: f32) -> u32 {
    ((std::f32::consts::TAU * radius / TOOTH_PITCH).round() as u32).max(6)
}

/// Phase offset of the teeth on every disk that meshes, so meshing pairs
/// interleave: each gear's teeth sit in its partner's gaps at the contact.
/// Worked out along each train from its first gear.
fn gear_phases(world: &World) -> Vec<Option<f32>> {
    let bodies = &world.bodies;
    let mut phase: Vec<Option<f32>> = vec![None; bodies.len()];
    for c in world.constraints.iter().filter(|c| c.on) {
        let Some(g) = c.as_any().downcast_ref::<GearJoint>() else { continue };
        if g.kind != GearKind::Mesh || g.body_a >= bodies.len() || g.body_b >= bodies.len() { continue; }
        let (a, b) = (g.body_a, g.body_b);
        let (Some(ra), Some(rb)) = (disk_r(&bodies[a]), disk_r(&bodies[b])) else { continue };
        let (pa, pb) = (phase[a], phase[b]);
        // Orient the pair so `from` already has a phase if either does.
        let (from, to, rf, rt) = if pa.is_none() && pb.is_some() { (b, a, rb, ra) } else { (a, b, ra, rb) };
        let pf = *phase[from].get_or_insert(0.0);
        if phase[to].is_some() { continue; }
        let d = bodies[to].pos32() - bodies[from].pos32();
        let psi = d.y.atan2(d.x);
        let (nf, nt) = (tooth_count(rf) as f32, tooth_count(rt) as f32);
        let tau = std::f32::consts::TAU;
        // Fraction of a tooth pitch `from`'s nearest tooth is from the contact line.
        let ff = ((psi - bodies[from].angle32() - pf) * nf / tau).rem_euclid(1.0);
        // Mirror it on `to` (they turn opposite ways), half a pitch over.
        let ft = 0.5 - ff;
        phase[to] = Some(psi + std::f32::consts::PI - bodies[to].angle32() - ft * tau / nt);
        let _ = rt;
    }
    phase
}

fn disk_r(b: &Body) -> Option<f32> {
    match b.shape { BodyShape::Disk { radius } => Some(radius), _ => None }
}

/// Trapezoid teeth around the pitch circle `r`, starting at angle `start`.
fn draw_teeth(geo: &mut GeometryBuilder, c: Vec2, r: f32, start: f32, color: [f32; 4]) {
    let n = tooth_count(r);
    let step = std::f32::consts::TAU / n as f32;
    let (root, tip) = (r - TOOTH_DED, r + TOOTH_ADD);
    for (grow, col) in [(OUTLINE, BG), (0.0, color)] {
        for k in 0..n {
            let a = start + k as f32 * step;
            let dir = Vec2::new(a.cos(), a.sin());
            let side = dir.perp();
            // Wider at the root than the tip.
            let (w_root, w_tip) = (step * r * 0.30 + grow, step * r * 0.18 + grow);
            let p0 = c + dir * root;
            let p1 = c + dir * (tip + grow);
            geo.draw_quad(p0 - side * w_root, p0 + side * w_root, p1 + side * w_tip, p1 - side * w_tip, col);
        }
    }
}

/// Points along an arc (inclusive of both ends).
fn arc_points(c: Vec2, r: f32, start: f32, sweep: f32, out: &mut Vec<Vec2>) {
    let segs = ((sweep.abs() * r / 0.03).ceil() as usize).clamp(2, 128);
    for k in 0..=segs {
        let a = start + sweep * k as f32 / segs as f32;
        out.push(c + Vec2::new(a.cos(), a.sin()) * r);
    }
}

/// Thick polyline with round joins; `closed` joins the ends.
fn polyline(geo: &mut GeometryBuilder, pts: &[Vec2], w: f32, color: [f32; 4], closed: bool) {
    let n = pts.len();
    if n < 2 { return; }
    let segs = if closed { n } else { n - 1 };
    for k in 0..segs {
        let (a, b) = (pts[k], pts[(k + 1) % n]);
        geo.draw_line(a, b, w, color);
        geo.draw_circle(b, w * 0.5, 8, color);
    }
}

/// Open belt around two pulleys, with marks that travel with it.
fn draw_belt(geo: &mut GeometryBuilder, a: &Body, b: &Body) {
    let (Some(ra), Some(rb)) = (disk_r(a), disk_r(b)) else { return };
    let (ca, cb) = (a.pos32(), b.pos32());
    let d = cb - ca;
    let len = d.length();
    let (ra, rb) = (ra + BELT_W * 0.5, rb + BELT_W * 0.5);
    if len <= (ra - rb).abs() + 1e-4 { return; }
    let base = d.y.atan2(d.x);
    let gamma = ((ra - rb) / len).clamp(-1.0, 1.0).acos();
    let tau = std::f32::consts::TAU;
    // Wrapped arcs: round the back of a, and the front of b.
    let mut pts = Vec::new();
    arc_points(ca, ra, base + gamma, tau - 2.0 * gamma, &mut pts);
    arc_points(cb, rb, base - gamma, 2.0 * gamma, &mut pts);
    polyline(geo, &pts, BELT_W + 2.0 * OUTLINE, BG, true);
    polyline(geo, &pts, BELT_W, WHITE, true);
    // Marks every so often along the belt, carried round by pulley a.
    let spacing = 0.16;
    let mut offset = (a.angle32() * ra).rem_euclid(spacing);
    let m = pts.len();
    for k in 0..m {
        let (p, q) = (pts[k], pts[(k + 1) % m]);
        let seg = (q - p).length();
        let dir = (q - p) / seg.max(1e-6);
        while offset < seg {
            let at = p + dir * offset;
            geo.draw_line(at - dir * 0.012, at + dir * 0.012, BELT_W * 0.6, BG);
            offset += spacing;
        }
        offset -= seg;
    }
}

/// Rope as a thin line: straight (or sagging, when slack) between its ends,
/// or two strands and the arc it lies on over a pulley.
fn draw_rope(geo: &mut GeometryBuilder, r: &Rope, bodies: &[Body]) {
    let path = r.path(bodies);
    let mut pts = Vec::new();
    match path.over {
        Some((c, pr, start, sweep)) => {
            pts.push(path.a);
            arc_points(c, pr + ROPE_W * 0.5, start, sweep, &mut pts);
            pts.push(path.b);
        }
        None => {
            let (a, b) = (path.a, path.b);
            let dist = (b - a).length();
            let excess = (r.length - dist).max(0.0);
            // A parabola of the same length hangs about √(3·d·excess/8) deep.
            let sag = if path.slack { (3.0 * dist * excess / 8.0).sqrt() } else { 0.0 };
            let segs = if sag > 1e-3 { 24 } else { 1 };
            for k in 0..=segs {
                let t = k as f32 / segs as f32;
                pts.push(a + (b - a) * t - Vec2::Y * (4.0 * sag * t * (1.0 - t)));
            }
        }
    }
    polyline(geo, &pts, ROPE_W + 2.0 * OUTLINE, BG, false);
    polyline(geo, &pts, ROPE_W, WHITE, false);
}

/// Flat spiral spring: inner end on the body at `angle`, unwinding as it
/// is twisted.
fn draw_torsion(geo: &mut GeometryBuilder, p: Vec2, angle: f32, twist: f32) {
    let (r0, r1, turns) = (0.07, 0.2, 2.25);
    let sweep = turns * std::f32::consts::TAU - twist;
    let segs = 72;
    let pts: Vec<Vec2> = (0..=segs).map(|k| {
        let t = k as f32 / segs as f32;
        let a = angle + sweep * t;
        p + Vec2::new(a.cos(), a.sin()) * (r0 + (r1 - r0) * t)
    }).collect();
    polyline(geo, &pts, 0.02 + 2.0 * OUTLINE, BG, false);
    polyline(geo, &pts, 0.02, WHITE, false);
    // Outer end hooks onto a short post.
    let end = *pts.last().unwrap();
    geo.draw_circle(end, 0.03, 12, WHITE);
}

fn cylinders(world: &World) -> impl Iterator<Item = &Cylinder> {
    let n = world.bodies.len();
    world.constraints.iter().filter(|c| c.on)
        .filter_map(|c| c.as_any().downcast_ref::<Cylinder>())
        .filter(move |cy| cy.piston < n && cy.barrel < n)
}

const WALL:      f32 = 0.045;
const HEAD_T:    f32 = 0.07;
const FIN:       f32 = 0.11;
const FIN_PITCH: f32 = 0.075;
const MAX_LIFT:  f32 = 0.045;
const HOT:       [f32; 4] = [1.0, 0.42, 0.10, 1.0];
const FLASH:     [f32; 4] = [1.0, 0.90, 0.55, 1.0];
const CHARGE:    [f32; 4] = [0.16, 0.26, 0.40, 1.0];
const SMOKE:     [f32; 4] = [0.34, 0.32, 0.31, 1.0];

fn lerp_color(a: [f32; 4], b: [f32; 4], t: f32) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t)
}

/// Axis-aligned (in the bore frame) rectangle: `s0..s1` along `u` from
/// `o`, half width `hw` across.
fn bore_rect(geo: &mut GeometryBuilder, o: Vec2, u: Vec2, s0: f32, s1: f32, hw: f32, color: [f32; 4]) {
    geo.draw_line(o + u * s0, o + u * s1, 2.0 * hw, color);
}

/// Barrel frame: centre, axis (towards the head), half length, bore.
fn barrel_frame(barrel: &Body) -> (Vec2, Vec2, f32, f32) {
    let (s, c) = barrel.angle32().sin_cos();
    let (hl, bore) = match barrel.shape {
        BodyShape::Rod { half_len, half_width } => (half_len, half_width),
        _ => (0.0, 0.0),
    };
    (barrel.pos32(), Vec2::new(c, s), hl, bore)
}

/// Barrel plus live gas: cooling fins, barrel and head, base flange, the
/// charge coloured by stroke and pressure, valves lifting on their strokes
/// (4-stroke) and the spark plug firing at ignition.
fn draw_cylinder(geo: &mut GeometryBuilder, cy: &Cylinder, barrel: &Body, st: CylinderState) {
    let (o, u, hl, bore) = barrel_frame(barrel);
    let m = u.perp();
    let (base, head) = (-hl, hl);
    let outer = bore + WALL;
    let combustion = cy.mode != GasMode::Sealed;

    // Cooling fins on combustion cylinders, from above the flange over the
    // head; a sealed air strut stays smooth.
    let mut f = base + 0.24;
    while combustion && f < head + HEAD_T {
        bore_rect(geo, o, u, f - 0.015 - OUTLINE, f + 0.015 + OUTLINE, outer + FIN + OUTLINE, BG);
        bore_rect(geo, o, u, f - 0.015, f + 0.015, outer + FIN, WHITE);
        f += FIN_PITCH;
    }
    // Barrel and head as one block, bore hollowed out.
    bore_rect(geo, o, u, base - OUTLINE, head + HEAD_T + OUTLINE, outer + OUTLINE, BG);
    bore_rect(geo, o, u, base, head + HEAD_T, outer, WHITE);
    // Base flange with two bolts.
    bore_rect(geo, o, u, base - OUTLINE, base + 0.06 + OUTLINE, outer + 0.05 + OUTLINE, BG);
    bore_rect(geo, o, u, base, base + 0.06, outer + 0.05, WHITE);
    for side in [-1.0, 1.0] {
        geo.draw_circle(o + u * (base + 0.03) + m * side * (outer + 0.02), 0.014, 10, BG);
    }

    // Gas between crown and head.
    let gas = match (cy.mode, st.stroke) {
        (GasMode::Sealed, _) | (_, Stroke::Compression) => lerp_color(BG, CHARGE, 0.35 + st.pressure * 3.0),
        (_, Stroke::Power) => {
            let hot = lerp_color(BG, HOT, (st.pressure.max(0.0) * 2.5).sqrt());
            lerp_color(hot, FLASH, (st.pressure - 0.5) * 2.0)
        }
        (_, Stroke::Exhaust) => lerp_color(BG, SMOKE, 0.8 * (1.0 - st.progress)),
        (_, Stroke::Intake) => lerp_color(BG, CHARGE, 0.35 * (std::f32::consts::PI * st.progress).sin().max(st.progress)),
    };
    bore_rect(geo, o, u, base, head, bore, BG);
    bore_rect(geo, o, u, st.crown.min(head), head, bore, gas);

    // Valves (4-stroke): intake on one side of the axis, exhaust on the other.
    if cy.mode == GasMode::FourStroke {
        let lift = |on: bool| if on { MAX_LIFT * (std::f32::consts::PI * st.progress).sin() } else { 0.0 };
        for (side, open) in [(1.0, st.stroke == Stroke::Intake), (-1.0, st.stroke == Stroke::Exhaust)] {
            let l = lift(open);
            let c = o + m * side * bore * 0.52;
            geo.draw_line(c + u * (head - l), c + u * (head + HEAD_T + 0.06), 0.02, WHITE);
            bore_rect(geo, c, u, head - l - 0.022, head - l, 0.055, WHITE);
            bore_rect(geo, c, u, head + HEAD_T + 0.03, head + HEAD_T + 0.05, 0.035, WHITE);
        }
    }

    // Spark plug through the head; it fires at the start of the power stroke.
    if combustion {
        bore_rect(geo, o, u, head + HEAD_T - OUTLINE, head + HEAD_T + 0.09 + OUTLINE, 0.03 + OUTLINE, BG);
        bore_rect(geo, o, u, head + HEAD_T, head + HEAD_T + 0.09, 0.03, WHITE);
        if st.stroke == Stroke::Power && st.progress < 0.1 {
            let k = 1.0 - st.progress / 0.1;
            geo.draw_circle(o + u * (head - 0.02), 0.025 + 0.06 * k, 16, lerp_color(HOT, FLASH, k));
        }
    }
}

/// Piston block with ring grooves at the wrist pin, covering the piston
/// body's own shape.
fn draw_piston(geo: &mut GeometryBuilder, cy: &Cylinder, bodies: &[Body]) {
    let (_, u, _, bore) = barrel_frame(&bodies[cy.barrel]);
    let p = bodies[cy.piston].world_point(cy.local);
    let hw = bore - 0.012;
    bore_rect(geo, p, u, -cy.crown, cy.crown, hw, WHITE);
    for g in [0.03, 0.058] {
        bore_rect(geo, p, u, cy.crown - g - 0.006, cy.crown - g + 0.006, hw, BG);
    }
    // Skirt edge
    bore_rect(geo, p, u, -cy.crown - OUTLINE, -cy.crown, hw, BG);
}

fn outlined_circle(geo: &mut GeometryBuilder, c: Vec2, r: f32, segs: u32) {
    geo.draw_circle(c, r + OUTLINE, segs, BG);
    geo.draw_circle(c, r, segs, WHITE);
}

fn outlined_rod(geo: &mut GeometryBuilder, a: Vec2, b: Vec2, w: f32) {
    geo.draw_rod(a, b, w + 2.0 * OUTLINE, BG);
    geo.draw_rod(a, b, w, WHITE);
}

/// Rectangle centred on `c` with axis `u`, half extents `hl` (along `u`) and
/// `hw`, corners rounded with radius `r`.
fn rounded_rect(geo: &mut GeometryBuilder, c: Vec2, u: Vec2, hl: f32, hw: f32, r: f32, color: [f32; 4]) {
    let m = u.perp();
    geo.draw_line(c - u * hl, c + u * hl, 2.0 * (hw - r), color);
    geo.draw_line(c - u * (hl - r), c + u * (hl - r), 2.0 * hw, color);
    for (a, b) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
        geo.draw_circle(c + u * a * (hl - r) + m * b * (hw - r), r, 8, color);
    }
}

/// Joint: white boss with a dark hole and a white pin core.
fn draw_pin(geo: &mut GeometryBuilder, p: Vec2) {
    outlined_circle(geo, p, PIN_BOSS_R, 24);
    draw_pin_hole(geo, p);
}

fn draw_pin_hole(geo: &mut GeometryBuilder, p: Vec2) {
    geo.draw_circle(p, PIN_HOLE_R, 20, BG);
    geo.draw_circle(p, PIN_CORE_R, 12, WHITE);
}

/// Mass-weighted mean height of the free bodies, if there are any.
pub fn free_com_y(world: &World) -> Option<f32> {
    let (mut m_sum, mut my_sum) = (0.0f32, 0.0f32);
    for b in world.bodies.iter().filter(|b| !b.fixed) {
        m_sum += b.mass;
        my_sum += b.mass * b.pos32().y;
    }
    (m_sum > 0.0).then(|| my_sum / m_sum)
}

/// Live mount direction for a point, as used while editing (see `draw_world`).
pub fn anchor_is_ceiling(world: &World, p: Vec2) -> bool {
    free_com_y(world).is_some_and(|y| p.y > y)
}

/// Fixed mount: round-topped pedestal on a ground line, pin at `p`.
fn draw_pedestal(geo: &mut GeometryBuilder, p: Vec2, ceiling: bool) {
    let s = if ceiling { 1.0 } else { -1.0 };
    let base = p + Vec2::new(0.0, s * PED_H);
    geo.draw_line(p, base, 2.0 * (PED_HW + OUTLINE), BG);
    geo.draw_circle(p, PED_HW + OUTLINE, 24, BG);
    draw_pedestal_tinted(geo, p, ceiling, WHITE);
}

/// Pedestal fill and ground line only, in `color` (used for editor ghosts).
pub fn draw_pedestal_tinted(geo: &mut GeometryBuilder, p: Vec2, ceiling: bool, color: [f32; 4]) {
    let s = if ceiling { 1.0 } else { -1.0 };
    let base = p + Vec2::new(0.0, s * PED_H);
    geo.draw_line(p, base, 2.0 * PED_HW, color);
    geo.draw_circle(p, PED_HW, 24, color);
    let g = base + Vec2::new(0.0, s * GROUND_W * 0.5);
    geo.draw_line(g - Vec2::new(GROUND_HW, 0.0), g + Vec2::new(GROUND_HW, 0.0), GROUND_W, color);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::forces::Gravity;
    use crate::sim::world::Integrator;

    fn energy(w: &World) -> f32 {
        let g = w.forces.iter()
            .find_map(|f| f.as_any().downcast_ref::<Gravity>().map(|g| g.g))
            .unwrap_or(0.0);
        w.kinetic_energy() + w.potential_energy(g)
    }

    /// Every scene draws (with the force overlay and a butterfly ghost)
    /// without panicking or producing non-finite vertices, also mid-run.
    #[test]
    fn scenes_draw() {
        for def in SCENES {
            let mut w = (def.build)();
            w.record_reactions = true;
            let ghost = crate::scene_file::clone_world(&w);
            for frame in 0..3 {
                let mut geo = GeometryBuilder::new();
                let (tints, load) = force_tints(&w, 1.0);
                draw_world(&w, free_com_y(&w), &tints, &mut geo);
                draw_force_arrows(&w, &mut geo, 0.01, load.max(1.0));
                draw_ghost(&ghost, &mut geo, 0.01);
                assert!(w.bodies.is_empty() || !geo.vertices.is_empty(), "{} drew nothing", def.name);
                assert!(
                    geo.vertices.iter().all(|v| v.position.iter().chain(&v.color).all(|x| x.is_finite())),
                    "{}: non-finite vertex on frame {frame}", def.name,
                );
                for _ in 0..120 { w.step(1.0 / 240.0); }
            }
        }
    }

    /// Every scene runs 20 s at the app's step size without blowing up,
    /// keeps its constraints, and stays inside (a margin around) its view.
    #[test]
    fn scenes_are_stable() {
        let dt = 1.0 / 240.0;
        for (def, integ) in SCENES.iter().flat_map(|d| [(d, Integrator::Xpbd), (d, Integrator::Rk4)]) {
            let mut w = (def.build)();
            w.integrator = integ;
            // XPBD is built for many small steps (target-fps mode gives it
            // thousands per frame); a single sweep at 240 Hz is below its
            // operating range, so it gets 4 sub-steps per frame step.
            let sub = if integ == Integrator::Xpbd { 4 } else { 1 };
            let e0 = energy(&w);
            let (mut max_err, mut max_out, mut max_ke) = (0.0f32, 0.0f32, 0.0f32);
            let c = Vec2::from(def.camera_center);
            let half = Vec2::from(def.view_size) * 0.5;
            for _ in 0..(20.0 / dt) as usize {
                for _ in 0..sub { w.step(dt / sub as f32); }
                max_err = max_err.max(w.constraint_error());
                max_ke = max_ke.max(w.kinetic_energy());
                for b in w.bodies.iter().filter(|b| !b.fixed) {
                    let out = ((b.pos32() - c).abs() - half).max_element();
                    max_out = max_out.max(out);
                }
            }
            let e1 = energy(&w);
            eprintln!(
                "{:<20} {:<5} max C err {:.1e}  max outside view {:+.2}  KE peak {:.1}  E {:.2} → {:.2}",
                def.name, format!("{integ:?}"), max_err, max_out, max_ke, e0, e1,
            );
            assert!(w.bodies.iter().all(|b| b.pos32().is_finite() && b.vel.is_finite()), "{} blew up", def.name);
            assert!(max_err < 0.02, "{}: constraint error {max_err}", def.name);
            assert!(max_out < 0.3, "{}: left its view by {max_out}", def.name);
        }
    }
}

// ── Force overlay ─────────────────────────────────────────────────────────────

const TENSION:     [f32; 4] = [0.95, 0.30, 0.22, 1.0];
const COMPRESSION: [f32; 4] = [0.30, 0.55, 1.0, 1.0];
const ARROW:       [f32; 4] = [1.0, 0.78, 0.25, 1.0];
/// Longest arrow drawn, in world units, for the largest force on show.
const ARROW_MAX: f32 = 0.9;

/// Where each constraint's force acts, and on which body: a joint's
/// location and the body it pushes (the second of a pair, the hanging one
/// in a chain; the supported body at a world pin).
fn force_sites(world: &World) -> Vec<(usize, usize, Vec2)> {
    let b = &world.bodies;
    let n = b.len();
    let mut out = Vec::new();
    for (ci, c) in world.constraints.iter().enumerate() {
        let any = c.as_any();
        if let Some(p) = any.downcast_ref::<PinJoint>() {
            if p.body_b < n { out.push((ci, p.body_b, b[p.body_b].world_point(p.local_b))); }
        } else if let Some(p) = any.downcast_ref::<PinWorld>() {
            out.push((ci, p.body, p.target));
        } else if let Some(d) = any.downcast_ref::<DistanceConstraint>() {
            if d.body_a < n && d.body_b < n {
                out.push((ci, d.body_a, b[d.body_a].world_point(d.local_a)));
                out.push((ci, d.body_b, b[d.body_b].world_point(d.local_b)));
            }
        } else if let Some(s) = any.downcast_ref::<SliderJoint>() {
            if s.rider < n { out.push((ci, s.rider, b[s.rider].world_point(s.local))); }
        } else if let Some(r) = any.downcast_ref::<Rope>() {
            if r.body_a < n && r.body_b < n {
                out.push((ci, r.body_a, b[r.body_a].world_point(r.local_a)));
                out.push((ci, r.body_b, b[r.body_b].world_point(r.local_b)));
            }
        } else if let Some(r) = any.downcast_ref::<crate::sim::constraints::RollingContact>() {
            if r.body_a < n && r.body_b < n {
                let (pa, pb) = (b[r.body_a].pos32(), b[r.body_b].pos32());
                let ra = disk_r(&b[r.body_a]).unwrap_or(0.0);
                out.push((ci, r.body_b, pa + (pb - pa).normalize_or_zero() * ra));
            }
        } else if let Some(r) = any.downcast_ref::<crate::sim::constraints::RollingOnRod>() {
            if r.on && r.disk < n && r.rod < n {
                let rr = disk_r(&b[r.disk]).unwrap_or(0.0);
                let (s, c) = b[r.rod].angle32().sin_cos();
                let m = Vec2::new(-s, c);
                let side = if m.dot(b[r.disk].pos32() - b[r.rod].pos32()) < 0.0 { 1.0 } else { -1.0 };
                out.push((ci, r.disk, b[r.disk].pos32() + m * side * rr));
            }
        }
    }
    out
}

/// Axial load in every rod from the joints at its ends, tension positive,
/// and a tint from white towards red (tension) or blue (compression),
/// scaled by `scale` (the load drawn at full colour).
pub fn force_tints(world: &World, scale: f32) -> (Vec<[f32; 4]>, f32) {
    let b = &world.bodies;
    let mut load = vec![0.0f32; b.len()];
    if world.reactions.len() == world.constraints.len() {
        for (ci, body, p) in force_sites(world) {
            let BodyShape::Rod { half_len, .. } = b[body].shape else { continue };
            let (s, c) = b[body].angle32().sin_cos();
            let u = Vec2::new(c, s);
            let along = u.dot(p - b[body].pos32());
            if along.abs() < 0.25 * half_len { continue; }
            let f = world.reactions[ci].on(body).as_vec2();
            // Pulling an end outward stretches the rod.
            load[body] += along.signum() * f.dot(u) * 0.5;
        }
    }
    let max = load.iter().fold(0.0f32, |m, l| m.max(l.abs()));
    let tints = load.iter().map(|&l| {
        let k = (l.abs() / scale.max(1e-6)).min(1.0).sqrt();
        lerp_color(WHITE, if l > 0.0 { TENSION } else { COMPRESSION }, k)
    }).collect();
    (tints, max)
}

fn draw_arrow(geo: &mut GeometryBuilder, from: Vec2, v: Vec2, w: f32, color: [f32; 4]) {
    let len = v.length();
    if len < 1e-4 { return; }
    let dir = v / len;
    let head = (w * 4.0).min(len * 0.5);
    let tip = from + v;
    geo.draw_line(from, tip - dir * head * 0.8, w, color);
    let side = dir.perp() * head * 0.55;
    geo.draw_quad(tip, tip - dir * head + side, tip - dir * head * 0.75, tip - dir * head - side, color);
}

/// Arrows for the force each joint (and each contact) applies, scaled so
/// a force of `scale` draws `ARROW_MAX` long. Returns the largest force.
pub fn draw_force_arrows(world: &World, geo: &mut GeometryBuilder, px: f32, scale: f32) -> f32 {
    let k = ARROW_MAX / scale.max(1e-6);
    let w = 2.5 * px;
    let mut max = 0.0f32;
    if world.reactions.len() == world.constraints.len() {
        for (ci, body, p) in force_sites(world) {
            let f = world.reactions[ci].on(body).as_vec2();
            max = max.max(f.length());
            geo.draw_circle(p, 3.0 * px, 10, ARROW);
            draw_arrow(geo, p, f * k, w, ARROW);
        }
    }
    for c in &world.contacts.last {
        let f = (c.n * c.f_n + c.n.perp() * c.f_t).as_vec2();
        max = max.max(f.length());
        draw_arrow(geo, c.p.as_vec2(), f * k, w, ARROW);
    }
    max
}

/// The butterfly ghost: each body as a translucent outline.
pub fn draw_ghost(world: &World, geo: &mut GeometryBuilder, px: f32) {
    const GHOST: [f32; 4] = [0.95, 0.35, 0.25, 0.75];
    let w = 2.0 * px;
    for b in world.bodies.iter().filter(|b| !b.fixed) {
        match b.shape {
            BodyShape::Disk { radius } => {
                geo.draw_arc(b.pos32(), radius, 0.0, std::f32::consts::TAU, 48, w, GHOST);
                let (s, c) = b.angle32().sin_cos();
                geo.draw_line(b.pos32(), b.pos32() + Vec2::new(c, s) * radius, w, GHOST);
            }
            BodyShape::Rod { half_len, half_width } => {
                let (s, c) = b.angle32().sin_cos();
                let (u, m) = (Vec2::new(c, s), Vec2::new(-s, c));
                let p = b.pos32();
                for side in [-1.0, 1.0] {
                    geo.draw_line(p - u * half_len + m * side * half_width, p + u * half_len + m * side * half_width, w, GHOST);
                }
                let a = s.atan2(c);
                let half = std::f32::consts::FRAC_PI_2;
                geo.draw_arc(p + u * half_len, half_width, a - half, a + half, 8, w, GHOST);
                geo.draw_arc(p - u * half_len, half_width, a + half, a + 3.0 * half, 8, w, GHOST);
            }
            BodyShape::Point => {}
        }
    }
}
