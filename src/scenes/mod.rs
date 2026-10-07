use glam::Vec2;
use crate::renderer::geometry::GeometryBuilder;
use crate::sim::{
    body::{disk_inertia, rod_inertia, Body, BodyShape},
    constraints::{
        cylinder::{CylinderState, GasMode, Stroke}, slider::{COLLAR_HALF, STOP_W},
        Cylinder, DistanceConstraint, PinJoint, PinWorld, SliderJoint,
    },
    forces::spring::SpringDamper,
    world::World,
};

pub mod air_struts;
pub mod coupled_pendulums;
pub mod double_pendulum;
pub mod elastic_pendulum;
pub mod planetary_pendulum;
pub mod radial_engine;
pub mod rocker_linkage;
pub mod rolling_crane;
pub mod sandbox;
pub mod trammel;
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
        name:          "Sandbox",
        description:   "Build your own simulation  ·  press E to edit",
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
pub fn draw_world(world: &World, anchor_ref_y: Option<f32>, geo: &mut GeometryBuilder) {
    let bodies = &world.bodies;
    let n = bodies.len();

    let ceiling = |p: Vec2| anchor_ref_y.is_some_and(|y| p.y > y);

    let mut pins: Vec<Vec2> = Vec::new();

    // 1. Springs (behind everything)
    for f in &world.forces {
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
    for c in &world.constraints {
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

    // 4. Disks
    for b in bodies {
        if let BodyShape::Disk { radius } = b.shape {
            outlined_circle(geo, b.pos32(), radius, 64);
            let hub = (radius * 0.25).min(0.035);
            geo.draw_circle(b.pos32(), hub, 16, BG);
            // Off-centre dot so rotation is visible
            let (s, c) = b.angle32().sin_cos();
            let dot = b.pos32() + Vec2::new(c, s) * (radius * 0.55);
            geo.draw_circle(dot, (radius * 0.08).max(0.012), 12, BG);
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
            outlined_rod(geo, b.pos32() - d, b.pos32() + d, 2.0 * half_width);
        }
    }

    // 6. Rails: a dark groove between two end stops, then the carriages
    let sliders = || world.constraints.iter()
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
    for c in &world.constraints {
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

    // 9. Pins on top
    for p in pins {
        draw_pin(geo, p);
    }
}

fn cylinders(world: &World) -> impl Iterator<Item = &Cylinder> {
    let n = world.bodies.len();
    world.constraints.iter()
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
