use glam::Vec2;

use crate::sim::{
    body::{Body, BodyShape, disk_inertia, rod_inertia},
    constraint::Constraint,
    constraints::{
        Cylinder, DistanceConstraint, GearJoint, GearKind, PinJoint, PinWorld, RollingContact, WeldJoint,
        RollingOnRod, Rope, SliderJoint,
    },
    forces::{spring::SpringDamper, TorsionSpring},
    world::{Integrator, World},
};

// ── Body palette ──────────────────────────────────────────────────────────────

#[derive(Clone, Debug, PartialEq)]
pub enum BodyTemplate {
    Disk,
    Rod,
    Anchor, // fixed point body
}

#[derive(Clone, Debug)]
pub struct BodyProps {
    pub template: BodyTemplate,
    pub mass:     f32,
    pub radius:   f32, // for Disk
    pub half_len: f32, // for Rod
    pub half_width: f32, // for Rod
    pub fixed:    bool,
    pub collide:  bool,
}

impl Default for BodyProps {
    fn default() -> Self {
        Self { template: BodyTemplate::Disk, mass: 1.0, radius: 0.35, half_len: 0.5, half_width: 0.06, fixed: false, collide: false }
    }
}

// ── Body property editor ──────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub struct BodyEdit {
    pub mass:     f32,
    pub fixed:    bool,
    pub collide:  bool,
    /// Contact material, shown only while `collide` is on.
    pub friction:    f32,
    pub restitution: f32,
    pub plane: u8,
    pub radius:   f32,   // Disk only
    pub half_len: f32,   // Rod only
    pub half_width: f32, // Rod only
    /// Orientation in degrees, wrapped to (-180, 180].
    pub angle:    f32,
    /// `angle` as last read from or written to the body: if the slider
    /// hasn't moved since, the body's own (simulated) angle wins.
    pub angle_synced: f32,
}

/// A body's angle in degrees, wrapped to (-180, 180].
pub fn wrapped_degrees(body: &Body) -> f32 {
    let d = body.angle.to_degrees().rem_euclid(360.0);
    (if d > 180.0 { d - 360.0 } else { d }) as f32
}

/// Turn a body to `deg` degrees, by the smallest change from where it is,
/// so a body that has spun many laps keeps its accumulated angle.
pub fn set_angle_degrees(body: &mut Body, deg: f32) {
    let delta = (deg - wrapped_degrees(body)) as f64;
    let delta = (delta + 180.0).rem_euclid(360.0) - 180.0;
    body.angle += delta.to_radians();
}

/// Which dimension a resize handle changes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ResizeKind {
    Radius,
    /// Drags the rod end on the side `end` (±1); the other end stays put.
    Length { end: i8 },
    Width,
}

/// Gap between a rod's end (or face) and its length (or width) handle.
const LEN_GAP: f32 = 0.14;
const WIDTH_GAP: f32 = 0.12;

/// The resize handles of a body: a disk's on its rim (local +y), a rod's
/// length handles just past both ends and width handle off its +y face.
pub fn resize_handles(body: &Body) -> Vec<(ResizeKind, Vec2)> {
    let at = |local: Vec2| body.world_point(local);
    match body.shape {
        BodyShape::Disk { radius } => vec![(ResizeKind::Radius, at(Vec2::new(0.0, radius)))],
        BodyShape::Rod { half_len, half_width } => vec![
            (ResizeKind::Length { end: -1 }, at(Vec2::new(-(half_len + LEN_GAP), 0.0))),
            (ResizeKind::Length { end: 1 }, at(Vec2::new(half_len + LEN_GAP, 0.0))),
            (ResizeKind::Width, at(Vec2::new(0.0, half_width + WIDTH_GAP))),
        ],
        BodyShape::Point => vec![],
    }
}

/// Resize a body so the dragged handle follows `cursor` (see `set_shape`).
/// A disk keeps its centre; a rod keeps the end opposite the dragged one,
/// so whatever is attached there stays put. `snap` rounds to tidy values.
pub fn resize_to(world: &mut World, idx: usize, kind: ResizeKind, cursor: Vec2, snap: bool) {
    let body = &world.bodies[idx];
    let l = world_to_local(body, cursor);
    let tidy = |v: f32, step: f32| if snap { (v / step).round() * step } else { v };
    let shape = match (kind, body.shape.clone()) {
        (ResizeKind::Radius, BodyShape::Disk { .. }) => {
            BodyShape::Disk { radius: tidy(l.length(), 0.05).clamp(0.05, 2.0) }
        }
        (ResizeKind::Length { end }, BodyShape::Rod { half_len, half_width }) => {
            // Length from the fixed end to the cursor, along the rod.
            let e = end as f32;
            let full = e * l.x + half_len - LEN_GAP;
            let new_hl = tidy(full, 0.1).clamp(0.2, 8.0) * 0.5;
            set_shape(world, idx, BodyShape::Rod { half_len: new_hl, half_width });
            // Slide the centre so the fixed end hasn't moved.
            let b = &mut world.bodies[idx];
            let (sn, cs) = b.angle.sin_cos();
            b.pos += glam::DVec2::new(cs, sn) * (e * (new_hl - half_len)) as f64;
            sync_world_pins(world, idx);
            return;
        }
        (ResizeKind::Width, BodyShape::Rod { half_len, .. }) => {
            BodyShape::Rod { half_len, half_width: tidy(l.y - WIDTH_GAP, 0.01).clamp(0.02, 0.4) }
        }
        _ => return,
    };
    set_shape(world, idx, shape);
    sync_world_pins(world, idx);
}

/// Change a body's size, keeping its centre and recomputing inertia. Every
/// attachment on it scales along: a rod's along its length (and across its
/// width), a disk's with its radius, so a pin on a rod end or a disk rim
/// stays on the end or rim.
pub fn set_shape(world: &mut World, idx: usize, shape: BodyShape) {
    let body = &world.bodies[idx];
    let ratio = |new: f32, old: f32| if old > 1e-6 { new / old } else { 1.0 };
    let scale = match (&body.shape, &shape) {
        (BodyShape::Disk { radius: old }, BodyShape::Disk { radius: new }) => Vec2::splat(ratio(*new, *old)),
        (BodyShape::Rod { half_len: ol, half_width: ow }, BodyShape::Rod { half_len: nl, half_width: nw }) => {
            Vec2::new(ratio(*nl, *ol), ratio(*nw, *ow))
        }
        _ => Vec2::ONE,
    };
    if scale != Vec2::ONE {
        scale_attachments(world, idx, scale);
    }
    let body = &mut world.bodies[idx];
    let resized_disk = matches!((&body.shape, &shape), (BodyShape::Disk { radius: a }, BodyShape::Disk { radius: b }) if a != b);
    body.inertia = match shape {
        BodyShape::Disk { radius } => disk_inertia(body.mass, radius),
        BodyShape::Rod { half_len, .. } => rod_inertia(body.mass, half_len),
        BodyShape::Point => body.inertia,
    };
    body.shape = shape;
    if resized_disk {
        // A rope over a resized pulley keeps its strands as they are.
        for c in world.constraints.iter_mut() {
            if let Some(r) = c.as_any_mut().downcast_mut::<Rope>() {
                if r.pulley == Some(idx) { r.length = r.current_length(&world.bodies) as f32; }
            }
        }
    }
}

/// Multiply every body-local attachment point on `idx` by `s` (per axis).
fn scale_attachments(world: &mut World, idx: usize, s: Vec2) {
    let scale = |body: usize, local: &mut Vec2| if body == idx { *local *= s };
    for c in world.constraints.iter_mut() {
        let any = c.as_any_mut();
        if let Some(pj) = any.downcast_mut::<PinJoint>() {
            scale(pj.body_a, &mut pj.local_a);
            scale(pj.body_b, &mut pj.local_b);
        } else if let Some(wj) = any.downcast_mut::<WeldJoint>() {
            scale(wj.body_a, &mut wj.local_a);
            scale(wj.body_b, &mut wj.local_b);
        } else if let Some(pw) = any.downcast_mut::<PinWorld>() {
            scale(pw.body, &mut pw.local);
        } else if let Some(dc) = any.downcast_mut::<DistanceConstraint>() {
            scale(dc.body_a, &mut dc.local_a);
            scale(dc.body_b, &mut dc.local_b);
        } else if let Some(sj) = any.downcast_mut::<SliderJoint>() {
            scale(sj.rider, &mut sj.local);
        } else if let Some(cy) = any.downcast_mut::<Cylinder>() {
            scale(cy.piston, &mut cy.local);
            // The crown sits on the piston's face across the bore axis.
            if cy.piston == idx { cy.crown *= s.y; }
        } else if let Some(r) = any.downcast_mut::<Rope>() {
            scale(r.body_a, &mut r.local_a);
            scale(r.body_b, &mut r.local_b);
        }
    }
    for f in world.forces.iter_mut() {
        let any = f.as_any_mut();
        if let Some(sd) = any.downcast_mut::<SpringDamper>() {
            scale(sd.body_a, &mut sd.local_a);
            scale(sd.body_b, &mut sd.local_b);
        } else if let Some(t) = any.downcast_mut::<TorsionSpring>() {
            scale(t.body_a, &mut t.local_a);
        }
    }
    for t in world.tracers.iter_mut() {
        scale(t.body, &mut t.local);
    }
}

/// Where the rotate handle of a body sits: just outside a disk in the
/// direction of its local +x axis, or above a rod's +x end (clear of the
/// length handle). Anchors have none.
pub fn rotate_handle(body: &Body) -> Option<Vec2> {
    let local = match body.shape {
        BodyShape::Disk { radius } => Vec2::new(radius + 0.22, 0.0),
        BodyShape::Rod { half_len, half_width } => Vec2::new(half_len, half_width + 0.24),
        BodyShape::Point => return None,
    };
    Some(body.world_point(local))
}

impl BodyEdit {
    pub fn from_body(body: &Body) -> Self {
        let (radius, half_len, half_width) = match body.shape {
            BodyShape::Disk { radius }                => (radius, 0.5, 0.06),
            BodyShape::Rod  { half_len, half_width }  => (0.35, half_len, half_width),
            BodyShape::Point                          => (0.35, 0.5, 0.06),
        };
        let angle = wrapped_degrees(body);
        Self { mass: body.mass, fixed: body.fixed, collide: body.collide, friction: body.friction, restitution: body.restitution, plane: body.plane, radius, half_len, half_width, angle, angle_synced: angle }
    }

    /// Apply edited values back to body `idx`, recalculating inertia; a
    /// size change goes through `set_shape` so attachments follow.
    pub fn apply_to(&mut self, world: &mut World, idx: usize) {
        let body = &mut world.bodies[idx];
        let turned = self.angle != self.angle_synced;
        if turned {
            set_angle_degrees(body, self.angle);
        } else {
            // Follow the body while it turns (simulation running in edit mode).
            self.angle = wrapped_degrees(body);
        }
        self.angle_synced = self.angle;
        body.mass  = self.mass;
        body.fixed = self.fixed;
        body.collide = self.collide;
        body.friction = self.friction;
        body.restitution = self.restitution;
        body.plane = self.plane;
        if self.fixed {
            // A fixed body isn't integrated, but constraints still read its
            // velocity; a leftover spin would drive whatever rolls on it.
            body.vel = glam::DVec2::ZERO;
            body.ang_vel = 0.0;
        }
        let shape = match body.shape {
            BodyShape::Disk { .. } => BodyShape::Disk { radius: self.radius },
            BodyShape::Rod { .. } => BodyShape::Rod { half_len: self.half_len, half_width: self.half_width },
            BodyShape::Point => BodyShape::Point,
        };
        let resized = match (&body.shape, &shape) {
            (BodyShape::Disk { radius: a }, BodyShape::Disk { radius: b }) => a != b,
            (BodyShape::Rod { half_len: l0, half_width: w0 }, BodyShape::Rod { half_len: l1, half_width: w1 }) => l0 != l1 || w0 != w1,
            _ => false,
        };
        set_shape(world, idx, shape);
        if turned || resized { sync_world_pins(world, idx); }
    }
}

/// After the editor moves, turns or resizes body `idx`, bring its world
/// pins along: each pin's world point becomes wherever its body point now
/// is. (Only for editor moves: the simulation must never drag pins.)
pub fn sync_world_pins(world: &mut World, idx: usize) {
    let Some(body) = world.bodies.get(idx) else { return };
    for c in world.constraints.iter_mut() {
        if let Some(pw) = c.as_any_mut().downcast_mut::<PinWorld>() {
            if pw.body == idx { pw.target = body.world_point(pw.local); }
        }
    }
}

// ── Constraint picker ─────────────────────────────────────────────────────────

#[derive(Clone, Debug, PartialEq)]
pub enum ConstraintKind {
    PinJoint,
    Weld,
    PinToWorld,
    RollingContact,
    Distance,
    Spring,
    Slider,
    Cylinder,
    Gear,
    Belt,
    Rope,
    /// A rope that runs over a pulley picked next (`EditorMode::PulleyPending`).
    RopeOverPulley,
    Torsion,
}

// ── Editor state machine ───────────────────────────────────────────────────────

#[derive(Clone, Debug)]
pub enum EditorMode {
    /// Normal: idle, ready to select a body
    Idle,
    /// User is hovering to place a new body (click to confirm placement)
    PlacingBody,
    /// First body selected; waiting for second body or world click
    FirstSelected {
        body_idx:  usize,
        world_pos: Vec2, // where on the body was clicked
    },
    /// Two bodies selected; show constraint picker popup
    BothSelected {
        body_a:    usize,
        attach_a:  Vec2, // world position clicked on body a
        body_b:    usize,
        attach_b:  Vec2, // world position clicked on body b
    },
    /// One body selected, user chose "Pin to world" — now click a world anchor
    PinWorldPending {
        body_idx:  usize,
        attach:    Vec2, // local attach on body
    },
    /// Two ends chosen for a rope; waiting for a click on the pulley disk.
    PulleyPending {
        body_a:   usize,
        attach_a: Vec2,
        body_b:   usize,
        attach_b: Vec2,
    },
    /// Dragging a body
    Dragging {
        body_idx: usize,
    },
    /// Clicked on a body to inspect/edit properties
    Inspecting {
        body_idx: usize,
    },
    /// Waiting for a click that places a new trace point on the body
    PlacingTrace {
        body_idx: usize,
    },
    /// Turning a body with its rotate handle; `offset` is body angle minus
    /// the cursor's bearing from the body centre when the drag started.
    Rotating {
        body_idx: usize,
        offset:   f32,
    },
    /// Dragging one of the selected body's resize handles
    Resizing {
        body_idx: usize,
        kind:     ResizeKind,
    },
    /// Dragging a box over empty space from `start` (world); on release
    /// the bodies whose centres lie inside become the selection.
    BoxSelecting {
        start: Vec2,
    },
    /// Several bodies selected (`Editor::selection`): drag one to move them
    /// all, Delete, Ctrl+C / Ctrl+X.
    Selection,
    /// Moving the whole selection, grabbed by `body` at `start`; `last` is
    /// the previous cursor point. Released without moving, it selects `body`.
    GroupDragging {
        last:  Vec2,
        start: Vec2,
        body:  usize,
    },
    /// Dragging one of the selected body's connection points
    DraggingHandle {
        body_idx: usize,
        handle:   HandleRef,
    },
}

/// A draggable connection point: which constraint/spring, and which end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HandleRef {
    Constraint { idx: usize, end: HandleEnd },
    Spring     { idx: usize, end: HandleEnd },
    Tracer     { idx: usize },
}

/// `Both` is a shared pivot (pin joint / world pin): moving it moves the
/// attachment on every side so the joint stays satisfied.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HandleEnd { A, B, Both }

impl Default for EditorMode {
    fn default() -> Self { EditorMode::Idle }
}

pub struct Editor {
    pub active:              bool,
    pub mode:                EditorMode,
    pub body_props:          BodyProps,
    pub inspect_idx:         Option<usize>,
    pub selected:            Option<usize>,
    pub pending_constraint:  Option<ConstraintKind>,
    pub delete_requested:    bool,
    pub spring_k:            f32,
    pub spring_d:            f32,
    /// Live property edit for the currently inspected body.
    pub body_edit:           Option<BodyEdit>,
    /// Shape tag of inspected body: "Disk" | "Rod" | "Point"
    pub inspect_shape:       Option<String>,
    /// Scene-level gravity (m/s²). Applied to the Gravity force each frame.
    pub gravity:             f32,
    /// Physics sub-steps per rendered frame (set by the app in `TargetFps` mode).
    pub sub_steps:           u32,
    pub step_mode:           StepMode,
    /// How the world is advanced each step.
    pub integrator:          Integrator,
    /// Physics halted. Entering edit mode pauses and leaving resumes, but
    /// it can be toggled freely (PAUSE button, Space), editing included.
    pub paused:              bool,
    /// Frame rate `StepMode::TargetFps` steers towards.
    pub target_fps:          f32,
    /// Simulation speed: sim seconds per wall-clock second.
    pub time_scale:          f32,
    /// The current BothSelected pair can roll (disk+disk or disk+rod).
    pub pair_can_roll:       bool,
    /// The current BothSelected pair can slide (at least one is a rod).
    pub pair_can_slide:      bool,
    /// The current BothSelected pair can gear (two disks).
    pub pair_can_gear:       bool,
    /// New torsion springs.
    pub torsion_k:           f32,
    pub torsion_d:           f32,
    /// What the connection row under the pointer links to, for scene highlighting.
    pub link_hover:          Option<LinkHover>,
    /// Bodies of a multi-selection (`EditorMode::Selection`), ascending.
    pub selection:           Vec<usize>,
    /// Something has been copied (Ctrl+V would paste).
    pub can_paste:           bool,
    /// The Parts window's list of every body is open.
    pub show_body_list:      bool,
    /// Body under the pointer in that list, highlighted in the scene.
    pub list_hover:          Option<usize>,
    /// Shift is held where a click would pick a connection's first body
    /// (set by the app each frame; shown in the scene and the status line).
    pub connect_ready:       bool,
    /// The Parts window's CONNECT switch: acts as a held Shift for picking
    /// connections, for touch screens. Off again once two bodies are picked.
    pub connect_mode:        bool,
    /// HUD windows folded to their title row. The main panel's is `None`
    /// until the user folds or unfolds it: folded on narrow screens.
    pub main_folded:         Option<bool>,
    pub parts_folded:        bool,
    pub selection_folded:    bool,
}

/// How the number of physics steps per frame is chosen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StepMode {
    /// The user sets the step count directly.
    Fixed,
    /// The app picks as many steps as fit the target frame rate.
    TargetFps,
}

/// The far end of a connection the user is hovering in the inspector.
#[derive(Clone, Copy, Debug)]
pub enum LinkHover {
    Body(usize),
    Point(Vec2),
}

impl Default for Editor {
    fn default() -> Self {
        Self {
            active:             false,
            mode:               EditorMode::Idle,
            body_props:         BodyProps::default(),
            inspect_idx:        None,
            selected:           None,
            pending_constraint: None,
            delete_requested:   false,
            spring_k:           80.0,
            spring_d:           8.0,
            body_edit:          None,
            inspect_shape:      None,
            gravity:            9.81,
            sub_steps:          4,
            step_mode:          StepMode::TargetFps,
            integrator:         Integrator::Rk4,
            paused:             false,
            target_fps:         60.0,
            time_scale:         1.0,
            pair_can_roll:      false,
            pair_can_slide:     false,
            pair_can_gear:      false,
            torsion_k:          20.0,
            torsion_d:          0.5,
            link_hover:         None,
            connect_ready:      false,
            connect_mode:       false,
            main_folded:        None,
            parts_folded:       false,
            selection_folded:   false,
            selection:          Vec::new(),
            can_paste:          false,
            show_body_list:     false,
            list_hover:         None,
        }
    }
}

impl Editor {
    pub fn toggle(&mut self) {
        self.active = !self.active;
        self.paused = self.active;
        self.mode = EditorMode::Idle;
        self.inspect_idx = None;
        self.selected = None;
        self.body_edit = None;
        self.inspect_shape = None;
        self.pending_constraint = None;
        self.link_hover = None;
        self.list_hover = None;
        self.selection.clear();
    }

    /// The bodies an edit command (copy, cut, delete) acts on: the group,
    /// or the one inspected body.
    pub fn selected_bodies(&self) -> Vec<usize> {
        match self.mode {
            EditorMode::Selection | EditorMode::GroupDragging { .. } => self.selection.clone(),
            EditorMode::Inspecting { body_idx } => vec![body_idx],
            _ => Vec::new(),
        }
    }

    /// Delete several bodies (and everything attached to them).
    pub fn delete_bodies(&self, bodies: &[usize], world: &mut World) {
        let mut ids = bodies.to_vec();
        ids.sort_unstable();
        ids.dedup();
        for &i in ids.iter().rev() { self.delete_body(i, world); }
    }

    /// Place a new body at world position.
    pub fn place_body(&self, world_pos: Vec2, world: &mut World) -> usize {
        let bp = &self.body_props;
        let body = match bp.template {
            BodyTemplate::Disk => {
                let inertia = disk_inertia(bp.mass, bp.radius);
                let mut b = Body::new(world_pos, 0.0, bp.mass, inertia,
                                      BodyShape::Disk { radius: bp.radius });
                if bp.fixed { b = b.fixed(); }
                b.collide = bp.collide;
                b
            }
            BodyTemplate::Rod => {
                let inertia = rod_inertia(bp.mass, bp.half_len);
                let mut b = Body::new(world_pos, 0.0, bp.mass, inertia,
                                      BodyShape::Rod { half_len: bp.half_len, half_width: bp.half_width });
                if bp.fixed { b = b.fixed(); }
                b.collide = bp.collide;
                b
            }
            BodyTemplate::Anchor => {
                Body::new(world_pos, 0.0, 1.0, 1.0, BodyShape::Point).fixed()
            }
        };
        world.add_body(body)
    }

    /// Add a constraint between two bodies (or body+world) from UI selection.
    pub fn add_constraint(
        &self,
        kind:     ConstraintKind,
        body_a:   usize,
        attach_a: Vec2, // world position
        body_b:   Option<usize>,
        attach_b: Vec2, // world position (or world anchor if body_b is None)
        world:    &mut World,
        spring_k: f32,
        spring_d: f32,
    ) {
        match kind {
            ConstraintKind::PinJoint => {
                if let Some(b) = body_b {
                    let local_a = world_to_local(&world.bodies[body_a], attach_a);
                    let local_b = world_to_local(&world.bodies[b], attach_b);
                    world.add_constraint(PinJoint::new(body_a, local_a, b, local_b));
                }
            }
            ConstraintKind::Weld => {
                if let Some(b) = body_b {
                    world.add_constraint(WeldJoint::new(body_a, b, attach_a, &world.bodies));
                }
            }
            ConstraintKind::PinToWorld => {
                let local_a = world_to_local(&world.bodies[body_a], attach_a);
                world.add_constraint(PinWorld::new(body_a, local_a, attach_b));
            }
            ConstraintKind::RollingContact => {
                if let Some(b) = body_b {
                    add_rolling(body_a, b, world);
                }
            }
            ConstraintKind::Distance => {
                if let Some(b) = body_b {
                    let local_a = world_to_local(&world.bodies[body_a], attach_a);
                    let local_b = world_to_local(&world.bodies[b], attach_b);
                    let rest = (attach_b - attach_a).length();
                    world.add_constraint(DistanceConstraint::new(body_a, local_a, b, local_b, rest));
                }
            }
            ConstraintKind::Slider => {
                if let Some(b) = body_b {
                    add_slider(body_a, attach_a, b, attach_b, world, false);
                }
            }
            ConstraintKind::Cylinder => {
                if let Some(b) = body_b {
                    add_slider(body_a, attach_a, b, attach_b, world, true);
                }
            }
            ConstraintKind::Gear | ConstraintKind::Belt => {
                if let Some(b) = body_b {
                    let kind = if kind == ConstraintKind::Gear { GearKind::Mesh } else { GearKind::Belt };
                    if let Some(g) = GearJoint::new(body_a, b, kind, &world.bodies) {
                        world.add_constraint(g);
                    }
                }
            }
            ConstraintKind::Rope | ConstraintKind::RopeOverPulley => {
                if let Some(b) = body_b {
                    let local_a = world_to_local(&world.bodies[body_a], attach_a);
                    let local_b = world_to_local(&world.bodies[b], attach_b);
                    world.add_constraint(Rope::new(body_a, local_a, b, local_b, &world.bodies));
                }
            }
            ConstraintKind::Torsion => {
                if let Some(b) = body_b {
                    let local_a = world_to_local(&world.bodies[body_a], attach_a);
                    world.add_force(TorsionSpring::new(body_a, b, local_a, self.torsion_k, self.torsion_d, &world.bodies));
                }
            }
            ConstraintKind::Spring => {
                if let Some(b) = body_b {
                    let local_a = world_to_local(&world.bodies[body_a], attach_a);
                    let local_b = world_to_local(&world.bodies[b], attach_b);
                    let rest = (attach_b - attach_a).length();
                    world.add_force(SpringDamper::new(
                        body_a, local_a, b, local_b, rest, spring_k, spring_d,
                    ));
                }
            }
        }
    }

    /// Delete a body, remove constraints/forces that touch it, remap surviving indices.
    pub fn delete_body(&self, idx: usize, world: &mut World) -> bool {
        if idx >= world.bodies.len() { return false; }
        world.bodies.remove(idx);

        // Drop constraints that reference the deleted body.
        world.constraints.retain(|c| !c.body_indices().contains(&idx));

        // Remap surviving constraints: every body index > idx shifts down by 1.
        let n = world.bodies.len();
        for c in &mut world.constraints {
            for new_i in (idx..n).rev() {
                c.remap_body(new_i + 1, new_i);
            }
        }

        world.follow = match world.follow {
            Some(f) if f == idx => None,
            Some(f) if f > idx => Some(f - 1),
            f => f,
        };
        world.tracers.retain(|t| t.body != idx);
        for t in &mut world.tracers {
            if t.body > idx { t.body -= 1; }
        }

        // Same for forces (Gravity/MouseSpring have empty body_indices, so they survive).
        world.forces.retain(|f| !f.body_indices().contains(&idx));
        for f in &mut world.forces {
            for new_i in (idx..n).rev() {
                f.remap_body(new_i + 1, new_i);
            }
        }

        true
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Add a rope from (a, attach_a) to (b, attach_b) over the disk `pulley`.
/// False if it can't be routed (not a disk, or an end inside it).
pub fn add_rope_over(a: usize, attach_a: Vec2, b: usize, attach_b: Vec2, pulley: usize, world: &mut World) -> bool {
    let local_a = world_to_local(&world.bodies[a], attach_a);
    let local_b = world_to_local(&world.bodies[b], attach_b);
    match Rope::new(a, local_a, b, local_b, &world.bodies).over(pulley, &world.bodies) {
        Some(r) => { world.add_constraint(r); true }
        None => false,
    }
}

/// The body that `idx`'s axle is pinned to (a pin joint at its centre),
/// where a motor driving it would be mounted.
pub fn axle_mount(world: &World, idx: usize) -> Option<usize> {
    world.constraints.iter().filter(|c| c.on).find_map(|c| {
        let pj = c.as_any().downcast_ref::<PinJoint>()?;
        if pj.body_a == idx && pj.local_a.length() < 1e-3 { Some(pj.body_b) }
        else if pj.body_b == idx && pj.local_b.length() < 1e-3 { Some(pj.body_a) }
        else { None }
    })
}

/// Points that can be pinned to the world with one click from the
/// inspector: a disk's centre, a rod's two ends. Rod ends are named by
/// where they are right now (left/right, or top/bottom when upright).
pub fn quick_pin_points(body: &Body) -> Vec<(&'static str, Vec2)> {
    match body.shape {
        BodyShape::Disk { .. } => vec![("PIN CENTRE", Vec2::ZERO)],
        BodyShape::Rod { half_len, .. } => {
            let (a, b) = (Vec2::new(-half_len, 0.0), Vec2::new(half_len, 0.0));
            let d = body.world_point(b) - body.world_point(a);
            let upright = d.y.abs() > d.x.abs();
            let (lo, hi) = if upright { ("PIN BOTTOM", "PIN TOP") } else { ("PIN LEFT", "PIN RIGHT") };
            // `a` comes first along the axis that names them.
            let a_first = if upright { d.y > 0.0 } else { d.x > 0.0 };
            if a_first { vec![(lo, a), (hi, b)] } else { vec![(lo, b), (hi, a)] }
        }
        BodyShape::Point => vec![],
    }
}

/// The world pin holding body-local point `local` of `body`, if any.
pub fn world_pin_at(world: &World, body: usize, local: Vec2) -> Option<usize> {
    world.constraints.iter().position(|c| {
        c.as_any().downcast_ref::<PinWorld>()
            .is_some_and(|pw| pw.body == body && (pw.local - local).length() < 1e-3)
    })
}

/// Pin `local` of `body` to the world where it is now, or remove that pin.
pub fn toggle_world_pin(world: &mut World, body: usize, local: Vec2) {
    match world_pin_at(world, body, local) {
        Some(ci) => { world.constraints.remove(ci); }
        None => {
            let at = world.bodies[body].world_point(local);
            world.add_constraint(PinWorld::new(body, local, at));
        }
    }
}

/// Gears and belts join two disks.
pub fn can_gear(a: &Body, b: &Body) -> bool {
    is_disk(a) && is_disk(b)
}

fn disk_radius_of(body: &Body) -> Option<f32> {
    match body.shape {
        BodyShape::Disk { radius } => Some(radius),
        _ => None,
    }
}

pub fn is_disk(body: &Body) -> bool {
    disk_radius_of(body).is_some()
}

fn is_rod(body: &Body) -> bool {
    matches!(body.shape, BodyShape::Rod { .. })
}

/// Rolling is defined for disk-on-disk and disk-on-rod, in either order.
pub fn can_roll(a: &Body, b: &Body) -> bool {
    (is_disk(a) && (is_disk(b) || is_rod(b))) || (is_rod(a) && is_disk(b))
}

/// Snap the pair into contact, then add the matching rolling constraint.
/// The free body is the one that moves; a fixed one stays where it was put.
fn add_rolling(a: usize, b: usize, world: &mut World) {
    let (pa, pb) = (world.bodies[a].pos32(), world.bodies[b].pos32());
    let move_a = world.bodies[b].fixed && !world.bodies[a].fixed;

    if let (Some(ra), Some(rb)) = (disk_radius_of(&world.bodies[a]), disk_radius_of(&world.bodies[b])) {
        let d = pb - pa;
        let len = d.length();
        if len > 1e-4 {
            let dir = d / len;
            if move_a { world.bodies[a].pos = (pb - dir * (ra + rb)).as_dvec2(); }
            else      { world.bodies[b].pos = (pa + dir * (ra + rb)).as_dvec2(); }
        }
        if let Some(rc) = RollingContact::new(a, b, &world.bodies) {
            world.add_constraint(rc);
        }
        return;
    }

    let (disk, rod) = if is_disk(&world.bodies[a]) { (a, b) } else { (b, a) };
    let (Some(r), BodyShape::Rod { half_width, .. }) =
        (disk_radius_of(&world.bodies[disk]), &world.bodies[rod].shape) else { return };
    // Put the disk on the face of the rod it is currently nearest to.
    let (s, c) = world.bodies[rod].angle32().sin_cos();
    let m = Vec2::new(-s, c);
    let off = m.dot(world.bodies[disk].pos32() - world.bodies[rod].pos32());
    let side = if off < 0.0 { -1.0 } else { 1.0 };
    let shift = m * (side * (r + *half_width) - off);
    if world.bodies[disk].fixed && !world.bodies[rod].fixed {
        world.bodies[rod].pos -= shift.as_dvec2();
    } else {
        world.bodies[disk].pos += shift.as_dvec2();
    }
    if let Some(rr) = RollingOnRod::new(disk, rod, &world.bodies) {
        world.add_constraint(rr);
    }
}

/// Sliding needs a rod to run on, in either order.
pub fn can_slide(a: &Body, b: &Body) -> bool {
    is_rod(a) || is_rod(b)
}

fn rod_half_len(body: &Body) -> f32 {
    match body.shape { BodyShape::Rod { half_len, .. } => half_len, _ => 0.0 }
}

/// The rail is the rod; if both are rods, the one clicked at (or nearest to)
/// one of its ends rides on the other, since a rod end running along a rod is
/// what that click means. Only when neither is clearly nearer an end does the
/// longer rod become the rail. The rider rides at the point clicked on it. That point is snapped onto
/// the rail's centre line within the travel; the free body is the one moved.
/// With `cylinder` the rail becomes the barrel (head at its +x end) and the
/// rider the piston, sealed where it is.
fn add_slider(a: usize, attach_a: Vec2, b: usize, attach_b: Vec2, world: &mut World, cylinder: bool) {
    let (ba, bb) = (&world.bodies[a], &world.bodies[b]);
    let a_is_rail = match (is_rod(ba), is_rod(bb)) {
        (true, true) => {
            let (ea, eb) = (end_closeness(ba, attach_a), end_closeness(bb, attach_b));
            if (ea - eb).abs() > 0.05 { ea < eb } else { rod_half_len(ba) > rod_half_len(bb) }
        }
        (a_rod, _) => a_rod,
    };
    let (rider, attach, rail) = if a_is_rail { (b, attach_b, a) } else { (a, attach_a, b) };
    if !is_rod(&world.bodies[rail]) { return; }

    let local = world_to_local(&world.bodies[rider], attach);
    let target = on_rail(&world.bodies[rail], attach);
    let shift = (target - attach).as_dvec2();
    if world.bodies[rider].fixed && !world.bodies[rail].fixed {
        world.bodies[rail].pos -= shift;
    } else {
        world.bodies[rider].pos += shift;
    }
    if cylinder {
        if let Some(cy) = Cylinder::new(rider, local, rail, &world.bodies) {
            world.add_constraint(cy);
        }
    } else if let Some(sj) = SliderJoint::new(rider, local, rail, &world.bodies) {
        world.add_constraint(sj);
    }
}

/// How close `p` lies to an end of the rod along its length: 0 at the
/// centre, 1 at (or past) an end.
fn end_closeness(rod: &Body, p: Vec2) -> f32 {
    let hl = rod_half_len(rod);
    if hl <= 0.0 { return 0.0; }
    (world_to_local(rod, p).x.abs() / hl).min(1.0)
}

/// Nearest point to `p` on the rail's centre line, within its travel.
fn on_rail(rail: &Body, p: Vec2) -> Vec2 {
    let (s, c) = rail.angle32().sin_cos();
    let u = Vec2::new(c, s);
    let lim = SliderJoint::travel(rail);
    rail.pos32() + u * u.dot(p - rail.pos32()).clamp(-lim, lim)
}

/// Convert a world position to a body-local position.
pub fn world_to_local(body: &Body, world_pos: Vec2) -> Vec2 {
    let r = world_pos - body.pos32();
    let (s, c) = body.angle32().sin_cos();
    Vec2::new(c * r.x + s * r.y, -s * r.x + c * r.y)
}

// ── Connection handles ────────────────────────────────────────────────────────

/// Every movable point on `body_idx` (connection ends and trace points),
/// with its world position.
pub fn connection_handles(world: &World, body_idx: usize) -> Vec<(HandleRef, Vec2)> {
    let b = &world.bodies;
    let mut out = Vec::new();
    for (idx, c) in world.constraints.iter().enumerate() {
        if !c.body_indices().contains(&body_idx) { continue; }
        let any = c.as_any();
        let h = |end| HandleRef::Constraint { idx, end };
        if let Some(pj) = any.downcast_ref::<PinJoint>() {
            out.push((h(HandleEnd::Both), b[pj.body_a].world_point(pj.local_a)));
        } else if let Some(wj) = any.downcast_ref::<WeldJoint>() {
            out.push((h(HandleEnd::Both), b[wj.body_a].world_point(wj.local_a)));
        } else if let Some(pw) = any.downcast_ref::<PinWorld>() {
            out.push((h(HandleEnd::Both), pw.target));
        } else if let Some(sj) = any.downcast_ref::<SliderJoint>() {
            out.push((h(HandleEnd::Both), b[sj.rider].world_point(sj.local)));
        } else if let Some(dc) = any.downcast_ref::<DistanceConstraint>() {
            out.push((h(HandleEnd::A), b[dc.body_a].world_point(dc.local_a)));
            out.push((h(HandleEnd::B), b[dc.body_b].world_point(dc.local_b)));
        } else if let Some(r) = any.downcast_ref::<Rope>() {
            if r.body_a == body_idx { out.push((h(HandleEnd::A), b[r.body_a].world_point(r.local_a))); }
            if r.body_b == body_idx { out.push((h(HandleEnd::B), b[r.body_b].world_point(r.local_b))); }
        }
    }
    for (idx, f) in world.forces.iter().enumerate() {
        if let Some(t) = f.as_any().downcast_ref::<TorsionSpring>() {
            if t.body_a == body_idx || t.body_b == body_idx {
                out.push((HandleRef::Spring { idx, end: HandleEnd::A }, b[t.body_a].world_point(t.local_a)));
            }
            continue;
        }
        let Some(sd) = f.as_any().downcast_ref::<SpringDamper>() else { continue };
        if sd.body_a != body_idx && sd.body_b != body_idx { continue; }
        out.push((HandleRef::Spring { idx, end: HandleEnd::A }, b[sd.body_a].world_point(sd.local_a)));
        out.push((HandleRef::Spring { idx, end: HandleEnd::B }, b[sd.body_b].world_point(sd.local_b)));
    }
    for (idx, t) in world.tracers.iter().enumerate() {
        if t.body == body_idx {
            out.push((HandleRef::Tracer { idx }, b[t.body].world_point(t.local)));
        }
    }
    out
}

/// Move a connection point to world position `p`. Distance rods take the
/// new length; springs keep their rest length (it has its own slider).
pub fn move_handle(world: &mut World, handle: HandleRef, p: Vec2) {
    let bodies = &world.bodies;
    match handle {
        HandleRef::Constraint { idx, end } => {
            let Some(c) = world.constraints.get_mut(idx) else { return };
            let any = c.as_any_mut();
            if let Some(pj) = any.downcast_mut::<PinJoint>() {
                pj.local_a = world_to_local(&bodies[pj.body_a], p);
                pj.local_b = world_to_local(&bodies[pj.body_b], p);
            } else if let Some(wj) = any.downcast_mut::<WeldJoint>() {
                wj.local_a = world_to_local(&bodies[wj.body_a], p);
                wj.local_b = world_to_local(&bodies[wj.body_b], p);
            } else if let Some(pw) = any.downcast_mut::<PinWorld>() {
                pw.local = world_to_local(&bodies[pw.body], p);
                pw.target = p;
            } else if let Some(sj) = any.downcast_mut::<SliderJoint>() {
                // The attachment stays on the rail: move it along, not off.
                sj.local = world_to_local(&bodies[sj.rider], on_rail(&bodies[sj.rail], p));
            } else if let Some(dc) = any.downcast_mut::<DistanceConstraint>() {
                match end {
                    HandleEnd::A => dc.local_a = world_to_local(&bodies[dc.body_a], p),
                    _            => dc.local_b = world_to_local(&bodies[dc.body_b], p),
                }
                let pa = bodies[dc.body_a].world_point(dc.local_a);
                let pb = bodies[dc.body_b].world_point(dc.local_b);
                dc.rest_len = (pb - pa).length();
            } else if let Some(r) = any.downcast_mut::<Rope>() {
                // Like a distance rod, the rope stays taut at its new length.
                match end {
                    HandleEnd::A => r.local_a = world_to_local(&bodies[r.body_a], p),
                    _            => r.local_b = world_to_local(&bodies[r.body_b], p),
                }
                r.length = r.current_length(bodies) as f32;
                r.rebase(bodies);
            }
        }
        HandleRef::Tracer { idx } => {
            let Some(t) = world.tracers.get_mut(idx) else { return };
            t.local = world_to_local(&bodies[t.body], p);
        }
        HandleRef::Spring { idx, end } => {
            let Some(f) = world.forces.get_mut(idx) else { return };
            if let Some(t) = f.as_any_mut().downcast_mut::<TorsionSpring>() {
                t.local_a = world_to_local(&bodies[t.body_a], p);
                return;
            }
            let Some(sd) = f.as_any_mut().downcast_mut::<SpringDamper>() else { return };
            match end {
                HandleEnd::A => sd.local_a = world_to_local(&bodies[sd.body_a], p),
                _            => sd.local_b = world_to_local(&bodies[sd.body_b], p),
            }
        }
    }
}

// ── Snapping ──────────────────────────────────────────────────────────────────

/// Snap `raw` to the nearest feature within `radius` (world units). Point
/// features (anchors, world pins, disk centres, rod ends and midpoints) win
/// over the continuous disk edge. `exclude` skips one body, e.g. the one
/// being dragged.
pub fn snap_point(world: &World, raw: Vec2, radius: f32, exclude: Option<usize>) -> Option<Vec2> {
    let mut points: Vec<Vec2> = Vec::new();
    let mut edge: Option<(f32, Vec2)> = None;

    for (i, b) in world.bodies.iter().enumerate() {
        if Some(i) == exclude { continue; }
        match b.shape {
            BodyShape::Point => points.push(b.pos32()),
            BodyShape::Disk { radius: r } => {
                points.push(b.pos32());
                let d = raw - b.pos32();
                if d.length() > 1e-6 {
                    let on_edge = b.pos32() + d.normalize() * r;
                    let dist = (on_edge - raw).length();
                    if edge.is_none_or(|(best, _)| dist < best) {
                        edge = Some((dist, on_edge));
                    }
                }
            }
            BodyShape::Rod { half_len, .. } => {
                let (s, c) = b.angle32().sin_cos();
                let u = Vec2::new(c, s) * half_len;
                points.extend([b.pos32() - u, b.pos32(), b.pos32() + u]);
            }
        }
    }
    for c in &world.constraints {
        if let Some(pw) = c.as_any().downcast_ref::<PinWorld>() {
            points.push(pw.target);
        }
    }

    let nearest_point = points
        .into_iter()
        .map(|p| ((p - raw).length(), p))
        .filter(|(d, _)| *d <= radius)
        .min_by(|a, b| a.0.total_cmp(&b.0));
    nearest_point
        .or(edge.filter(|(d, _)| *d <= radius))
        .map(|(_, p)| p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::body::rod_inertia;

    fn rod(pos: Vec2, hl: f32) -> Body {
        Body::new(pos, 0.3, 1.0, rod_inertia(1.0, hl), BodyShape::Rod { half_len: hl, half_width: 0.06 })
    }

    /// Dragging a rod's −x end: the +x end and the pin on it stay put in
    /// the world, and a pin on the −x end stays on that end.
    #[test]
    fn resizing_rod_keeps_attachments_on_ends() {
        let mut w = World::new();
        let a = w.add_body(rod(Vec2::ZERO, 1.0));
        let b = w.add_body(rod(Vec2::new(3.0, 0.0), 0.5));
        w.add_constraint(PinJoint::new(a, Vec2::new(1.0, 0.0), b, Vec2::new(-0.5, 0.0)));
        w.add_constraint(PinWorld::new(a, Vec2::new(-1.0, 0.0), Vec2::ZERO));
        w.add_tracer(a, Vec2::new(0.5, 0.0));
        let plus_end = w.bodies[a].world_point(Vec2::new(1.0, 0.0));

        let u = Vec2::new(0.3f32.cos(), 0.3f32.sin());
        let cursor = plus_end - u * (3.0 + LEN_GAP); // make it 3 long
        resize_to(&mut w, a, ResizeKind::Length { end: -1 }, cursor, false);

        let BodyShape::Rod { half_len, .. } = w.bodies[a].shape else { panic!() };
        assert!((half_len - 1.5).abs() < 1e-4, "half_len {half_len}");
        let pj = w.constraints[0].as_any().downcast_ref::<PinJoint>().unwrap();
        let pw = w.constraints[1].as_any().downcast_ref::<PinWorld>().unwrap();
        assert!((pj.local_a - Vec2::new(1.5, 0.0)).length() < 1e-4);
        assert!((pw.local - Vec2::new(-1.5, 0.0)).length() < 1e-4);
        assert!((w.tracers[0].local - Vec2::new(0.75, 0.0)).length() < 1e-4);
        let still = w.bodies[a].world_point(pj.local_a);
        assert!((still - plus_end).length() < 1e-4, "fixed end moved: {still} vs {plus_end}");
    }

    /// Quick pins toggle on and off at a rod's ends, named by position.
    #[test]
    fn quick_pins_toggle() {
        let mut w = World::new();
        let r = w.add_body(Body::new(Vec2::ZERO, std::f32::consts::PI, 1.0, 1.0, BodyShape::Rod { half_len: 1.0, half_width: 0.06 }));
        let spots = quick_pin_points(&w.bodies[r]);
        // Turned half round: the local +x end is now on the left.
        assert_eq!(spots[0].0, "PIN LEFT");
        assert!((spots[0].1 - Vec2::new(1.0, 0.0)).length() < 1e-6);
        toggle_world_pin(&mut w, r, spots[0].1);
        let pw = w.constraints[0].as_any().downcast_ref::<PinWorld>().unwrap();
        assert!((pw.target - Vec2::new(-1.0, 0.0)).length() < 1e-5);
        assert!(world_pin_at(&w, r, spots[0].1).is_some());
        toggle_world_pin(&mut w, r, spots[0].1);
        assert!(w.constraints.is_empty());
    }

    /// World pins follow their body through editor moves, turns and resizes.
    #[test]
    fn world_pins_follow_editor_moves() {
        let mut w = World::new();
        let r = w.add_body(rod(Vec2::ZERO, 1.0));
        let end = quick_pin_points(&w.bodies[r])[1].1;
        toggle_world_pin(&mut w, r, end);
        let pin = |w: &World| w.constraints[0].as_any().downcast_ref::<PinWorld>().unwrap().clone();
        let on_end = |w: &World| {
            let p = pin(w);
            let BodyShape::Rod { half_len, .. } = w.bodies[r].shape else { return false };
            (p.local - Vec2::new(half_len, 0.0)).length() < 1e-5
                && (p.target - w.bodies[r].world_point(p.local)).length() < 1e-5
        };
        assert!((end - Vec2::new(1.0, 0.0)).length() < 1e-6);

        w.bodies[r].pos += glam::DVec2::new(2.0, 1.0);
        sync_world_pins(&mut w, r);
        assert!(on_end(&w), "drag");
        set_angle_degrees(&mut w.bodies[r], 70.0);
        sync_world_pins(&mut w, r);
        assert!(on_end(&w), "rotate");
        let mut edit = BodyEdit::from_body(&w.bodies[r]);
        edit.half_len = 1.6;
        edit.apply_to(&mut w, r);
        assert!(on_end(&w), "resize by slider");
        assert!(w.constraint_error() < 1e-5);
    }

    /// A pin on a disk's rim stays on the rim when the radius changes.
    #[test]
    fn resizing_disk_scales_rim_attachments() {
        let mut w = World::new();
        let d = w.add_body(Body::new(Vec2::ZERO, 0.0, 1.0, 1.0, BodyShape::Disk { radius: 0.5 }));
        let r = w.add_body(rod(Vec2::new(2.0, 0.0), 0.5));
        w.add_constraint(PinJoint::new(d, Vec2::new(0.0, 0.5), r, Vec2::ZERO));
        set_shape(&mut w, d, BodyShape::Disk { radius: 1.0 });
        let pj = w.constraints[0].as_any().downcast_ref::<PinJoint>().unwrap();
        assert!((pj.local_a - Vec2::new(0.0, 1.0)).length() < 1e-6);
        assert!((pj.local_b - Vec2::ZERO).length() < 1e-6, "the other body's point is untouched");
    }
}
