use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use glam::Vec2;
use winit::{
    event::{ElementState, KeyEvent, MouseButton, MouseScrollDelta, WindowEvent},
    keyboard::{KeyCode, PhysicalKey},
    window::Window,
};

use crate::analysis::{Analysis, Butterfly, UndoStack};
use crate::scene_file::SceneFile;
use crate::editor::{
    connection_handles, move_handle, snap_point, BodyEdit, ConstraintKind, Editor, EditorMode,
    HandleRef, LinkHover, StepMode,
};
use crate::sim::forces::Gravity;
use crate::renderer::{
    camera::Camera,
    geometry::GeometryBuilder,
    hud::{Hud, HudOutput, Metrics, UndoState},
    state::RenderState,
};
use crate::scenes::SCENES;
use crate::sim::{
    body::BodyShape,
    forces::{MouseSpring, MouseSpringData},
    world::World,
};

/// Upper bound on automatically chosen steps per frame. Only a runaway
/// guard: the frame-time budget is what normally limits the count.
const MAX_AUTO_STEPS: f32 = 1_000_000.0;
/// Trace samples per second of trail (one sample per rendered frame).
const TRACE_FPS: f32 = 60.0;

/// Overlay scale that follows the largest value on show: up at once (so
/// nothing overflows), down slowly (so it doesn't flicker).
fn smooth_scale(current: f32, max: f32) -> f32 {
    if !max.is_finite() || max <= 0.0 { return current; }
    if max > current { max } else { (current * 0.98 + max * 0.02).max(1e-3) }
}

/// Zoom (world units per pixel) that fits `view` (world w × h) in `screen`.
fn fit_scale(view: [f32; 2], screen: Vec2) -> f32 {
    (view[0] / screen.x.max(1.0)).max(view[1] / screen.y.max(1.0))
}

pub struct App {
    pub render_state: RenderState,
    pub camera:       Camera,
    pub geo:          GeometryBuilder,
    pub world:        World,
    pub hud:          Hud,
    pub editor:       Editor,

    window: &'static Window,

    mouse_spring:       Arc<Mutex<MouseSpringData>>,
    picked:             Option<(usize, Vec2)>,
    current_scene_idx:  usize,
    /// Smoothed wall time of one physics step (seconds), for target-fps mode.
    step_cost:          f32,
    /// Wall time the last frame spent in physics, and the smoothed time each
    /// frame spends on everything else (drawing, present, events).
    physics_time:       f32,
    other_cost:         f32,
    /// Recent path of each `world.tracers` entry, oldest first.
    traces:             Vec<VecDeque<Vec2>>,
    /// Height that decides anchor mount direction; frozen while simulating.
    anchor_ref_y:       Option<f32>,

    mouse_pos:      Vec2,
    middle_pressed: bool,
    last_mouse_pos: Vec2,
    ctrl_held:      bool,
    shift_held:     bool,
    /// Body pressed in editor but not yet dragged (becomes Dragging only after movement)
    drag_candidate: Option<usize>,

    last_frame:  web_time::Instant,
    /// The camera still shows the scene's own framing (no pan or zoom since
    /// it loaded), so window resizes refit it. This also covers start-up,
    /// where the real window size often arrives after creation.
    camera_fitted: bool,
    fps_smooth:  f32,
    pub window_size: winit::dpi::PhysicalSize<u32>,
    pub frame_count: u64,

    /// Rewind, butterfly, phase plot, overlays, share panel.
    analysis: Analysis,
    undo:     UndoStack,
    /// Left button held in the viewport (an edit in progress).
    left_down: bool,
    /// Smoothed largest force and rod load on show, for overlay scaling.
    force_scale: f32,
    load_scale:  f32,
    /// A world loaded from a share code (JSON, so RESET reloads it) and the
    /// view it was shared with, which replaces the scene's for fitting.
    custom: Option<String>,
    custom_view: Option<[f32; 2]>,
    #[cfg(target_arch = "wasm32")]
    last_hash: String,
}

impl App {
    pub async fn new(window: &'static Window) -> Self {
        let render_state = RenderState::new(window).await;
        let hud = Hud::new(window, &render_state.device, render_state.view_format);

        let shared = Arc::new(Mutex::new(MouseSpringData::default()));
        let world = Self::build_scene(0, shared.clone());
        let anchor_ref_y = crate::scenes::free_com_y(&world);

        let def = &SCENES[0];
        let camera = {
            let mut c = Camera::new();
            c.center = Vec2::from(def.camera_center);
            let size = window.inner_size();
            c.scale  = fit_scale(def.view_size, Vec2::new(size.width as f32, size.height as f32));
            c
        };

        let mut app = Self {
            render_state,
            camera,
            geo: GeometryBuilder::new(),
            world,
            hud,
            editor: Editor::default(),
            window,
            mouse_spring: shared,
            picked: None,
            current_scene_idx: 0,
            anchor_ref_y,
            traces: Vec::new(),
            step_cost: 0.0,
            physics_time: 0.0,
            other_cost: 0.0,
            mouse_pos: Vec2::ZERO,
            middle_pressed: false,
            last_mouse_pos: Vec2::ZERO,
            shift_held: false,
            ctrl_held: false,
            drag_candidate: None,
            last_frame: web_time::Instant::now(),
            camera_fitted: true,
            fps_smooth: 60.0,
            window_size: window.inner_size(),
            frame_count: 0,
            analysis: Analysis::default(),
            undo: UndoStack::default(),
            left_down: false,
            force_scale: 1.0,
            load_scale: 1.0,
            custom: None,
            custom_view: None,
            #[cfg(target_arch = "wasm32")]
            last_hash: String::new(),
        };
        app.undo.reset(&app.world);
        #[cfg(target_arch = "wasm32")]
        app.check_url_hash();
        app
    }

    /// Swap in a whole new world (undo, redo, a loaded share code), keeping
    /// the app's mouse spring and solver choice, and dropping everything
    /// tied to the old one.
    fn replace_world(&mut self, mut w: World) {
        w.add_force(MouseSpring(self.mouse_spring.clone()));
        w.integrator = self.editor.integrator;
        self.world = w;
        self.mouse_spring.lock().unwrap().active = false;
        self.picked = None;
        self.drag_candidate = None;
        self.traces.clear();
        self.analysis.timeline.clear();
        self.analysis.butterfly = None;
        self.analysis.phase.clear();
        self.editor.mode = EditorMode::Idle;
        self.editor.body_edit = None;
        self.editor.inspect_shape = None;
        self.anchor_ref_y = crate::scenes::free_com_y(&self.world);
        if let Some(g) = self.world.forces.iter().find_map(|f| f.as_any().downcast_ref::<Gravity>().map(|g| g.g)) {
            self.editor.gravity = g;
        }
    }

    /// The current world as a share code, with the camera's view.
    fn share_code(&self) -> String {
        let mut file = SceneFile::from_world(&self.world);
        let screen = self.screen_size() * self.camera.scale;
        file.view = Some([self.camera.center.x, self.camera.center.y, screen.x, screen.y]);
        file.to_code()
    }

    /// Load a share code (or link). False if it doesn't decode.
    fn load_code(&mut self, code: &str) -> bool {
        let Some(file) = SceneFile::from_code(code) else { return false };
        self.replace_world(file.to_world());
        if let Some([cx, cy, w, h]) = file.view {
            self.camera.center = Vec2::new(cx, cy);
            self.custom_view = Some([w, h]);
            self.camera.scale = fit_scale([w, h], self.screen_size());
            self.camera_fitted = true;
        }
        self.custom = Some(file.to_json());
        self.undo.reset(&self.world);
        self.editor.active = false;
        self.editor.paused = false;
        true
    }

    /// In the browser a share link carries the scene in `#s=…`; load it at
    /// start and whenever the hash changes (a pasted link).
    #[cfg(target_arch = "wasm32")]
    fn check_url_hash(&mut self) {
        let Some(hash) = web_sys::window().and_then(|w| w.location().hash().ok()) else { return };
        if hash == self.last_hash { return; }
        self.last_hash = hash.clone();
        if let Some(code) = hash.strip_prefix("#s=") {
            if !self.load_code(code) { log::warn!("share link did not decode"); }
        }
    }

    /// Publish the share code: clipboard natively, URL hash (and clipboard,
    /// if the browser allows) on the web.
    fn share(&mut self) {
        let code = self.share_code();
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.hud.copy_text(code.clone());
            self.analysis.share_note = format!("Copied // {} chars", code.len());
            self.analysis.share_code = code;
        }
        #[cfg(target_arch = "wasm32")]
        {
            let Some(win) = web_sys::window() else { return };
            let loc = win.location();
            let hash = format!("#s={code}");
            let _ = loc.set_hash(&hash);
            self.last_hash = hash;
            let base = loc.href().ok().map(|h| h.split('#').next().unwrap_or("").to_owned()).unwrap_or_default();
            let link = format!("{base}#s={code}");
            let _ = win.navigator().clipboard().write_text(&link);
            self.analysis.share_note = "Link in the address bar and clipboard".to_owned();
            self.analysis.share_code = link;
        }
    }

    fn do_undo(&mut self, redo: bool) {
        let w = if redo { self.undo.redo() } else { self.undo.undo() };
        if let Some(w) = w { self.replace_world(w); }
    }

    /// Enter/leave edit mode. Editor moves teleport bodies, so constraints
    /// with reference state (rolling) re-capture it before physics resumes.
    fn toggle_editor(&mut self) {
        self.editor.toggle();
        // The ghost can't follow edits.
        self.analysis.butterfly = None;
        if self.editor.active {
            self.undo.commit(&self.world);
        } else {
            self.world.rebase();
            self.traces.clear();
            self.analysis.timeline.clear();
            self.analysis.butterfly = None;
            self.analysis.phase.clear();
        }
    }

    /// While the simulation runs in edit mode, a body being dragged, turned
    /// or resized stays where the editor puts it instead of flying off.
    fn hold_grabbed_body(&mut self) {
        let idx = match self.editor.mode {
            EditorMode::Dragging { body_idx }
            | EditorMode::Rotating { body_idx, .. }
            | EditorMode::Resizing { body_idx, .. } => body_idx,
            _ => return,
        };
        if idx >= self.world.bodies.len() { return; }
        if let EditorMode::Dragging { .. } = self.editor.mode {
            let wp = self.editor_point(Some(idx));
            self.world.bodies[idx].pos = wp.as_dvec2();
        }
        let b = &mut self.world.bodies[idx];
        b.vel = glam::DVec2::ZERO;
        b.ang_vel = 0.0;
    }

    fn record_traces(&mut self) {
        let tracers = &self.world.tracers;
        if self.traces.len() != tracers.len() {
            self.traces = vec![VecDeque::new(); tracers.len()];
        }
        for (trail, t) in self.traces.iter_mut().zip(tracers) {
            let Some(body) = self.world.bodies.get(t.body) else { continue };
            let cap = ((t.seconds * TRACE_FPS) as usize).max(2);
            while trail.len() >= cap { trail.pop_front(); }
            trail.push_back(body.world_point(t.local));
        }
    }

    /// Thin red trails, fading out towards their oldest end.
    fn draw_traces(&mut self) {
        let width = 1.6 * self.camera.scale;
        for trail in &self.traces {
            let n = trail.len();
            for (i, (a, b)) in trail.iter().zip(trail.iter().skip(1)).enumerate() {
                // Squared ramp: bright near the head, gone well before the tail.
                let f = (i + 1) as f32 / n as f32;
                let alpha = 0.85 * f * f;
                self.geo.draw_line(*a, *b, width, [0.90, 0.27, 0.22, alpha]);
            }
        }
    }

    fn build_scene(idx: usize, shared: Arc<Mutex<MouseSpringData>>) -> World {
        let mut world = (SCENES[idx].build)();
        world.add_force(MouseSpring(shared));
        world
    }

    fn load_scene(&mut self, idx: usize) {
        let switching = idx != self.current_scene_idx;
        self.current_scene_idx = idx;
        let def = &SCENES[idx];
        self.camera.center = Vec2::from(def.camera_center);
        self.camera_fitted = true;
        self.camera.scale  = fit_scale(def.view_size, self.screen_size());
        self.traces.clear();
        self.mouse_spring.lock().unwrap().active = false;
        self.picked = None;
        self.drag_candidate = None;
        self.world = Self::build_scene(idx, self.mouse_spring.clone());
        self.anchor_ref_y = crate::scenes::free_com_y(&self.world);
        self.custom = None;
        self.custom_view = None;
        self.analysis.timeline.clear();
        self.analysis.butterfly = None;
        self.analysis.phase.clear();
        self.undo.reset(&self.world);
        self.editor.mode = EditorMode::Idle;
        self.editor.body_edit = None;
        self.editor.inspect_shape = None;
        self.editor.pending_constraint = None;
        // Sync gravity slider from world
        for f in &self.world.forces {
            if let Some(grav) = f.as_any().downcast_ref::<Gravity>() {
                self.editor.gravity = grav.g;
                break;
            }
        }
        // The sandbox opens in edit mode; switching to any other scene
        // returns to simulating. A reset keeps whichever mode (and pause
        // state) you were in.
        let sandbox = idx == SCENES.len() - 1;
        if sandbox || switching {
            self.editor.active = sandbox;
            self.editor.paused = sandbox;
        }
    }

    /// Enter Inspecting mode for a body and populate the editor's body_edit fields.
    fn inspect_body(&mut self, body_idx: usize) {
        if body_idx >= self.world.bodies.len() { return; }
        let body = &self.world.bodies[body_idx];
        let shape_tag = match body.shape {
            BodyShape::Disk { .. }  => "Disk",
            BodyShape::Rod  { .. }  => "Rod",
            BodyShape::Point        => "Point",
        }.to_string();
        self.editor.mode          = EditorMode::Inspecting { body_idx };
        self.editor.body_edit     = Some(BodyEdit::from_body(body));
        self.editor.inspect_shape = Some(shape_tag);
    }

    fn screen_size(&self) -> Vec2 {
        Vec2::new(self.window_size.width as f32, self.window_size.height as f32)
    }

    fn world_mouse(&self) -> Vec2 {
        self.camera.screen_to_world(self.mouse_pos, self.screen_size())
    }

    fn pick_body(&self, world_pos: Vec2) -> Option<(usize, Vec2)> {
        const GRAB_RADIUS: f32 = 0.5;
        let mut best_dist = GRAB_RADIUS;
        let mut result = None;

        for (i, body) in self.world.bodies.iter().enumerate() {
            if body.fixed && !self.editor.active { continue; }
            let r = world_pos - body.pos32();
            let (s, c) = body.angle32().sin_cos();
            let lx =  c * r.x + s * r.y;
            let ly = -s * r.x + c * r.y;

            let (dist, local) = match &body.shape {
                BodyShape::Disk { radius } => {
                    let d = r.length() - radius;
                    let local = if r.length() < 1e-6 {
                        Vec2::ZERO
                    } else {
                        Vec2::new(lx, ly).clamp_length_max(*radius)
                    };
                    (d, local)
                }
                BodyShape::Rod { half_len, half_width } => {
                    let cx = lx.clamp(-half_len, *half_len);
                    let dist = ((lx - cx).powi(2) + ly.powi(2)).sqrt() - half_width;
                    (dist, Vec2::new(cx, 0.0))
                }
                BodyShape::Point => {
                    let d = r.length() - 0.15;
                    (d, Vec2::ZERO)
                }
            };

            if dist < best_dist {
                best_dist = dist;
                result = Some((i, local));
            }
        }
        result
    }

    /// Snap radius: a fixed number of screen pixels, whatever the zoom.
    fn snap_radius(&self) -> f32 { 14.0 * self.camera.scale }

    /// Where an editor action at the cursor lands: the snapped feature when
    /// Ctrl is held and one is in range, otherwise the raw cursor.
    fn editor_point(&self, exclude: Option<usize>) -> Vec2 {
        let raw = self.world_mouse();
        if !self.ctrl_held { return raw; }
        snap_point(&self.world, raw, self.snap_radius(), exclude).unwrap_or(raw)
    }

    /// Is the cursor on the selected body's rotate handle?
    fn on_rotate_handle(&self, body_idx: usize, raw: Vec2) -> bool {
        let reach = 10.0 * self.camera.scale;
        self.world.bodies.get(body_idx)
            .and_then(crate::editor::rotate_handle)
            .is_some_and(|p| (p - raw).length() <= reach)
    }

    /// Resize handle of the selected body under the cursor, if any.
    fn resize_handle_at(&self, body_idx: usize, raw: Vec2) -> Option<crate::editor::ResizeKind> {
        let reach = 10.0 * self.camera.scale;
        let b = self.world.bodies.get(body_idx)?;
        crate::editor::resize_handles(b).into_iter()
            .find(|(_, p)| (*p - raw).length() <= reach)
            .map(|(k, _)| k)
    }

    /// Connection handle of the selected body under the cursor, if any.
    fn handle_at(&self, body_idx: usize, raw: Vec2) -> Option<HandleRef> {
        let reach = 10.0 * self.camera.scale;
        connection_handles(&self.world, body_idx)
            .into_iter()
            .map(|(h, p)| (h, (p - raw).length()))
            .filter(|(_, d)| *d <= reach)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(h, _)| h)
    }

    /// Handle a left-click in editor mode. Bodies are picked at the raw
    /// cursor `raw`; positions (placement, attach points, anchors) use `at`,
    /// which is the Ctrl-snapped point when snapping.
    fn editor_click(&mut self, raw: Vec2, at: Vec2) {
        match self.editor.mode.clone() {
            EditorMode::PlacingBody => {
                let idx = self.editor.place_body(at, &mut self.world);
                self.inspect_body(idx);
            }

            EditorMode::Idle | EditorMode::Inspecting { .. } => {
                if self.shift_held {
                    if let Some((idx, _local)) = self.pick_body(raw) {
                        self.editor.mode = EditorMode::FirstSelected {
                            body_idx:  idx,
                            world_pos: at,
                        };
                        self.editor.body_edit = None;
                        self.editor.inspect_shape = None;
                    }
                } else if let Some((idx, _local)) = self.pick_body(raw) {
                    self.inspect_body(idx);
                } else {
                    self.editor.mode = EditorMode::Idle;
                    self.editor.body_edit = None;
                    self.editor.inspect_shape = None;
                }
            }

            EditorMode::FirstSelected { body_idx: first, world_pos: attach_a } => {
                if let Some((second, _)) = self.pick_body(raw) {
                    if second != first {
                        let (fa, fb) = (&self.world.bodies[first], &self.world.bodies[second]);
                        self.editor.pair_can_roll = crate::editor::can_roll(fa, fb);
                        self.editor.pair_can_slide = crate::editor::can_slide(fa, fb);
                        self.editor.pair_can_gear = crate::editor::can_gear(fa, fb);
                        self.editor.mode = EditorMode::BothSelected {
                            body_a:   first,
                            attach_a,
                            body_b:   second,
                            attach_b: at,
                        };
                    } else {
                        // Clicked same body — deselect
                        self.editor.mode = EditorMode::Idle;
                    }
                } else {
                    self.editor.mode = EditorMode::Idle;
                }
            }

            EditorMode::BothSelected { .. } => {
                // Clicks in viewport ignored while picker is open
            }

            EditorMode::PinWorldPending { body_idx, attach } => {
                let k = self.editor.spring_k;
                let d = self.editor.spring_d;
                self.editor.add_constraint(
                    ConstraintKind::PinToWorld,
                    body_idx, attach,
                    None, at,
                    &mut self.world,
                    k, d,
                );
                self.editor.mode = EditorMode::Idle;
            }

            EditorMode::PulleyPending { body_a, attach_a, body_b, attach_b } => {
                if let Some((p, _)) = self.pick_body(raw) {
                    if crate::editor::add_rope_over(body_a, attach_a, body_b, attach_b, p, &mut self.world) {
                        self.editor.mode = EditorMode::Idle;
                    }
                } else {
                    self.editor.mode = EditorMode::Idle;
                }
            }

            EditorMode::PlacingTrace { body_idx } => {
                // The point may lie off the body: it rides along rigidly.
                if let Some(body) = self.world.bodies.get(body_idx) {
                    let local = crate::editor::world_to_local(body, at);
                    self.world.add_tracer(body_idx, local);
                }
                self.inspect_body(body_idx);
            }

            EditorMode::Dragging { .. } | EditorMode::DraggingHandle { .. }
            | EditorMode::Rotating { .. } | EditorMode::Resizing { .. } => {}
        }
    }

    pub fn handle_event(&mut self, event: &WindowEvent) -> bool {
        // A release over the HUD still ends a press that began in the scene.
        if let WindowEvent::MouseInput { state: ElementState::Released, button: MouseButton::Left, .. } = event {
            self.left_down = false;
        }
        if self.hud.on_window_event(self.window, event) {
            return true;
        }

        match event {
            WindowEvent::Resized(size) => {
                self.render_state.resize(*size);
                self.window_size = *size;
                if self.camera_fitted && size.width > 0 && size.height > 0 {
                    let view = self.custom_view.unwrap_or(SCENES[self.current_scene_idx].view_size);
                    self.camera.scale = fit_scale(view, self.screen_size());
                }
            }
            WindowEvent::KeyboardInput { event: KeyEvent { physical_key, state, .. }, .. } => {
                if *state == ElementState::Pressed {
                    match physical_key {
                        PhysicalKey::Code(KeyCode::KeyE) => {
                            self.toggle_editor();
                        }
                        PhysicalKey::Code(KeyCode::Delete) | PhysicalKey::Code(KeyCode::Backspace) => {
                            if self.editor.active {
                                if let EditorMode::Inspecting { body_idx } = self.editor.mode {
                                    self.editor.delete_body(body_idx, &mut self.world);
                                    self.editor.mode = EditorMode::Idle;
                                }
                            }
                        }
                        PhysicalKey::Code(KeyCode::Space) => {
                            self.editor.paused = !self.editor.paused;
                        }
                        PhysicalKey::Code(KeyCode::KeyR) => {
                            if self.editor.active {
                                if let EditorMode::Inspecting { body_idx } = self.editor.mode {
                                    let step = if self.shift_held { -15.0f64 } else { 15.0 };
                                    if let Some(b) = self.world.bodies.get_mut(body_idx) {
                                        b.angle += step.to_radians();
                                    }
                                    self.inspect_body(body_idx);
                                }
                            }
                        }
                        PhysicalKey::Code(KeyCode::KeyZ) if self.editor.active && self.ctrl_held => {
                            self.do_undo(self.shift_held);
                        }
                        PhysicalKey::Code(KeyCode::KeyY) if self.editor.active && self.ctrl_held => {
                            self.do_undo(true);
                        }
                        PhysicalKey::Code(KeyCode::Escape) => {
                            if self.editor.active {
                                self.editor.mode = EditorMode::Idle;
                            }
                        }
                        _ => {}
                    }
                }
                match physical_key {
                    PhysicalKey::Code(KeyCode::ShiftLeft) | PhysicalKey::Code(KeyCode::ShiftRight) => {
                        self.shift_held = *state == ElementState::Pressed;
                    }
                    PhysicalKey::Code(KeyCode::ControlLeft) | PhysicalKey::Code(KeyCode::ControlRight) => {
                        self.ctrl_held = *state == ElementState::Pressed;
                    }
                    _ => {}
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let new_pos = Vec2::new(position.x as f32, position.y as f32);
                let delta = new_pos - self.last_mouse_pos;
                if self.middle_pressed {
                    self.camera.pan(delta);
                    self.camera_fitted = false;
                }
                self.last_mouse_pos = new_pos;
                self.mouse_pos      = new_pos;

                if self.editor.active {
                    // Promote drag candidate to actual drag once mouse moves > 4 px
                    if let Some(cand_idx) = self.drag_candidate {
                        if delta.length() > 4.0 || matches!(self.editor.mode, EditorMode::Dragging { .. }) {
                            self.editor.mode = EditorMode::Dragging { body_idx: cand_idx };
                        }
                    }
                    match self.editor.mode {
                        EditorMode::Dragging { body_idx } if body_idx < self.world.bodies.len() => {
                            // Snap the body's centre, never onto its own features.
                            let wp = self.editor_point(Some(body_idx));
                            self.world.bodies[body_idx].pos = wp.as_dvec2();
                        }
                        EditorMode::DraggingHandle { handle, .. } => {
                            let wp = self.editor_point(None);
                            move_handle(&mut self.world, handle, wp);
                        }
                        EditorMode::Resizing { body_idx, kind } if body_idx < self.world.bodies.len() => {
                            let wp = self.world_mouse();
                            let snap = self.ctrl_held;
                            crate::editor::resize_to(&mut self.world, body_idx, kind, wp, snap);
                        }
                        EditorMode::Rotating { body_idx, offset } if body_idx < self.world.bodies.len() => {
                            let d = self.world_mouse() - self.world.bodies[body_idx].pos32();
                            let mut deg = (d.y.atan2(d.x) + offset).to_degrees();
                            if self.ctrl_held { deg = (deg / 15.0).round() * 15.0; }
                            crate::editor::set_angle_degrees(&mut self.world.bodies[body_idx], deg);
                        }
                        _ => {}
                    }
                } else if self.picked.is_some() {
                    self.mouse_spring.lock().unwrap().target = self.world_mouse();
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                match button {
                    MouseButton::Middle => {
                        self.middle_pressed = *state == ElementState::Pressed;
                    }
                    MouseButton::Left => {
                        self.left_down = *state == ElementState::Pressed;
                        let wp = self.world_mouse();
                        if *state == ElementState::Pressed {
                            if self.editor.active {
                                // Connection handles of the selected body take priority.
                                if let EditorMode::Inspecting { body_idx } = self.editor.mode {
                                    if self.on_rotate_handle(body_idx, wp) {
                                        let b = &self.world.bodies[body_idx];
                                        let d = wp - b.pos32();
                                        let offset = b.angle32() - d.y.atan2(d.x);
                                        self.editor.mode = EditorMode::Rotating { body_idx, offset };
                                        return false;
                                    }
                                    if let Some(kind) = self.resize_handle_at(body_idx, wp) {
                                        self.editor.mode = EditorMode::Resizing { body_idx, kind };
                                        return false;
                                    }
                                    if let Some(handle) = self.handle_at(body_idx, wp) {
                                        self.editor.mode = EditorMode::DraggingHandle { body_idx, handle };
                                        return false;
                                    }
                                }
                                // In body-moveable modes without shift: record a drag candidate.
                                // The candidate becomes a real drag on movement, or a click on release.
                                if !self.shift_held
                                    && !matches!(self.editor.mode,
                                        EditorMode::PlacingBody
                                        | EditorMode::PlacingTrace { .. }
                                        | EditorMode::FirstSelected { .. }
                                        | EditorMode::BothSelected { .. }
                                        | EditorMode::PulleyPending { .. }
                                        | EditorMode::PinWorldPending { .. })
                                {
                                    if let Some((idx, _)) = self.pick_body(wp) {
                                        self.drag_candidate = Some(idx);
                                        return false; // wait to see if it's a drag or click
                                    }
                                }
                                let at = self.editor_point(None);
                                self.editor_click(wp, at);
                            } else if let Some((idx, local)) = self.pick_body(wp) {
                                self.picked = Some((idx, local));
                                let mut ms = self.mouse_spring.lock().unwrap();
                                ms.active       = true;
                                ms.body_idx     = idx;
                                ms.local_attach = local;
                                ms.target       = wp;
                            }
                        } else {
                            // Mouse released
                            if self.editor.active {
                                if let Some(cand) = self.drag_candidate.take() {
                                    self.inspect_body(cand);
                                } else if let EditorMode::Dragging { body_idx }
                                    | EditorMode::DraggingHandle { body_idx, .. }
                                    | EditorMode::Rotating { body_idx, .. }
                                    | EditorMode::Resizing { body_idx, .. } = self.editor.mode
                                {
                                    // Running while editing: the edit teleported
                                    // things, so re-capture constraint references.
                                    if !self.editor.paused { self.world.rebase(); }
                                    self.inspect_body(body_idx);
                                }
                            } else {
                                self.picked = None;
                                self.mouse_spring.lock().unwrap().active = false;
                            }
                        }
                    }
                    _ => {}
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => *y,
                    MouseScrollDelta::PixelDelta(p)   => p.y as f32 * 0.01,
                };
                let factor = if scroll > 0.0 {
                    0.9f32.powf(scroll)
                } else {
                    1.0 / 0.9f32.powf(-scroll)
                };
                self.camera.zoom(factor, self.mouse_pos, self.screen_size());
                self.camera_fitted = false;
            }
            _ => {}
        }
        false
    }

    pub fn update(&mut self) {
        let now = web_time::Instant::now();
        let frame_time = (now - self.last_frame).as_secs_f32();
        let dt  = frame_time.min(0.05);
        self.last_frame = now;

        // The frame that just ended was its physics plus everything else.
        let other = (frame_time - self.physics_time).max(0.0);
        self.other_cost = if self.other_cost > 0.0 { self.other_cost * 0.9 + other * 0.1 } else { other };
        self.physics_time = 0.0;

        if dt > 0.0 {
            self.fps_smooth = self.fps_smooth * 0.92 + (1.0 / dt) * 0.08;
        }

        if !self.editor.paused {
            self.world.integrator = self.editor.integrator;
            let steps = self.editor.sub_steps.max(1);
            let sub_dt = dt * self.editor.time_scale / steps as f32;
            // After a jump in step cost (e.g. a heavier scene) the count is
            // briefly too high; stop at twice the frame budget instead of
            // freezing. That frame's sim time is cut short, as with the dt cap.
            let limit = 2.0 / self.editor.target_fps.max(1.0);
            let guard = self.editor.step_mode == StepMode::TargetFps;
            self.world.record_reactions = self.analysis.show_forces;
            if self.analysis.show_chaos && self.analysis.butterfly.is_none() && !self.editor.active {
                self.analysis.butterfly = Some(Butterfly::seed(&self.world, &self.mouse_spring));
            }
            if let Some(bf) = &mut self.analysis.butterfly {
                bf.ghost.integrator = self.editor.integrator;
                for f in bf.ghost.forces.iter_mut() {
                    if let Some(g) = f.as_any_mut().downcast_mut::<Gravity>() { g.g = self.editor.gravity; }
                }
            }
            let t0 = web_time::Instant::now();
            let mut done = 0;
            while done < steps {
                self.world.step(sub_dt);
                if let Some(bf) = &mut self.analysis.butterfly { bf.ghost.step(sub_dt); }
                if self.analysis.show_phase { self.analysis.phase.after_step(&self.world); }
                done += 1;
                if guard && done % 256 == 0 && t0.elapsed().as_secs_f32() > limit { break; }
            }
            self.physics_time = t0.elapsed().as_secs_f32();
            self.hold_grabbed_body();
            let sim_dt = (sub_dt * done as f32) as f64;
            if let Some(bf) = &mut self.analysis.butterfly { bf.sample(&self.world, sim_dt); }
            if self.analysis.show_phase {
                self.analysis.phase.ensure_bodies(&self.world);
                self.analysis.phase.after_frame(&self.world);
            }
            if !self.editor.active { self.analysis.timeline.record(&self.world, sim_dt); }
            let per_step = self.physics_time / done as f32;
            self.step_cost = if self.step_cost > 0.0 { self.step_cost * 0.9 + per_step * 0.1 } else { per_step };

            if self.editor.step_mode == StepMode::TargetFps && self.step_cost > 0.0 {
                // Physics gets whatever the target frame time leaves after
                // the measured cost of everything else, so the real frame
                // rate settles on the target. Smoothed so the count doesn't jitter.
                let frame = 1.0 / self.editor.target_fps.max(1.0);
                let budget = (frame - self.other_cost).max(0.1 * frame);
                let want = (budget / self.step_cost).clamp(1.0, MAX_AUTO_STEPS);
                let next = self.editor.sub_steps as f32 * 0.85 + want * 0.15;
                self.editor.sub_steps = next.round().max(1.0) as u32;
            }
        }

        if !self.editor.paused {
            self.record_traces();
        }

        self.frame_count += 1;
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let idx = self.current_scene_idx;

        // Build geometry
        self.geo.clear();
        if self.editor.active {
            self.anchor_ref_y = crate::scenes::free_com_y(&self.world);
        }
        let tints = if self.analysis.show_forces {
            let (t, max) = crate::scenes::force_tints(&self.world, self.load_scale);
            self.load_scale = smooth_scale(self.load_scale, max);
            t
        } else {
            Vec::new()
        };
        crate::scenes::draw_world(&self.world, self.anchor_ref_y, &tints, &mut self.geo);
        self.draw_traces();
        if let Some(bf) = &self.analysis.butterfly {
            crate::scenes::draw_ghost(&bf.ghost, &mut self.geo, self.camera.scale);
        }
        if self.analysis.show_forces {
            let max = crate::scenes::draw_force_arrows(&self.world, &mut self.geo, self.camera.scale, self.force_scale);
            self.force_scale = smooth_scale(self.force_scale, max);
        }

        // Editor overlays
        if self.editor.active {
            self.draw_editor_overlay();
        } else if let Some((idx, local)) = self.picked {
            if idx < self.world.bodies.len() {
                let body   = &self.world.bodies[idx];
                let attach = body.world_point(local);
                let cursor = self.world_mouse();
                let orange = [1.0f32, 0.55, 0.05, 1.0];
                self.geo.draw_line(attach, cursor, 0.018, [1.0, 0.55, 0.05, 0.5]);
                self.geo.draw_circle(attach, 0.055, 16, orange);
                self.geo.draw_circle(cursor,  0.040, 16, orange);
            }
        }

        let (output, surface_view, mut encoder) =
            self.render_state.render_geometry(&self.camera, &self.geo)?;

        let ke  = self.world.kinetic_energy();
        let pe  = self.world.potential_energy(self.editor.gravity);
        let err = self.world.constraint_error();
        let metrics = Metrics {
            fps:              self.fps_smooth,
            sub_steps:        self.editor.sub_steps,
            kinetic_energy:   ke,
            potential_energy: pe,
            constraint_error: err,
        };

        let scene_count = SCENES.len();
        let def         = &SCENES[idx];
        let size        = [self.window_size.width, self.window_size.height];
        let (name, desc) = if self.custom.is_some() {
            ("Shared Scene", "Loaded from a share code  ·  RESET reloads it")
        } else {
            (def.name, def.description)
        };
        let undo_state = UndoState { can_undo: self.undo.can_undo(), can_redo: self.undo.can_redo() };

        let hud_out = self.hud.encode(
            self.window,
            &self.render_state.device,
            &self.render_state.queue,
            &mut encoder,
            &surface_view,
            size,
            &metrics,
            idx,
            scene_count,
            name,
            desc,
            &mut self.editor,
            &mut self.world,
            &mut self.analysis,
            undo_state,
        );
        self.handle_analysis(&hud_out);

        // Handle pending constraint from editor panel
        if let Some(kind) = self.editor.pending_constraint.take() {
            if let EditorMode::BothSelected { body_a, attach_a, body_b, attach_b } = self.editor.mode.clone() {
                if kind == ConstraintKind::PinToWorld {
                    // Re-enter pin-world pending mode for the first body
                    self.editor.mode = EditorMode::PinWorldPending {
                        body_idx: body_a,
                        attach:   attach_a,
                    };
                } else if kind == ConstraintKind::RopeOverPulley {
                    self.editor.mode = EditorMode::PulleyPending { body_a, attach_a, body_b, attach_b };
                } else {
                    let k = self.editor.spring_k;
                    let d = self.editor.spring_d;
                    self.editor.add_constraint(
                        kind,
                        body_a, attach_a,
                        Some(body_b), attach_b,
                        &mut self.world,
                        k, d,
                    );
                    self.editor.mode = EditorMode::Idle;
                }
            }
        }

        // Apply live body property edits to the selected body
        if let EditorMode::Inspecting { body_idx } = self.editor.mode {
            if let Some(edit) = &mut self.editor.body_edit {
                if body_idx < self.world.bodies.len() {
                    edit.apply_to(&mut self.world, body_idx);
                }
            }
        }

        // Apply gravity slider to the world's Gravity force
        let target_g = self.editor.gravity;
        for f in &mut self.world.forces {
            if let Some(grav) = f.as_any_mut().downcast_mut::<Gravity>() {
                grav.g = target_g;
            }
        }

        // Handle delete request from editor panel
        if self.editor.delete_requested {
            self.editor.delete_requested = false;
            if let EditorMode::Inspecting { body_idx } = self.editor.mode {
                self.editor.delete_body(body_idx, &mut self.world);
                self.editor.mode = EditorMode::Idle;
                self.editor.body_edit = None;
                self.editor.inspect_shape = None;
            }
        }

        // Process scene navigation
        if hud_out.toggle_edit {
            self.toggle_editor();
        } else if hud_out.reset_scene {
            // Keep the user's gravity across a reset (the rebuilt world starts at 9.81).
            let g = self.editor.gravity;
            match self.custom.take().and_then(|j| SceneFile::from_json(&j)) {
                Some(file) => {
                    self.replace_world(file.to_world());
                    self.custom = Some(file.to_json());
                    self.undo.reset(&self.world);
                }
                None => self.load_scene(idx),
            }
            self.editor.gravity = g;
        } else if hud_out.next_scene {
            let next = (idx + 1) % scene_count;
            self.load_scene(next);
        } else if hud_out.prev_scene {
            let prev = (idx + scene_count - 1) % scene_count;
            self.load_scene(prev);
        }

        // Undo snapshots: whenever a paused edit has settled (no button
        // held in the scene or on a slider), record it if anything changed.
        if self.editor.active && self.editor.paused && !self.left_down && !hud_out.pointer_busy
            && matches!(self.editor.mode, EditorMode::Idle | EditorMode::Inspecting { .. })
        {
            self.undo.commit(&self.world);
        }

        #[cfg(target_arch = "wasm32")]
        if self.frame_count % 30 == 0 { self.check_url_hash(); }

        self.render_state.queue.submit(std::iter::once(encoder.finish()));
        output.present();
        Ok(())
    }

    /// Act on the HUD's analysis requests.
    fn handle_analysis(&mut self, out: &HudOutput) {
        if out.undo { self.do_undo(false); }
        if out.redo { self.do_undo(true); }
        if out.share { self.share(); }
        if out.load_code {
            let code = std::mem::take(&mut self.analysis.paste);
            if !self.load_code(&code) {
                self.analysis.paste = code;
                self.analysis.share_note = "That code did not load".to_owned();
            } else {
                self.analysis.share_note = "Loaded".to_owned();
            }
        }
        if out.toggle_chaos {
            self.analysis.show_chaos = !self.analysis.show_chaos;
            self.analysis.butterfly = None;
        }
        if out.reseed_chaos {
            self.analysis.butterfly = Some(Butterfly::seed(&self.world, &self.mouse_spring));
        }
        if let Some(i) = out.scrub {
            if self.analysis.timeline.seek(&mut self.world, i) {
                self.editor.paused = true;
                self.traces.clear();
                self.analysis.butterfly = None;
                self.analysis.phase.trail.clear();
            }
        }
    }

    /// Editor overlay in the same visual language as the scene: white
    /// CAD-style corner brackets for selection, red for the second pick,
    /// reticles on attach points, translucent ghosts for placement.
    fn draw_editor_overlay(&mut self) {
        const SEL:   [f32; 4] = [1.0, 1.0, 1.0, 1.0];
        const SEL_B: [f32; 4] = [0.90, 0.27, 0.22, 1.0];
        const HOVER: [f32; 4] = [1.0, 1.0, 1.0, 0.35];
        const GHOST: [f32; 4] = [1.0, 1.0, 1.0, 0.28];
        let raw = self.world_mouse();
        let dragged = match self.editor.mode {
            EditorMode::Dragging { body_idx } => Some(body_idx),
            _ => None,
        };
        // Where cursor-following previews go (Ctrl-snapped when snapping).
        let cursor = self.editor_point(dragged);

        // Snap target marker
        if self.ctrl_held {
            if let Some(p) = snap_point(&self.world, raw, self.snap_radius(), dragged) {
                self.draw_snap_marker(p, SEL_B);
            }
        }

        // Hover hint when not already busy with a body
        if matches!(self.editor.mode, EditorMode::Idle | EditorMode::Inspecting { .. } | EditorMode::FirstSelected { .. }) {
            if let Some((i, _)) = self.pick_body(raw) {
                let selected = match self.editor.mode {
                    EditorMode::Inspecting { body_idx } | EditorMode::FirstSelected { body_idx, .. } => body_idx == i,
                    _ => false,
                };
                if !selected { self.draw_brackets(i, HOVER); }
            }
        }

        match self.editor.mode.clone() {
            EditorMode::PlacingBody => {
                let props = &self.editor.body_props;
                match props.template {
                    crate::editor::BodyTemplate::Disk => {
                        self.geo.draw_circle(cursor, props.radius, 64, GHOST);
                        self.geo.draw_dashed_ring(cursor, props.radius + 0.05, 32, 0.016, SEL);
                    }
                    crate::editor::BodyTemplate::Rod => {
                        let d = Vec2::new(props.half_len, 0.0);
                        self.geo.draw_rod(cursor - d, cursor + d, 0.12, GHOST);
                        self.geo.draw_dashed(cursor - d, cursor + d, 0.012, 0.08, 0.06, SEL);
                    }
                    crate::editor::BodyTemplate::Anchor => {
                        let ceiling = crate::scenes::anchor_is_ceiling(&self.world, cursor);
                        crate::scenes::draw_pedestal_tinted(&mut self.geo, cursor, ceiling, GHOST);
                    }
                }
                self.draw_reticle(cursor, SEL);
            }
            EditorMode::DraggingHandle { body_idx, handle } => {
                self.draw_brackets(body_idx, SEL);
                for (h, p) in connection_handles(&self.world, body_idx) {
                    self.draw_handle(p, if h == handle { Some(SEL_B) } else { None });
                }
            }
            EditorMode::Rotating { body_idx, .. } => {
                self.draw_brackets(body_idx, SEL);
                self.draw_rotate_handle(body_idx, Some(SEL_B));
            }
            EditorMode::Resizing { body_idx, kind } => {
                self.draw_brackets(body_idx, SEL);
                self.draw_resize_handles(body_idx, Some(kind), SEL_B);
            }
            EditorMode::Inspecting { body_idx } | EditorMode::Dragging { body_idx } => {
                self.draw_brackets(body_idx, SEL);
                if matches!(self.editor.mode, EditorMode::Inspecting { .. }) {
                    let over = self.on_rotate_handle(body_idx, raw);
                    self.draw_rotate_handle(body_idx, over.then_some(SEL));
                    let over = self.resize_handle_at(body_idx, raw);
                    self.draw_resize_handles(body_idx, over, SEL);
                    let hovered = self.handle_at(body_idx, raw);
                    for (h, p) in connection_handles(&self.world, body_idx) {
                        self.draw_handle(p, (Some(h) == hovered).then_some(SEL));
                    }
                }
                if let (Some(hover), Some(own)) = (self.editor.link_hover, self.world.bodies.get(body_idx)) {
                    let from = own.pos32();
                    let to = match hover {
                        LinkHover::Body(o) => {
                            self.draw_brackets(o, SEL_B);
                            self.world.bodies.get(o).map(|b| b.pos32())
                        }
                        LinkHover::Point(p) => {
                            self.draw_reticle(p, SEL_B);
                            Some(p)
                        }
                    };
                    if let Some(to) = to {
                        self.geo.draw_dashed(from, to, 0.016, 0.08, 0.06, SEL_B);
                    }
                }
            }
            EditorMode::FirstSelected { body_idx, world_pos } => {
                self.draw_brackets(body_idx, SEL);
                self.draw_reticle(world_pos, SEL);
                self.geo.draw_dashed(world_pos, cursor, 0.014, 0.08, 0.06, HOVER);
            }
            EditorMode::BothSelected { body_a, attach_a, body_b, attach_b } => {
                self.draw_brackets(body_a, SEL);
                self.draw_brackets(body_b, SEL_B);
                self.geo.draw_dashed(attach_a, attach_b, 0.016, 0.08, 0.06, SEL);
                self.draw_reticle(attach_a, SEL);
                self.draw_reticle(attach_b, SEL_B);
            }
            EditorMode::PulleyPending { body_a, attach_a, body_b, attach_b } => {
                self.draw_brackets(body_a, SEL);
                self.draw_brackets(body_b, SEL);
                self.draw_reticle(attach_a, SEL);
                self.draw_reticle(attach_b, SEL);
                let target = self.pick_body(raw)
                    .filter(|&(i, _)| self.world.bodies.get(i).is_some_and(crate::editor::is_disk))
                    .map(|(i, _)| i);
                let via = match target {
                    Some(i) => { self.draw_brackets(i, SEL_B); self.world.bodies[i].pos32() }
                    None => cursor,
                };
                self.geo.draw_dashed(attach_a, via, 0.016, 0.08, 0.06, SEL);
                self.geo.draw_dashed(via, attach_b, 0.016, 0.08, 0.06, SEL);
            }
            EditorMode::PinWorldPending { body_idx, attach } => {
                self.draw_brackets(body_idx, SEL);
                self.draw_reticle(attach, SEL);
                self.geo.draw_dashed(attach, cursor, 0.016, 0.08, 0.06, SEL);
                let ceiling = crate::scenes::anchor_is_ceiling(&self.world, cursor);
                crate::scenes::draw_pedestal_tinted(&mut self.geo, cursor, ceiling, GHOST);
                self.draw_reticle(cursor, SEL_B);
            }
            EditorMode::PlacingTrace { body_idx } => {
                self.draw_brackets(body_idx, SEL);
                if let Some(b) = self.world.bodies.get(body_idx) {
                    let from = b.pos32();
                    self.geo.draw_dashed(from, cursor, 0.014, 0.08, 0.06, HOVER);
                }
                self.draw_reticle(cursor, SEL_B);
            }
            EditorMode::Idle => {}
        }

        // Every trace point, so they can be found and grabbed.
        for t in &self.world.tracers {
            if let Some(b) = self.world.bodies.get(t.body) {
                let p = b.world_point(t.local);
                self.geo.draw_arc(p, 6.0 * self.camera.scale, 0.0, std::f32::consts::TAU, 16,
                    2.0 * self.camera.scale, SEL_B);
            }
        }
    }

    /// Corner brackets around a body's oriented bounding box.
    fn draw_brackets(&mut self, idx: usize, color: [f32; 4]) {
        let Some(body) = self.world.bodies.get(idx) else { return };
        const PAD: f32 = 0.10;
        let (u, hx, hy) = match body.shape {
            BodyShape::Disk { radius } => (Vec2::X, radius + PAD, radius + PAD),
            BodyShape::Rod { half_len, half_width } => {
                let (s, c) = body.angle32().sin_cos();
                (Vec2::new(c, s), half_len + half_width + PAD, half_width + PAD)
            }
            BodyShape::Point => (Vec2::X, 0.22, 0.22),
        };
        let v = Vec2::new(-u.y, u.x);
        let arm = (hx.min(hy) * 0.7).min(0.22);
        let w = 0.02;
        for (sx, sy) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
            let corner = body.pos32() + u * (sx * hx) + v * (sy * hy);
            self.geo.draw_line(corner + u * (sx * w * 0.5), corner - u * (sx * arm), w, color);
            self.geo.draw_line(corner + v * (sy * w * 0.5), corner - v * (sy * arm), w, color);
        }
    }

    /// Square grip on a movable connection point; `fill` when hot.
    /// Rotate handle: a guide arc on the handle's circle, a spoke from the
    /// centre, and a round knob (filled when hovered or dragged).
    fn draw_rotate_handle(&mut self, body_idx: usize, fill: Option<[f32; 4]>) {
        use crate::scenes::{BG, WHITE};
        let Some(b) = self.world.bodies.get(body_idx) else { return };
        let Some(p) = crate::editor::rotate_handle(b) else { return };
        let c = b.pos32();
        let r = (p - c).length();
        let a = (p - c).y.atan2((p - c).x);
        let px = self.camera.scale;
        self.geo.draw_arc(c, r, a - 0.5, a + 0.5, 24, 1.5 * px, [1.0, 1.0, 1.0, 0.5]);
        self.geo.draw_dashed(c, p, 1.5 * px, 6.0 * px, 5.0 * px, [1.0, 1.0, 1.0, 0.5]);
        self.geo.draw_circle(p, 8.0 * px, 20, BG);
        self.geo.draw_circle(p, 6.5 * px, 20, fill.unwrap_or(WHITE));
        if fill.is_none() {
            self.geo.draw_circle(p, 3.5 * px, 16, BG);
        }
    }

    /// Resize handles: diamonds pointing along the dimension they change,
    /// on a faint guide from the body; `active` is drawn filled in `fill`.
    fn draw_resize_handles(&mut self, body_idx: usize, active: Option<crate::editor::ResizeKind>, fill: [f32; 4]) {
        use crate::scenes::{BG, WHITE};
        let Some(b) = self.world.bodies.get(body_idx) else { return };
        let c = b.pos32();
        let handles = crate::editor::resize_handles(b);
        let px = self.camera.scale;
        for (kind, p) in handles {
            let d = (p - c).normalize_or_zero();
            let n = d.perp();
            self.geo.draw_dashed(c + (p - c) * 0.55, p, 1.5 * px, 4.0 * px, 4.0 * px, [1.0, 1.0, 1.0, 0.5]);
            // Diamond elongated along the direction it resizes in.
            let (along, across) = (d * 10.0 * px, n * 6.0 * px);
            let on = active == Some(kind);
            self.geo.draw_diamond(p, along * 1.3, across * 1.45, BG);
            self.geo.draw_diamond(p, along, across, if on { fill } else { WHITE });
            if !on { self.geo.draw_diamond(p, along * 0.45, across * 0.45, BG); }
        }
    }

    fn draw_handle(&mut self, p: Vec2, fill: Option<[f32; 4]>) {
        use crate::scenes::{BG, WHITE};
        let hs = 6.0 * self.camera.scale; // constant on-screen size
        let o = 2.0 * self.camera.scale;
        let x = Vec2::new(hs + o, 0.0);
        self.geo.draw_line(p - x, p + x, 2.0 * (hs + o), BG);
        let x = Vec2::new(hs, 0.0);
        self.geo.draw_line(p - x, p + x, 2.0 * hs, fill.unwrap_or(WHITE));
        if fill.is_none() {
            let x = Vec2::new(hs * 0.45, 0.0);
            self.geo.draw_line(p - x, p + x, 0.9 * hs, BG);
        }
    }

    /// Hollow diamond marking the point a Ctrl-snap will use.
    fn draw_snap_marker(&mut self, p: Vec2, color: [f32; 4]) {
        use crate::scenes::BG;
        let a = 7.0 * self.camera.scale;
        let diag = |k: f32| Vec2::new(k, k);
        // A segment along (1,1) with equal width is a square rotated 45°.
        self.geo.draw_line(p - diag(a * 0.5), p + diag(a * 0.5), a * std::f32::consts::SQRT_2, color);
        self.geo.draw_line(p - diag(a * 0.25), p + diag(a * 0.25), a * 0.5 * std::f32::consts::SQRT_2, BG);
    }

    /// Ring with four outward ticks marking an exact attach point.
    fn draw_reticle(&mut self, p: Vec2, color: [f32; 4]) {
        let r = 0.07;
        self.geo.draw_arc(p, r, 0.0, std::f32::consts::TAU, 24, 0.016, color);
        for d in [Vec2::X, Vec2::Y, -Vec2::X, -Vec2::Y] {
            self.geo.draw_line(p + d * (r + 0.02), p + d * (r + 0.08), 0.016, color);
        }
        self.geo.draw_circle(p, 0.015, 8, color);
    }
}
