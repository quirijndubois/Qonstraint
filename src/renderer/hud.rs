use std::collections::VecDeque;

use egui::{
    Align2, Color32, FontData, FontDefinitions, FontFamily, FontId, Frame, Margin, RichText,
    Rounding, Sense, Stroke, Vec2,
};
use egui_wgpu::ScreenDescriptor;
use winit::window::Window;

use crate::editor::{BodyTemplate, ConstraintKind, Editor, EditorMode, LinkHover, StepMode};
use crate::sim::{
    body::BodyShape,
    constraints::{cylinder::{GasMode, Stroke as GasStroke}, Cylinder, DistanceConstraint, PinJoint, PinWorld, RollingContact, RollingOnRod, SliderJoint},
    forces::{spring::SpringDamper, Motor},
    world::{Integrator, World},
};

// ── Style ─────────────────────────────────────────────────────────────────────
//
// Terminal-style panels: black fill, hard white border, heavy uppercase mono,
// one red accent for destructive actions and warnings.

const WHITE: Color32 = Color32::WHITE;
const BLACK: Color32 = Color32::BLACK;
const PANEL: Color32 = Color32::from_rgba_premultiplied(0, 0, 0, 235);
const MUTED: Color32 = Color32::from_rgb(130, 130, 130);
const RED:   Color32 = Color32::from_rgb(230, 70, 55);

const TITLE_SIZE:  f32 = 19.0;
const HEAD_SIZE:   f32 = 14.0;
const BODY_SIZE:   f32 = 11.5;
const SMALL_SIZE:  f32 = 9.5;
const BORDER_W:    f32 = 1.5;

fn font(size: f32) -> FontId { FontId::new(size, FontFamily::Monospace) }

fn text(s: impl Into<String>, size: f32, color: Color32) -> RichText {
    RichText::new(s.into().to_uppercase())
        .font(font(size))
        .color(color)
        .extra_letter_spacing(size * 0.06)
}

fn panel_frame() -> Frame {
    Frame::none()
        .fill(PANEL)
        .stroke(Stroke::new(BORDER_W, WHITE))
        .rounding(Rounding::ZERO)
        .inner_margin(Margin::same(12.0))
}

// ── Public types ──────────────────────────────────────────────────────────────

pub struct Metrics {
    pub fps:              f32,
    pub sub_steps:        u32,
    pub kinetic_energy:   f32,
    pub potential_energy: f32,
    pub constraint_error: f32,
    pub running:          bool,
}

#[derive(Default)]
pub struct HudOutput {
    pub prev_scene:   bool,
    pub next_scene:   bool,
    pub reset_scene:  bool,
    pub toggle_edit:  bool,
}

// ── HUD ──────────────────────────────────────────────────────────────────────

pub struct Hud {
    ctx:         egui::Context,
    winit_state: egui_winit::State,
    renderer:    egui_wgpu::Renderer,
    history:     History,
}

const HISTORY_LEN: usize = 150;

/// Recent per-frame values behind the INFO sparklines.
#[derive(Default)]
struct History {
    fps:    VecDeque<f32>,
    sr:     VecDeque<f32>,
    steps:  VecDeque<f32>,
    energy: VecDeque<f32>,
    c_err:  VecDeque<f32>,
}

impl History {
    fn push(&mut self, m: &Metrics) {
        let sr = m.fps * m.sub_steps as f32;
        for (buf, v) in [
            (&mut self.fps,    m.fps),
            (&mut self.sr,     sr),
            (&mut self.steps,  m.sub_steps as f32),
            (&mut self.energy, m.kinetic_energy + m.potential_energy),
            (&mut self.c_err,  m.constraint_error.max(1e-12).log10()),
        ] {
            if buf.len() == HISTORY_LEN { buf.pop_front(); }
            buf.push_back(v);
        }
    }
}

impl Hud {
    pub fn new(
        window:         &'static Window,
        device:         &wgpu::Device,
        surface_format: wgpu::TextureFormat,
    ) -> Self {
        let ctx = egui::Context::default();
        install_font(&ctx);
        install_style(&ctx);

        let winit_state = egui_winit::State::new(
            ctx.clone(),
            egui::ViewportId::ROOT,
            window as &dyn egui_winit::winit::raw_window_handle::HasDisplayHandle,
            Some(window.scale_factor() as f32),
            None,
            None,
        );
        let renderer = egui_wgpu::Renderer::new(device, surface_format, None, 1, false);

        Self { ctx, winit_state, renderer, history: History::default() }
    }

    pub fn on_window_event(&mut self, window: &Window, event: &winit::event::WindowEvent) -> bool {
        self.winit_state.on_window_event(window, event).consumed
    }

    #[allow(clippy::too_many_arguments)]
    pub fn encode(
        &mut self,
        window:       &Window,
        device:       &wgpu::Device,
        queue:        &wgpu::Queue,
        encoder:      &mut wgpu::CommandEncoder,
        surface_view: &wgpu::TextureView,
        size:         [u32; 2],
        metrics:      &Metrics,
        scene_idx:    usize,
        scene_count:  usize,
        scene_name:   &str,
        scene_desc:   &str,
        editor:       &mut Editor,
        world:        &mut World,
    ) -> HudOutput {
        self.history.push(metrics);
        let raw_input = self.winit_state.take_egui_input(window);

        let mut out = HudOutput::default();
        let history = &self.history;
        let full_output = self.ctx.run(raw_input, |ctx| {
            out = draw_main_panel(ctx, metrics, history, scene_idx, scene_count, scene_name, scene_desc, editor);
            if editor.active {
                draw_editor_panel(ctx, editor, world);
            }
        });

        self.winit_state.handle_platform_output(window, full_output.platform_output);

        let ppp     = full_output.pixels_per_point;
        let clipped = self.ctx.tessellate(full_output.shapes, ppp);
        let screen_desc = ScreenDescriptor { size_in_pixels: size, pixels_per_point: ppp };

        for (id, delta) in &full_output.textures_delta.set {
            self.renderer.update_texture(device, queue, *id, delta);
        }
        self.renderer.update_buffers(device, queue, encoder, &clipped, &screen_desc);

        {
            let rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view:           surface_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load:  wgpu::LoadOp::Load,
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes:         None,
                occlusion_query_set:      None,
            });
            let mut rpass = rpass.forget_lifetime();
            self.renderer.render(&mut rpass, &clipped, &screen_desc);
        }

        for id in &full_output.textures_delta.free {
            self.renderer.free_texture(id);
        }

        out
    }
}

fn install_font(ctx: &egui::Context) {
    let mut fonts = FontDefinitions::default();
    fonts.font_data.insert(
        "jbm-xb".to_owned(),
        FontData::from_static(include_bytes!("../../assets/fonts/JetBrainsMono-ExtraBold.ttf")).into(),
    );
    for family in [FontFamily::Monospace, FontFamily::Proportional] {
        fonts.families.entry(family).or_default().insert(0, "jbm-xb".to_owned());
    }
    ctx.set_fonts(fonts);
}

fn install_style(ctx: &egui::Context) {
    let mut style = (*ctx.style()).clone();
    style.interaction.selectable_labels = false;
    style.spacing.slider_width = 120.0;
    style.spacing.item_spacing = Vec2::new(6.0, 4.0);

    let v = &mut style.visuals;
    *v = egui::Visuals::dark();
    v.window_shadow = egui::Shadow::NONE;
    v.override_text_color = Some(WHITE);
    v.selection.bg_fill = WHITE;
    v.slider_trailing_fill = true;
    v.handle_shape = egui::style::HandleShape::Rect { aspect_ratio: 0.55 };
    for w in [
        &mut v.widgets.noninteractive,
        &mut v.widgets.inactive,
        &mut v.widgets.hovered,
        &mut v.widgets.active,
        &mut v.widgets.open,
    ] {
        w.rounding = Rounding::ZERO;
        w.expansion = 0.0;
        w.bg_fill = Color32::from_gray(55);
        w.weak_bg_fill = BLACK;
        w.bg_stroke = Stroke::new(1.0_f32, WHITE);
        w.fg_stroke = Stroke::new(BORDER_W, WHITE);
    }
    v.widgets.hovered.bg_fill = Color32::from_gray(90);
    v.widgets.active.bg_fill = WHITE;

    ctx.set_style(style);
}

// ── Main panel ────────────────────────────────────────────────────────────────

#[allow(clippy::too_many_arguments)]
fn draw_main_panel(
    ctx:         &egui::Context,
    m:           &Metrics,
    h:           &History,
    scene_idx:   usize,
    scene_count: usize,
    scene_name:  &str,
    scene_desc:  &str,
    editor:      &mut Editor,
) -> HudOutput {
    let mut out = HudOutput::default();

    egui::Area::new(egui::Id::new("hud"))
        .fixed_pos(egui::pos2(16.0, 16.0))
        .show(ctx, |ui| {
            panel_frame().show(ui, |ui| {
                ui.set_width(340.0);

                // Title + navigation
                ui.horizontal(|ui| {
                    ui.label(text(scene_name, TITLE_SIZE, WHITE));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(text(format!("{}/{}", scene_idx + 1, scene_count), SMALL_SIZE, MUTED));
                    });
                });
                ui.label(text(scene_desc, SMALL_SIZE, MUTED));
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    if hud_btn(ui, "<", false, 28.0).clicked()      { out.prev_scene  = true; }
                    if hud_btn(ui, ">", false, 28.0).clicked()      { out.next_scene  = true; }
                    if hud_btn(ui, "RESET", false, 0.0).clicked()   { out.reset_scene = true; }
                    if hud_btn(ui, "EDIT", editor.active, 0.0).clicked() { out.toggle_edit = true; }
                    let lbl = if editor.paused { "PLAY" } else { "PAUSE" };
                    if hud_btn(ui, lbl, editor.paused, 0.0).clicked() { editor.paused = !editor.paused; }
                });

                ui.add_space(10.0);

                // INFO
                ui.label(text("Info", HEAD_SIZE, WHITE));
                kv_spark(ui, "FRAME RATE",  &format!("{:.0} FPS", m.fps), &h.fps);
                kv_spark(ui, "SAMPLE RATE", &format!("{:.0} HZ", m.fps * m.sub_steps as f32), &h.sr);
                kv_spark(ui, "STEPS",       &m.sub_steps.to_string(), &h.steps);
                kv_spark(ui, "ENERGY",      &format!("{:.4}", m.kinetic_energy + m.potential_energy), &h.energy);
                kv_spark(ui, "CONSTRAINT ERROR", &format!("{:.1E}", m.constraint_error), &h.c_err);

                ui.add_space(8.0);
                slider_row(ui, "GRAVITY", &mut editor.gravity, 0.0, 30.0);
                ui.horizontal(|ui| {
                    let (rect, _) = ui.allocate_exact_size(Vec2::new(58.0, 18.0), Sense::hover());
                    ui.painter().text(rect.left_center(), Align2::LEFT_CENTER, "SOLVER", font(BODY_SIZE), WHITE);
                    for (integ, lbl) in [(Integrator::Rk4, "RK4"), (Integrator::Xpbd, "XPBD")] {
                        if hud_btn(ui, lbl, editor.integrator == integ, 0.0).clicked() {
                            editor.integrator = integ;
                        }
                    }
                });
                ui.horizontal(|ui| {
                    let (rect, _) = ui.allocate_exact_size(Vec2::new(58.0, 18.0), Sense::hover());
                    ui.painter().text(rect.left_center(), Align2::LEFT_CENTER, "STEPS", font(BODY_SIZE), WHITE);
                    for (mode, lbl) in [(StepMode::Fixed, "FIXED"), (StepMode::TargetFps, "TARGET FPS")] {
                        if hud_btn(ui, lbl, editor.step_mode == mode, 0.0).clicked() {
                            editor.step_mode = mode;
                        }
                    }
                });
                match editor.step_mode {
                    StepMode::Fixed => int_slider_row(ui, "", &mut editor.sub_steps, 1, 2000),
                    StepMode::TargetFps => {
                        // Floor of 20: frame dt is capped at 0.05 s, below that the sim would lag real time.
                        slider_row(ui, "", &mut editor.target_fps, 20.0, 240.0);
                        ui.label(text(
                            format!("Auto: {} steps per frame", editor.sub_steps),
                            SMALL_SIZE, MUTED,
                        ));
                    }
                }
                log_slider_row(ui, "SPEED", &mut editor.time_scale, 0.05, 20.0);
                ui.add_space(10.0);

                // Status line
                let state = match (m.running, (editor.time_scale - 1.0).abs() > 0.005) {
                    (false, _)    => "Paused".to_owned(),
                    (true, false) => "Running".to_owned(),
                    (true, true)  => format!("Running {:.2}x", editor.time_scale),
                };
                let (health, col) = if editor.active {
                    ("Editing", WHITE)
                } else if m.constraint_error > 1e-2 {
                    ("Drift", RED)
                } else {
                    ("Okay", WHITE)
                };
                ui.horizontal(|ui| {
                    ui.label(text(format!("{state} //"), HEAD_SIZE + 1.0, WHITE));
                    ui.label(text(health, HEAD_SIZE + 1.0, col));
                });
            });
        });

    out
}

/// "KEY = VALUE" with a small sparkline of its recent history on the right.
fn kv_spark(ui: &mut egui::Ui, key: &str, value: &str, history: &VecDeque<f32>) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(200.0, 16.0), Sense::hover());
        ui.painter().text(
            rect.left_center(), Align2::LEFT_CENTER,
            format!("{key} = {value}"), font(BODY_SIZE), WHITE,
        );
        sparkline(ui, history);
    });
}

/// Thin white polyline in a hairline box, auto-scaled to its own range.
fn sparkline(ui: &mut egui::Ui, history: &VecDeque<f32>) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 14.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_stroke(rect, Rounding::ZERO, Stroke::new(1.0_f32, Color32::from_gray(55)));
    let n = history.len();
    if n < 2 { return; }
    let (lo, hi) = history.iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &v| (lo.min(v), hi.max(v)));
    let range = hi - lo;
    let inner = rect.shrink(2.0);
    let points: Vec<egui::Pos2> = history.iter().enumerate().map(|(i, &v)| {
        let x = inner.left() + inner.width() * i as f32 / (n - 1) as f32;
        // A flat signal sits mid-height instead of jittering at float noise.
        let t = if range > 1e-6 * hi.abs().max(1.0) { (v - lo) / range } else { 0.5 };
        egui::pos2(x, inner.bottom() - t * inner.height())
    }).collect();
    p.add(egui::Shape::line(points, Stroke::new(1.0_f32, WHITE)));
}

// ── Editor panel ──────────────────────────────────────────────────────────────

fn draw_editor_panel(ctx: &egui::Context, editor: &mut Editor, world: &mut World) {
    egui::Area::new(egui::Id::new("editor_panel"))
        .anchor(Align2::RIGHT_TOP, egui::vec2(-16.0, 16.0))
        .show(ctx, |ui| {
            // Panel never extends past the window bottom; overflow scrolls.
            let max_h = (ctx.screen_rect().height() - 32.0 - 24.0).max(120.0);
            panel_frame().show(ui, |ui| {
                ui.set_width(270.0);
                // Grow with the content; only scroll once it would pass the
                // window bottom. Without min_scrolled_height egui may shrink
                // the area to its 64px default whenever it has to scroll.
                egui::ScrollArea::vertical()
                    .max_height(max_h)
                    .min_scrolled_height(max_h)
                    .auto_shrink([false, true])
                    .show(ui, |ui| editor_panel_contents(ui, editor, world));
            });
        });
}

fn editor_panel_contents(ui: &mut egui::Ui, editor: &mut Editor, world: &mut World) {
    ui.label(text("Editor", TITLE_SIZE, WHITE));
    ui.add_space(8.0);

    // New body: only while nothing is selected, so a selection's
    // properties and connections sit near the top of the panel.
    if matches!(editor.mode, EditorMode::Idle | EditorMode::PlacingBody) {
        ui.label(text("New body", HEAD_SIZE, WHITE));
        ui.horizontal(|ui| {
            for (tmpl, lbl) in [
                (BodyTemplate::Disk,   "DISK"),
                (BodyTemplate::Rod,    "ROD"),
                (BodyTemplate::Anchor, "ANCHOR"),
            ] {
                let selected = editor.body_props.template == tmpl;
                if hud_btn(ui, lbl, selected, 0.0).clicked() {
                    editor.body_props.template = tmpl;
                }
            }
        });
        ui.add_space(4.0);
        match editor.body_props.template {
            BodyTemplate::Disk => {
                slider_row(ui, "RADIUS", &mut editor.body_props.radius, 0.1, 1.5);
                slider_row(ui, "MASS",   &mut editor.body_props.mass,   0.1, 10.0);
            }
            BodyTemplate::Rod => {
                slider_row(ui, "LENGTH", &mut editor.body_props.half_len, 0.2, 3.0);
                slider_row(ui, "WIDTH",  &mut editor.body_props.half_width, 0.02, 0.4);
                slider_row(ui, "MASS",   &mut editor.body_props.mass,     0.1, 10.0);
            }
            BodyTemplate::Anchor => {
                ui.label(text("Fixed world point", SMALL_SIZE, MUTED));
            }
        }
        if editor.body_props.template != BodyTemplate::Anchor {
            toggle_row(ui, "FIXED", &mut editor.body_props.fixed);
        }
        ui.add_space(4.0);
        let placing = matches!(editor.mode, EditorMode::PlacingBody);
        let place_lbl = if placing { "CANCEL PLACE" } else { "+ PLACE BODY" };
        if hud_btn(ui, place_lbl, placing, ui.available_width()).clicked() {
            editor.mode = if placing { EditorMode::Idle } else { EditorMode::PlacingBody };
        }
    }

    // Connection picker
    if let EditorMode::BothSelected { .. } = editor.mode {
        ui.add_space(10.0);
        ui.label(text("Connect with", HEAD_SIZE, WHITE));
        for (lbl, kind) in [
            ("PIN JOINT",    ConstraintKind::PinJoint),
            ("DISTANCE ROD", ConstraintKind::Distance),
            ("ROLLING",      ConstraintKind::RollingContact),
            ("SLIDER",       ConstraintKind::Slider),
            ("CYLINDER",     ConstraintKind::Cylinder),
            ("SPRING",       ConstraintKind::Spring),
        ] {
            if kind == ConstraintKind::RollingContact && !editor.pair_can_roll {
                continue;
            }
            if matches!(kind, ConstraintKind::Slider | ConstraintKind::Cylinder) && !editor.pair_can_slide {
                continue;
            }
            if hud_btn(ui, lbl, false, ui.available_width()).clicked() {
                editor.pending_constraint = Some(kind);
            }
        }
        if !editor.pair_can_slide {
            ui.label(text("Slider and cylinder need a rod", SMALL_SIZE, MUTED));
        }
        if !editor.pair_can_roll {
            ui.label(text("Rolling needs a disk + disk or rod", SMALL_SIZE, MUTED));
        }
        if hud_btn(ui, "CANCEL", false, ui.available_width()).clicked() {
            editor.mode = EditorMode::Idle;
        }
        ui.add_space(4.0);
        ui.label(text("New spring", SMALL_SIZE, MUTED));
        slider_row(ui, "STIFF", &mut editor.spring_k, 1.0, 500.0);
        slider_row(ui, "DAMP",  &mut editor.spring_d, 0.0, 50.0);
    }

    // Selected body
    if matches!(editor.mode, EditorMode::Inspecting { .. }) {
        let shape = editor.inspect_shape.clone().unwrap_or_else(|| "Body".into());
        if let Some(edit) = &mut editor.body_edit {
            ui.add_space(10.0);
            ui.label(text(format!("Selected // {shape}"), HEAD_SIZE, WHITE));
            slider_row(ui, "MASS", &mut edit.mass, 0.1, 20.0);
            match shape.as_str() {
                "Disk" => slider_row(ui, "RADIUS", &mut edit.radius,   0.05, 2.0),
                "Rod"  => {
                    slider_row(ui, "LENGTH", &mut edit.half_len, 0.1,  4.0);
                    slider_row(ui, "WIDTH",  &mut edit.half_width, 0.02, 0.4);
                }
                _      => {}
            }
            if shape != "Point" {
                slider_row(ui, "ANGLE", &mut edit.angle, -180.0, 180.0);
            }
            toggle_row(ui, "FIXED", &mut edit.fixed);
            ui.add_space(4.0);
            if danger_btn(ui, "DELETE", ui.available_width()).clicked() {
                editor.delete_requested = true;
            }
        }
    }

    editor.link_hover = None;
    if let EditorMode::Inspecting { body_idx } = editor.mode {
        if body_idx < world.bodies.len() {
            ui.add_space(10.0);
            editor.link_hover = draw_connections(ui, world, body_idx);
            ui.add_space(10.0);
            let (trace_hover, add) = draw_traces(ui, world, body_idx);
            editor.link_hover = editor.link_hover.or(trace_hover);
            if add {
                editor.mode = EditorMode::PlacingTrace { body_idx };
            }
        }
    }

    if let EditorMode::PlacingTrace { body_idx } = editor.mode {
        ui.add_space(10.0);
        ui.label(text("Place trace", HEAD_SIZE, WHITE));
        ui.label(text("Click where the trail should be drawn from", SMALL_SIZE, MUTED));
        if hud_btn(ui, "CANCEL", false, ui.available_width()).clicked() {
            editor.mode = EditorMode::Inspecting { body_idx };
        }
    }

    // Status + key hints
    ui.add_space(10.0);
    let status = match &editor.mode {
        EditorMode::Idle                 => "Shift-click to connect",
        EditorMode::PlacingBody          => "Click to place",
        EditorMode::FirstSelected { .. } => "Click 2nd body",
        EditorMode::BothSelected { .. }  => "Pick a connection",
        EditorMode::PinWorldPending { .. } => "Click anchor point",
        EditorMode::Dragging { .. }      => "Dragging",
        EditorMode::DraggingHandle { .. } => "Moving point",
        EditorMode::Rotating { .. }      => "Rotating (CTRL snaps 15 deg)",
        EditorMode::Resizing { .. }      => "Resizing (CTRL snaps)",
        EditorMode::PlacingTrace { .. }  => "Click to place trace",
        EditorMode::Inspecting { .. }    => "Drag knobs: rotate, resize, move points",
    };
    ui.label(text(format!("> {status}"), BODY_SIZE, WHITE));
    ui.label(text("CTRL snap // R rotate // DEL delete // ESC deselect", SMALL_SIZE, MUTED));
}

// ── Connections inspector ─────────────────────────────────────────────────────

/// Lists every constraint and spring touching `body_idx` as an editable card.
/// Returns the far end of the card under the pointer, for scene highlighting.
fn draw_connections(ui: &mut egui::Ui, world: &mut World, body_idx: usize) -> Option<LinkHover> {
    ui.label(text("Connections", HEAD_SIZE, WHITE));

    let mut hover = None;
    let mut remove_constraint = None;
    let mut remove_force = None;
    let mut count = 0;
    let bodies = &world.bodies;
    let other = |a: usize, b: usize| if a == body_idx { b } else { a };

    for (ci, c) in world.constraints.iter_mut().enumerate() {
        if !c.body_indices().contains(&body_idx) { continue; }
        count += 1;
        let any = c.as_any_mut();
        let (title, far) = if let Some(pj) = any.downcast_ref::<PinJoint>() {
            let o = other(pj.body_a, pj.body_b);
            (format!("Pin joint > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else if let Some(pw) = any.downcast_ref::<PinWorld>() {
            (format!("World pin @ {:.1}, {:.1}", pw.target.x, pw.target.y), LinkHover::Point(pw.target))
        } else if let Some(dc) = any.downcast_ref::<DistanceConstraint>() {
            let o = other(dc.body_a, dc.body_b);
            (format!("Distance > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else if let Some(rc) = any.downcast_ref::<RollingContact>() {
            let o = other(rc.body_a, rc.body_b);
            (format!("Rolling > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else if let Some(rr) = any.downcast_ref::<RollingOnRod>() {
            let o = other(rr.disk, rr.rod);
            (format!("Rolling > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else if let Some(cy) = any.downcast_ref::<Cylinder>() {
            let o = other(cy.piston, cy.barrel);
            let what = if cy.barrel == body_idx { "Cylinder, piston" } else { "Cylinder, barrel" };
            (format!("{what} > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else if let Some(sj) = any.downcast_ref::<SliderJoint>() {
            let o = other(sj.rider, sj.rail);
            let what = if sj.rail == body_idx { "Carries" } else { "Slides on" };
            (format!("{what} > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else {
            ("Constraint".to_owned(), LinkHover::Body(body_idx))
        };

        let (delete, hovered) = link_card(ui, &title, |ui| {
            if let Some(dc) = any.downcast_mut::<DistanceConstraint>() {
                slider_row(ui, "LENGTH", &mut dc.rest_len, 0.05, 8.0);
            } else if let Some(cy) = any.downcast_mut::<Cylinder>() {
                ui.horizontal(|ui| {
                    for (mode, lbl) in [(GasMode::Sealed, "SEALED"), (GasMode::TwoStroke, "2-STROKE"), (GasMode::FourStroke, "4-STROKE")] {
                        if hud_btn(ui, lbl, cy.mode == mode, 0.0).clicked() && cy.mode != mode {
                            cy.mode = mode;
                            // Start on compression; the cycle syncs from there.
                            cy.stroke = GasStroke::Compression;
                        }
                    }
                });
                if cy.mode == GasMode::Sealed {
                    ui.label(text("Air spring // rests where it is now", SMALL_SIZE, MUTED));
                } else {
                    slider_row(ui, "THROTTLE", &mut cy.throttle, 0.0, 1.0);
                    ui.label(text("Fires at top dead centre", SMALL_SIZE, MUTED));
                }
            } else if let Some(sj) = any.downcast_mut::<SliderJoint>() {
                toggle_row(ui, "LOCK ROT", &mut sj.lock_rotation);
                ui.label(text("Rides the rail between its end stops", SMALL_SIZE, MUTED));
            } else if any.is::<RollingContact>() || any.is::<RollingOnRod>() {
                ui.label(text("No slip // rotation locked to travel", SMALL_SIZE, MUTED));
            } else {
                ui.label(text("No parameters", SMALL_SIZE, MUTED));
            }
        });
        if delete { remove_constraint = Some(ci); }
        if hovered { hover = Some(far); }
    }

    let mut has_motor = false;
    for (fi, f) in world.forces.iter_mut().enumerate() {
        if !f.body_indices().contains(&body_idx) { continue; }
        if let Some(mo) = f.as_any_mut().downcast_mut::<Motor>() {
            count += 1;
            has_motor = true;
            let (delete, _) = link_card(ui, "Motor", |ui| {
                slider_row(ui, "TORQUE", &mut mo.torque, -50.0, 50.0);
                slider_row(ui, "DRAG",   &mut mo.drag,   0.0, 10.0);
                ui.label(text("Drive torque minus drag x spin", SMALL_SIZE, MUTED));
            });
            if delete { remove_force = Some(fi); }
            continue;
        }
        let Some(sd) = f.as_any_mut().downcast_mut::<SpringDamper>() else { continue };
        count += 1;
        let o = other(sd.body_a, sd.body_b);
        let title = format!("Spring > {}", body_name(bodies, o));
        let (delete, hovered) = link_card(ui, &title, |ui| {
            slider_row(ui, "STIFF", &mut sd.stiffness, 1.0, 500.0);
            slider_row(ui, "DAMP",  &mut sd.damping,   0.0, 50.0);
            slider_row(ui, "REST",  &mut sd.rest_len,  0.05, 8.0);
        });
        if delete { remove_force = Some(fi); }
        if hovered { hover = Some(LinkHover::Body(o)); }
    }

    if count == 0 {
        ui.label(text("None", SMALL_SIZE, MUTED));
    }
    if !has_motor && hud_btn(ui, "+ MOTOR", false, ui.available_width()).clicked() {
        world.add_force(Motor::new(body_idx, 0.0, 0.0));
    }
    if let Some(ci) = remove_constraint { world.constraints.remove(ci); }
    if let Some(fi) = remove_force { world.forces.remove(fi); }
    hover
}

/// Trace cards for `body_idx`: trail length and delete, plus an add button.
/// Returns (hovered trace point, add clicked).
fn draw_traces(ui: &mut egui::Ui, world: &mut World, body_idx: usize) -> (Option<LinkHover>, bool) {
    ui.label(text("Traces", HEAD_SIZE, WHITE));
    let mut hover = None;
    let mut remove = None;
    let mut n = 0;
    let bodies = &world.bodies;
    for (i, t) in world.tracers.iter_mut().enumerate() {
        if t.body != body_idx { continue; }
        n += 1;
        let p = bodies[t.body].world_point(t.local);
        let (delete, hovered) = link_card(ui, &format!("Trace {n}"), |ui| {
            slider_row(ui, "TRAIL S", &mut t.seconds, 0.5, 30.0);
        });
        if delete { remove = Some(i); }
        if hovered { hover = Some(LinkHover::Point(p)); }
    }
    if n == 0 {
        ui.label(text("None", SMALL_SIZE, MUTED));
    }
    if let Some(i) = remove { world.tracers.remove(i); }
    let add = hud_btn(ui, "+ ADD TRACE", false, ui.available_width()).clicked();
    (hover, add)
}

/// Bordered card with a title row and a small delete button.
/// Returns (delete clicked, pointer over card).
fn link_card(ui: &mut egui::Ui, title: &str, body: impl FnOnce(&mut egui::Ui)) -> (bool, bool) {
    let mut delete = false;
    let resp = Frame::none()
        .stroke(Stroke::new(1.0_f32, MUTED))
        .inner_margin(Margin::same(6.0))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.label(text(title, BODY_SIZE, WHITE));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    delete = danger_btn(ui, "X", 22.0).clicked();
                });
            });
            body(ui);
        })
        .response;
    (delete, ui.rect_contains_pointer(resp.rect))
}

fn body_name(bodies: &[crate::sim::body::Body], i: usize) -> String {
    let kind = match bodies.get(i).map(|b| &b.shape) {
        Some(BodyShape::Disk { .. }) => "Disk",
        Some(BodyShape::Rod { .. })  => "Rod",
        Some(BodyShape::Point)       => "Anchor",
        None                         => "?",
    };
    format!("{kind} {i}")
}

// ── Widgets ───────────────────────────────────────────────────────────────────

/// Flat bordered button. `active` renders it inverted (white on black).
fn hud_btn(ui: &mut egui::Ui, label: &str, active: bool, min_width: f32) -> egui::Response {
    styled_btn(ui, label, active, min_width, WHITE)
}

fn danger_btn(ui: &mut egui::Ui, label: &str, min_width: f32) -> egui::Response {
    styled_btn(ui, label, false, min_width, RED)
}

fn styled_btn(ui: &mut egui::Ui, label: &str, active: bool, min_width: f32, color: Color32) -> egui::Response {
    let f = font(BODY_SIZE);
    let galley = ui.painter().layout_no_wrap(label.to_owned(), f.clone(), color);
    let size = Vec2::new(min_width.max(galley.size().x + 16.0), 22.0);
    let (rect, resp) = ui.allocate_exact_size(size, Sense::click());

    let (bg, fg) = if active {
        (color, BLACK)
    } else if resp.hovered() {
        (Color32::from_gray(45), color)
    } else {
        (BLACK, color)
    };
    let p = ui.painter();
    p.rect(rect, Rounding::ZERO, bg, Stroke::new(BORDER_W, color));
    p.text(rect.center(), Align2::CENTER_CENTER, label, f, fg);
    resp
}

fn slider_row(ui: &mut egui::Ui, label: &str, value: &mut f32, min: f32, max: f32) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(58.0, 18.0), Sense::hover());
        ui.painter().text(rect.left_center(), Align2::LEFT_CENTER, label, font(BODY_SIZE), WHITE);
        ui.add(egui::Slider::new(value, min..=max).show_value(false));
        ui.label(text(format!("{value:.2}"), BODY_SIZE, WHITE));
    });
}

/// Like `slider_row` but logarithmic, for ranges spanning orders of magnitude.
fn log_slider_row(ui: &mut egui::Ui, label: &str, value: &mut f32, min: f32, max: f32) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(58.0, 18.0), Sense::hover());
        ui.painter().text(rect.left_center(), Align2::LEFT_CENTER, label, font(BODY_SIZE), WHITE);
        ui.add(egui::Slider::new(value, min..=max).logarithmic(true).show_value(false));
        ui.label(text(format!("{value:.2}"), BODY_SIZE, WHITE));
    });
}

fn int_slider_row(ui: &mut egui::Ui, label: &str, value: &mut u32, min: u32, max: u32) {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(58.0, 18.0), Sense::hover());
        ui.painter().text(rect.left_center(), Align2::LEFT_CENTER, label, font(BODY_SIZE), WHITE);
        // Logarithmic so the low, common values stay easy to hit.
        ui.add(egui::Slider::new(value, min..=max).logarithmic(true).show_value(false));
        ui.label(text(value.to_string(), BODY_SIZE, WHITE));
    });
}

fn toggle_row(ui: &mut egui::Ui, label: &str, value: &mut bool) {
    let lbl = format!("[{}] {label}", if *value { "X" } else { " " });
    if hud_btn(ui, &lbl, *value, 0.0).clicked() {
        *value = !*value;
    }
}
