use std::collections::VecDeque;

use egui::{
    Align2, Color32, FontData, FontDefinitions, FontFamily, FontId, Frame, Margin, RichText,
    Rounding, Sense, Stroke, Vec2,
};
use egui_wgpu::ScreenDescriptor;
use winit::window::Window;

use crate::analysis::{free_bodies, Analysis, PlotMode};
use crate::library::{clean_name, Library, Source, Tile};
use crate::editor::{BodyTemplate, ConstraintKind, Editor, EditorMode, LinkHover, StepMode};
use crate::sim::{
    body::BodyShape,
    constraint::Constraint,
    constraints::{
        cylinder::{GasMode, Stroke as GasStroke}, Cylinder, DistanceConstraint, GearJoint, GearKind,
        PinJoint, PinWorld, RollingContact, RollingOnRod, Rope, SliderJoint, WeldJoint,
    },
    forces::{spring::SpringDamper, Motor, TorsionSpring},
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
}

#[derive(Default)]
pub struct HudOutput {
    /// Start an empty scene.
    pub new_scene:    bool,
    /// The LOAD gallery was opened: (re)build its tiles.
    pub open_gallery: bool,
    pub load:         Option<Source>,
    /// SAVE with this (cleaned) name.
    pub save_as:      Option<String>,
    pub delete_saved: Option<String>,
    pub reset_scene:  bool,
    pub toggle_edit:  bool,
    pub undo:         bool,
    pub redo:         bool,
    pub share:        bool,
    pub load_code:    bool,
    /// Rewind to this timeline frame.
    pub scrub:        Option<usize>,
    pub toggle_chaos: bool,
    pub reseed_chaos: bool,
    /// A pointer button is held over the HUD (a slider being dragged).
    pub pointer_busy: bool,
    /// Editor clipboard buttons (same as Ctrl+C / X / V).
    pub copy:         bool,
    pub cut:          bool,
    pub paste:        bool,
    /// A part tile was dropped on the scene: place `editor.body_props` at the cursor.
    pub drop_part:    bool,
    /// A body picked in the Parts list; `true` with Ctrl (toggle it in the
    /// group selection).
    pub select_body:  Option<(usize, bool)>,
}

/// What the editor panel needs to know about undo.
#[derive(Clone, Copy, Default)]
pub struct UndoState { pub can_undo: bool, pub can_redo: bool }

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

    /// A window covers this point (physical pixels). Unlike egui's
    /// own pointer checks it needs no hover beforehand, so it is right for
    /// a finger that has just touched down.
    pub fn is_over_ui(&self, pos: glam::Vec2) -> bool {
        let ppp = self.ctx.pixels_per_point();
        // egui registers the whole screen as a background layer; only the
        // windows above it count.
        self.ctx.layer_id_at(egui::pos2(pos.x / ppp, pos.y / ppp))
            .is_some_and(|l| l.order != egui::Order::Background)
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
        scene_name:   &str,
        scene_desc:   &str,
        editor:       &mut Editor,
        world:        &mut World,
        analysis:     &mut Analysis,
        undo:         UndoState,
        library:      &mut Library,
    ) -> HudOutput {
        self.history.push(metrics);
        let raw_input = self.winit_state.take_egui_input(window);

        let mut out = HudOutput::default();
        let history = &self.history;
        let full_output = self.ctx.run(raw_input, |ctx| {
            out = draw_main_panel(ctx, metrics, history, scene_name, scene_desc, editor, analysis, library);
            if library.show_save { draw_save_panel(ctx, library, &mut out); }
            if library.show_load { draw_gallery(ctx, library, &mut out); }
            if editor.active {
                draw_editor_panel(ctx, editor, world, undo, &mut out);
            } else if analysis.show_timeline {
                draw_timeline(ctx, analysis, &mut out);
            }
            if analysis.show_share { draw_share_panel(ctx, analysis, &mut out); }
            if analysis.show_phase { draw_phase_panel(ctx, analysis, world); }
            if analysis.show_chaos { draw_chaos_panel(ctx, analysis, &mut out); }
            out.pointer_busy = ctx.input(|i| i.pointer.any_down()) && ctx.is_pointer_over_area();
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

    /// Make a rendered texture (a gallery thumbnail) drawable by egui.
    pub fn register_texture(&mut self, device: &wgpu::Device, view: &wgpu::TextureView) -> egui::TextureId {
        self.renderer.register_native_texture(device, view, wgpu::FilterMode::Linear)
    }

    pub fn free_texture(&mut self, id: egui::TextureId) {
        self.renderer.free_texture(&id);
    }

    /// Put text on the clipboard (native; the browser has no egui clipboard).
    pub fn copy_text(&self, s: String) {
        self.ctx.copy_text(s);
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
    scene_name:  &str,
    scene_desc:  &str,
    editor:      &mut Editor,
    analysis:    &mut Analysis,
    library:     &mut Library,
) -> HudOutput {
    let mut out = HudOutput::default();
    // Folded by default on a phone, where it would cover the whole scene.
    let mut folded = editor.main_folded.unwrap_or_else(|| narrow(ctx));

    egui::Area::new(egui::Id::new("hud"))
        .fixed_pos(egui::pos2(16.0, 16.0))
        .show(ctx, |ui| {
            panel_frame().show(ui, |ui| {
                ui.set_width(panel_width(ctx, 340.0));

                // Title + scene files
                ui.horizontal(|ui| {
                    ui.label(text(scene_name, TITLE_SIZE, WHITE));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let was = folded;
                        fold_button(ui, &mut folded);
                        if folded != was { editor.main_folded = Some(folded); }
                    });
                });
                if !folded { ui.label(text(scene_desc, SMALL_SIZE, MUTED)); }
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    let w = equal_width(ui, 3);
                    if icon_btn(ui, Icon::New, "NEW", false, w).on_hover_text("Start an empty scene").clicked() {
                        out.new_scene = true;
                    }
                    if icon_btn(ui, Icon::Load, "LOAD", library.show_load, w).on_hover_text("Pick a scene or one you saved").clicked() {
                        library.show_load = true;
                        out.open_gallery = true;
                    }
                    if icon_btn(ui, Icon::Save, "SAVE", library.show_save, w).on_hover_text("Keep this scene under a name").clicked() {
                        library.show_save = true;
                        library.save_focus = true;
                        library.save_name = scene_name.to_owned();
                        library.save_note.clear();
                    }
                });
                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    let w = equal_width(ui, 4);
                    if hud_btn(ui, "RESET", false, w).clicked()   { out.reset_scene = true; }
                    if hud_btn(ui, "EDIT", editor.active, w).clicked() { out.toggle_edit = true; }
                    let lbl = if editor.paused { "PLAY" } else { "PAUSE" };
                    if hud_btn(ui, lbl, editor.paused, w).clicked() { editor.paused = !editor.paused; }
                    if hud_btn(ui, "SHARE", analysis.show_share, w).clicked() {
                        analysis.show_share = !analysis.show_share;
                        if analysis.show_share { out.share = true; }
                    }
                });

                if folded { return; }
                ui.add_space(10.0);

                // INFO
                ui.label(text("Info", HEAD_SIZE, WHITE));
                kv_spark(ui, "FRAME RATE",  &format!("{:.0} FPS", m.fps), &h.fps);
                kv_spark(ui, "SAMPLE RATE", &format!("{:.0} HZ", m.fps * m.sub_steps as f32), &h.sr);
                kv_spark(ui, "STEPS",       &m.sub_steps.to_string(), &h.steps);
                kv_spark(ui, "ENERGY",      &format!("{:.4}", m.kinetic_energy + m.potential_energy), &h.energy);
                kv_spark(ui, "CONSTRAINT ERROR", &format!("{:.1E}", m.constraint_error), &h.c_err);

                ui.add_space(10.0);
                ui.label(text("Simulation", HEAD_SIZE, WHITE));
                ui.add_space(2.0);
                slider_row(ui, "GRAVITY", &mut editor.gravity, 0.0, 30.0);
                log_slider_row(ui, "SPEED", &mut editor.time_scale, 0.05, 20.0);
                ui.add_space(2.0);
                labelled(ui, "SOLVER", |ui| {
                    segmented(ui, &mut editor.integrator, &[(Integrator::Rk4, "RK4"), (Integrator::Xpbd, "XPBD")]);
                });
                ui.add_space(2.0);
                labelled(ui, "STEPS", |ui| {
                    segmented(ui, &mut editor.step_mode, &[(StepMode::TargetFps, "TARGET FPS"), (StepMode::Fixed, "FIXED")]);
                });
                ui.add_space(2.0);
                match editor.step_mode {
                    StepMode::Fixed => int_slider_row(ui, "COUNT", &mut editor.sub_steps, 1, 2000),
                    // Floor of 20: frame dt is capped at 0.05 s, below that the sim would lag real time.
                    StepMode::TargetFps => slider_row(ui, "TARGET", &mut editor.target_fps, 20.0, 240.0),
                }
                ui.add_space(10.0);
                ui.label(text("Show", HEAD_SIZE, WHITE));
                ui.add_space(2.0);
                // Two columns of switches.
                let col = equal_width(ui, 2);
                ui.horizontal(|ui| {
                    toggle_cell(ui, "FORCES", &mut analysis.show_forces, col);
                    toggle_cell(ui, "PHASE", &mut analysis.show_phase, col);
                });
                ui.horizontal(|ui| {
                    let mut chaos = analysis.show_chaos;
                    if toggle_cell(ui, "CHAOS", &mut chaos, col).clicked() { out.toggle_chaos = true; }
                    toggle_cell(ui, "REWIND", &mut analysis.show_timeline, col);
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

/// The editor as two windows stacked at the top right: Parts (adding new
/// things) and, right below it when something is selected, the selection
/// (changing what's there). The second sits at the first's measured bottom.
fn draw_editor_panel(ctx: &egui::Context, editor: &mut Editor, world: &mut World, undo: UndoState, out: &mut HudOutput) {
    let screen = ctx.screen_rect();
    // On a phone, under the main panel rather than over it.
    let parts_top = match ctx.memory(|m| m.area_rect(egui::Id::new("hud"))) {
        Some(main) if narrow(ctx) => main.bottom() + 12.0 - screen.top(),
        _ => 16.0,
    };
    let parts = egui::Area::new(egui::Id::new("editor_panel"))
        .anchor(Align2::RIGHT_TOP, egui::vec2(-16.0, parts_top))
        .show(ctx, |ui| {
            panel_frame().show(ui, |ui| {
                ui.set_width(panel_width(ctx, 270.0));
                parts_panel_contents(ui, editor, world, undo, out);
            });
        });

    editor.link_hover = None;
    let Some(title) = selection_title(editor, world) else { return };
    let top = parts.response.rect.bottom() + 12.0;
    egui::Area::new(egui::Id::new("selection_panel"))
        .anchor(Align2::RIGHT_TOP, egui::vec2(-16.0, top - screen.top()))
        .show(ctx, |ui| {
            // Never past the window bottom; overflow scrolls.
            let max_h = (screen.bottom() - top - 16.0 - 24.0).max(120.0);
            panel_frame().show(ui, |ui| {
                ui.set_width(panel_width(ctx, 270.0));
                ui.horizontal(|ui| {
                    ui.label(text(title, TITLE_SIZE, WHITE));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        fold_button(ui, &mut editor.selection_folded);
                    });
                });
                if editor.selection_folded { return; }
                ui.add_space(8.0);
                // Grow with the content; only scroll once it would pass the
                // window bottom. Without min_scrolled_height egui may shrink
                // the area to its 64px default whenever it has to scroll.
                egui::ScrollArea::vertical()
                    .max_height(max_h)
                    .min_scrolled_height(max_h)
                    .auto_shrink([false, true])
                    .show(ui, |ui| selection_panel_contents(ui, editor, world, out));
            });
        });
}

/// What the selection window is about, or `None` when there is nothing
/// selected (it is hidden then).
fn selection_title(editor: &Editor, world: &World) -> Option<String> {
    match editor.mode {
        EditorMode::Inspecting { body_idx } if editor.body_edit.is_some() && body_idx < world.bodies.len() => {
            Some(body_name(&world.bodies, body_idx))
        }
        EditorMode::Selection | EditorMode::GroupDragging { .. } => Some(format!("{} bodies", editor.selection.len())),
        EditorMode::BothSelected { .. } => Some("Connect with".to_owned()),
        EditorMode::PulleyPending { .. } => Some("Rope over pulley".to_owned()),
        EditorMode::PlacingTrace { .. } => Some("Place trace".to_owned()),
        _ => None,
    }
}

/// Parts window: undo / redo and the palette of new parts.
fn parts_panel_contents(ui: &mut egui::Ui, editor: &mut Editor, world: &World, undo: UndoState, out: &mut HudOutput) {
    ui.horizontal(|ui| {
        ui.label(text("Parts", TITLE_SIZE, WHITE));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            fold_button(ui, &mut editor.parts_folded);
            if dim_btn(ui, "REDO", undo.can_redo).clicked() { out.redo = true; }
            if dim_btn(ui, "UNDO", undo.can_undo).clicked() { out.undo = true; }
            if icon_square(ui, Icon::List, editor.show_body_list).on_hover_text("List every part in the scene").clicked() {
                editor.show_body_list = !editor.show_body_list;
            }
        });
    });
    if editor.parts_folded { return; }
    ui.add_space(8.0);

    // Drag a tile into the scene (or click it, then click the scene). New
    // parts use defaults; edit them once placed.
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 6.0;
        let w = (ui.available_width() - 12.0) / 3.0;
        for (tmpl, lbl) in [
            (BodyTemplate::Disk,   "DISK"),
            (BodyTemplate::Rod,    "ROD"),
            (BodyTemplate::Anchor, "ANCHOR"),
        ] {
            let placing = matches!(editor.mode, EditorMode::PlacingBody) && editor.body_props.template == tmpl;
            let resp = part_tile(ui, &tmpl, lbl, placing, w);
            if resp.drag_started() {
                editor.body_props.template = tmpl.clone();
                editor.mode = EditorMode::PlacingBody;
            } else if resp.clicked() {
                editor.mode = if placing { EditorMode::Idle } else { EditorMode::PlacingBody };
                editor.body_props.template = tmpl.clone();
            }
            if resp.drag_stopped() {
                // Dropped back on a panel: cancel.
                if ui.ctx().is_pointer_over_area() {
                    editor.mode = EditorMode::Idle;
                } else {
                    out.drop_part = true;
                }
            }
        }
    });
    // Shift for touch screens: while on, a tap picks a connection's bodies.
    ui.add_space(6.0);
    if hud_btn(ui, "CONNECT", editor.connect_mode, ui.available_width())
        .on_hover_text("Tap two bodies to connect them (same as Shift-click)")
        .clicked()
    {
        editor.connect_mode = !editor.connect_mode;
    }
    if editor.can_paste {
        ui.add_space(6.0);
        if hud_btn(ui, "PASTE", false, ui.available_width()).clicked() { out.paste = true; }
    }
    editor.list_hover = None;
    if editor.show_body_list {
        ui.add_space(8.0);
        body_list(ui, editor, world, out);
    }
}

/// Every body in the scene, for picking ones that are hard to click in the
/// scene: click selects, Ctrl-click adds to / removes from the group,
/// hovering highlights it in the scene.
fn body_list(ui: &mut egui::Ui, editor: &mut Editor, world: &World, out: &mut HudOutput) {
    let selected = editor.selected_bodies();
    if world.bodies.is_empty() {
        ui.label(text("Nothing in the scene yet", SMALL_SIZE, MUTED));
        return;
    }
    egui::ScrollArea::vertical()
        .id_salt("body_list")
        .max_height(220.0)
        .auto_shrink([false, true])
        .show(ui, |ui| {
            ui.spacing_mut().item_spacing.y = 2.0;
            for (i, b) in world.bodies.iter().enumerate() {
                let (rect, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 22.0), Sense::click());
                let is_sel = selected.contains(&i);
                let (bg, fg) = if is_sel {
                    (WHITE, BLACK)
                } else if resp.hovered() {
                    (Color32::from_gray(45), WHITE)
                } else {
                    (BLACK, Color32::from_gray(220))
                };
                let p = ui.painter();
                p.rect_filled(rect, Rounding::ZERO, bg);
                let icon = egui::Rect::from_center_size(egui::pos2(rect.left() + 12.0, rect.center().y), Vec2::splat(14.0));
                let shape_icon = match b.shape {
                    BodyShape::Disk { .. } => Icon::Disk,
                    BodyShape::Rod { .. } => Icon::Rod,
                    BodyShape::Point => Icon::Anchor,
                };
                paint_icon(p, icon, shape_icon, fg);
                p.text(egui::pos2(rect.left() + 26.0, rect.center().y), Align2::LEFT_CENTER,
                    body_name(&world.bodies, i).to_uppercase(), font(BODY_SIZE), fg);
                if b.fixed && !matches!(b.shape, BodyShape::Point) {
                    let muted = if is_sel { Color32::from_gray(90) } else { MUTED };
                    p.text(egui::pos2(rect.right() - 6.0, rect.center().y), Align2::RIGHT_CENTER, "FIXED", font(SMALL_SIZE), muted);
                }
                if resp.hovered() { editor.list_hover = Some(i); }
                if resp.clicked() {
                    let ctrl = ui.input(|inp| inp.modifiers.ctrl || inp.modifiers.command);
                    out.select_body = Some((i, ctrl));
                }
            }
        });
}

/// A small square button holding just an icon.
fn icon_square(ui: &mut egui::Ui, icon: Icon, active: bool) -> egui::Response {
    let (rect, resp) = ui.allocate_exact_size(Vec2::splat(22.0), Sense::click());
    let (bg, fg) = if active {
        (WHITE, BLACK)
    } else if resp.hovered() {
        (Color32::from_gray(45), WHITE)
    } else {
        (BLACK, WHITE)
    };
    ui.painter().rect(rect, Rounding::ZERO, bg, Stroke::new(BORDER_W, WHITE));
    paint_icon(ui.painter(), rect.shrink(4.0), icon, fg);
    resp
}

/// Square chevron that folds a window down to its title row and back.
fn fold_button(ui: &mut egui::Ui, folded: &mut bool) {
    let icon = if *folded { Icon::Unfold } else { Icon::Fold };
    let tip = if *folded { "Show the whole window" } else { "Fold the window to its title" };
    if icon_square(ui, icon, false).on_hover_text(tip).clicked() { *folded = !*folded; }
}

/// Window inner width: `want`, or less on a screen too narrow for it.
fn panel_width(ctx: &egui::Context, want: f32) -> f32 {
    want.min(ctx.screen_rect().width() - 32.0 - 24.0).max(160.0)
}

/// Too narrow for the main panel and the editor windows side by side (a
/// phone): the editor windows stack under the main panel instead.
fn narrow(ctx: &egui::Context) -> bool {
    ctx.screen_rect().width() < 340.0 + 270.0 + 3.0 * 16.0 + 4.0 * 12.0
}

/// Selection window: whatever is selected (one body, a group) or being
/// connected, below its title.
fn selection_panel_contents(ui: &mut egui::Ui, editor: &mut Editor, world: &mut World, out: &mut HudOutput) {
    if let EditorMode::Selection | EditorMode::GroupDragging { .. } = editor.mode {
        ui.label(text("Drag one to move all  ·  Ctrl+A selects all", SMALL_SIZE, MUTED));
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let w = equal_width(ui, 3);
            if hud_btn(ui, "COPY", false, w).clicked() { out.copy = true; }
            if hud_btn(ui, "CUT", false, w).clicked() { out.cut = true; }
            if danger_btn(ui, "DELETE", w).clicked() { editor.delete_requested = true; }
        });
    }

    // Connection picker
    if let EditorMode::BothSelected { .. } = editor.mode {
        for (lbl, kind) in [
            ("PIN JOINT",    ConstraintKind::PinJoint),
            ("WELD",         ConstraintKind::Weld),
            ("DISTANCE ROD", ConstraintKind::Distance),
            ("ROLLING",      ConstraintKind::RollingContact),
            ("GEAR",         ConstraintKind::Gear),
            ("BELT",         ConstraintKind::Belt),
            ("SLIDER",       ConstraintKind::Slider),
            ("CYLINDER",     ConstraintKind::Cylinder),
            ("ROPE",         ConstraintKind::Rope),
            ("ROPE OVER PULLEY", ConstraintKind::RopeOverPulley),
            ("SPRING",       ConstraintKind::Spring),
            ("TORSION SPRING", ConstraintKind::Torsion),
        ] {
            if kind == ConstraintKind::RollingContact && !editor.pair_can_roll {
                continue;
            }
            if matches!(kind, ConstraintKind::Slider | ConstraintKind::Cylinder) && !editor.pair_can_slide {
                continue;
            }
            if matches!(kind, ConstraintKind::Gear | ConstraintKind::Belt) && !editor.pair_can_gear {
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
        if !editor.pair_can_gear {
            ui.label(text("Gear and belt need two disks", SMALL_SIZE, MUTED));
        }
        if hud_btn(ui, "CANCEL", false, ui.available_width()).clicked() {
            editor.mode = EditorMode::Idle;
        }
    }

    if let EditorMode::PulleyPending { .. } = editor.mode {
        ui.label(text("Click the disk the rope runs over", SMALL_SIZE, MUTED));
        if hud_btn(ui, "CANCEL", false, ui.available_width()).clicked() {
            editor.mode = EditorMode::Idle;
        }
    }

    // Selected body
    if let EditorMode::Inspecting { body_idx } = editor.mode {
        let shape = editor.inspect_shape.clone().unwrap_or_else(|| "Body".into());
        if let Some(edit) = &mut editor.body_edit {
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
            ui.horizontal(|ui| {
                toggle_row(ui, "FIXED", &mut edit.fixed);
                if shape != "Point" { toggle_row(ui, "COLLIDE", &mut edit.collide); }
            });
            if shape != "Point" {
                let mut follow = world.follow == Some(body_idx);
                toggle_row(ui, "CAMERA FOLLOWS", &mut follow);
                if follow { world.follow = Some(body_idx); } else if world.follow == Some(body_idx) { world.follow = None; }
            }
            if edit.collide && shape != "Point" {
                slider_row(ui, "FRICTION", &mut edit.friction, 0.0, 1.5);
                slider_row(ui, "BOUNCE", &mut edit.restitution, 0.0, 1.0);
                labelled(ui, "PLANE", |ui| segmented(ui, &mut edit.plane, &[(0, "ALL"), (1, "1"), (2, "2"), (3, "3"), (4, "4")]));
            }
            // One-click world pins on the natural spots.
            if let Some(body) = world.bodies.get(body_idx) {
                let spots = crate::editor::quick_pin_points(body);
                if !spots.is_empty() {
                    ui.horizontal(|ui| {
                        for (lbl, local) in spots {
                            let mut on = crate::editor::world_pin_at(world, body_idx, local).is_some();
                            let was = on;
                            toggle_row(ui, lbl, &mut on);
                            if on != was { crate::editor::toggle_world_pin(world, body_idx, local); }
                        }
                    });
                }
            }
            ui.add_space(4.0);
            if danger_btn(ui, "DELETE", ui.available_width()).clicked() {
                editor.delete_requested = true;
            }
        }
    }

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
        ui.label(text("Click where the trail should be drawn from", SMALL_SIZE, MUTED));
        if hud_btn(ui, "CANCEL", false, ui.available_width()).clicked() {
            editor.mode = EditorMode::Inspecting { body_idx };
        }
    }
}

// ── Analysis panels ───────────────────────────────────────────────────────────

/// Scrub bar along the bottom: drag to rewind, PLAY resumes from there.
fn draw_timeline(ctx: &egui::Context, a: &mut Analysis, out: &mut HudOutput) {
    let n = a.timeline.len();
    let screen = ctx.screen_rect();
    let width = (screen.width() - 32.0).clamp(200.0, 720.0);
    egui::Area::new(egui::Id::new("timeline"))
        .anchor(Align2::CENTER_BOTTOM, egui::vec2(0.0, -16.0))
        .show(ctx, |ui| {
            panel_frame().inner_margin(Margin::symmetric(12.0, 8.0)).show(ui, |ui| {
                ui.set_width(width);
                ui.horizontal(|ui| {
                    ui.label(text("Rewind", BODY_SIZE, WHITE));
                    if n < 2 {
                        ui.label(text("No history yet // records while running", BODY_SIZE, MUTED));
                    } else {
                        let mut i = a.timeline.cursor.unwrap_or(n - 1);
                        let before = i;
                        ui.spacing_mut().slider_width = width - 220.0;
                        ui.add(egui::Slider::new(&mut i, 0..=n - 1).show_value(false));
                        let age = a.timeline.age(i);
                        let lbl = if a.timeline.cursor.is_some() { format!("-{age:.1} S") } else { "LIVE".to_owned() };
                        ui.label(text(lbl, BODY_SIZE, if a.timeline.cursor.is_some() { RED } else { WHITE }));
                        if i != before { out.scrub = Some(i); }
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if hud_btn(ui, "X", false, 22.0).clicked() { a.show_timeline = false; }
                    });
                });
            });
        });
}

/// SAVE: a small centred dialog naming the scene. The name starts
/// selected, Enter saves, Esc or a click outside cancels.
fn draw_save_panel(ctx: &egui::Context, lib: &mut Library, out: &mut HudOutput) {
    let modal = egui::Modal::new(egui::Id::new("save"))
        .frame(panel_frame())
        .backdrop_color(Color32::from_black_alpha(140))
        .show(ctx, |ui| {
            ui.set_width(320.0);
            ui.horizontal(|ui| {
                let (r, _) = ui.allocate_exact_size(Vec2::splat(18.0), Sense::hover());
                paint_icon(ui.painter(), r, Icon::Save, WHITE);
                ui.label(text("Save scene", HEAD_SIZE, WHITE));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if hud_btn(ui, "X", false, 22.0).clicked() { lib.show_save = false; }
                });
            });
            ui.add_space(8.0);
            ui.label(text("Name", SMALL_SIZE, MUTED));
            let mut edit = egui::TextEdit::singleline(&mut lib.save_name)
                .font(font(BODY_SIZE))
                .desired_width(f32::INFINITY)
                .show(ui);
            if lib.save_focus {
                lib.save_focus = false;
                edit.response.request_focus();
                let all = egui::text::CCursorRange::two(
                    egui::text::CCursor::new(0),
                    egui::text::CCursor::new(lib.save_name.chars().count()),
                );
                edit.state.cursor.set_char_range(Some(all));
                edit.state.store(ui.ctx(), edit.response.id);
            }
            let name = clean_name(&lib.save_name);
            let enter = edit.response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            ui.add_space(4.0);
            let (note, color) = if name.is_empty() {
                ("Type a name".to_owned(), MUTED)
            } else if !lib.save_note.is_empty() {
                (lib.save_note.clone(), RED)
            } else if lib.saved_names.contains(&name) {
                ("Replaces the saved scene with this name".to_owned(), RED)
            } else {
                ("Shows up under Saved in LOAD".to_owned(), MUTED)
            };
            ui.label(text(note, SMALL_SIZE, color));
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let half = (ui.available_width() - ui.spacing().item_spacing.x) / 2.0;
                if hud_btn(ui, "CANCEL", false, half).clicked() { lib.show_save = false; }
                let color = if name.is_empty() { Color32::from_gray(70) } else { WHITE };
                if (styled_btn(ui, "SAVE", !name.is_empty(), half, color).clicked() || enter) && !name.is_empty() {
                    out.save_as = Some(name);
                }
            });
        });
    if modal.should_close() { lib.show_save = false; }
}

const THUMB_W: f32 = 200.0;
const THUMB_H: f32 = 125.0;
/// Tile name + up to two description lines under the thumbnail.
const TILE_TEXT_H: f32 = 52.0;
const GAP: f32 = 14.0;

/// LOAD: every built-in scene, then the saved ones, as thumbnail tiles.
/// Sized explicitly from the screen, never from last frame's content.
fn draw_gallery(ctx: &egui::Context, lib: &mut Library, out: &mut HudOutput) {
    let screen = ctx.screen_rect();
    let margin = 40.0;
    let pad = 2.0 * 12.0; // panel_frame inner margin
    let scroll_bar = 16.0;
    let cols = (((screen.width() - 2.0 * margin - pad - scroll_bar + GAP) / (THUMB_W + GAP)).floor() as usize).clamp(1, 5);
    let grid_w = cols as f32 * (THUMB_W + GAP) - GAP;
    let header_h = 44.0;
    let body_h = (screen.height() - 2.0 * margin - pad - header_h).max(THUMB_H + TILE_TEXT_H);

    let modal = egui::Modal::new(egui::Id::new("gallery"))
        .frame(panel_frame())
        .backdrop_color(Color32::from_black_alpha(170))
        .show(ctx, |ui| {
            ui.set_width(grid_w + scroll_bar);
            ui.horizontal(|ui| {
                let (r, _) = ui.allocate_exact_size(Vec2::splat(20.0), Sense::hover());
                paint_icon(ui.painter(), r, Icon::Load, WHITE);
                ui.label(text("Load scene", TITLE_SIZE, WHITE));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if hud_btn(ui, "X", false, 22.0).clicked() { lib.show_load = false; }
                });
            });
            ui.add_space(10.0);
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .max_height(body_h)
                .min_scrolled_height(body_h)
                .show(ui, |ui| {
                    ui.set_width(grid_w);
                    if lib.tiles.is_empty() {
                        ui.label(text("Drawing thumbnails...", SMALL_SIZE, MUTED));
                        return;
                    }
                    let current = lib.current.clone();
                    let builtin: Vec<&Tile> = lib.tiles.iter().filter(|t| matches!(t.source, Source::Builtin(_))).collect();
                    let saved: Vec<&Tile> = lib.tiles.iter().filter(|t| matches!(t.source, Source::Saved(_))).collect();
                    ui.label(text("Scenes", HEAD_SIZE, WHITE));
                    ui.add_space(6.0);
                    gallery_grid(ui, &builtin, cols, current.as_ref(), out);
                    ui.add_space(10.0);
                    ui.label(text("Saved", HEAD_SIZE, WHITE));
                    ui.add_space(6.0);
                    if saved.is_empty() {
                        ui.label(text("Nothing saved yet // SAVE keeps the current scene here", SMALL_SIZE, MUTED));
                    }
                    gallery_grid(ui, &saved, cols, current.as_ref(), out);
                });
        });
    if modal.should_close() { lib.show_load = false; }
    if out.load.is_some() { lib.show_load = false; }
}

/// Rows of fixed-size tiles, allocated directly so nothing reflows.
fn gallery_grid(ui: &mut egui::Ui, tiles: &[&Tile], cols: usize, current: Option<&Source>, out: &mut HudOutput) {
    for row in tiles.chunks(cols) {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = GAP;
            for tile in row {
                gallery_tile(ui, tile, current == Some(&tile.source), out);
            }
        });
        ui.add_space(GAP);
    }
}

/// Thumbnail with the name and description under it; click to load.
/// Saved scenes get a delete corner on hover.
fn gallery_tile(ui: &mut egui::Ui, tile: &Tile, is_current: bool, out: &mut HudOutput) {
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(THUMB_W, THUMB_H + TILE_TEXT_H), Sense::click());
    let thumb = egui::Rect::from_min_size(rect.min, Vec2::new(THUMB_W, THUMB_H));
    let del = egui::Rect::from_min_size(egui::pos2(thumb.right() - 24.0, thumb.top() + 4.0), Vec2::splat(20.0));
    let del_resp = match &tile.source {
        Source::Saved(_) => Some(ui.interact(del, resp.id.with("delete"), Sense::click()).on_hover_text("Delete")),
        Source::Builtin(_) => None,
    };
    let on_delete = del_resp.as_ref().is_some_and(|r| r.hovered());
    let resp = resp.on_hover_cursor(egui::CursorIcon::PointingHand);
    let hovered = resp.hovered() || on_delete;

    let p = ui.painter();
    let uv = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
    p.image(tile.thumb, thumb, uv, if hovered { Color32::WHITE } else { Color32::from_gray(210) });
    let border = if hovered || is_current { Stroke::new(BORDER_W * 2.0, WHITE) } else { Stroke::new(BORDER_W, Color32::from_gray(80)) };
    p.rect_stroke(thumb, Rounding::ZERO, border);
    if is_current {
        let tag = egui::Rect::from_min_size(thumb.min, Vec2::new(66.0, 16.0));
        p.rect_filled(tag, Rounding::ZERO, WHITE);
        p.text(tag.center(), Align2::CENTER_CENTER, "CURRENT", font(SMALL_SIZE), BLACK);
    }

    // Name, and the description wrapped to at most two lines.
    let name_pos = egui::pos2(rect.left(), thumb.bottom() + 6.0);
    p.text(name_pos, Align2::LEFT_TOP, tile.name.to_uppercase(), font(BODY_SIZE), WHITE);
    if !tile.desc.is_empty() {
        let mut job = egui::text::LayoutJob::simple(tile.desc.to_uppercase(), font(SMALL_SIZE), MUTED, THUMB_W);
        job.wrap.max_rows = 2;
        let galley = ui.fonts(|f| f.layout_job(job));
        ui.painter().galley(egui::pos2(rect.left(), name_pos.y + 18.0), galley, MUTED);
    }

    if let (Source::Saved(name), Some(dr)) = (&tile.source, &del_resp) {
        if hovered {
            let (bg, fg) = if on_delete { (RED, BLACK) } else { (BLACK, RED) };
            ui.painter().rect(del, Rounding::ZERO, bg, Stroke::new(BORDER_W, RED));
            paint_icon(ui.painter(), del.shrink(4.0), Icon::Delete, fg);
        }
        if dr.clicked() {
            out.delete_saved = Some(name.clone());
            return;
        }
    }
    if resp.clicked() { out.load = Some(tile.source.clone()); }
}

#[derive(Clone, Copy)]
enum Icon { New, Load, Save, Delete, Power, List, Disk, Rod, Anchor, Fold, Unfold }

/// Line icons drawn in `r` on a 16 × 16 design grid, in the HUD's flat style.
fn paint_icon(p: &egui::Painter, r: egui::Rect, icon: Icon, color: Color32) {
    let u = r.width().min(r.height()) / 16.0;
    let c = r.center();
    let at = |x: f32, y: f32| egui::pos2(c.x + (x - 8.0) * u, c.y + (y - 8.0) * u);
    let st = Stroke::new((1.5 * u).max(1.2), color);
    match icon {
        Icon::New => {
            // Page with a folded corner and a plus.
            p.add(egui::Shape::line(vec![at(10.0, 1.5), at(2.5, 1.5), at(2.5, 14.5), at(13.5, 14.5), at(13.5, 5.0), at(10.0, 1.5), at(10.0, 5.0), at(13.5, 5.0)], st));
            p.line_segment([at(8.0, 7.5), at(8.0, 12.5)], st);
            p.line_segment([at(5.5, 10.0), at(10.5, 10.0)], st);
        }
        Icon::Load => {
            // Open folder.
            p.add(egui::Shape::line(vec![at(13.0, 6.5), at(13.0, 4.5), at(7.5, 4.5), at(6.0, 2.5), at(1.5, 2.5), at(1.5, 13.5)], st));
            p.add(egui::Shape::closed_line(vec![at(1.5, 13.5), at(4.0, 7.0), at(15.0, 7.0), at(12.5, 13.5)], st));
        }
        Icon::Save => {
            // Floppy disk: body with a cut corner, shutter, label.
            p.add(egui::Shape::closed_line(vec![at(1.5, 1.5), at(11.5, 1.5), at(14.5, 4.5), at(14.5, 14.5), at(1.5, 14.5)], st));
            p.add(egui::Shape::line(vec![at(4.5, 1.5), at(4.5, 5.5), at(10.5, 5.5), at(10.5, 1.5)], st));
            p.add(egui::Shape::line(vec![at(4.0, 14.5), at(4.0, 9.5), at(12.0, 9.5), at(12.0, 14.5)], st));
        }
        Icon::Power => {
            // Ring open at the top, with a stem through the gap.
            let r = 5.5 * u;
            let centre = at(8.0, 8.8);
            let gap = 0.65;
            let start = -std::f32::consts::FRAC_PI_2 + gap;
            let pts: Vec<egui::Pos2> = (0..=24)
                .map(|k| {
                    let a = start + (std::f32::consts::TAU - 2.0 * gap) * k as f32 / 24.0;
                    egui::pos2(centre.x + r * a.cos(), centre.y + r * a.sin())
                })
                .collect();
            p.add(egui::Shape::line(pts, st));
            p.line_segment([at(8.0, 1.5), at(8.0, 8.0)], st);
        }
        Icon::Fold => {
            // Chevron up: fold the window away.
            p.add(egui::Shape::line(vec![at(3.0, 10.5), at(8.0, 5.5), at(13.0, 10.5)], st));
        }
        Icon::Unfold => {
            p.add(egui::Shape::line(vec![at(3.0, 5.5), at(8.0, 10.5), at(13.0, 5.5)], st));
        }
        Icon::List => {
            for y in [3.5, 8.0, 12.5] {
                p.line_segment([at(1.5, y), at(3.0, y)], st);
                p.line_segment([at(5.5, y), at(14.5, y)], st);
            }
        }
        Icon::Disk => {
            p.circle_filled(at(8.0, 8.0), 6.0 * u, color);
        }
        Icon::Rod => {
            p.line_segment([at(2.0, 11.0), at(14.0, 5.0)], Stroke::new(3.5 * u, color));
        }
        Icon::Anchor => {
            p.add(egui::Shape::convex_polygon(vec![at(8.0, 3.0), at(13.0, 11.0), at(3.0, 11.0)], color, Stroke::NONE));
            p.line_segment([at(1.5, 13.5), at(14.5, 13.5)], st);
        }
        Icon::Delete => {
            p.line_segment([at(3.0, 3.0), at(13.0, 13.0)], st);
            p.line_segment([at(13.0, 3.0), at(3.0, 13.0)], st);
        }
    }
}

/// Button with a line icon before its label, the pair centred.
fn icon_btn(ui: &mut egui::Ui, icon: Icon, label: &str, active: bool, min_width: f32) -> egui::Response {
    let f = font(BODY_SIZE);
    let galley = ui.painter().layout_no_wrap(label.to_owned(), f.clone(), WHITE);
    let size = Vec2::new(min_width.max(galley.size().x + 40.0), 28.0);
    let (rect, resp) = ui.allocate_exact_size(size, Sense::click());
    let (bg, fg) = if active {
        (WHITE, BLACK)
    } else if resp.hovered() {
        (Color32::from_gray(45), WHITE)
    } else {
        (BLACK, WHITE)
    };
    let p = ui.painter();
    p.rect(rect, Rounding::ZERO, bg, Stroke::new(BORDER_W, WHITE));
    let content_w = 15.0 + 8.0 + galley.size().x;
    let x0 = rect.center().x - content_w / 2.0;
    paint_icon(p, egui::Rect::from_center_size(egui::pos2(x0 + 7.5, rect.center().y), Vec2::splat(15.0)), icon, fg);
    p.text(egui::pos2(x0 + 23.0, rect.center().y), Align2::LEFT_CENTER, label, f, fg);
    resp
}

fn draw_share_panel(ctx: &egui::Context, a: &mut Analysis, out: &mut HudOutput) {
    egui::Area::new(egui::Id::new("share"))
        .default_pos(egui::pos2(372.0, 16.0))
        .show(ctx, |ui| {
            panel_frame().show(ui, |ui| {
                ui.set_width(300.0);
                ui.horizontal(|ui| {
                    ui.label(text("Share", HEAD_SIZE, WHITE));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if hud_btn(ui, "X", false, 22.0).clicked() { a.show_share = false; }
                    });
                });
                ui.label(text("This scene as a code (or link, in the browser)", SMALL_SIZE, MUTED));
                code_box(ui, &mut a.share_code, false);
                ui.horizontal(|ui| {
                    if hud_btn(ui, "COPY AGAIN", false, 0.0).clicked() { out.share = true; }
                    ui.label(text(&a.share_note, SMALL_SIZE, MUTED));
                });
                ui.add_space(8.0);
                ui.label(text("Load // paste a code or link", SMALL_SIZE, MUTED));
                code_box(ui, &mut a.paste, true);
                if hud_btn(ui, "LOAD", false, ui.available_width()).clicked() { out.load_code = true; }
            });
        });
}

/// Monospace text box in the panel style; `editable` false still selects.
fn code_box(ui: &mut egui::Ui, s: &mut String, editable: bool) {
    let mut view = s.clone();
    let te = egui::TextEdit::multiline(if editable { s } else { &mut view })
        .font(font(SMALL_SIZE))
        .desired_rows(3)
        .desired_width(f32::INFINITY);
    egui::ScrollArea::vertical().id_salt(editable).max_height(70.0).show(ui, |ui| { ui.add(te); });
}

/// Phase portrait (θ, ω) of one body, or its Poincaré section.
fn draw_phase_panel(ctx: &egui::Context, a: &mut Analysis, world: &World) {
    let screen = ctx.screen_rect();
    a.phase.ensure_bodies(world);
    egui::Area::new(egui::Id::new("phase"))
        .default_pos(egui::pos2(16.0, (screen.height() - 360.0).max(16.0)))
        .show(ctx, |ui| {
            panel_frame().show(ui, |ui| {
                ui.set_width(300.0);
                ui.horizontal(|ui| {
                    ui.label(text("Phase space", HEAD_SIZE, WHITE));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if hud_btn(ui, "X", false, 22.0).clicked() { a.show_phase = false; }
                    });
                });
                let p = &mut a.phase;
                ui.horizontal(|ui| {
                    for (mode, lbl) in [(PlotMode::Phase, "TRAJECTORY"), (PlotMode::Poincare, "POINCARE")] {
                        if hud_btn(ui, lbl, p.mode == mode, 0.0).clicked() && p.mode != mode {
                            p.mode = mode;
                            p.clear();
                        }
                    }
                    if hud_btn(ui, "CLEAR", false, 0.0).clicked() { p.clear(); }
                });
                let free = free_bodies(world);
                if let Some(b) = body_picker(ui, "BODY", p.body, &free, &world.bodies) {
                    p.body = Some(b);
                    p.clear();
                }
                if p.mode == PlotMode::Poincare {
                    if let Some(b) = body_picker(ui, "SECTION", p.section, &free, &world.bodies) {
                        p.section = Some(b);
                        p.clear();
                    }
                    ui.label(text("A dot each time the section body's angle passes 0", SMALL_SIZE, MUTED));
                }

                let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 210.0), Sense::hover());
                let painter = ui.painter_at(rect);
                painter.rect_stroke(rect, Rounding::ZERO, Stroke::new(1.0_f32, Color32::from_gray(70)));
                let c = rect.center();
                painter.line_segment([egui::pos2(rect.left(), c.y), egui::pos2(rect.right(), c.y)], Stroke::new(1.0_f32, Color32::from_gray(40)));
                painter.line_segment([egui::pos2(c.x, rect.top()), egui::pos2(c.x, rect.bottom())], Stroke::new(1.0_f32, Color32::from_gray(40)));
                let w_max = p.w_max * 1.1;
                let to_screen = |(t, w): (f32, f32)| egui::pos2(
                    c.x + t / std::f32::consts::PI * rect.width() * 0.5,
                    c.y - w / w_max * rect.height() * 0.5,
                );
                // Trajectory: fading polyline, broken where θ wraps.
                let n = p.trail.len();
                for (k, (a0, a1)) in p.trail.iter().zip(p.trail.iter().skip(1)).enumerate() {
                    if (a1.0 - a0.0).abs() > std::f32::consts::PI { continue; }
                    let f = (k + 1) as f32 / n as f32;
                    let col = if p.mode == PlotMode::Phase {
                        Color32::from_white_alpha((40.0 + 215.0 * f) as u8)
                    } else {
                        Color32::from_white_alpha((12.0 + 30.0 * f) as u8)
                    };
                    painter.line_segment([to_screen(*a0), to_screen(*a1)], Stroke::new(1.0_f32, col));
                }
                for &d in &p.dots {
                    painter.circle_filled(to_screen(d), 1.2, RED);
                }
                if let Some(&last) = p.trail.back() {
                    painter.circle_filled(to_screen(last), 3.0, WHITE);
                }
                painter.text(rect.left_top() + egui::vec2(4.0, 2.0), Align2::LEFT_TOP,
                    format!("OMEGA {:+.1}", w_max), font(SMALL_SIZE), MUTED);
                painter.text(rect.right_bottom() - egui::vec2(4.0, 2.0), Align2::RIGHT_BOTTOM,
                    "ANGLE -PI..PI", font(SMALL_SIZE), MUTED);
                if p.mode == PlotMode::Poincare {
                    painter.text(rect.right_top() + egui::vec2(-4.0, 2.0), Align2::RIGHT_TOP,
                        format!("{} DOTS", p.dots.len()), font(SMALL_SIZE), MUTED);
                }
            });
        });
}

/// `LABEL < Disk 3 >`, cycling through `options`. Returns a new choice.
fn body_picker(ui: &mut egui::Ui, label: &str, current: Option<usize>, options: &[usize], bodies: &[crate::sim::body::Body]) -> Option<usize> {
    let mut pick = None;
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(58.0, 18.0), Sense::hover());
        ui.painter().text(rect.left_center(), Align2::LEFT_CENTER, label, font(BODY_SIZE), WHITE);
        let pos = current.and_then(|c| options.iter().position(|&o| o == c));
        if !options.is_empty() {
            let k = pos.unwrap_or(0);
            if hud_btn(ui, "<", false, 24.0).clicked() { pick = Some(options[(k + options.len() - 1) % options.len()]); }
            ui.label(text(current.map_or("None".to_owned(), |c| body_name(bodies, c)), BODY_SIZE, WHITE));
            if hud_btn(ui, ">", false, 24.0).clicked() { pick = Some(options[(k + 1) % options.len()]); }
        } else {
            ui.label(text("No free bodies", BODY_SIZE, MUTED));
        }
    });
    pick
}

/// Butterfly mode: divergence of the ghost and the Lyapunov estimate.
fn draw_chaos_panel(ctx: &egui::Context, a: &mut Analysis, out: &mut HudOutput) {
    let screen = ctx.screen_rect();
    egui::Area::new(egui::Id::new("chaos"))
        .default_pos(egui::pos2(332.0, (screen.height() - 300.0).max(16.0)))
        .show(ctx, |ui| {
            panel_frame().show(ui, |ui| {
                ui.set_width(280.0);
                ui.horizontal(|ui| {
                    ui.label(text("Butterfly", HEAD_SIZE, WHITE));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if hud_btn(ui, "X", false, 22.0).clicked() { out.toggle_chaos = true; }
                        if hud_btn(ui, "RESEED", false, 0.0).clicked() { out.reseed_chaos = true; }
                    });
                });
                let Some(bf) = &a.butterfly else {
                    ui.label(text("Starts when the simulation runs", SMALL_SIZE, MUTED));
                    return;
                };
                ui.label(text("Red ghost started 1E-9 away in velocity", SMALL_SIZE, MUTED));
                let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 120.0), Sense::hover());
                let painter = ui.painter_at(rect);
                painter.rect_stroke(rect, Rounding::ZERO, Stroke::new(1.0_f32, Color32::from_gray(70)));
                // log10 distance from -10 to +1 against time.
                let (lo, hi) = (-10.0f64, 1.0f64);
                for g in [-8.0, -6.0, -4.0, -2.0, 0.0] {
                    let y = rect.bottom() - ((g - lo) / (hi - lo)) as f32 * rect.height();
                    painter.line_segment([egui::pos2(rect.left(), y), egui::pos2(rect.right(), y)], Stroke::new(1.0_f32, Color32::from_gray(35)));
                }
                if let (Some(&(t0, _)), Some(&(t1, _))) = (bf.history.front(), bf.history.back()) {
                    let span = (t1 - t0).max(1e-6);
                    let pts: Vec<egui::Pos2> = bf.history.iter().map(|&(t, l)| egui::pos2(
                        rect.left() + ((t - t0) / span) as f32 * rect.width(),
                        rect.bottom() - ((l.clamp(lo, hi) - lo) / (hi - lo)) as f32 * rect.height(),
                    )).collect();
                    painter.add(egui::Shape::line(pts, Stroke::new(1.5_f32, RED)));
                }
                painter.text(rect.left_top() + egui::vec2(4.0, 2.0), Align2::LEFT_TOP, "LOG10 DISTANCE", font(SMALL_SIZE), MUTED);
                let d = bf.history.back().map_or(f64::NAN, |&(_, l)| l);
                ui.label(text(format!("Distance = 1E{d:.1}"), BODY_SIZE, WHITE));
                match bf.lyapunov {
                    Some(l) => ui.label(text(format!("Lyapunov = {l:.2} /S"), BODY_SIZE, WHITE)),
                    None => ui.label(text("Lyapunov = measuring...", BODY_SIZE, MUTED)),
                };
                let note = if bf.saturated() {
                    "Fully diverged // estimate frozen"
                } else if bf.lyapunov.is_some_and(|l| l < 0.05) {
                    "Not growing // regular motion"
                } else {
                    "Slope of log distance over time"
                };
                ui.label(text(note, SMALL_SIZE, MUTED));
            });
        });
}

// ── Connections inspector ─────────────────────────────────────────────────────

/// Load a connection breaks at when first made breakable (N).
const DEFAULT_BREAK_FORCE: f32 = 100.0;

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
        let mut on = c.on;
        let mut break_force = c.break_force;
        let broken = c.broken;
        let any = c.as_any_mut();
        let (title, far) = if let Some(pj) = any.downcast_ref::<PinJoint>() {
            let o = other(pj.body_a, pj.body_b);
            (format!("Pin joint > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else if let Some(wj) = any.downcast_ref::<WeldJoint>() {
            let o = other(wj.body_a, wj.body_b);
            (format!("Weld > {}", body_name(bodies, o)), LinkHover::Body(o))
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
        } else if let Some(g) = any.downcast_ref::<GearJoint>() {
            let o = other(g.body_a, g.body_b);
            let what = if g.kind == GearKind::Mesh { "Gear" } else { "Belt" };
            (format!("{what} > {}", body_name(bodies, o)), LinkHover::Body(o))
        } else if let Some(r) = any.downcast_ref::<Rope>() {
            if r.pulley == Some(body_idx) {
                (format!("Pulley for {} + {}", body_name(bodies, r.body_a), body_name(bodies, r.body_b)), LinkHover::Body(r.body_a))
            } else {
                let o = other(r.body_a, r.body_b);
                (format!("Rope > {}", body_name(bodies, o)), LinkHover::Body(o))
            }
        } else {
            ("Constraint".to_owned(), LinkHover::Body(body_idx))
        };

        let title = if broken { format!("{title} // broken") } else { title };
        let (delete, hovered) = link_card(ui, &title, Some(&mut on), |ui| {
            if any.is::<WeldJoint>() {
                ui.label(text("Locks position and angle // one rigid part", SMALL_SIZE, MUTED));
            } else if let Some(dc) = any.downcast_mut::<DistanceConstraint>() {
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
            } else if let Some(g) = any.downcast_mut::<GearJoint>() {
                ui.horizontal(|ui| {
                    for (kind, lbl) in [(GearKind::Mesh, "MESH"), (GearKind::Belt, "BELT")] {
                        if hud_btn(ui, lbl, g.kind == kind, 0.0).clicked() && g.kind != kind {
                            g.kind = kind;
                            g.rebase(bodies);
                        }
                    }
                });
                let r = g.ratio(bodies);
                let (wa, wb) = (body_name(bodies, g.body_a), body_name(bodies, g.body_b));
                ui.label(text(format!("{wb} turns {:.2}x {wa}{}", r.abs(), if r < 0.0 { ", reversed" } else { "" }), SMALL_SIZE, MUTED));
                if g.kind == GearKind::Belt {
                    toggle_row(ui, "COLLIDE", &mut g.collide);
                    if g.collide {
                        ui.label(text("Carries bodies marked collide, like a conveyor", SMALL_SIZE, MUTED));
                    }
                }
            } else if let Some(r) = any.downcast_mut::<Rope>() {
                slider_row(ui, "LENGTH", &mut r.length, 0.05, 12.0);
                if r.pulley.is_some() {
                    toggle_row(ui, "GRIPS PULLEY", &mut r.grip);
                }
                ui.label(text("Pulls, never pushes", SMALL_SIZE, MUTED));
            } else if any.is::<RollingContact>() || any.is::<RollingOnRod>() {
                ui.label(text("No slip // rotation locked to travel", SMALL_SIZE, MUTED));
            } else if !any.is::<PinJoint>() && !any.is::<PinWorld>() {
                ui.label(text("No parameters", SMALL_SIZE, MUTED));
            }
            let mut breakable = break_force.is_some();
            toggle_row(ui, "BREAKABLE", &mut breakable);
            match (breakable, &mut break_force) {
                (true, Some(f)) => log_slider_row(ui, "BREAK AT", f, 1.0, 5000.0),
                (true, f) => *f = Some(DEFAULT_BREAK_FORCE),
                (false, f) => *f = None,
            }
        });
        if on && !c.on { c.broken = false; }
        c.on = on;
        c.break_force = break_force;
        if delete { remove_constraint = Some(ci); }
        if hovered { hover = Some(far); }
    }

    let mut has_motor = false;
    let mount = crate::editor::axle_mount(world, body_idx);
    let bodies = &world.bodies;
    for (fi, f) in world.forces.iter_mut().enumerate() {
        if !f.body_indices().contains(&body_idx) { continue; }
        let mut on = f.on;
        if let Some(mo) = f.as_any_mut().downcast_mut::<Motor>() {
            // Listed on the driven body, not on the one it's mounted on.
            if mo.body != body_idx { continue; }
            count += 1;
            has_motor = true;
            let (delete, _) = link_card(ui, "Motor", Some(&mut on), |ui| {
                slider_row(ui, "TORQUE", &mut mo.torque, -50.0, 50.0);
                slider_row(ui, "DRAG",   &mut mo.drag,   0.0, 10.0);
                if let Some(mount) = mount {
                    let mut mounted = mo.stator.is_some();
                    toggle_row(ui, "MOUNTED", &mut mounted);
                    mo.stator = mounted.then_some(mount);
                    let on_what = if mounted { body_name(bodies, mount) } else { "world".to_owned() };
                    ui.label(text(format!("Pushes back on {on_what}"), SMALL_SIZE, MUTED));
                }
                ui.label(text("Drive torque minus drag x spin", SMALL_SIZE, MUTED));
            });
            f.on = on;
            if delete { remove_force = Some(fi); }
            continue;
        }
        if let Some(t) = f.as_any_mut().downcast_mut::<TorsionSpring>() {
            count += 1;
            let o = other(t.body_a, t.body_b);
            let title = format!("Torsion > {}", body_name(bodies, o));
            let mut rest = t.rest.to_degrees();
            let (delete, hovered) = link_card(ui, &title, Some(&mut on), |ui| {
                slider_row(ui, "STIFF", &mut t.stiffness, 0.5, 200.0);
                slider_row(ui, "DAMP",  &mut t.damping,   0.0, 10.0);
                slider_row(ui, "REST",  &mut rest, -180.0, 180.0);
                ui.label(text("Springs the relative angle", SMALL_SIZE, MUTED));
            });
            // Only write back a real change: deg → rad → deg isn't exact.
            if rest != t.rest.to_degrees() { t.rest = rest.to_radians(); }
            f.on = on;
            if delete { remove_force = Some(fi); }
            if hovered { hover = Some(LinkHover::Body(o)); }
            continue;
        }
        let Some(sd) = f.as_any_mut().downcast_mut::<SpringDamper>() else { continue };
        count += 1;
        let o = other(sd.body_a, sd.body_b);
        let title = format!("Spring > {}", body_name(bodies, o));
        let (delete, hovered) = link_card(ui, &title, Some(&mut on), |ui| {
            slider_row(ui, "STIFF", &mut sd.stiffness, 1.0, 500.0);
            slider_row(ui, "DAMP",  &mut sd.damping,   0.0, 50.0);
            slider_row(ui, "REST",  &mut sd.rest_len,  0.05, 8.0);
        });
        f.on = on;
        if delete { remove_force = Some(fi); }
        if hovered { hover = Some(LinkHover::Body(o)); }
    }

    if count == 0 {
        ui.label(text("None", SMALL_SIZE, MUTED));
    }
    if !has_motor && hud_btn(ui, "+ MOTOR", false, ui.available_width()).clicked() {
        let mut m = Motor::new(body_idx, 0.0, 0.0);
        m.stator = mount;
        world.add_force(m);
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
        let (delete, hovered) = link_card(ui, &format!("Trace {n}"), None, |ui| {
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

/// Bordered card with a title row, an on/off icon (when `on` is given)
/// and a small delete button. Returns (delete clicked, pointer over card).
fn link_card(ui: &mut egui::Ui, title: &str, on: Option<&mut bool>, body: impl FnOnce(&mut egui::Ui)) -> (bool, bool) {
    let mut delete = false;
    let enabled = on.as_ref().is_none_or(|v| **v);
    let resp = Frame::none()
        .stroke(Stroke::new(1.0_f32, MUTED))
        .inner_margin(Margin::same(6.0))
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.label(text(title, BODY_SIZE, if enabled { WHITE } else { MUTED }));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    delete = danger_btn(ui, "X", 22.0).clicked();
                    if let Some(on) = on { power_toggle(ui, on); }
                });
            });
            if enabled {
                body(ui);
            } else {
                ui.label(text("Off // not simulated", SMALL_SIZE, MUTED));
            }
        })
        .response;
    (delete, ui.rect_contains_pointer(resp.rect))
}

/// A small power icon that switches a connection on and off.
fn power_toggle(ui: &mut egui::Ui, on: &mut bool) {
    let (rect, resp) = ui.allocate_exact_size(Vec2::splat(22.0), Sense::click());
    let resp = resp.on_hover_text(if *on { "Switch off" } else { "Switch on" });
    if resp.clicked() { *on = !*on; }
    let color = match (*on, resp.hovered()) {
        (true, _) => WHITE,
        (false, true) => Color32::from_gray(170),
        (false, false) => Color32::from_gray(90),
    };
    if resp.hovered() {
        ui.painter().rect_filled(rect, Rounding::ZERO, Color32::from_gray(40));
    }
    paint_icon(ui.painter(), rect.shrink(4.0), Icon::Power, color);
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

/// Button that greys out when there's nothing to do.
fn dim_btn(ui: &mut egui::Ui, label: &str, enabled: bool) -> egui::Response {
    let r = styled_btn(ui, label, false, 0.0, if enabled { WHITE } else { Color32::from_gray(70) });
    if enabled { r } else { r.on_hover_text("") }
}

/// A palette tile: the part's outline above its name. Click or drag.
fn part_tile(ui: &mut egui::Ui, tmpl: &BodyTemplate, label: &str, active: bool, width: f32) -> egui::Response {
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(width, 58.0), Sense::click_and_drag());
    let resp = resp.on_hover_cursor(egui::CursorIcon::Grab);
    let (bg, fg) = if active {
        (WHITE, BLACK)
    } else if resp.hovered() {
        (Color32::from_gray(45), WHITE)
    } else {
        (BLACK, WHITE)
    };
    let p = ui.painter();
    p.rect(rect, Rounding::ZERO, bg, Stroke::new(BORDER_W, WHITE));
    let c = rect.center() - Vec2::new(0.0, 8.0);
    match tmpl {
        BodyTemplate::Disk => {
            p.circle_filled(c, 11.0, fg);
            p.circle_filled(c, 2.5, bg);
        }
        BodyTemplate::Rod => {
            p.line_segment([c - Vec2::new(16.0, 0.0), c + Vec2::new(16.0, 0.0)], Stroke::new(6.0_f32, fg));
            p.circle_filled(c - Vec2::new(16.0, 0.0), 3.0, fg);
            p.circle_filled(c + Vec2::new(16.0, 0.0), 3.0, fg);
        }
        BodyTemplate::Anchor => {
            let top = c - Vec2::new(0.0, 8.0);
            p.add(egui::Shape::convex_polygon(
                vec![top, c + Vec2::new(9.0, 6.0), c + Vec2::new(-9.0, 6.0)],
                fg, Stroke::NONE,
            ));
            p.line_segment([c + Vec2::new(-13.0, 9.0), c + Vec2::new(13.0, 9.0)], Stroke::new(2.0_f32, fg));
        }
    }
    p.text(rect.center_bottom() - Vec2::new(0.0, 11.0), Align2::CENTER_CENTER, label, font(SMALL_SIZE), fg);
    resp
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

/// Width of the label column of every labelled row.
const LABEL_W: f32 = 64.0;
/// Width of the value column at the right of slider rows.
const VALUE_W: f32 = 50.0;

/// Width that fits `n` controls side by side across the row.
fn equal_width(ui: &egui::Ui, n: usize) -> f32 {
    (ui.available_width() - (n as f32 - 1.0) * ui.spacing().item_spacing.x) / n as f32
}

/// A row with `label` in the label column and `content` filling the rest.
fn labelled<R>(ui: &mut egui::Ui, label: &str, content: impl FnOnce(&mut egui::Ui) -> R) -> R {
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(LABEL_W, 20.0), Sense::hover());
        ui.painter().text(rect.left_center(), Align2::LEFT_CENTER, label, font(BODY_SIZE), WHITE);
        content(ui)
    }).inner
}

/// Label, a slider stretched across the row, and its value right-aligned.
fn slider_layout(ui: &mut egui::Ui, label: &str, value: String, slider: impl FnOnce(&mut egui::Ui)) {
    labelled(ui, label, |ui| {
        let w = (ui.available_width() - VALUE_W - ui.spacing().item_spacing.x).max(40.0);
        ui.spacing_mut().slider_width = w;
        slider(ui);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width().min(VALUE_W), 20.0), Sense::hover());
        ui.painter().text(rect.right_center(), Align2::RIGHT_CENTER, value, font(BODY_SIZE), WHITE);
    });
}

fn slider_row(ui: &mut egui::Ui, label: &str, value: &mut f32, min: f32, max: f32) {
    let shown = format!("{value:.2}");
    slider_layout(ui, label, shown, |ui| { ui.add(egui::Slider::new(value, min..=max).show_value(false)); });
}

/// Like `slider_row` but logarithmic, for ranges spanning orders of magnitude.
fn log_slider_row(ui: &mut egui::Ui, label: &str, value: &mut f32, min: f32, max: f32) {
    let shown = format!("{value:.2}");
    slider_layout(ui, label, shown, |ui| { ui.add(egui::Slider::new(value, min..=max).logarithmic(true).show_value(false)); });
}

fn int_slider_row(ui: &mut egui::Ui, label: &str, value: &mut u32, min: u32, max: u32) {
    let shown = value.to_string();
    // Logarithmic so the low, common values stay easy to hit.
    slider_layout(ui, label, shown, |ui| { ui.add(egui::Slider::new(value, min..=max).logarithmic(true).show_value(false)); });
}

/// A one-of switch across the rest of the row: a bordered track with a
/// white thumb that slides to the chosen option.
fn segmented<T: PartialEq + Copy>(ui: &mut egui::Ui, value: &mut T, options: &[(T, &str)]) {
    let n = options.len().max(1);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 24.0), Sense::hover());
    let seg_w = rect.width() / n as f32;
    let id = ui.id().with(("segmented", options[0].1));
    let chosen = options.iter().position(|(v, _)| v == value).unwrap_or(0);
    let x = ui.ctx().animate_value_with_time(id, chosen as f32, 0.12);

    let p = ui.painter();
    p.rect(rect, Rounding::ZERO, BLACK, Stroke::new(BORDER_W, WHITE));
    let inset = 3.0;
    let thumb = egui::Rect::from_min_size(
        egui::pos2(rect.left() + seg_w * x + inset, rect.top() + inset),
        Vec2::new(seg_w - 2.0 * inset, rect.height() - 2.0 * inset),
    );
    for (i, (v, _)) in options.iter().enumerate() {
        let seg = egui::Rect::from_min_size(egui::pos2(rect.left() + seg_w * i as f32, rect.top()), Vec2::new(seg_w, rect.height()));
        let resp = ui.interact(seg, id.with(i), Sense::click());
        if resp.hovered() && i != chosen {
            ui.painter().rect_filled(seg.shrink(inset), Rounding::ZERO, Color32::from_gray(40));
        }
        if resp.clicked() { *value = *v; }
    }
    ui.painter().rect_filled(thumb, Rounding::ZERO, WHITE);
    for (i, (_, lbl)) in options.iter().enumerate() {
        // Black where the thumb covers the label, white elsewhere.
        let cover = (1.0 - (x - i as f32).abs()).clamp(0.0, 1.0);
        let g = (255.0 * (1.0 - cover)) as u8;
        let seg_c = egui::pos2(rect.left() + seg_w * (i as f32 + 0.5), rect.center().y);
        ui.painter().text(seg_c, Align2::CENTER_CENTER, *lbl, font(BODY_SIZE), Color32::from_gray(g));
    }
}

/// A small square switch: the knob slides right and the track fills when on.
fn paint_switch(ui: &egui::Ui, rect: egui::Rect, id: egui::Id, on: bool, hovered: bool) {
    let t = ui.ctx().animate_bool_with_time(id, on, 0.12);
    let p = ui.painter();
    let track_fill = if t > 0.5 { WHITE } else if hovered { Color32::from_gray(45) } else { BLACK };
    p.rect(rect, Rounding::ZERO, track_fill, Stroke::new(BORDER_W, WHITE));
    let k = rect.height() - 6.0;
    let x = rect.left() + 3.0 + t * (rect.width() - 6.0 - k);
    let knob = egui::Rect::from_min_size(egui::pos2(x, rect.top() + 3.0), Vec2::splat(k));
    p.rect_filled(knob, Rounding::ZERO, if t > 0.5 { BLACK } else { WHITE });
}

/// Switch then its label, as one clickable cell `width` wide (0: just
/// fits), so switches line up in columns and sit next to their text.
fn toggle_cell(ui: &mut egui::Ui, label: &str, value: &mut bool, width: f32) -> egui::Response {
    let f = font(BODY_SIZE);
    let galley = ui.painter().layout_no_wrap(label.to_owned(), f.clone(), WHITE);
    let sw = Vec2::new(30.0, 16.0);
    let gap = 8.0;
    let natural = sw.x + gap + galley.size().x;
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(width.max(natural), 22.0), Sense::click());
    let resp = resp.on_hover_cursor(egui::CursorIcon::PointingHand);
    if resp.clicked() { *value = !*value; }
    let sr = egui::Rect::from_min_size(egui::pos2(rect.left(), rect.center().y - sw.y / 2.0), sw);
    paint_switch(ui, sr, resp.id, *value, resp.hovered());
    let fg = if resp.hovered() { WHITE } else { Color32::from_gray(220) };
    ui.painter().text(egui::pos2(sr.right() + gap, rect.center().y), Align2::LEFT_CENTER, label, f, fg);
    resp
}

fn toggle_row(ui: &mut egui::Ui, label: &str, value: &mut bool) {
    toggle_cell(ui, label, value, 0.0);
    ui.add_space(10.0);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::Butterfly;
    use crate::scenes::SCENES;
    use crate::sim::forces::MouseSpringData;
    use std::sync::{Arc, Mutex};

    /// Lay out every panel, in every editor mode that has its own UI, for
    /// each scene, inspecting every body (so every connection card shows).
    #[test]
    fn panels_lay_out() {
        let ctx = egui::Context::default();
        install_style(&ctx);
        let mouse = Arc::new(Mutex::new(MouseSpringData::default()));
        for def in SCENES {
            let mut world = (def.build)();
            let mut editor = Editor { active: true, ..Default::default() };
            let mut a = Analysis { show_phase: true, show_chaos: true, show_share: true, show_forces: true, show_timeline: true, ..Default::default() };
            let mut lib = Library { show_load: true, show_save: true, save_name: "Mine".into(), saved_names: vec!["Mine".into()], ..Default::default() };
            for source in [Source::Builtin(0), Source::Saved("Mine".into())] {
                lib.tiles.push(Tile { source, name: "Mine".into(), desc: "A scene".into(), thumb: egui::TextureId::default() });
            }
            a.share_code = "psim1:abc".into();
            for _ in 0..30 {
                world.step(1.0 / 240.0);
                a.timeline.record(&world, 1.0 / 240.0);
                a.phase.ensure_bodies(&world);
                a.phase.after_frame(&world);
            }
            let mut bf = Butterfly::seed(&world, &mouse);
            bf.sample(&world, 0.01);
            a.butterfly = Some(bf);
            let metrics = Metrics { fps: 60.0, sub_steps: 10, kinetic_energy: 1.0, potential_energy: 2.0, constraint_error: 1e-6 };
            let history = History::default();
            let mut modes = vec![EditorMode::Idle, EditorMode::PlacingBody, EditorMode::Selection, EditorMode::BoxSelecting { start: glam::Vec2::ZERO }];
            editor.selection = (0..world.bodies.len()).collect();
            editor.can_paste = true;
            editor.show_body_list = true;
            if world.bodies.len() >= 2 {
                modes.push(EditorMode::BothSelected { body_a: 0, attach_a: glam::Vec2::ZERO, body_b: 1, attach_b: glam::Vec2::ONE });
                modes.push(EditorMode::PulleyPending { body_a: 0, attach_a: glam::Vec2::ZERO, body_b: 1, attach_b: glam::Vec2::ONE });
            }
            for i in 0..world.bodies.len() { modes.push(EditorMode::Inspecting { body_idx: i }); }
            for mode in modes {
                if let EditorMode::Inspecting { body_idx } = mode {
                    editor.body_edit = Some(crate::editor::BodyEdit::from_body(&world.bodies[body_idx]));
                    editor.inspect_shape = Some("Rod".into());
                }
                editor.mode = mode;
                for active in [true, false] {
                    editor.active = active;
                    let _ = ctx.run(egui::RawInput::default(), |ctx| {
                        let mut out = draw_main_panel(ctx, &metrics, &history, def.name, def.description, &mut editor, &mut a, &mut lib);
                        draw_save_panel(ctx, &mut lib, &mut out);
                        draw_gallery(ctx, &mut lib, &mut out);
                        if editor.active {
                            draw_editor_panel(ctx, &mut editor, &mut world, UndoState::default(), &mut out);
                        } else {
                            draw_timeline(ctx, &mut a, &mut out);
                        }
                        draw_share_panel(ctx, &mut a, &mut out);
                        draw_phase_panel(ctx, &mut a, &world);
                        draw_chaos_panel(ctx, &mut a, &mut out);
                    });
                }
            }
        }
    }
}
