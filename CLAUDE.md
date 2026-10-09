# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
cargo build          # compile
cargo run            # launch the simulation window
cargo check          # fast type-check without linking
cargo clippy         # lints
cargo test --release bench_scenes -- --ignored --nocapture   # physics benchmark
PROBE="Scene Name" cargo test --release probe -- --ignored --nocapture --exact scenes::probe::probe   # one scene's energy, error, extent, contacts, joint load over time
trunk serve          # browser build at http://127.0.0.1:8080 (trunk build --release → dist/)
cargo check --target wasm32-unknown-unknown   # type-check the web build
./android.sh build   # Android APK → target/release/apk/qonstraint.apk (./android.sh run installs and starts it over USB)
```

Web build needs the wasm target and trunk (Arch: `pacman -S rust-wasm trunk`). It uses WebGPU and falls back to WebGL2. Everything web-specific sits behind `cfg(target_arch = "wasm32")` (window/canvas setup, async init via a `UserEvent` and no `ControlFlow::Poll` — it starves Firefox's animation frames — in `lib.rs`, WebGL2 limits and the sRGB `view_format` in `renderer/state.rs`, refitting the camera on the first resize in `app.rs`); keep the native path free of web changes. Use `web_time::Instant`, never `std::time::Instant` (it panics on wasm).

Android build: `android.sh` runs `cargo apk` (install: `cargo install cargo-apk`) through a user-local rustup in `~/.cargo/bin` with the `aarch64-linux-android` / `x86_64-linux-android` targets (the Arch system Rust has none), the SDK in `~/Android/Sdk` and its newest NDK; cargo-apk needs an SDK platform the NDK supports (NDK 26: `platforms;android-34`). Release, signed with `~/.android/debug.keystore`. APK settings are `[package.metadata.android]` in `Cargo.toml`. The app is a library (`qonstraint`, `src/lib.rs`, `crate-type = cdylib + rlib`) so Android can load it through `android_main`; `src/main.rs` (the `physics_sim` binary trunk builds, `data-bin` in `index.html`) just calls `qonstraint::main`. Android-specific: `Suspended` drops the wgpu surface and `Resumed` recreates it on the same device (`RenderState::drop_surface` / `recreate_surface`; no redraws in between), Vulkan is tried before OpenGL ES and any backend whose surface won't configure is skipped (`RenderState::with_backends`), device limits are the WebGL2-level ones as on the web, saved scenes go to the app's internal data folder (`library::set_data_dir`), logs and panics go to logcat (tag `qonstraint`), and the egui clipboard (share codes, copy/paste text) does nothing there.

The app opens a wgpu window. Pan with middle-mouse drag, zoom with scroll wheel. Left-click drag to pull bodies with a mouse spring.

Touch (`App::on_touch`; winit delivers touch as `WindowEvent::Touch`, never as mouse events): egui sees every finger; one that lands off the panels (`Hud::is_over_ui`, a layer hit test that needs no prior hover and skips egui's full-screen background layer) is replayed into `App::scene_event` as the left mouse button, so picking, the mouse spring and editor gestures work unchanged. A second finger cancels that press and the pair pans and pinch-zooms; outside the editor one finger on empty space pans. The Parts window's CONNECT switch (`Editor::connect_mode`) stands in for Shift when picking connections. On narrow screens (`hud::narrow`) the main panel starts folded to its title and button rows and the editor windows stack under it instead of beside it; every HUD window has a fold chevron (`Editor::main_folded` / `parts_folded` / `selection_folded`) and shrinks to fit (`panel_width`). `index.html` sets `touch-action: none` on the canvas.

## Architecture

Rust library plus a thin binary (see the Android paragraph). Top-level modules: `sim` (physics), `renderer` (wgpu + egui), `scenes` (scene definitions), `app` (input/update/render loop), `editor`, `analysis` (rewind, butterfly, phase plot, undo), `scene_file` (save/load/share).

### App (`src/app.rs`)

`App` owns `RenderState`, `Camera`, `GeometryBuilder`, `World`, and `Hud`. `update()` runs 4 sub-steps per frame (`dt/4` each). `handle_event()` processes pan/zoom/mouse-spring input. `render()` builds geometry, calls `render_geometry`, then encodes the egui HUD pass. Scene switching (`load_scene`) resets world, camera and traces from the `SCENES` table.

Camera follow: `World::follow` names a body (a vehicle) the camera pans along with while simulating outside the editor (`App::follow_camera`: its x exactly, its height eased); the inspector's CAMERA FOLLOWS toggle sets it, scene files save it, deleting the body clears it.

Scene files (main panel NEW / LOAD / SAVE, `library.rs`): NEW loads the last `SCENES` entry (empty, opens in edit mode). SAVE stores `App::scene_file` JSON by name (native: `$XDG_DATA_HOME`/`~/.local/share` or `%APPDATA%` `/qonstraint/scenes/<name>.json`; web: `localStorage` `qonstraint.scene.<name>`) and makes it the scene RESET returns to (`custom` / `custom_name`). LOAD is an egui `Modal` gallery of `library::Tile`s: built-in scenes (all but the empty one), then saved ones with DELETE. Thumbnails are drawn by `draw_world` through `RenderState::render_to_texture` and registered with egui (`Hud::register_texture`); since they share the renderer's uniform and vertex buffers, `App::build_gallery` runs at the start of a frame (`gallery_pending`), never mid-frame.

Pause: `Editor::paused` gates physics in `App::update` (PAUSE/PLAY button, Space). Entering edit mode pauses and leaving resumes, but it can run while editing: a grabbed body is held in place (`hold_grabbed_body`), the ANGLE slider follows the simulated angle unless moved (`BodyEdit::angle_synced`), and finishing a drag/rotate/resize while running calls `World::rebase`.

Editor rotation and sizing: the selected body has a rotate handle (round knob, Ctrl snaps 15°), an ANGLE slider, and R / Shift+R turn it 15°; diamond resize handles (`editor::resize_handles` / `resize_to`: disk radius on the rim, rod length past each end — the opposite end stays put — rod width off its face; Ctrl snaps). All size changes go through `editor::set_shape`, which scales every attachment on the body (pins, links, springs, slider/cylinder points, traces) so end and rim attachments stay on the end or rim; they mirror the RADIUS / LENGTH / WIDTH sliders. `editor::set_angle_degrees` changes the angle by the smallest step so accumulated spin is kept.

Analysis and sharing (`App::analysis`, an `analysis::Analysis`; main panel SHOW row FORCES / PHASE / CHAOS / REWIND, SHARE next to PAUSE):
- **Rewind**: `Timeline` records `World::save_state` (body states plus `Constraint::save_state`, e.g. the engine cycle) once per frame while simulating outside the editor, last 30 s, whether or not the bar is open; the bottom REWIND bar (only when toggled on) scrubs (pauses, `seek`), PLAY truncates the future and continues. Cleared on structure changes and on leaving the editor.
- **Butterfly** (CHAOS): `Butterfly` holds a ghost `World` cloned via `scene_file::clone_world`, velocities nudged 1e-9, stepped in lockstep in `App::update`, drawn by `scenes::draw_ghost`; the panel plots log10 phase-space distance and a least-squares Lyapunov estimate over the exponential phase. Dropped on edits, scrubs and scene loads.
- **Phase** panel: `PhasePlot`, (θ, ω) trajectory of one body, or a Poincaré section (a dot when the section body's angle crosses 0 forward, checked after every step).
- **Forces**: sets `World::record_reactions`; `scenes::force_tints` colours rods by axial load (red tension, blue compression), `draw_force_arrows` draws joint and contact forces; both auto-scale (`smooth_scale`).
- **Share**: `SceneFile` (serde records for every body/constraint/force type, per-body contact material and plane, breakable joints, the followed body, gear phases (so a reload continues exactly), plus view; serde_json with `float_roundtrip`, so positions survive bit for bit; old codes' world-wide `friction` / `restitution` fill in bodies without their own) → JSON → deflate → base64url `psim1:` code. Native copies to the clipboard; the web build puts it in the URL hash `#s=` (and the clipboard), loads it at start and on hash change. A loaded scene shows as "Shared Scene"; RESET reloads it.
- **Undo/redo** (editor UNDO/REDO, Ctrl+Z / Ctrl+Shift+Z / Ctrl+Y): `UndoStack` of JSON snapshots, committed each frame the editor is paused and no button is held, when the world changed. Restores go through `App::replace_world`.

Editor parts: the PARTS palette (DISK / ROD / ANCHOR tiles, `hud::part_tile`) is dragged into the scene (`HudOutput::drop_part`, placed at the cursor, Ctrl snaps) or clicked then the scene clicked (`EditorMode::PlacingBody`); new bodies and springs use defaults (`BodyProps::default`, `Editor::spring_k` / `torsion_k`) and are edited afterwards in the inspector and connection cards. `App::handle_event` tracks the cursor even over the HUD so the dragged ghost follows it.

Connection cards (body inspector) have a power icon (`power_toggle`) that switches the connection's `Slot::on` without deleting it, and every constraint card a BREAKABLE toggle with a BREAK AT force (`Slot::break_force`; a broken joint's card says so, and switching it back on mends it). The motor card shows MOUNTED when the body's axle is pinned to another body (`editor::axle_mount`): the motor then pushes back on that body (`Motor::stator`). With COLLIDE on, the inspector also has a PLANE switch (`Body::plane`). Editor overlays (selection brackets, box select, handles) use `SELECT` sky blue so they show on white parts.

Editor selection: a left drag on empty space is a box select (`EditorMode::BoxSelecting`; bodies whose centres are inside): one body opens the inspector, more make `EditorMode::Selection` with `Editor::selection`. Dragging a selected body moves the group (`GroupDragging`; a click without moving selects just that body); Ctrl+A selects all, Delete deletes (`App::delete_selected`). Ctrl+C / Ctrl+X / Ctrl+V (and COPY / CUT / PASTE buttons) go through `scene_file::extract` (the bodies plus every constraint, force and trace involving only them, as a `SceneFile` in `App::clipboard`) and `scene_file::paste_into` (centred on the cursor, indices shifted like `bench::merged`, gravity not doubled, world pins moved along); the pasted bodies become the selection.

Editor connections: Shift-click two bodies (holding Shift sets `Editor::connect_ready`: red hover brackets and attach reticle; attach reticles and the link line are red over a `BG` halo so they show on white parts), then PIN JOINT, WELD, DISTANCE ROD, ROLLING, GEAR / BELT (two disks), SLIDER, CYLINDER, ROPE, ROPE OVER PULLEY (then click the pulley disk: `EditorMode::PulleyPending`), SPRING or TORSION SPRING. FRICTION / BOUNCE sliders show in the body inspector only while COLLIDE is on. The body inspector's PIN CENTRE (disk) and PIN LEFT/RIGHT (or BOTTOM/TOP) (rod ends) toggles add or remove a `PinWorld` at that spot (`editor::toggle_world_pin`), so no anchor needs placing. Every world pin on a body follows it through editor drags, turns and resizes (`editor::sync_world_pins`, called only from editor actions, never while simulating).

Mouse interaction: left-click picks a body (hit-tests disks and rods), attaches a `MouseSpring` force via `Arc<Mutex<MouseSpringData>>` shared between `App` and the force.

### Physics (`src/sim/`)

**`body.rs`** — `Body` struct: pos, angle, vel, ang_vel, force/torque accumulators in **f64** (with thousands of steps per frame, f32 loses a large share of every `v·dt` increment to rounding), mass, inertia and shape in f32; `collide` opts it into contacts (editor COLLIDE toggle), `friction` / `restitution` are its contact material, `plane` its depth plane for contacts (0 = all planes; others only meet their own plane and plane 0, so overlapping parts can sit side by side front to back, like a walker's legs or wheels tucked under a chassis). UI/render code reads `pos32()` / `angle32()` / `world_point(local: Vec2) -> Vec2`; sim code uses `world_point_d`, `rot()` (a cached `(cos, sin)` `Rot`) and `point_vel(r)`. Helper fns `rod_inertia` and `disk_inertia`. `apply_force_at_world_point` accumulates both force and torque correctly.

**`constraint.rs`** — `Constraint` trait. One call, `evaluate(bodies, vel, &mut ConstraintEval)`, fills C, the Jacobian blocks (`JBlock { body, j[row][col] }`, col = x, y, θ, at most `MAX_DIM = 3` rows and `MAX_BLOCKS = 3` blocks; XPBD ignores cross terms if one body appears in two blocks, so don't do that) and, if `vel`, Ċ and J̇q̇ (`bias`), so shared geometry is computed once and nothing allocates. `n_blocks = 0` marks a momentarily degenerate constraint. `Attach` (a body point: world position, offset `r`, `block`, `vel`, `centripetal`) is the shared helper for position constraints.

**`constraints/`** — `PinJoint` (body-to-body, 2 eqs), `WeldJoint` (pin plus the relative angle held: 3 eqs; builds compound parts from disks and rods, e.g. a toothed wheel or a domino on a foot), `PinWorld` (body to fixed world point, 2 eqs), `DistanceConstraint` (1 eq), `RollingContact` (disk on disk), `RollingOnRod` (disk on a possibly moving rod's face; `on` drops in `post_step` once the contact passes a rod end so the disk falls, and comes back with `k` re-captured if it lands on the same face; saved in `save_state` and the scene file) `Cylinder` (gas cylinder between a barrel rod — head at its local +x end, bore = its half width — and a piston body whose point rides the barrel axis with rotation locked; gas pushes piston and head apart. `GasMode::Sealed` is an adiabatic air spring, `TwoStroke`/`FourStroke` run a combustion cycle that times itself from the piston's own dead centres in `post_step`, igniting at TDC, so it works in any mechanism; `begin(stroke)` sets the phase, `state()` feeds the drawing) and `SliderJoint` (a point on any body rides a rod's centre line, rotation free or locked via `lock_rotation`; travel is limited by end stops implemented as stiff damped bumpers in `Constraint::apply_forces`, the hook for one-sided effects an equality solver can't express). The on-rail rows and the bumpers are shared helpers (`eval_on_rail`, `rail_stops`) that `Cylinder` reuses. `Constraint::post_step(&mut self)` runs once after each full step for discrete state (the engine cycle); `Constraint::potential_energy` adds stored energy (sealed gas) to `World::potential_energy`. `GearJoint` (two disks, `GearKind::Mesh` opposite turning or `Belt` same way; a belt with `collide` is a contact surface, see `contact.rs` — also an internal gear: one row r_a(θ_a−φ) ± r_b(θ_b−φ) = k with φ the line of centres, so planets on carriers are right; world frame when coaxial; only rotation is constrained, axles are held by pins). `Rope` (end points on two bodies, optionally `over(pulley)` any disk: strands to tangent points plus the wrapped arc, unwrapped by whole turns; one-sided — inactive once slack beyond `SLACK_TOL`; row 1 is no-slip on the pulley rim when `grip` and the pulley is free, so a heavy pulley turns; 3 blocks). Both rolling constraints hold no-slip at *position* level (rotation locked to rolled distance via an offset `k`), not just velocity level, so it can't drift. They keep reference state, re-captured by `Constraint::rebase`, which `App::toggle_editor` calls on leaving edit mode because editor moves teleport bodies. Tests for both live next to them (`cargo test`).

**`contact.rs`** — `Contacts` (owned by `World`; material is per body, `Body::friction` / `restitution`, a pair uses √(μa·μb) and the larger restitution): collisions between bodies with `Body::collide` (disks as circles, rods as capsules, anchors never; pairs sharing any constraint never collide). Stiff damped penalty normal forces (ω = 200, capped at ω·dt ≤ 0.5 like the slider stops, damping from restitution) plus Coulomb friction with a stick spring per contact (`post_step` advances it and clamps at μ·N), so blocks below the friction angle don't creep. Colliding belts (`GearJoint::collide`): each straight run (`gear::belt_strands`) is a capsule of `BELT_WIDTH` whose side of the contact (`BeltSide`) takes its velocity from, and puts its reaction on, the two pulleys at the run's tangent points weighted by position along the run, so friction carries bodies like a conveyor and loads the drive; the wrapped arcs are circles round pulleys that don't collide themselves. Called in both integrators next to `apply_forces`; `prepare` once per step, `last` holds the contacts for the force overlay.
- Broad phase: `prepare` builds the candidate pairs once per step by sort-and-sweep along x (a persistent, insertion-sorted list keyed by (start, index)), each body's bound padded by twice its speed × dt; the integrator stages and `post_step` only run the exact tests on those. Stick springs are kept sorted for lookup.
- Exclusions: besides pairs sharing a constraint, bodies meeting at one pin node (members pinned in a chain at a truss joint: pin and weld ends merged by union–find) and pairs in different planes. A pair whose joint breaks or is switched off stays out of contact (`separating`) until the two no longer overlap, so freed parts drift apart instead of being blasted apart.
- Welded pieces: the springs are tuned to the effective mass of the whole welded piece a body belongs to (`find_pieces`: union–find over welds, mass, centre and inertia per piece), and a contact force on a welded body is spread over its piece as one rigid body (`push`), so a light tooth on a heavy wheel neither lets things sink through nor gets kicked by a spring meant for the wheel.
- Capsules (belt runs, a belt's wrap round a pulley) test ends both ways against a rod, then the middles, as rod–rod pairs do.

**`solver.rs`** — `WitkinSolver` (owned by `World`) implements Witkin eq. 11: `JWJ^T λ = -J̇q̇ - JWQ - ks·C - kd·Ċ`, solved **exactly** by a sparse envelope LDLᵀ. Constraints are ordered by reverse Cuthill–McKee over the "shares a free body" graph; ordering and layout are rebuilt only when that graph changes (a signature is compared each call). Near-zero pivots (redundant rows in over-constrained loops) get λ = 0; pivots that have lost almost all but not all of their diagonal are held at `PIVOT_FLOOR` (1e-5) of it, which bounds the multipliers of nearly dependent rows (a parallelogram linkage at its change point while a redundant twin holds it, e.g. a locomotive's two quartered coupling rods); well-conditioned systems never reach it. All buffers persist, so a solve allocates nothing. Scatters `J^T λ` into the accumulators. Feedback constants `KS=150`, `KD=15`. `reactions` exposes Jᵀλ per constraint and block (`constraint::Reaction`); `World` records them on RK4 stage 0 (XPBD: Jᵀ·Δλ/dt²) when `record_reactions` is set.

**`slot.rs`** — `Slot<T>` wraps every entry of `World::constraints` / `forces`: derefs to the item (so reading code is unchanged) plus `on`, the editor's per-connection power toggle, and for constraints `break_force` / `broken`: `World::step` records reactions whenever a breakable joint exists and switches off (`broken`) any whose largest force on a body exceeds its limit (`World::break_overloaded`; `World::break_last_at` makes the last added constraint breakable). `save_state` carries whether each breakable joint holds, so a rewind mends it; scene files save `breaks`; a broken joint isn't drawn at all. Off: the solvers treat it as `n_blocks = 0`, its forces / `apply_forces` / `post_step` / potential energy are skipped, contacts stop excluding its bodies, `constraint_error` ignores it, and `draw_world` draws only a faint dashed trace (`draw_switched_off`). Saved as `SceneFile::switched_off` (item indices).

**`world.rs`** — `World` holds bodies, constraints, forces and an `integrator` (`Integrator::Rk4`, the default, or `Xpbd`; the HUD SOLVER buttons set it via `Editor::integrator`). RK4 writes each stage straight into the bodies and folds derivatives into a running sum (no per-step allocation). `follow` is the camera-follow body. `rebase()` re-captures constraint reference state and flags the next XPBD step to settle positions first; `App::toggle_editor` calls it. Also: `kinetic_energy()`, `potential_energy(g)` (gravitational + spring elastic), `constraint_error()` returns `|C|`.

**`xpbd.rs`** — Position-based "small steps" integrator: symplectic-Euler predict, one Gauss–Seidel sweep projecting each constraint (its rows solved together with `(J W Jᵀ) Δλ = −C`, reusing `evaluate`), velocities from the position change. 4–10× cheaper per step than RK4 and holds constraints tighter, but bleeds energy (worse at low step counts), which is why RK4 is the default for these energy-sensitive scenes. Its single sweep diverges when a light body holds many heavier parts (a hub with ten welded spokes and a fraction of their inertia, a tiny pulley between heavy loads): give such hubs at least the inertia of what hangs on them, and keep link masses within a reasonable ratio of the loads they carry.

**`forces/`** — `Gravity` (adds `m·g` downward), `Motor` (torque − drag·ω on one body: drive, brake or load; editor: + MOTOR in the body inspector; with a `stator` (`Motor::on`) it pushes back on that body and its drag acts on the relative spin, so a vehicle's drive is internal), `SpringDamper` (damped spring between two body attachment points), `TorsionSpring` (damped spring on the relative angle of two bodies, rest captured at creation; `local_a` is only where its spiral is drawn), `MouseSpring` (interactive drag force, reads from `Arc<Mutex<MouseSpringData>>`).

**`force.rs`** — `Force` trait: `apply(&self, bodies: &mut [Body])` and `potential_energy(&self, bodies: &[Body]) -> f64` (default 0.0).

### Renderer (`src/renderer/`)

**`state.rs`** — wgpu setup with 4× MSAA. `render_geometry(camera, geo)` uploads `GeometryBuilder` buffers and runs two render passes: grid (fullscreen triangle + SDF shader) then geometry. Returns `(output, surface_view, encoder)` so the caller can append further passes (e.g. egui HUD).

**`geometry.rs`** — `GeometryBuilder` builds CPU triangle lists each frame. Methods: `draw_line`, `draw_rod` (line + rounded end-caps), `draw_circle` (triangle fan), `draw_arc`, `draw_spring` (zigzag with end crossbars). Vertex layout: `[position: [f32;2], color: [f32;4]]`.

**`camera.rs`** — Orthographic camera with pan/zoom. `view_proj(w, h)` and `inv_view_proj` are both uploaded as uniforms. `screen_to_world` used for zoom-towards-cursor.

**`hud.rs`** — egui overlay rendered after the geometry pass, styled as terminal panels (black fill, hard white border, uppercase JetBrains Mono ExtraBold bundled from `assets/fonts/`, red only for warnings/delete). Main panel: scene title, `NEW LOAD SAVE` then `RESET EDIT PAUSE SHARE` buttons, INFO (frame rate, sample rate, steps, energy, constraint error, each with a sparkline; labels are spelled out, no abbreviations), Simulation (GRAVITY / SPEED sliders, SOLVER and STEPS switches, step count or target slider; stored on `Editor`, read by `App::update`) and SHOW. Rows span the panel: `labelled` (fixed `LABEL_W` label column), `slider_layout` (slider stretched, value right-aligned in `VALUE_W`), `equal_width` for button rows, `segmented` (one-of, sliding thumb) and `toggle_cell` / `toggle_row` (label + square switch, `paint_switch`) for on/off settings. In edit mode, two windows stacked at the top right (`draw_editor_panel`): Parts (UNDO / REDO, a list button that opens `body_list`: every body, click selects, Ctrl-click toggles it in the group, hover highlights it in the scene via `Editor::list_hover`; the part palette, PASTE) for adding things, and right under it, placed at Parts' measured bottom, the selection window (`selection_panel_contents`, titled by `selection_title`: the body's name, "N bodies", "Connect with", ...) for changing what's selected; it's hidden when nothing is. Returns `HudOutput` flags that `App::render` acts on.

**`shaders/`** — `grid.wgsl`: fullscreen quad, unprojects to world space, SDF grid lines. `geometry.wgsl`: simple vertex transform + pass-through color.

### Scenes (`src/scenes/`)

`SceneDef` bundles `name`, `description`, `build: fn() -> World`, `camera_center`, `view_size`. The static `SCENES` slice holds all scenes in order:

| Index | Name | Description |
|-------|------|-------------|
| 0 | Rocker Linkage | Disk rolls on a sprung lever (held by contact, not a constraint: both COLLIDE, friction 1.2, bounce 0.05), coupled via a rod to a rocker with a small disk (default) |
| 1 | Radial Engine | 5-cylinder 4-stroke radial from editor parts (flywheel, fixed barrels, pistons, rods, Cylinders, Motor drag as load), firing 1-3-5-2-4 |
| 2 | Double Pendulum | Two links and a bob, chaotic |
| 3 | Elastic Pendulum | Spring pendulum near 2:1 resonance |
| 4 | Coupled Pendulums | Weak spring, energy beats between two pendulums |
| 5 | Rolling Crane | Wheel rolls under a fixed rail, double pendulum hangs from it |
| 6 | Planetary Pendulum | Disk rolls around a fixed disk, pendulum on its rim |
| 7 | Trammel | Bar slides on two crossed rails, tip traces an ellipse |
| 8 | Air Struts | Hub bounces on two sealed Cylinders (air springs), pendulum below; lossless chaos |
| 9 | Strandbeest | Two mirrored Jansen legs on one motor-driven crank, triangles as three pinned rods, feet traced |
| 10 | Peaucellier-Lipkin | Swinging crank + inverting rhombus, P on an exact straight line |
| 11 | Watt's Linkage | Torsion-sprung arms, coupler midpoint near-straight with figure-eight ends |
| 12 | Kapitza's Pendulum | Motor flywheel + conrod shakes a carriage on a rail; inverted pendulum stands (velocities start consistent) |
| 13 | Gear Train | Motor pinion 3:1 gear, 1:2 belt, crank-rocker four-bar |
| 14 | Swinging Atwood | Rope over a pulley, counterweight vs swinging mass (μ = 1.8, chosen to stay clear of the pulley) |
| 15 | Tumbler | Contacts: closed box, motor paddle, disks and bars |
| 16 | Flexible Beam | Cantilever of rods on torsion-sprung hinges |
| 17 | Pendulum Wave | 15 ideal pendulums tuned to 15..29 swings in 30 s; back in step after 30 s (`realigns_after_period` test) |
| 18 | Barton's Pendulums | Heavy driver and seven light pendulums on a beam sliding on a rail between springs; the length-matched one (retuned for the beam's give) resonates |
| 19 | Galton Board | 200 balls through 11 rows of fixed pegs into bins (broad-phase stress test) |
| 20 | Soft Bodies | Six blobs: rings of colliding disks on damped springs, shelves and a motor paddle in a tub |
| 21 | Domino Cascade | Ten dominoes (bar welded on a foot), each 1.32× the last, tapped by a pendulum |
| 22 | Pumpjacks | Three crank-rocker pumpjacks: belt-driven cranks with welded counterweights, horsehead disk welded to the beam, bridle rope over it to a sucker rod on a slider |
| 23 | Planetary Press | Sun, three planets on a carrier, fixed internal ring (drawn with inward teeth); carrier crank drives a ram with a sprung punch over a colliding conveyor carrying blocks into a bin |
| 24 | Geneva Drive | Crank pin (welded disk) on a heavy flywheel hub indexes a four-slot wheel (welded bars, walls ending at the pin's exit radius, flared mouth guides) through contact; a belt-driven eccentric cam rocks a sprung roller lever |
| 25 | Pendulum Clock | Weight-driven anchor escapement: spike-toothed escape wheel (welded), slanted bar pallets welded to the pendulum, endless rope over a drum belted from a pinion; one tooth per period |
| 26 | Rimless Wheel | Ten welded spokes rolling down a slope into a limit-cycle gait, then onto the flat; camera follows |
| 27 | Truss Bridge | Warren deck truss, members pinned in chains at nodes, every pin breakable; a motor cart crosses and it folds into the river |
| 28 | Trebuchet | Hinged counterweight, 3:1 arm, breakable rope sling that releases at its breaking pull; the stone flies ~20 m into a block wall |
| 29 | Walking Strandbeest | Four Jansen legs on a free frame (two cranks half a turn apart on a chain, motor mounted on the frame), each leg's feet in their own plane; camera follows |
| 30 | Steam Locomotive | Three coupled drivers on a rail by friction, two quartered coupling rods, a 2-stroke Cylinder welded to the frame on the middle driver, tender and two coaches; camera follows |
| 31 | Hot Rod | Exposed inline four (geared cranks, 1-3-4-2) belted to the rear wheel, sprung swing arms, rolling ground; camera follows |
| 32 | Tank | Closed track of pinned links round sprocket (motor mounted on the hull), idler and road wheels, over two logs; camera follows |
| 33 | Rube Goldberg | Ball, ramps, dominoes, a breakable trapdoor latch, a weight on a rope over a pulley lifting a gate, balls into a basket, a gear waving a flag |
| 34 | Untitled | Empty; what NEW opens, in edit mode; not in the gallery |

Scenes give a `view_size` (world w × h) rather than a zoom; the camera is fitted to the window on load, and refitted on every window resize until the user pans or zooms (`App::camera_fitted`), which also fixes the start-up framing when the real window size arrives late. Shared builders (`disk`, `rod_between`, `anchor`, `from_down`) live in `scenes/mod.rs`. Scenes can register `World::tracers` (body-local points) which the app draws as fading red trails; the editor's TRACE toggle adds one too. `scenes_are_stable` runs every scene for 20 s under both integrators and checks constraint error, energy and that bodies stay in view (judged from the followed body for scenes with `follow`), reporting every failing scene at once; run it after changing a scene. `scenes/probe.rs` holds ignored dev tests: `probe` (one scene over time; `PROBE_INTEG=xpbd`, `PROBE_T`, `PROBE_EVERY`, `PROBE_BODIES=1,4` with positions and peak displacement, `PROBE_FAST` for the fastest body, `PROBE_CONTACTS=k` for body k's contacts, `PROBE_DUMP=file PROBE_AT=t1,t2` for every body's shape and pose, and a line at every joint break) and `probe_reload` (where a world and its reloaded copy part). `scenes/bench.rs` (ignored test `bench_scenes`) times µs/step and accuracy at a fixed frame budget for every scene plus larger stress worlds (merged scene copies, a 20-link chain). Initial velocities must agree with the constraints (e.g. a disk spinning on its own pin), otherwise Baumgarte has to fight a startup transient.

**Scenes are built only from general, editor-usable elements** (bodies, constraints, forces the user can also place and edit); never add a part that exists for one scene — generalise it and expose it in the editor instead. Each scene module only has a `build()` function. All scenes render through the shared `draw_world` in `scenes/mod.rs`, so every element type looks identical everywhere: flat white parts with a background-coloured (`BG`) outline separating overlaps, pin bosses at every joint/spring attachment, a square plate with four bolts at every weld, and pedestal anchors (fixed bodies that collide are scenery, ground, walls, pegs, and get no pedestal) (ceiling- or floor-mounted depending on whether they sit above a reference height: the free bodies' centre of mass, tracked live while editing and frozen by `App` while simulating so mounts never flip mid-run). Meshing gears get interleaved teeth (`gear_phases` works out each train's tooth phase, internal pairs included), a disk that is the outer member of a belt-kind pair with the other inside it is drawn as an internal ring gear with inward teeth (`ring_gears`, `draw_ring_gear`), belts a band with marks that travel, ropes a thin line (sagging when slack) over their pulley arc, torsion springs a spiral that winds with the twist, driving motors an end-on rotor hub over the body's centre: dark disk, thin white rim, three white chevrons that turn with the body and point the torque's way, white axle dot (`draw_motor`; drag-only motors, brakes and loads, aren't drawn). Cylinders get finned barrels, a gas column coloured by stroke and pressure, lifting valves, a spark plug that flashes at ignition, and a ringed piston drawn over the piston body and its rod. Slider rails get a dark groove between end stops and each rider a carriage (pin boss if rotation is free, bolt heads if locked). Don't add per-scene drawing; extend `draw_world` instead.

## Solver Math Reference

The constraint force equation (Witkin 1997, eq. 11):

```
JWJ^T λ = -J̇q̇ - JWQ - ks·C - kd·Ċ
Constraint force: Q̂ = J^T λ
```

Where:
- `J` = Jacobian of C w.r.t. q (block-sparse, each constraint contributes `dim × 3` blocks per affected body)
- `W` = diagonal inverse-mass matrix `[1/m, 1/m, 1/I]` per body
- `Q` = applied forces vector
- `ks`, `kd` = Baumgarte drift-feedback constants (not physically meaningful, prevent accumulation)

## Adding a New Constraint

1. Create `src/sim/constraints/my_constraint.rs` implementing the `Constraint` trait
2. Implement `evaluate`: fill `c`, `blocks` (`j[row][col]` = `∂C_row/∂q_col`) and `n_blocks`; when `vel` is set also `c_dot` and `bias` (J̇q̇, the centripetal/Coriolis terms). Use `Attach` for body points.
3. Both integrators pick it up automatically; run `cargo test` (tests cover both).
4. Export from `src/sim/constraints/mod.rs`
5. Make it a full editor part: an `Item` in `scene_file.rs` (or sharing/undo silently drop it), `ConstraintKind` + picker button + connection card in `hud.rs`, `scale_attachments` / `connection_handles` / `move_handle` in `editor.rs` for body-local points, its look in `draw_world` and, if it joins two points, an entry in `force_sites`. Any reference state that drifts while simulating (like `GearJoint`'s phase) belongs in the record too, or a reload won't continue exactly.

## Adding a New Scene

1. Create `src/scenes/my_scene.rs` with `pub fn build() -> World`
2. Declare the module in `src/scenes/mod.rs`
3. Add a `SceneDef` entry to the `SCENES` slice in `src/scenes/mod.rs`

## Adding a New Force

Implement the `Force` trait (`apply(&self, bodies: &mut [Body])`) and optionally `potential_energy`. Call `body.apply_force_at_world_point(force, world_point)` to accumulate both force and torque.
