# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
cargo build          # compile
cargo run            # launch the simulation window
cargo check          # fast type-check without linking
cargo clippy         # lints
cargo test --release bench_scenes -- --ignored --nocapture   # physics benchmark
trunk serve          # browser build at http://127.0.0.1:8080 (trunk build --release → dist/)
cargo check --target wasm32-unknown-unknown   # type-check the web build
```

Web build needs the wasm target and trunk (Arch: `pacman -S rust-wasm trunk`). It uses WebGPU and falls back to WebGL2. Everything web-specific sits behind `cfg(target_arch = "wasm32")` (window/canvas setup and async init via a `UserEvent` in `main.rs`, WebGL2 limits and the sRGB `view_format` in `renderer/state.rs`, refitting the camera on the first resize in `app.rs`); keep the native path free of web changes. Use `web_time::Instant`, never `std::time::Instant` (it panics on wasm).

The app opens a wgpu window. Pan with middle-mouse drag, zoom with scroll wheel. Left-click drag to pull bodies with a mouse spring.

## Architecture

Single Rust binary. Top-level modules: `sim` (physics), `renderer` (wgpu + egui), `scenes` (scene definitions), `app` (input/update/render loop).

### App (`src/app.rs`)

`App` owns `RenderState`, `Camera`, `GeometryBuilder`, `World`, and `Hud`. `update()` runs 4 sub-steps per frame (`dt/4` each). `handle_event()` processes pan/zoom/mouse-spring input. `render()` builds geometry, calls `render_geometry`, then encodes the egui HUD pass. Scene switching (`load_scene`) resets world, camera and traces from the `SCENES` table.

Pause: `Editor::paused` gates physics in `App::update` (PAUSE/PLAY button, Space). Entering edit mode pauses and leaving resumes, but it can run while editing: a grabbed body is held in place (`hold_grabbed_body`), the ANGLE slider follows the simulated angle unless moved (`BodyEdit::angle_synced`), and finishing a drag/rotate/resize while running calls `World::rebase`.

Editor rotation and sizing: the selected body has a rotate handle (round knob, Ctrl snaps 15°), an ANGLE slider, and R / Shift+R turn it 15°; diamond resize handles (`editor::resize_handles` / `resize_to`: disk radius on the rim, rod length past each end — the opposite end stays put — rod width off its face; Ctrl snaps). All size changes go through `editor::set_shape`, which scales every attachment on the body (pins, links, springs, slider/cylinder points, traces) so end and rim attachments stay on the end or rim; they mirror the RADIUS / LENGTH / WIDTH sliders. `editor::set_angle_degrees` changes the angle by the smallest step so accumulated spin is kept.

Mouse interaction: left-click picks a body (hit-tests disks and rods), attaches a `MouseSpring` force via `Arc<Mutex<MouseSpringData>>` shared between `App` and the force.

### Physics (`src/sim/`)

**`body.rs`** — `Body` struct: pos, angle, vel, ang_vel, force/torque accumulators in **f64** (with thousands of steps per frame, f32 loses a large share of every `v·dt` increment to rounding), mass, inertia and shape in f32. UI/render code reads `pos32()` / `angle32()` / `world_point(local: Vec2) -> Vec2`; sim code uses `world_point_d`, `rot()` (a cached `(cos, sin)` `Rot`) and `point_vel(r)`. Helper fns `rod_inertia` and `disk_inertia`. `apply_force_at_world_point` accumulates both force and torque correctly.

**`constraint.rs`** — `Constraint` trait. One call, `evaluate(bodies, vel, &mut ConstraintEval)`, fills C, the Jacobian blocks (`JBlock { body, j[row][col] }`, col = x, y, θ, at most `MAX_DIM = 3` rows and 2 blocks) and, if `vel`, Ċ and J̇q̇ (`bias`), so shared geometry is computed once and nothing allocates. `n_blocks = 0` marks a momentarily degenerate constraint. `Attach` (a body point: world position, offset `r`, `block`, `vel`, `centripetal`) is the shared helper for position constraints.

**`constraints/`** — `PinJoint` (body-to-body, 2 eqs), `PinWorld` (body to fixed world point, 2 eqs), `DistanceConstraint` (1 eq), `RollingContact` (disk on disk), `RollingOnRod` (disk on a possibly moving rod's face, rod treated as an infinite line) `Cylinder` (gas cylinder between a barrel rod — head at its local +x end, bore = its half width — and a piston body whose point rides the barrel axis with rotation locked; gas pushes piston and head apart. `GasMode::Sealed` is an adiabatic air spring, `TwoStroke`/`FourStroke` run a combustion cycle that times itself from the piston's own dead centres in `post_step`, igniting at TDC, so it works in any mechanism; `begin(stroke)` sets the phase, `state()` feeds the drawing) and `SliderJoint` (a point on any body rides a rod's centre line, rotation free or locked via `lock_rotation`; travel is limited by end stops implemented as stiff damped bumpers in `Constraint::apply_forces`, the hook for one-sided effects an equality solver can't express). The on-rail rows and the bumpers are shared helpers (`eval_on_rail`, `rail_stops`) that `Cylinder` reuses. `Constraint::post_step(&mut self)` runs once after each full step for discrete state (the engine cycle); `Constraint::potential_energy` adds stored energy (sealed gas) to `World::potential_energy`. Both rolling constraints hold no-slip at *position* level (rotation locked to rolled distance via an offset `k`), not just velocity level, so it can't drift. They keep reference state, re-captured by `Constraint::rebase`, which `App::toggle_editor` calls on leaving edit mode because editor moves teleport bodies. Tests for both live next to them (`cargo test`).

**`solver.rs`** — `WitkinSolver` (owned by `World`) implements Witkin eq. 11: `JWJ^T λ = -J̇q̇ - JWQ - ks·C - kd·Ċ`, solved **exactly** by a sparse envelope LDLᵀ. Constraints are ordered by reverse Cuthill–McKee over the "shares a free body" graph; ordering and layout are rebuilt only when that graph changes (a signature is compared each call). Near-zero pivots (redundant rows in over-constrained loops) get λ = 0. All buffers persist, so a solve allocates nothing. Scatters `J^T λ` into the accumulators. Feedback constants `KS=150`, `KD=15`.

**`world.rs`** — `World` holds bodies, constraints, forces and an `integrator` (`Integrator::Rk4`, the default, or `Xpbd`; the HUD SOLVER buttons set it via `Editor::integrator`). RK4 writes each stage straight into the bodies and folds derivatives into a running sum (no per-step allocation). `rebase()` re-captures constraint reference state and flags the next XPBD step to settle positions first; `App::toggle_editor` calls it. Also: `kinetic_energy()`, `potential_energy(g)` (gravitational + spring elastic), `constraint_error()` returns `|C|`.

**`xpbd.rs`** — Position-based "small steps" integrator: symplectic-Euler predict, one Gauss–Seidel sweep projecting each constraint (its rows solved together with `(J W Jᵀ) Δλ = −C`, reusing `evaluate`), velocities from the position change. 4–10× cheaper per step than RK4 and holds constraints tighter, but bleeds energy (worse at low step counts), which is why RK4 is the default for these energy-sensitive scenes.

**`forces/`** — `Gravity` (adds `m·g` downward), `Motor` (torque − drag·ω on one body: drive, brake or load; editor: + MOTOR in the body inspector), `SpringDamper` (damped spring between two body attachment points), `MouseSpring` (interactive drag force, reads from `Arc<Mutex<MouseSpringData>>`).

**`force.rs`** — `Force` trait: `apply(&self, bodies: &mut [Body])` and `potential_energy(&self, bodies: &[Body]) -> f64` (default 0.0).

### Renderer (`src/renderer/`)

**`state.rs`** — wgpu setup with 4× MSAA. `render_geometry(camera, geo)` uploads `GeometryBuilder` buffers and runs two render passes: grid (fullscreen triangle + SDF shader) then geometry. Returns `(output, surface_view, encoder)` so the caller can append further passes (e.g. egui HUD).

**`geometry.rs`** — `GeometryBuilder` builds CPU triangle lists each frame. Methods: `draw_line`, `draw_rod` (line + rounded end-caps), `draw_circle` (triangle fan), `draw_arc`, `draw_spring` (zigzag with end crossbars). Vertex layout: `[position: [f32;2], color: [f32;4]]`.

**`camera.rs`** — Orthographic camera with pan/zoom. `view_proj(w, h)` and `inv_view_proj` are both uploaded as uniforms. `screen_to_world` used for zoom-towards-cursor.

**`hud.rs`** — egui overlay rendered after the geometry pass, styled as terminal panels (black fill, hard white border, uppercase JetBrains Mono ExtraBold bundled from `assets/fonts/`, red only for warnings/delete). Main panel: scene title, `< > RESET EDIT` buttons, INFO (frame rate, sample rate, steps, energy, constraint error, each with a sparkline; labels are spelled out, no abbreviations), GRAVITY / STEPS / SPEED sliders (stored on `Editor`, read by `App::update`), `RUNNING // OKAY` status line. Editor panel on the right when edit mode is on. Returns `HudOutput` flags that `App::render` acts on.

**`shaders/`** — `grid.wgsl`: fullscreen quad, unprojects to world space, SDF grid lines. `geometry.wgsl`: simple vertex transform + pass-through color.

### Scenes (`src/scenes/`)

`SceneDef` bundles `name`, `description`, `build: fn() -> World`, `camera_center`, `view_size`. The static `SCENES` slice holds all scenes in order:

| Index | Name | Description |
|-------|------|-------------|
| 0 | Rocker Linkage | Disk rolls on a sprung lever, coupled via a rod to a rocker with a small disk (default) |
| 1 | Radial Engine | 5-cylinder 4-stroke radial from editor parts (flywheel, fixed barrels, pistons, rods, Cylinders, Motor drag as load), firing 1-3-5-2-4 |
| 2 | Double Pendulum | Two links and a bob, chaotic |
| 3 | Elastic Pendulum | Spring pendulum near 2:1 resonance |
| 4 | Coupled Pendulums | Weak spring, energy beats between two pendulums |
| 5 | Rolling Crane | Wheel rolls under a fixed rail, double pendulum hangs from it |
| 6 | Planetary Pendulum | Disk rolls around a fixed disk, pendulum on its rim |
| 7 | Trammel | Bar slides on two crossed rails, tip traces an ellipse |
| 8 | Air Struts | Hub bounces on two sealed Cylinders (air springs), pendulum below; lossless chaos |
| 9 | Sandbox | Empty; opens in edit mode |

Scenes give a `view_size` (world w × h) rather than a zoom; the camera is fitted to the window on load, and refitted on every window resize until the user pans or zooms (`App::camera_fitted`), which also fixes the start-up framing when the real window size arrives late. Shared builders (`disk`, `rod_between`, `anchor`, `from_down`) live in `scenes/mod.rs`. Scenes can register `World::tracers` (body-local points) which the app draws as fading red trails; the editor's TRACE toggle adds one too. `scenes_are_stable` runs every scene for 20 s under both integrators and checks constraint error, energy and that bodies stay in view; run it after changing a scene. `scenes/bench.rs` (ignored test `bench_scenes`) times µs/step and accuracy at a fixed frame budget for every scene plus larger stress worlds (merged scene copies, a 20-link chain). Initial velocities must agree with the constraints (e.g. a disk spinning on its own pin), otherwise Baumgarte has to fight a startup transient.

**Scenes are built only from general, editor-usable elements** (bodies, constraints, forces the user can also place and edit); never add a part that exists for one scene — generalise it and expose it in the editor instead. Each scene module only has a `build()` function. All scenes render through the shared `draw_world` in `scenes/mod.rs`, so every element type looks identical everywhere: flat white parts with a background-coloured (`BG`) outline separating overlaps, pin bosses at every joint/spring attachment, and pedestal anchors (ceiling- or floor-mounted depending on whether they sit above a reference height: the free bodies' centre of mass, tracked live while editing and frozen by `App` while simulating so mounts never flip mid-run). Cylinders get finned barrels, a gas column coloured by stroke and pressure, lifting valves, a spark plug that flashes at ignition, and a ringed piston drawn over the piston body and its rod. Slider rails get a dark groove between end stops and each rider a carriage (pin boss if rotation is free, bolt heads if locked). Don't add per-scene drawing; extend `draw_world` instead.

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

## Adding a New Scene

1. Create `src/scenes/my_scene.rs` with `pub fn build() -> World`
2. Declare the module in `src/scenes/mod.rs`
3. Add a `SceneDef` entry to the `SCENES` slice in `src/scenes/mod.rs`

## Adding a New Force

Implement the `Force` trait (`apply(&self, bodies: &mut [Body])`) and optionally `potential_energy`. Call `body.apply_force_at_world_point(force, world_point)` to accumulate both force and torque.
