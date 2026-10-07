**[▶ Run it in your browser](https://quirijndubois.github.io/Qonstraint/)**

> A Chromium-based browser (Chrome, Edge, Brave…) is highly recommended, and it is meant to be run on a computer: it needs a mouse (middle drag, scroll wheel) and a large screen, so phones and tablets are not supported.

# Qonstraint

An interactive 2D rigid-body physics sandbox written in Rust. Bodies are held together by exact constraint forces (Witkin's method) instead of penalty springs, so linkages, rolling wheels, sliders and gas cylinders stay together while energy stays close to constant. It runs natively and in the browser (WebGPU, with a WebGL2 fallback).

Inspired by [this video](https://www.youtube.com/watch?v=TtgS-b191V0) by AngeTheGreat.

## Scenes

| Scene | What happens |
|-------|--------------|
| Rocker Linkage | A disk rolls on a sprung lever, coupled by a rod to a rocker |
| Radial Engine | A five-cylinder four-stroke radial engine, firing order 1-3-5-2-4 |
| Double Pendulum | The classic chaotic double pendulum |
| Elastic Pendulum | A spring pendulum near 2:1 resonance |
| Coupled Pendulums | Two pendulums trade energy through a weak spring |
| Rolling Crane | A wheel rolls under a rail with a double pendulum hanging from it |
| Planetary Pendulum | A disk rolls around a fixed disk with a pendulum on its rim |
| Trammel | A bar slides on two crossed rails, and its tip traces an ellipse |
| Air Struts | A hub bounces on two sealed air springs with a pendulum below |
| Sandbox | Empty, opens in edit mode so you can build your own |

Every scene is built from the same parts you can place in the editor.

## Controls

| Input | Action |
|-------|--------|
| Left drag | Pull a body with a mouse spring |
| Middle drag | Pan |
| Scroll | Zoom toward the cursor |
| Space | Pause / resume |
| E | Toggle edit mode |
| R / Shift+R | Rotate the selected body 15° (edit mode) |
| Ctrl while dragging handles | Snap angle or size |
| Delete / Backspace | Delete the selected body (edit mode) |
| Esc | Deselect |

The HUD also has scene switching, reset, gravity, step count, simulation speed and a choice of integrator.

## Physics

- **Constraint solver:** Witkin's constraint force equation, `JWJᵀλ = −J̇q̇ − JWQ − ks·C − kd·Ċ`, solved exactly each step with a sparse LDLᵀ factorisation (reverse Cuthill–McKee ordering, no allocation per solve).
- **Integrators:** RK4 (the default, good at keeping energy) or XPBD (cheaper per step and holds constraints tighter, but loses some energy).
- **Constraints:** pin joints, world pins, distance links, rolling contact (disk on disk and disk on rod, no-slip at position level), slider joints with end stops, and gas cylinders (sealed air spring, or a self-timing two- or four-stroke combustion cycle).
- **Forces:** gravity, damped springs, motors/brakes and the mouse spring.
- State is kept in `f64`, because thousands of steps per frame would lose too much precision in `f32`.

## Building

You need a recent stable Rust toolchain (edition 2024).

```bash
cargo run --release
```

### Web

```bash
rustup target add wasm32-unknown-unknown
cargo install trunk          # or: pacman -S rust-wasm trunk
trunk serve                  # http://127.0.0.1:8080
trunk build --release        # static site in dist/
```

Add `?webgl` to the URL to force the WebGL2 backend.

Every push to `main` builds the web version and deploys it to GitHub Pages using [`.github/workflows/pages.yml`](.github/workflows/pages.yml).

### Tests and benchmarks

```bash
cargo test                                                    # constraint tests + 20 s stability run of every scene
cargo test --release bench_scenes -- --ignored --nocapture    # µs/step and accuracy per scene
```

## Project layout

```
src/
  app.rs        input, update and render loop
  editor.rs     scene editor
  sim/          bodies, constraints, forces, solver, integrators
  renderer/     wgpu renderer, camera, egui HUD, shaders
  scenes/       scene definitions and shared drawing
```
