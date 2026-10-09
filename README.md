**[▶ Run it in your browser](https://quirijndubois.github.io/Qonstraint/)** · **[Downloads](https://github.com/quirijndubois/Qonstraint/releases/latest)** (Linux, Windows, Android)

> Best on a computer with a mouse. Touch screens work too (drag, two-finger pan and pinch zoom), but the editor is roomier on a large screen.

# Qonstraint

An interactive 2D rigid-body physics sandbox written in Rust. Bodies are held together by exact constraint forces (Witkin's method) instead of penalty springs, so linkages, rolling wheels, sliders and gas cylinders stay together while energy stays close to constant. It runs natively on Linux and Windows, on Android, and in the browser (WebGPU, with a WebGL2 fallback).

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
| Strandbeest | Two mirrored Jansen legs on one motor-driven crank, feet traced |
| Peaucellier-Lipkin | A linkage that draws an exact straight line |
| Watt's Linkage | Sprung arms whose coupler midpoint moves nearly straight |
| Kapitza's Pendulum | A shaken pivot keeps an inverted pendulum standing |
| Gear Train | A motor pinion, a 3:1 gear and a belt drive a crank-rocker |
| Swinging Atwood | A rope over a pulley: counterweight against a swinging mass |
| Tumbler | Disks and bars tumbling in a box with a motor paddle |
| Flexible Beam | A cantilever of rods on torsion-sprung hinges |
| Pendulum Wave | Fifteen pendulums drift out of step and back in after 30 s |
| Barton's Pendulums | A driver on a sliding beam excites only the pendulum of matching length |
| Galton Board | 200 balls through rows of pegs into bins |
| Soft Bodies | Blobs of colliding disks on springs, shelves and a paddle |
| Domino Cascade | Ten dominoes, each 1.32× the last |
| Pumpjacks | Three belt-driven oil pumpjacks with horseheads and sucker rods |
| Planetary Press | A planetary gearset drives a punch over a conveyor of blocks |
| Geneva Drive | A crank pin indexes a four-slot wheel; a cam rocks a roller lever |
| Pendulum Clock | Anchor escapement, hands through a gear train, a bell every minute |
| Rimless Wheel | Spokes rolling down a slope into a steady gait |
| Truss Bridge | A cart crosses a truss with breakable joints until it folds |
| Trebuchet | A counterweight trebuchet whose sling lets go at its breaking pull |
| Walking Strandbeest | Four Jansen legs walk a free frame along |
| Steam Locomotive | A two-stroke cylinder drives coupled wheels, pulling a tender and coaches |
| Hot Rod | An exposed inline four belted to the rear wheel over rolling ground |
| Tank | A closed track of pinned links over two logs |
| Rube Goldberg | A ball sets off ramps, dominoes, a trapdoor, a pulley and a flag |

Every scene is built from the same parts you can place in the editor. LOAD opens a gallery of all scenes with thumbnails, NEW starts an empty scene in edit mode, and SAVE keeps the current scene under a name (in a data folder natively, in browser storage on the web); saved scenes show up in the LOAD gallery.

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

On a touch screen, one finger drags bodies (or pans outside the editor), two fingers pan and pinch-zoom, and the CONNECT switch in the editor stands in for Shift.

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

### Android

`android.sh` builds the APK with [cargo-apk](https://crates.io/crates/cargo-apk). It needs the Android SDK and NDK (as installed by Android Studio, in `~/Android/Sdk`), an SDK platform the NDK supports, and rustup with the Android target:

```bash
rustup target add aarch64-linux-android
cargo install cargo-apk
./android.sh build           # target/release/apk/qonstraint.apk
./android.sh run             # install and start on a phone connected over USB
```

### Windows

Build natively with `cargo build --release`, or cross-build from Linux with [cargo-xwin](https://github.com/rust-cross/cargo-xwin):

```bash
rustup target add x86_64-pc-windows-msvc
cargo install cargo-xwin
cargo xwin build --release --target x86_64-pc-windows-msvc
```

### Tests and benchmarks

```bash
cargo test                                                    # constraint tests + 20 s stability run of every scene
cargo test --release bench_scenes -- --ignored --nocapture    # µs/step and accuracy per scene
```

## Project layout

```
src/
  lib.rs        platform entry points (desktop, web, Android)
  app.rs        input, update and render loop
  editor.rs     scene editor
  sim/          bodies, constraints, forces, solver, integrators
  renderer/     wgpu renderer, camera, egui HUD, shaders
  scenes/       scene definitions and shared drawing
```

## License

Copyright (C) 2026 Quirijn du Bois

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version. See [LICENSE](LICENSE).

The bundled JetBrains Mono font (`assets/fonts/`) is under the SIL Open Font
License 1.1 (`assets/fonts/JetBrainsMono-OFL.txt`).
