// Release builds on Windows are GUI apps: no console window next to the sim.
#![cfg_attr(all(windows, not(debug_assertions)), windows_subsystem = "windows")]

fn main() {
    qonstraint::main();
}
