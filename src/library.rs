//! The user's saved scenes (SAVE / LOAD), stored as scene-file JSON by
//! name: one file each in a data folder natively, `localStorage` entries in
//! the browser. Also the LOAD gallery's state, thumbnails included.

/// A scene in the user's library.
pub struct Saved {
    pub name: String,
    pub json: String,
}

/// What a gallery tile loads.
#[derive(Clone, PartialEq)]
pub enum Source {
    Builtin(usize),
    Saved(String),
}

pub struct Tile {
    pub source: Source,
    pub name:   String,
    pub desc:   String,
    pub thumb:  egui::TextureId,
}

/// LOAD gallery and SAVE dialog state, shared by the app and the HUD.
#[derive(Default)]
pub struct Library {
    pub show_load: bool,
    pub show_save: bool,
    /// Name typed in the SAVE dialog.
    pub save_name: String,
    pub save_note: String,
    /// Focus the name field (all selected) when the dialog opens.
    pub save_focus: bool,
    /// The scene on screen, marked in the gallery (set by the app).
    pub current: Option<Source>,
    /// Gallery tiles; empty until the gallery is first opened, rebuilt
    /// (with fresh thumbnails) when the saved list changes.
    pub tiles: Vec<Tile>,
    /// Names already saved, for the SAVE dialog's overwrite warning.
    pub saved_names: Vec<String>,
}

/// Names become file names natively; keep them plain.
pub fn clean_name(name: &str) -> String {
    name.chars()
        .filter(|c| c.is_alphanumeric() || matches!(c, ' ' | '-' | '_' | '.' | '(' | ')'))
        .collect::<String>()
        .trim()
        .trim_start_matches('.')
        .chars()
        .take(48)
        .collect()
}

/// Where saved scenes go when the platform says so itself (Android: the
/// app's private data folder, which has no `HOME` to derive it from).
#[cfg(not(target_arch = "wasm32"))]
static DATA_DIR: std::sync::OnceLock<std::path::PathBuf> = std::sync::OnceLock::new();

#[cfg(not(target_arch = "wasm32"))]
#[cfg_attr(not(target_os = "android"), allow(dead_code))]
pub fn set_data_dir(dir: std::path::PathBuf) {
    let _ = DATA_DIR.set(dir);
}

#[cfg(not(target_arch = "wasm32"))]
mod store {
    use super::Saved;
    use std::path::PathBuf;

    fn dir() -> Option<PathBuf> {
        if let Some(d) = super::DATA_DIR.get() { return Some(d.join("scenes")); }
        let base = if cfg!(windows) {
            std::env::var_os("APPDATA").map(PathBuf::from)
        } else {
            std::env::var_os("XDG_DATA_HOME").map(PathBuf::from)
                .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))
        };
        base.map(|b| b.join("qonstraint").join("scenes"))
    }

    pub fn list() -> Vec<Saved> {
        let Some(entries) = dir().and_then(|d| std::fs::read_dir(d).ok()) else { return Vec::new() };
        let mut out: Vec<Saved> = entries
            .filter_map(|e| {
                let path = e.ok()?.path();
                if path.extension()? != "json" { return None; }
                let name = path.file_stem()?.to_str()?.to_owned();
                let json = std::fs::read_to_string(&path).ok()?;
                Some(Saved { name, json })
            })
            .collect();
        out.sort_by_key(|s| s.name.to_lowercase());
        out
    }

    pub fn save(name: &str, json: &str) -> Result<(), String> {
        let d = dir().ok_or("No data folder")?;
        std::fs::create_dir_all(&d).map_err(|e| e.to_string())?;
        std::fs::write(d.join(format!("{name}.json")), json).map_err(|e| e.to_string())
    }

    pub fn delete(name: &str) {
        if let Some(d) = dir() { let _ = std::fs::remove_file(d.join(format!("{name}.json"))); }
    }
}

#[cfg(target_arch = "wasm32")]
mod store {
    use super::Saved;

    const PREFIX: &str = "qonstraint.scene.";

    fn storage() -> Option<web_sys::Storage> {
        web_sys::window()?.local_storage().ok()?
    }

    pub fn list() -> Vec<Saved> {
        let Some(s) = storage() else { return Vec::new() };
        let n = s.length().unwrap_or(0);
        let mut out: Vec<Saved> = (0..n)
            .filter_map(|i| {
                let key = s.key(i).ok()??;
                let name = key.strip_prefix(PREFIX)?.to_owned();
                let json = s.get_item(&key).ok()??;
                Some(Saved { name, json })
            })
            .collect();
        out.sort_by_key(|s| s.name.to_lowercase());
        out
    }

    pub fn save(name: &str, json: &str) -> Result<(), String> {
        let s = storage().ok_or("Browser storage is unavailable")?;
        s.set_item(&format!("{PREFIX}{name}"), json).map_err(|_| "Browser storage is full".to_owned())
    }

    pub fn delete(name: &str) {
        if let Some(s) = storage() { let _ = s.remove_item(&format!("{PREFIX}{name}")); }
    }
}

pub use store::{delete, list, save};
