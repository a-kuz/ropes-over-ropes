#[cfg(target_arch = "wasm32")]
pub fn save_level_to_storage(level_id: usize) {
    let storage: Option<web_sys::Storage> = web_sys::window()
        .and_then(|w| w.local_storage().ok().flatten());
    if let Some(s) = storage {
        let _ = s.set_item("uzls_level", &level_id.to_string());
    }
}

#[cfg(target_arch = "wasm32")]
pub fn load_level_from_storage() -> Option<usize> {
    let storage: Option<web_sys::Storage> = web_sys::window()
        .and_then(|w| w.local_storage().ok().flatten());
    let val: Option<String> = storage
        .and_then(|s| s.get_item("uzls_level").ok().flatten());
    val.and_then(|v| v.parse::<usize>().ok())
        .filter(|&v| v >= 1)
}

#[cfg(not(target_arch = "wasm32"))]
fn save_file_path() -> Option<std::path::PathBuf> {
    dirs::data_local_dir().map(|d| d.join("uzls4").join("save.txt"))
}

#[cfg(not(target_arch = "wasm32"))]
pub fn save_level_to_storage(level_id: usize) {
    if let Some(path) = save_file_path() {
        if let Some(parent) = path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        let _ = std::fs::write(&path, level_id.to_string());
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub fn load_level_from_storage() -> Option<usize> {
    save_file_path()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|s| s.trim().parse::<usize>().ok())
        .filter(|&v| v >= 1)
}
