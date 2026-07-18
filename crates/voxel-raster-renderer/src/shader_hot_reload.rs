use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use notify::{RecursiveMode, Watcher};

pub struct RasterShaderHotReload {
	dirty: Arc<AtomicBool>,
	_watcher: notify::RecommendedWatcher,
}

impl RasterShaderHotReload {
	pub fn new() -> notify::Result<Self> {
		let dirty = Arc::new(AtomicBool::new(false));
		let dirty_flag = dirty.clone();
		let mut watcher = notify::recommended_watcher(move |result: notify::Result<notify::Event>| match result {
			Ok(event) if event.paths.iter().any(|path| path.extension().is_some_and(|extension| extension == "wesl")) => {
				dirty_flag.store(true, Ordering::Relaxed);
			}
			Ok(_) => {}
			Err(error) => log::error!("Raster shader hot reload watcher error: {error}"),
		})?;
		let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
		for root in [manifest_dir.join("src/shaders"), manifest_dir.join("../voxel-gpu/src/shaders")] {
			watcher.watch(&root, RecursiveMode::Recursive)?;
		}
		Ok(Self { dirty, _watcher: watcher })
	}

	pub fn take_dirty(&self) -> bool {
		self.dirty.swap(false, Ordering::Relaxed)
	}
}
