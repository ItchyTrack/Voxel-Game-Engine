use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use notify::{EventKind, RecursiveMode, Watcher};

pub struct VoxelShaderHotReload {
	dirty: Arc<AtomicBool>,
	_watcher: notify::RecommendedWatcher,
}

impl VoxelShaderHotReload {
	pub fn new() -> notify::Result<Self> {
		let dirty = Arc::new(AtomicBool::new(false));
		let dirty_flag = dirty.clone();
		let mut watcher = notify::recommended_watcher(move |result: notify::Result<notify::Event>| match result {
			Ok(event)
				if matches!(event.kind,
					EventKind::Create(_) |
					EventKind::Modify(_) |
					EventKind::Remove(_)
				) && event.paths.iter().any(|path| is_shader_file(path)) =>
			{
				dirty_flag.store(true, Ordering::Relaxed);
			}
			Ok(_) => {}
			Err(error) => log::error!("Voxel shader hot reload watcher error: {error}"),
		})?;

		let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
		watcher.watch(&manifest_dir.join("src/shaders"), RecursiveMode::Recursive)?;

		Ok(Self {
			dirty,
			_watcher: watcher,
		})
	}

	pub fn take_dirty(&self) -> bool {
		self.dirty.swap(false, Ordering::Relaxed)
	}
}

fn is_shader_file(path: &Path) -> bool {
	path.extension().is_some_and(|extension| extension == "slang")
}
