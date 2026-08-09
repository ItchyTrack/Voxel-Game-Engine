use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

use bevy::asset::{Asset, AssetEvent, AssetId, AssetLoader, Handle, LoadContext, io::Reader};
use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::ecs::system::{Single, SystemParam, SystemParamItem};
use bevy::prelude::{Component, Message, Res, TypePath, With};
use bevy::render::render_asset::{PrepareAssetError, RenderAsset, RenderAssets};
use bevy::render::Extract;
use serde::{Deserialize, Serialize};

use crate::shader_compiler::{self, SlangEntry, SlangLinkage, SlangModule, SlangStage};

static NEXT_TEMP_DIRECTORY: AtomicU64 = AtomicU64::new(0);

#[derive(Asset, TypePath, Clone, Debug)]
pub struct CompiledSlangShader {
	pub shaders: Vec<String>,
}

#[derive(Message, Clone, Copy)]
pub struct SlangShaderChanged(AssetId<CompiledSlangShader>);

impl SlangShaderChanged {
	pub fn asset_id(self) -> AssetId<CompiledSlangShader> {
		self.0
	}
}

#[derive(Component)]
pub struct SlangShader {
	handle: Handle<CompiledSlangShader>,
}

impl SlangShader {
	pub fn new(handle: Handle<CompiledSlangShader>) -> Self {
		Self { handle }
	}
}

pub struct LoadedSlangShader<'a> {
	shader: &'a CompiledSlangShader,
	changed: bool,
}

impl LoadedSlangShader<'_> {
	pub fn shader(&self) -> &CompiledSlangShader {
		self.shader
	}

	pub fn changed(&self) -> bool {
		self.changed
	}
}

#[derive(SystemParam)]
pub struct SlangShaderParam<'w, 's, Marker: Component> {
	shader: Single<'w, 's, &'static SlangShader, With<Marker>>,
	shaders: Res<'w, RenderAssets<CompiledSlangShader>>,
	changes: MessageReader<'w, 's, SlangShaderChanged>,
}

impl<'w, 's, Marker: Component> SlangShaderParam<'w, 's, Marker> {
	pub fn get(&mut self) -> Option<LoadedSlangShader<'_>> {
		let asset_id = self.shader.handle.id();
		let changed = self.changes.read().any(|change| change.asset_id() == asset_id);
		let shader = self.shaders.get(asset_id)?;
		Some(LoadedSlangShader { shader, changed })
	}
}

pub fn extract_shader_changes(
	mut events: Extract<MessageReader<AssetEvent<CompiledSlangShader>>>,
	mut changed: MessageWriter<SlangShaderChanged>,
) {
	for event in events.read() {
		if let AssetEvent::Added { id } | AssetEvent::Modified { id } = event {
			changed.write(SlangShaderChanged(*id));
		}
	}
}

impl RenderAsset for CompiledSlangShader {
	type SourceAsset = Self;
	type Param = ();

	fn prepare_asset(
		source_asset: Self::SourceAsset,
		_asset_id: AssetId<Self::SourceAsset>,
		_param: &mut SystemParamItem<Self::Param>,
		_previous_asset: Option<&Self>,
	) -> Result<Self, PrepareAssetError<Self::SourceAsset>> {
		Ok(source_asset)
	}
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SlangShaderSettings {
	pub files: Vec<SlangAssetFile>,
	pub base_dir: PathBuf,
	pub include_dirs: Vec<PathBuf>,
	pub entries: Vec<SlangAssetEntry>,
	pub linkages: Vec<SlangLinkage>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SlangAssetFile {
	pub asset_path: String,
	pub compile_path: PathBuf,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SlangAssetEntry {
	pub source: String,
	pub entry: String,
	pub stage: SlangStage,
}

#[derive(Default, TypePath)]
pub struct SlangShaderLoader;

impl AssetLoader for SlangShaderLoader {
	type Asset = CompiledSlangShader;
	type Settings = SlangShaderSettings;
	type Error = std::io::Error;

	async fn load(
		&self,
		_reader: &mut dyn Reader,
		settings: &Self::Settings,
		load_context: &mut LoadContext<'_>,
	) -> Result<Self::Asset, Self::Error> {
		let temp_dir = TempShaderDirectory::new()?;
		for file in &settings.files {
			let bytes = load_context
				.read_asset_bytes(file.asset_path.clone())
				.await
				.map_err(|error| std::io::Error::other(error.to_string()))?;
			let path = temp_dir.path.join(&file.compile_path);
			if let Some(parent) = path.parent() {
				std::fs::create_dir_all(parent)?;
			}
			std::fs::write(path, bytes)?;
		}

		let entries = settings.entries.iter()
			.map(|entry| SlangEntry {
				source: entry.source.as_str(),
				entry: entry.entry.as_str(),
				stage: entry.stage,
			})
			.collect::<Vec<_>>();
		let include_dirs = settings.include_dirs.iter()
			.map(|path| temp_dir.path.join(path))
			.collect::<Vec<_>>();
		let linkages = settings.linkages.iter()
			.map(|linkage| SlangLinkage {
				modules: linkage.modules.iter()
					.map(|module| SlangModule { path: temp_dir.path.join(&module.path) })
					.collect(),
				conformances: linkage.conformances.clone(),
			})
			.collect::<Vec<_>>();
		let shaders = shader_compiler::compile_slang_files(
			&temp_dir.path.join(&settings.base_dir),
			&include_dirs,
			&entries,
			&linkages,
		)
		.map_err(|error| std::io::Error::other(error.to_string()))?;
		Ok(CompiledSlangShader { shaders })
	}

	fn extensions(&self) -> &[&str] {
		&["slang"]
	}
}

struct TempShaderDirectory {
	path: PathBuf,
}

impl TempShaderDirectory {
	fn new() -> std::io::Result<Self> {
		let sequence = NEXT_TEMP_DIRECTORY.fetch_add(1, Ordering::Relaxed);
		let path = std::env::temp_dir().join(format!(
			"voxel-slang-assets-{}-{sequence}",
			std::process::id(),
		));
		std::fs::create_dir_all(&path)?;
		Ok(Self { path })
	}
}

impl Drop for TempShaderDirectory {
	fn drop(&mut self) {
		let _ = std::fs::remove_dir_all(&self.path);
	}
}
