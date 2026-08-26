use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

pub type ShaderResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SlangStage {
	Vertex,
	Fragment,
	Compute,
}

impl SlangStage {
	fn as_str(self) -> &'static str {
		match self {
			Self::Vertex => "vertex",
			Self::Fragment => "fragment",
			Self::Compute => "compute",
		}
	}
}

#[derive(Clone, Copy)]
pub struct SlangEntry<'a> {
	pub source: &'a str,
	pub entry: &'a str,
	pub stage: SlangStage,
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct SlangModule {
	pub path: PathBuf,
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct SlangConformance {
	pub ty: String,
	pub interface: String,
	pub id: u16,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct SlangLinkage {
	pub modules: Vec<SlangModule>,
	pub conformances: Vec<SlangConformance>,
}

struct CompileSource {
	path: PathBuf,
	_temp_file: Option<PathBuf>,
}

impl Drop for CompileSource {
	fn drop(&mut self) {
		if let Some(path) = self._temp_file.as_ref() {
			let _ = std::fs::remove_file(path);
		}
	}
}

pub fn compile_slang_file(
	source: &Path,
	include_dirs: &[PathBuf],
	entry: &str,
	stage: SlangStage,
	linkage: &SlangLinkage,
) -> ShaderResult<String> {
	let compile_source = compile_source(source, &linkage.modules)?;
	let mut command = Command::new(std::env::var_os("SLANGC").unwrap_or_else(|| "slangc".into()));
	command
		.arg(&compile_source.path)
		.arg("-target")
		.arg("wgsl")
		.arg("-entry")
		.arg(entry)
		.arg("-stage")
		.arg(stage.as_str());
	for include_dir in include_dirs {
		command.arg("-I").arg(include_dir);
	}
	for conformance in &linkage.conformances {
		command.arg("-conformance").arg(format!("{}:{}={}", conformance.ty, conformance.interface, conformance.id));
	}

	let output = command.output().map_err(|error| {
		std::io::Error::new(
			error.kind(),
			format!("failed to run slangc; install Slang or set SLANGC to its path: {error}"),
		)
	})?;
	if !output.status.success() {
		return Err(std::io::Error::other(format!(
			"slangc failed for {}:\n{}",
			source.display(),
			String::from_utf8_lossy(&output.stderr)
		))
		.into());
	}
	Ok(String::from_utf8(output.stdout)?)
}

pub fn compile_slang_files(
	base_dir: &Path,
	include_dirs: &[PathBuf],
	entries: &[SlangEntry<'_>],
	linkages: &[SlangLinkage],
) -> ShaderResult<Vec<String>> {
	let default_linkage = SlangLinkage::default();
	entries
		.iter()
		.enumerate()
		.map(|(index, entry)| {
			let linkage = linkages.get(index).unwrap_or(&default_linkage);
			compile_slang_file(&base_dir.join(entry.source), include_dirs, entry.entry, entry.stage, linkage)
		})
		.collect()
}

fn compile_source(source: &Path, linked_modules: &[SlangModule]) -> ShaderResult<CompileSource> {
	if linked_modules.is_empty() {
		return Ok(CompileSource { path: source.to_path_buf(), _temp_file: None });
	}

	let unique = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
	let path = std::env::temp_dir().join(format!("voxel-slang-{unique}.slang"));
	let mut temp_file = File::create(&path)?;
	writeln!(temp_file, "#include \"{}\"", source.display())?;
	for module in linked_modules {
		writeln!(temp_file, "#include \"{}\"", module.path.display())?;
	}
	drop(temp_file);
	Ok(CompileSource { path: path.clone(), _temp_file: Some(path) })
}
