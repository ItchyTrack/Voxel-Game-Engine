use std::error::Error;
use std::path::{Path, PathBuf};
use std::process::Command;

pub type ShaderResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

#[derive(Clone, Copy)]
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

pub fn compile_slang_file(source: &Path, include_dirs: &[PathBuf], entry: &str, stage: SlangStage) -> ShaderResult<String> {
	let mut command = Command::new(std::env::var_os("SLANGC").unwrap_or_else(|| "slangc".into()));
	command
		.arg(source)
		.arg("-target")
		.arg("wgsl")
		.arg("-entry")
		.arg(entry)
		.arg("-stage")
		.arg(stage.as_str());
	for include_dir in include_dirs {
		command.arg("-I").arg(include_dir);
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

pub fn compile_slang_files(base_dir: &Path, include_dirs: &[PathBuf], entries: &[SlangEntry<'_>]) -> ShaderResult<Vec<String>> {
	entries
		.iter()
		.map(|entry| compile_slang_file(&base_dir.join(entry.source), include_dirs, entry.entry, entry.stage))
		.collect()
}
