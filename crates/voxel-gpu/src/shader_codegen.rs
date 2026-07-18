use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};
use std::error::Error;

use wesl::{Resolver, VirtualResolver, Wesl};

pub type ShaderResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

#[derive(Clone, Copy)]
pub struct EmbeddedWeslModule {
	pub module: &'static str,
	pub source: &'static str,
}

#[derive(Clone, Debug)]
pub struct GeneratedWeslModule {
	pub module: String,
	pub source: String,
}

#[macro_export]
macro_rules! embedded_wesl_modules {
	($($module:literal => $path:literal),* $(,)?) => {
		&[$($crate::shader_codegen::EmbeddedWeslModule {
			module: $module,
			source: include_str!($path),
		}),*]
	};
}

pub fn compile_embedded(
	embedded_modules: &[EmbeddedWeslModule],
	generated_modules: &[GeneratedWeslModule],
	entry_modules: &[&str],
) -> ShaderResult<Vec<String>> {
	let mut resolver = VirtualResolver::new();
	for module in embedded_modules {
		resolver.add_module(module.module.parse()?, Cow::Borrowed(module.source));
	}
	for module in generated_modules {
		resolver.add_module(module.module.parse()?, Cow::Owned(module.source.clone()));
	}
	compile(Wesl::new("").set_custom_resolver(resolver), entry_modules)
}

pub fn compile<R: Resolver>(compiler: Wesl<R>, entry_modules: &[&str]) -> ShaderResult<Vec<String>> {
	entry_modules
		.iter()
		.map(|module| Ok(compiler.compile(&module.parse()?)?.to_string()))
		.collect()
}

pub fn specialize(template: &str, replacements: &[(&str, &str)]) -> ShaderResult<String> {
	let tokens = template_tokens(template)?;
	let replacements: BTreeMap<_, _> = replacements.iter().copied().collect();
	for token in &tokens {
		if !replacements.contains_key(token.as_str()) {
			return Err(std::io::Error::other(format!("missing replacement for shader template token `__{token}__`")).into());
		}
	}
	for token in replacements.keys() {
		if !tokens.contains(*token) {
			return Err(std::io::Error::other(format!("replacement provided for unknown shader template token `__{token}__`")).into());
		}
	}
	let mut source = template.to_owned();
	for (token, value) in replacements {
		source = source.replace(&format!("__{token}__"), value);
	}
	Ok(source)
}

fn template_tokens(template: &str) -> ShaderResult<BTreeSet<String>> {
	let mut tokens = BTreeSet::new();
	let mut remainder = template;
	while let Some(start) = remainder.find("__") {
		let after_start = &remainder[start + 2..];
		let Some(end) = after_start.find("__") else {
			return Err(std::io::Error::other("unterminated shader template token").into());
		};
		let token = &after_start[..end];
		if token.is_empty() || !token.bytes().all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit() || byte == b'_') {
			return Err(std::io::Error::other(format!("invalid shader template token `__{token}__`")).into());
		}
		tokens.insert(token.to_owned());
		remainder = &after_start[end + 2..];
	}
	Ok(tokens)
}
