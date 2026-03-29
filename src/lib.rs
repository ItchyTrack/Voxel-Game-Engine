#![allow(dead_code)]
mod voxels;
mod resources;
mod player;
mod app;
mod renderer;
mod physics;
mod audio;
mod state;
mod pose;
mod math;
mod world_gen;
mod grid_tree;
mod debug_draw;
mod entity_component_system;
mod gpu_objects {
	pub mod texture;
	pub mod mesh;
	pub mod matrix;
	pub mod packed_buffer;
	pub mod packed_mesh_buffer;
}

use winit::{event_loop::{EventLoop}};

use crate::{app::App};

pub fn run() -> anyhow::Result<()> {
	#[cfg(not(target_arch = "wasm32"))]
	{
		env_logger::init();
	}
	#[cfg(target_arch = "wasm32")]
	{
		console_log::init_with_level(log::Level::Info).unwrap_throw();
	}

	let event_loop = EventLoop::with_user_event().build()?;
	let mut app = App::new(
		#[cfg(target_arch = "wasm32")]
		&event_loop,
	);
	event_loop.run_app(&mut app)?;

	Ok(())
}

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub fn run_web() -> Result<(), JsValue> {
	console_error_panic_hook::set_once();
	use wasm_bindgen::UnwrapThrowExt;
	run().unwrap_throw();
	Ok(())
}
