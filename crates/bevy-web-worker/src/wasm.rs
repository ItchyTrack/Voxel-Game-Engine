#![cfg(target_arch = "wasm32")]

use std::{ptr::NonNull, sync::Once, thread::ThreadId};

use bevy::{
	app::PluginsState,
	input::{
		keyboard::{Key, KeyboardInput, NativeKey},
		mouse::{MouseButtonInput, MouseMotion},
		ButtonState,
	},
	prelude::*,
	window::{RawHandleWrapper, WindowResized, WindowWrapper},
};
use raw_window_handle::{HasDisplayHandle, HasWindowHandle};
use wasm_bindgen::prelude::*;
use web_sys::OffscreenCanvas;

#[wasm_bindgen(inline_js = r#"
export function install_async_worker_pool(module, memory, initial) {
  for (let i = 0; i < initial; i++) {
    const worker = new Worker('./async_worker.js', { type: 'module' });
    worker.postMessage({ module, memory, workerId: i });
  }
}

export function install_source_request_workers(module, memory) {
  const chunkWorker = new Worker('./source_request_worker.js', { type: 'module' });
  chunkWorker.postMessage({ module, memory, kind: 'chunk', workerId: 0 });

  const lodWorker = new Worker('./source_request_worker.js', { type: 'module' });
  lodWorker.postMessage({ module, memory, kind: 'lod', workerId: 0 });
}
"#)]
extern "C" {
	fn install_async_worker_pool(module: &JsValue, memory: &JsValue, initial: usize);
	fn install_source_request_workers(module: &JsValue, memory: &JsValue);
}

static WASM_INIT: Once = Once::new();

fn init_wasm_runtime() {
	let runtime = tokio::runtime::Builder::new_current_thread().build().unwrap();
	let guard = runtime.enter();
	std::mem::forget(guard);
	std::mem::forget(runtime);
}

#[wasm_bindgen]
pub struct BevyApp {
	app: App,
	frame_counter: u32,
}

#[wasm_bindgen]
impl BevyApp {
	#[wasm_bindgen(constructor)]
	pub fn new(canvas: OffscreenCanvas, width: f32, height: f32, shim_url: String) -> Self {
		WASM_INIT.call_once(|| {
			console_error_panic_hook::set_once();
			let _ = console_log::init_with_level(log::Level::Info);
			init_wasm_runtime();
			wasm_thread::Builder::new()
				.wasm_bindgen_shim_url(shim_url)
				.set_default();
			install_async_worker_pool(&wasm_bindgen::module(), &wasm_bindgen::memory(), 8);
			install_source_request_workers(&wasm_bindgen::module(), &wasm_bindgen::memory());
		});

		let mut app =
			voxel_app::build_app(Window { resolution: bevy::window::WindowResolution::new(width as u32, height as u32), ..Default::default() });
		app.insert_non_send_resource(canvas);
		app.add_systems(PreStartup, setup_added_window);
		Self { app, frame_counter: 0 }
	}

	pub fn update(&mut self) {
		self.frame_counter = self.frame_counter.wrapping_add(1);
		if self.app.plugins_state() != PluginsState::Cleaned {
			if self.app.plugins_state() == PluginsState::Ready {
				self.app.finish();
				self.app.cleanup();
			}
		} else {
			self.app.update();
		}
	}

	pub fn resize(&mut self, width: f32, height: f32) {
		let world = self.app.world_mut();
		let mut resized_windows = Vec::new();
		let mut query = world.query::<(Entity, &mut Window)>();
		for (entity, mut window) in query.iter_mut(world) {
			window.resolution.set(width, height);
			resized_windows.push(entity);
		}
		for window in resized_windows {
			let _ = world.write_message(WindowResized { window, width, height });
		}
	}

	pub fn key(&mut self, code: String, pressed: bool) {
		let Some(key_code) = map_key_code(&code) else {
			return;
		};
		let Some(window) = primary_window_entity(&mut self.app) else {
			return;
		};
		let _ = self.app.world_mut().write_message(KeyboardInput {
			key_code,
			logical_key: Key::Unidentified(NativeKey::Web(code.into())),
			state: if pressed { ButtonState::Pressed } else { ButtonState::Released },
			text: None,
			repeat: false,
			window,
		});
	}

	pub fn mouse_button(&mut self, button: u16, pressed: bool) {
		let Some(button) = map_mouse_button(button) else {
			return;
		};
		let Some(window) = primary_window_entity(&mut self.app) else {
			return;
		};
		let _ = self.app.world_mut().write_message(MouseButtonInput {
			button,
			state: if pressed { ButtonState::Pressed } else { ButtonState::Released },
			window,
		});
	}

	pub fn mouse_motion(&mut self, delta_x: f32, delta_y: f32) {
		let delta = Vec2::new(delta_x, delta_y);
		let _ = self.app.world_mut().write_message(MouseMotion { delta });
	}

}

impl BevyApp {}

fn primary_window_entity(app: &mut App) -> Option<Entity> {
	let world = app.world_mut();
	let mut query = world.query_filtered::<Entity, With<Window>>();
	query.iter(world).next()
}

fn map_key_code(code: &str) -> Option<KeyCode> {
	match code {
		"AltLeft" => Some(KeyCode::AltLeft),
		"AltRight" => Some(KeyCode::AltRight),
		"ArrowDown" => Some(KeyCode::ArrowDown),
		"ArrowLeft" => Some(KeyCode::ArrowLeft),
		"ArrowRight" => Some(KeyCode::ArrowRight),
		"ArrowUp" => Some(KeyCode::ArrowUp),
		"Backquote" => Some(KeyCode::Backquote),
		"Backslash" => Some(KeyCode::Backslash),
		"Backspace" => Some(KeyCode::Backspace),
		"BracketLeft" => Some(KeyCode::BracketLeft),
		"BracketRight" => Some(KeyCode::BracketRight),
		"CapsLock" => Some(KeyCode::CapsLock),
		"Comma" => Some(KeyCode::Comma),
		"ContextMenu" => Some(KeyCode::ContextMenu),
		"ControlLeft" => Some(KeyCode::ControlLeft),
		"ControlRight" => Some(KeyCode::ControlRight),
		"Delete" => Some(KeyCode::Delete),
		"Digit0" => Some(KeyCode::Digit0),
		"Digit1" => Some(KeyCode::Digit1),
		"Digit2" => Some(KeyCode::Digit2),
		"Digit3" => Some(KeyCode::Digit3),
		"Digit4" => Some(KeyCode::Digit4),
		"Digit5" => Some(KeyCode::Digit5),
		"Digit6" => Some(KeyCode::Digit6),
		"Digit7" => Some(KeyCode::Digit7),
		"Digit8" => Some(KeyCode::Digit8),
		"Digit9" => Some(KeyCode::Digit9),
		"End" => Some(KeyCode::End),
		"Enter" => Some(KeyCode::Enter),
		"Equal" => Some(KeyCode::Equal),
		"Escape" => Some(KeyCode::Escape),
		"F1" => Some(KeyCode::F1),
		"F10" => Some(KeyCode::F10),
		"F11" => Some(KeyCode::F11),
		"F12" => Some(KeyCode::F12),
		"F13" => Some(KeyCode::F13),
		"F14" => Some(KeyCode::F14),
		"F15" => Some(KeyCode::F15),
		"F16" => Some(KeyCode::F16),
		"F17" => Some(KeyCode::F17),
		"F18" => Some(KeyCode::F18),
		"F19" => Some(KeyCode::F19),
		"F2" => Some(KeyCode::F2),
		"F20" => Some(KeyCode::F20),
		"F21" => Some(KeyCode::F21),
		"F22" => Some(KeyCode::F22),
		"F23" => Some(KeyCode::F23),
		"F24" => Some(KeyCode::F24),
		"F3" => Some(KeyCode::F3),
		"F4" => Some(KeyCode::F4),
		"F5" => Some(KeyCode::F5),
		"F6" => Some(KeyCode::F6),
		"F7" => Some(KeyCode::F7),
		"F8" => Some(KeyCode::F8),
		"F9" => Some(KeyCode::F9),
		"Home" => Some(KeyCode::Home),
		"Insert" => Some(KeyCode::Insert),
		"KeyA" => Some(KeyCode::KeyA),
		"KeyB" => Some(KeyCode::KeyB),
		"KeyC" => Some(KeyCode::KeyC),
		"KeyD" => Some(KeyCode::KeyD),
		"KeyE" => Some(KeyCode::KeyE),
		"KeyF" => Some(KeyCode::KeyF),
		"KeyG" => Some(KeyCode::KeyG),
		"KeyH" => Some(KeyCode::KeyH),
		"KeyI" => Some(KeyCode::KeyI),
		"KeyJ" => Some(KeyCode::KeyJ),
		"KeyK" => Some(KeyCode::KeyK),
		"KeyL" => Some(KeyCode::KeyL),
		"KeyM" => Some(KeyCode::KeyM),
		"KeyN" => Some(KeyCode::KeyN),
		"KeyO" => Some(KeyCode::KeyO),
		"KeyP" => Some(KeyCode::KeyP),
		"KeyQ" => Some(KeyCode::KeyQ),
		"KeyR" => Some(KeyCode::KeyR),
		"KeyS" => Some(KeyCode::KeyS),
		"KeyT" => Some(KeyCode::KeyT),
		"KeyU" => Some(KeyCode::KeyU),
		"KeyV" => Some(KeyCode::KeyV),
		"KeyW" => Some(KeyCode::KeyW),
		"KeyX" => Some(KeyCode::KeyX),
		"KeyY" => Some(KeyCode::KeyY),
		"KeyZ" => Some(KeyCode::KeyZ),
		"MetaLeft" => Some(KeyCode::SuperLeft),
		"MetaRight" => Some(KeyCode::SuperRight),
		"Minus" => Some(KeyCode::Minus),
		"NumLock" => Some(KeyCode::NumLock),
		"Numpad0" => Some(KeyCode::Numpad0),
		"Numpad1" => Some(KeyCode::Numpad1),
		"Numpad2" => Some(KeyCode::Numpad2),
		"Numpad3" => Some(KeyCode::Numpad3),
		"Numpad4" => Some(KeyCode::Numpad4),
		"Numpad5" => Some(KeyCode::Numpad5),
		"Numpad6" => Some(KeyCode::Numpad6),
		"Numpad7" => Some(KeyCode::Numpad7),
		"Numpad8" => Some(KeyCode::Numpad8),
		"Numpad9" => Some(KeyCode::Numpad9),
		"NumpadAdd" => Some(KeyCode::NumpadAdd),
		"NumpadDecimal" => Some(KeyCode::NumpadDecimal),
		"NumpadDivide" => Some(KeyCode::NumpadDivide),
		"NumpadEnter" => Some(KeyCode::NumpadEnter),
		"NumpadMultiply" => Some(KeyCode::NumpadMultiply),
		"NumpadSubtract" => Some(KeyCode::NumpadSubtract),
		"PageDown" => Some(KeyCode::PageDown),
		"PageUp" => Some(KeyCode::PageUp),
		"Pause" => Some(KeyCode::Pause),
		"Period" => Some(KeyCode::Period),
		"PrintScreen" => Some(KeyCode::PrintScreen),
		"Quote" => Some(KeyCode::Quote),
		"ScrollLock" => Some(KeyCode::ScrollLock),
		"Semicolon" => Some(KeyCode::Semicolon),
		"ShiftLeft" => Some(KeyCode::ShiftLeft),
		"ShiftRight" => Some(KeyCode::ShiftRight),
		"Slash" => Some(KeyCode::Slash),
		"Space" => Some(KeyCode::Space),
		"Tab" => Some(KeyCode::Tab),
		_ => None,
	}
}

fn map_mouse_button(button: u16) -> Option<MouseButton> {
	Some(match button {
		0 => MouseButton::Left,
		1 => MouseButton::Middle,
		2 => MouseButton::Right,
		_ => return None,
	})
}

fn setup_added_window(mut commands: Commands, canvas: NonSend<OffscreenCanvas>, mut new_windows: Query<Entity, Added<Window>>) {
	let Some(entity) = new_windows.iter_mut().next() else {
		return;
	};
	let handle = OffscreenWindowHandle::new(&canvas);
	let handle = RawHandleWrapper::new(&WindowWrapper::new(handle)).expect("failed to create offscreen raw handle wrapper");
	commands.entity(entity).insert(handle);
}

struct OffscreenWindowHandle {
	window_handle: raw_window_handle::RawWindowHandle,
	display_handle: raw_window_handle::DisplayHandle<'static>,
	thread_id: ThreadId,
}

impl OffscreenWindowHandle {
	fn new(canvas: &OffscreenCanvas) -> Self {
		let ptr = NonNull::from(canvas).cast();
		let handle = raw_window_handle::WebOffscreenCanvasWindowHandle::new(ptr);
		let window_handle = raw_window_handle::RawWindowHandle::WebOffscreenCanvas(handle);
		let display_handle = raw_window_handle::DisplayHandle::web();
		Self { window_handle, display_handle, thread_id: std::thread::current().id() }
	}
}

unsafe impl Send for OffscreenWindowHandle {}
unsafe impl Sync for OffscreenWindowHandle {}

impl HasWindowHandle for OffscreenWindowHandle {
	fn window_handle(&self) -> Result<raw_window_handle::WindowHandle<'_>, raw_window_handle::HandleError> {
		if self.thread_id != std::thread::current().id() {
			return Err(raw_window_handle::HandleError::NotSupported);
		}
		Ok(unsafe { raw_window_handle::WindowHandle::borrow_raw(self.window_handle) })
	}
}

impl HasDisplayHandle for OffscreenWindowHandle {
	fn display_handle(&self) -> Result<raw_window_handle::DisplayHandle<'_>, raw_window_handle::HandleError> { Ok(self.display_handle) }
}
