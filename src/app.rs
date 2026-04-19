#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;
use std::sync::Arc;

use tracy_client::{frame_mark, span};
use winit::{application::ApplicationHandler, event::{DeviceEvent, DeviceId, KeyEvent, WindowEvent}, event_loop::{ActiveEventLoop}, keyboard::PhysicalKey, window::Window};
use crate::{player::camera, state::State};

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;
#[cfg(target_arch = "wasm32")]
use winit::event_loop::EventLoop;

pub struct App {
	#[cfg(target_arch = "wasm32")]
	proxy: Option<winit::event_loop::EventLoopProxy<State>>,
	state: Option<State>,
	last_update: Instant,
	dt_avg: f32,
}

impl App {
	pub fn new(#[cfg(target_arch = "wasm32")] event_loop: &EventLoop<State>) -> Self {
		#[cfg(target_arch = "wasm32")]
		let proxy = Some(event_loop.create_proxy());
		Self {
			state: None,
			#[cfg(target_arch = "wasm32")]
			proxy,
			last_update: Instant::now(),
			dt_avg: 0.0,
		}
	}
}

impl ApplicationHandler<State> for App {
	fn resumed(&mut self, event_loop: &ActiveEventLoop) {
		#[allow(unused_mut)]
		let mut window_attributes = Window::default_attributes();

		#[cfg(target_arch = "wasm32")]
		{
			use wasm_bindgen::JsCast;
			use winit::platform::web::WindowAttributesExtWebSys;

			const CANVAS_ID: &str = "canvas";

			let window = wgpu::web_sys::window().unwrap_throw();
			let document = window.document().unwrap_throw();
			let canvas = document.get_element_by_id(CANVAS_ID).unwrap_throw();
			let html_canvas_element = canvas.unchecked_into();
			window_attributes = window_attributes.with_canvas(Some(html_canvas_element));
		}

		let window = Arc::new(event_loop.create_window(window_attributes).unwrap());

		#[cfg(not(target_arch = "wasm32"))]
		{
			// If we are not on web we can use pollster to
			// await the
			self.state = Some(pollster::block_on(State::new(window)).unwrap());
		}

		#[cfg(target_arch = "wasm32")]
		{
			// Run the future asynchronously and use the
			// proxy to send the results to the event loop
			if let Some(proxy) = self.proxy.take() {
				wasm_bindgen_futures::spawn_local(async move {
					assert!(proxy.send_event(State::new(window).await.expect("Unable to create canvas!!!")).is_ok())
				});
			}
		}
	}

	#[allow(unused_mut)]
	fn user_event(&mut self, _event_loop: &ActiveEventLoop, mut event: State) {
		// This is where proxy.send_event() ends up
		#[cfg(target_arch = "wasm32")]
		{
			event.renderer.window.request_redraw();
			event.resize(event.renderer.window.inner_size().width, event.renderer.window.inner_size().height);
		}

		self.state = Some(event);
	}

	fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: winit::window::WindowId, event: WindowEvent) {
		if let Some(state) = &mut self.state {
			state.renderer.imgui_platform.handle_event::<WindowEvent>(
				state.renderer.imgui.io_mut(),
				&state.renderer.window,
				&winit::event::Event::WindowEvent {
					window_id,
					event: event.clone(),
				},
			);
		}

		let state = match &mut self.state {
			Some(canvas) => canvas,
			None => return,
		};

		match event {
			WindowEvent::CloseRequested => event_loop.exit(),
			WindowEvent::Resized(size) => state.resize(size.width, size.height),
			WindowEvent::RedrawRequested => {
				let _zone = span!("Redraw Requested");
				let now = Instant::now();
				let dt = (now - self.last_update).as_secs_f32();
				self.last_update = now;
				state.renderer.imgui.io_mut().update_delta_time(std::time::Duration::from_secs_f32(dt));
				self.dt_avg = self.dt_avg * 0.8 + dt * 0.2;
				state.renderer.set_dt_avg(self.dt_avg);
				state.update(dt);
				match state.render() {
					Ok(_) => {}
					// Reconfigure the surface if it's lost or outdated
					Err(wgpu::CurrentSurfaceTexture::Suboptimal(_) | wgpu::CurrentSurfaceTexture::Outdated) => {
						let size = state.renderer.window.inner_size();
						state.resize(size.width, size.height);
					}
					Err(wgpu::CurrentSurfaceTexture::Occluded) => {
						// nothing to render if its occluded
					}
					Err(e) => {
						log::error!("Unable to render {:?}", e);
					}
				}
				frame_mark();
				state.renderer.window.request_redraw();
			}
			WindowEvent::KeyboardInput { event: KeyEvent { physical_key: PhysicalKey::Code(code), state: key_state, .. }, .. } => {
				if !state.renderer.imgui.io().want_capture_keyboard {
					state.handle_key(event_loop, code, key_state.is_pressed())
				}
			}
			WindowEvent::MouseInput { state: button_state, .. } if button_state.is_pressed() => {
				if !state.renderer.imgui.io().want_capture_mouse {
					state.audio_engine.resume();
					state.set_mouse_captured(true);
				}
			}
			_ => {}
		}
	}

	fn device_event(&mut self, _event_loop: &ActiveEventLoop, _device_id: DeviceId, event: DeviceEvent) {
		let state = match &mut self.state {
			Some(canvas) => canvas,
			None => return,
		};

		match event {
			DeviceEvent::MouseMotion { delta: (dx, dy) } => {
				// state.handle_mouse_motion(dx, dy)
				if state.mouse_captured && !state.renderer.imgui.io().want_capture_mouse {
					state.ecs.run_on_single_component_pair_mut::<camera::Camera, camera::CameraController, _>(state.player_id, |_entity_id,player_camera, player_camera_controller|
						player_camera_controller.handle_mouse_motion(player_camera, dx, dy)
					);
				}
			},
			_ => {}
		}
	}
}
