#[cfg(not(target_arch = "wasm32"))]
mod desktop;
#[cfg(target_arch = "wasm32")]
mod web;

#[cfg(not(target_arch = "wasm32"))]
pub use desktop::audio_engine;
#[cfg(target_arch = "wasm32")]
pub use web::audio_engine;
