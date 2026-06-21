#[cfg(not(target_arch = "wasm32"))]
pub mod network_client;
#[cfg(not(target_arch = "wasm32"))]
pub mod network_common;
#[cfg(not(target_arch = "wasm32"))]
pub mod network_server;
