use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

/// A cheaply cloneable flag for cooperatively stopping queued or running work.
#[derive(Clone, Debug, Default)]
pub struct CancellationToken(Arc<AtomicBool>);

impl CancellationToken {
	pub fn new() -> Self { Self::default() }

	pub fn cancel(&self) {
		self.0.store(true, Ordering::Release);
	}

	pub fn is_cancelled(&self) -> bool {
		self.0.load(Ordering::Acquire)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn clones_observe_cancellation() {
		let token = CancellationToken::new();
		let worker_token = token.clone();
		token.cancel();
		assert!(worker_token.is_cancelled());
	}
}
