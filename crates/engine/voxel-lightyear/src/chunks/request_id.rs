use serde::{Deserialize, Serialize};

/// A request ID that is meaningful only within the voxel network protocol.
/// SourceManager request IDs remain local to the process that allocated them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub(crate) struct NetworkRequestId(pub(crate) u64);

#[derive(Default)]
pub(crate) struct NetworkRequestIdAllocator {
	next: u64,
}

impl NetworkRequestIdAllocator {
	pub(crate) fn allocate(&mut self) -> NetworkRequestId {
		self.next = self.next.checked_add(1).expect("network request ID space exhausted");
		NetworkRequestId(self.next)
	}
}
