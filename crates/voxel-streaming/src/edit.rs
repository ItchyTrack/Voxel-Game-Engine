use bevy::prelude::*;
use tracy_client::span;

use voxel_edit::{EditGate, GridEdit, GridEdits};

use crate::chunk::chunk_of;
use crate::presence::ChunkState;

use crate::GridStreaming;

impl GridStreaming {
	pub(crate) fn replay_stalled(&mut self, chunk: IVec3, edits: &mut Option<Mut<GridEdits>>) {
		let _zone = span!();
		let Some(stalled) = self.stalled_edits.remove(&chunk) else { return; };
		if let Some(edits) = edits.as_mut() {
			for edit in stalled {
				edits.push_edit(edit);
			}
		}
		if self.stalled_pinned.remove(&chunk) {
			self.release_completed(chunk);
		}
	}
}

impl EditGate for GridStreaming {
	fn admit(&mut self, edit: &GridEdit) -> bool {
		let (min, max) = edit.voxel_bounds();
		let chunk_min = chunk_of(min);
		let chunk_max = chunk_of(max - IVec3::ONE);
		let mut blocked_chunks = Vec::new();
		for z in chunk_min.z..=chunk_max.z {
			for y in chunk_min.y..=chunk_max.y {
				for x in chunk_min.x..=chunk_max.x {
					let chunk = IVec3::new(x, y, z);
					if matches!(
						self.presence.state(chunk),
						Some(ChunkState::Available | ChunkState::ExternalDirty | ChunkState::ExternalDirtyInFlight)
					) {
						blocked_chunks.push(chunk);
					}
				}
			}
		}
		if blocked_chunks.is_empty() {
			return true;
		}
		for chunk in blocked_chunks {
			self.stalled_edits.entry(chunk).or_default().push(edit.clone());
		}
		false
	}

	fn touched(&mut self, edit: &GridEdit) {
		let (min, max) = edit.voxel_bounds();
		let chunk_min = chunk_of(min);
		let chunk_max = chunk_of(max - IVec3::ONE);
		for z in chunk_min.z..=chunk_max.z {
			for y in chunk_min.y..=chunk_max.y {
				for x in chunk_min.x..=chunk_max.x {
					let chunk = IVec3::new(x, y, z);
					let was_absent = self.presence.state(chunk).is_none();
					if let None | Some(ChunkState::Loaded) | Some(ChunkState::InternalDirty) = self.presence.state(chunk) {
						self.presence.set_state(chunk, ChunkState::InternalDirty);
						self.newly_dirty.push(chunk);
						if was_absent {
							self.newly_present_dirty.push(chunk);
						}
					}
				}
			}
		}
	}
}
