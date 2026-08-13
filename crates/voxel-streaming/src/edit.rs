use bevy::prelude::*;
use tracy_client::span;

use voxel_edit::{EditGate, GridEdit, GridEdits};

use tile_data::chunks_covering_voxel_region;
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
		let region = chunks_covering_voxel_region(edit.voxel_bounds());
		let all_borrowed = (region.min().z..region.end().z).all(|z| {
			(region.min().y..region.end().y).all(|y| {
				(region.min().x..region.end().x).all(|x| self.borrowed_chunks.contains(&IVec3::new(x, y, z)))
			})
		});
		if all_borrowed { return true; }
		self.pending_borrow_edits.push(edit.clone());
		false
	}

	fn touched(&mut self, edit: &GridEdit) {
		let region = chunks_covering_voxel_region(edit.voxel_bounds());
		self.completed_edits.push(edit.clone());
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					let chunk = IVec3::new(x, y, z);
					let was_absent = self.presence.state(chunk).is_none();
					if let None | Some(ChunkState::Available) | Some(ChunkState::Loaded) | Some(ChunkState::InternalDirty) = self.presence.state(chunk) {
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
