use std::collections::HashMap;

use bevy::ecs::entity::Entity;

fn align_up(value: u32, alignment: u32) -> u32 {
	value.next_multiple_of(alignment)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PendingUpload {
	pub entity: Entity,
	pub generation: u64,
	pub src_offset: u32,
	pub size: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotEntry {
	pub entity: Entity,
	pub generation: u64,
	pub src_offset: u32,
	pub size: u32,
	pub dst_offset: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CopyRegion {
	pub src_offset: u64,
	pub dst_offset: u64,
	pub size: u64,
}

#[derive(Debug)]
pub struct ResidencyPlan {
	pub required_capacity: u64,
	pub copy_regions: Vec<CopyRegion>,
	pub offsets: HashMap<Entity, u32>,
	pub slot_entries: Vec<SlotEntry>,
}

#[derive(Clone, Copy)]
struct Hole {
	start: u32,
	size: u32,
}

const NO_DESIRED_INDEX: usize = usize::MAX;
const NO_DST_OFFSET: u32 = u32::MAX;

pub fn plan_residency(
	previous_slot_entries: &[SlotEntry], previous_slot_entry_indices: &HashMap<Entity, usize>, desired_entries: &[PendingUpload], alignment: u32,
	move_call_budget: usize,
) -> ResidencyPlan {
	let mut retained_previous_to_desired = vec![NO_DESIRED_INDEX; previous_slot_entries.len()];
	let mut desired_dst_offsets = vec![NO_DST_OFFSET; desired_entries.len()];
	for (desired_index, desired) in desired_entries.iter().enumerate() {
		let Some(&previous_index) = previous_slot_entry_indices.get(&desired.entity) else {
			continue;
		};
		let previous = previous_slot_entries[previous_index];
		if previous.generation != desired.generation || previous.src_offset != desired.src_offset || previous.size != desired.size {
			continue;
		}

		retained_previous_to_desired[previous_index] = desired_index;
		desired_dst_offsets[desired_index] = previous.dst_offset;
	}

	let mut holes = Vec::new();
	let mut retained_previous_indices = Vec::new();
	let mut tail = 0u32;
	for (previous_index, previous) in previous_slot_entries.iter().copied().enumerate() {
		let span = align_up(previous.size, alignment);
		if previous.dst_offset > tail {
			holes.push(Hole {
				start: tail,
				size: previous.dst_offset - tail,
			});
		}

		if retained_previous_to_desired[previous_index] == NO_DESIRED_INDEX {
			holes.push(Hole {
				start: previous.dst_offset,
				size: span,
			});
		} else {
			retained_previous_indices.push(previous_index);
		}
		tail = previous.dst_offset + span;
	}

	let mut slot_entries = Vec::with_capacity(desired_entries.len());
	let mut desired_slot_indices = vec![0usize; desired_entries.len()];
	let mut copy_regions = Vec::new();
	for (desired_index, desired) in desired_entries.iter().copied().enumerate() {
		let dst_offset = if desired_dst_offsets[desired_index] != NO_DST_OFFSET {
			desired_dst_offsets[desired_index]
		} else {
			let span = align_up(desired.size, alignment);
			let dst_offset = take_first_fit(&mut holes, span).unwrap_or_else(|| {
				let start = tail;
				tail += span;
				start
			});
			desired_dst_offsets[desired_index] = dst_offset;
			copy_regions.push(CopyRegion {
				src_offset: desired.src_offset as u64,
				dst_offset: dst_offset as u64,
				size: align_up(desired.size, wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64,
			});
			dst_offset
		};

		desired_slot_indices[desired_index] = slot_entries.len();
		slot_entries.push(SlotEntry {
			entity: desired.entity,
			generation: desired.generation,
			src_offset: desired.src_offset,
			size: desired.size,
			dst_offset,
		});
	}

	let mut remaining_move_calls = move_call_budget;
	for previous_index in retained_previous_indices {
		if remaining_move_calls == 0 {
			break;
		}

		let desired_index = retained_previous_to_desired[previous_index];
		let slot_index = desired_slot_indices[desired_index];
		let planned = &mut slot_entries[slot_index];
		let span = align_up(planned.size, alignment);
		let Some(new_dst_offset) = take_first_fit_before(&mut holes, span, planned.dst_offset) else {
			continue;
		};

		let old_dst_offset = planned.dst_offset;
		planned.dst_offset = new_dst_offset;
		desired_dst_offsets[desired_index] = new_dst_offset;
		remaining_move_calls -= 1;
		copy_regions.push(CopyRegion {
			src_offset: planned.src_offset as u64,
			dst_offset: new_dst_offset as u64,
			size: align_up(planned.size, wgpu::COPY_BUFFER_ALIGNMENT as u32) as u64,
		});
		insert_hole(&mut holes, Hole {
			start: old_dst_offset,
			size: span,
		});
	}

	let required_capacity = slot_entries
		.iter()
		.map(|entry| entry.dst_offset as u64 + align_up(entry.size, alignment) as u64)
		.max()
		.unwrap_or(0);
	let offsets = desired_entries
		.iter()
		.enumerate()
		.map(|(desired_index, desired)| (desired.entity, desired_dst_offsets[desired_index]))
		.collect();
	slot_entries.sort_unstable_by_key(|entry| entry.dst_offset);

	ResidencyPlan {
		required_capacity,
		copy_regions,
		offsets,
		slot_entries,
	}
}

fn take_first_fit(holes: &mut Vec<Hole>, size: u32) -> Option<u32> {
	for index in 0..holes.len() {
		if holes[index].size < size {
			continue;
		}

		let start = holes[index].start;
		holes[index].start += size;
		holes[index].size -= size;
		if holes[index].size == 0 {
			holes.remove(index);
		}
		return Some(start);
	}
	None
}

fn take_first_fit_before(holes: &mut Vec<Hole>, size: u32, limit: u32) -> Option<u32> {
	for index in 0..holes.len() {
		let hole = holes[index];
		if hole.start >= limit {
			break;
		}
		if hole.size < size || limit - hole.start < size {
			continue;
		}

		let start = hole.start;
		holes[index].start += size;
		holes[index].size -= size;
		if holes[index].size == 0 {
			holes.remove(index);
		}
		return Some(start);
	}
	None
}

fn insert_hole(holes: &mut Vec<Hole>, hole: Hole) {
	if hole.size == 0 {
		return;
	}

	let mut insert_at = 0usize;
	while insert_at < holes.len() && holes[insert_at].start < hole.start {
		insert_at += 1;
	}
	holes.insert(insert_at, hole);

	if insert_at > 0 {
		let prev = holes[insert_at - 1];
		let current = holes[insert_at];
		if prev.start + prev.size == current.start {
			holes[insert_at - 1].size += current.size;
			holes.remove(insert_at);
			insert_at -= 1;
		}
	}

	if insert_at + 1 < holes.len() {
		let current = holes[insert_at];
		let next = holes[insert_at + 1];
		if current.start + current.size == next.start {
			holes[insert_at].size += next.size;
			holes.remove(insert_at + 1);
		}
	}
}

#[cfg(test)]
mod tests {
	use std::collections::HashMap;

	use bevy::ecs::entity::Entity;

	use super::{plan_residency, PendingUpload, SlotEntry};

	fn entity(bits: u64) -> Entity {
		Entity::from_bits(bits)
	}

	fn slot_entry_indices(entries: &[SlotEntry]) -> HashMap<Entity, usize> {
		entries.iter().enumerate().map(|(index, entry)| (entry.entity, index)).collect()
	}

	#[test]
	fn reuses_matching_entries_without_copying() {
		let previous = [SlotEntry {
			entity: entity(1),
			generation: 7,
			src_offset: 64,
			size: 64,
			dst_offset: 128,
		}];
		let plan = plan_residency(
			&previous,
			&slot_entry_indices(&previous),
			&[PendingUpload {
				entity: entity(1),
				generation: 7,
				src_offset: 64,
				size: 64,
			}],
			64,
			0,
		);

		assert!(plan.copy_regions.is_empty());
		assert_eq!(plan.required_capacity, 192);
		assert_eq!(plan.offsets[&entity(1)], 128);
	}

	#[test]
	fn changed_entries_take_holes_even_without_move_budget() {
		let previous = [
			SlotEntry {
				entity: entity(1),
				generation: 1,
				src_offset: 0,
				size: 64,
				dst_offset: 0,
			},
			SlotEntry {
				entity: entity(2),
				generation: 1,
				src_offset: 64,
				size: 64,
				dst_offset: 64,
			},
		];
		let plan = plan_residency(
			&previous,
			&slot_entry_indices(&previous),
			&[
				PendingUpload {
					entity: entity(2),
					generation: 1,
					src_offset: 64,
					size: 64,
				},
				PendingUpload {
					entity: entity(3),
					generation: 1,
					src_offset: 128,
					size: 64,
				},
			],
			64,
			0,
		);

		assert_eq!(plan.offsets[&entity(2)], 64);
		assert_eq!(plan.offsets[&entity(3)], 0);
		assert_eq!(plan.required_capacity, 128);
		assert_eq!(plan.copy_regions.len(), 1);
		assert_eq!(plan.copy_regions[0].dst_offset, 0);
	}

	#[test]
	fn move_call_budget_allows_optional_compaction() {
		let previous = [
			SlotEntry {
				entity: entity(1),
				generation: 1,
				src_offset: 0,
				size: 64,
				dst_offset: 0,
			},
			SlotEntry {
				entity: entity(2),
				generation: 1,
				src_offset: 64,
				size: 64,
				dst_offset: 128,
			},
		];
		let plan = plan_residency(
			&previous,
			&slot_entry_indices(&previous),
			&[
				PendingUpload {
					entity: entity(1),
					generation: 1,
					src_offset: 0,
					size: 64,
				},
				PendingUpload {
					entity: entity(2),
					generation: 1,
					src_offset: 64,
					size: 64,
				},
			],
			64,
			1,
		);

		assert_eq!(plan.offsets[&entity(2)], 64);
		assert_eq!(plan.required_capacity, 128);
		assert_eq!(plan.copy_regions.len(), 1);
		assert_eq!(plan.copy_regions[0].dst_offset, 64);
	}

	#[test]
	fn move_call_budget_is_not_limited_by_entry_size() {
		let previous = [
			SlotEntry {
				entity: entity(1),
				generation: 1,
				src_offset: 0,
				size: 64,
				dst_offset: 0,
			},
			SlotEntry {
				entity: entity(2),
				generation: 1,
				src_offset: 4096,
				size: 4096,
				dst_offset: 8192,
			},
		];
		let plan = plan_residency(
			&previous,
			&slot_entry_indices(&previous),
			&[
				PendingUpload {
					entity: entity(1),
					generation: 1,
					src_offset: 0,
					size: 64,
				},
				PendingUpload {
					entity: entity(2),
					generation: 1,
					src_offset: 4096,
					size: 4096,
				},
			],
			64,
			1,
		);

		assert_eq!(plan.offsets[&entity(2)], 64);
		assert_eq!(plan.required_capacity, 4160);
		assert_eq!(plan.copy_regions.len(), 1);
		assert_eq!(plan.copy_regions[0].dst_offset, 64);
		assert_eq!(plan.copy_regions[0].size, 4096);
	}

	#[test]
	fn zero_move_call_budget_keeps_sparse_tail_when_needed() {
		let previous = [
			SlotEntry {
				entity: entity(1),
				generation: 1,
				src_offset: 0,
				size: 64,
				dst_offset: 0,
			},
			SlotEntry {
				entity: entity(2),
				generation: 1,
				src_offset: 64,
				size: 64,
				dst_offset: 128,
			},
		];
		let plan = plan_residency(
			&previous,
			&slot_entry_indices(&previous),
			&[
				PendingUpload {
					entity: entity(1),
					generation: 1,
					src_offset: 0,
					size: 64,
				},
				PendingUpload {
					entity: entity(2),
					generation: 1,
					src_offset: 64,
					size: 64,
				},
			],
			64,
			0,
		);

		assert_eq!(plan.offsets[&entity(2)], 128);
		assert_eq!(plan.required_capacity, 192);
		assert!(plan.copy_regions.is_empty());
	}
}
