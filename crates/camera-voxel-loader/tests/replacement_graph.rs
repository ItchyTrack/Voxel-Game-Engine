use bevy::prelude::*;

mod types {
	use bevy::prelude::*;
	use voxel_data::grid::GridId;
	#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub(crate) struct TileKey { pub(crate) grid: GridId, pub(crate) lod: u8, pub(crate) min: IVec3 }
}
mod coverage { pub(crate) type TileKey = crate::types::TileKey; }
#[allow(dead_code)]
#[path = "../src/replacement_graph.rs"]
mod replacement_graph;

use coverage::TileKey;
use replacement_graph::{DependencyRecord, ReplacementGraph};
use types::TileKey;

fn grid() -> Entity { Entity::from_bits(1) }
fn tile(lod: u8, min: IVec3) -> TileKey { TileKey { grid: grid(), lod, min } }
fn chunk(min: IVec3) -> TileKey { tile(0, min) }

#[test]
fn unresolved_replacements_block_removal_until_all_are_satisfied() {
	let mut graph = ReplacementGraph::default();
	let old = tile(2, IVec3::ZERO);
	let a = tile(1, IVec3::ZERO);
	let b = tile(1, IVec3::new(2, 0, 0));
	graph.add_record(DependencyRecord::new(old, [a, b]));
	assert!(graph.apply_satisfied(a).is_empty());
	assert_eq!(graph.apply_satisfied(b), vec![old]);
}

#[test]
fn removals_do_not_propagate_transitively() {
	let mut graph = ReplacementGraph::default();
	let old = tile(2, IVec3::ZERO);
	let mid = tile(1, IVec3::ZERO);
	let new = chunk(IVec3::ZERO);
	graph.add_record(DependencyRecord::new(old, [mid]));
	graph.add_record(DependencyRecord::new(mid, [new]));
	assert_eq!(graph.apply_satisfied(new), vec![mid]);
	assert_eq!(graph.apply_satisfied(mid), vec![old]);
}

#[test]
fn cancel_record_allows_same_source_to_retire_again() {
	let mut graph = ReplacementGraph::default();
	let old = tile(2, IVec3::ZERO);
	let replacement = tile(1, IVec3::ZERO);
	graph.add_record(DependencyRecord::new(old, [replacement]));
	graph.cancel_record(old);
	graph.add_record(DependencyRecord::new(old, [replacement]));
	assert_eq!(graph.apply_satisfied(replacement), vec![old]);
}

#[test]
fn one_satisfied_source_can_unlock_multiple_records() {
	let mut graph = ReplacementGraph::default();
	let old_a = tile(2, IVec3::ZERO);
	let old_b = tile(2, IVec3::new(4, 0, 0));
	let replacement = tile(1, IVec3::ZERO);
	graph.add_record(DependencyRecord::new(old_a, [replacement]));
	graph.add_record(DependencyRecord::new(old_b, [replacement]));
	let mut removed = graph.apply_satisfied(replacement);
	removed.sort_by_key(|source| source.min.x);
	assert_eq!(removed, vec![old_a, old_b]);
}

#[test]
#[should_panic(expected = "replacement dependency records must have at least one replacement")]
fn empty_replacement_sets_are_invalid() {
	ReplacementGraph::default().add_record(DependencyRecord::new(tile(2, IVec3::ZERO), []));
}

#[test]
#[should_panic(expected = "replacement dependency record already exists")]
fn duplicate_records_are_invalid() {
	let mut graph = ReplacementGraph::default();
	let old = tile(2, IVec3::ZERO);
	let replacement = tile(1, IVec3::ZERO);
	graph.add_record(DependencyRecord::new(old, [replacement]));
	graph.add_record(DependencyRecord::new(old, [replacement]));
}
