use bevy::camera::Camera;
use bevy::ecs::entity::Entity;
use bevy::ecs::query::{Changed, Or};
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Query, Res, ResMut};
use bevy::math::{Affine3A, Vec3};
use bevy::render::Extract;
use bevy::render::renderer::WgpuWrapper;
use bevy::render::sync_world::RenderEntity;
use bevy::transform::components::{GlobalTransform, Transform};
use rustc_hash::FxHashMap;

use crate::direction_feedback::DirectionFeedback;
use crate::gpu_bvh::{BvhDataSource, BvhItemData};
use crate::gpu_data::RayWorldGpuData;
use crate::incoming_ray_directions::IncomingRayDirections;
use crate::residency::{ResidencyBuffers, ResidencyDirections, ResidentVoxels};
use crate::{tile_data::RayTileData, voxel_camera::VoxelCamera};
use voxel_data::aabb::aabb_of_transformed_aabb;
use voxel_data::bvh::BVH;
use voxel_gpu::PackedBufferAllocation;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

/// What one view renders this frame. BVH items are slot indices into the view's [`TileTable`].
pub struct ExtractedVoxelScene {
	pub bvh: Option<BVH<u32>>,
	pub tree_buffer: GpuBuffer,
	pub voxel_buffer: GpuBuffer,
	pub main_tree_buffer: GpuBuffer,
	pub main_voxel_buffer: GpuBuffer,
	/// Allocations of tiles that left the table this frame, kept alive until this frame has rendered.
	_retired_leases: Vec<(PackedBufferAllocation, PackedBufferAllocation)>,
}

#[derive(Resource, Default)]
pub struct ExtractedVoxelScenes(pub FxHashMap<Entity, ExtractedVoxelScene>);

/// Render-world mirror of the tiles each view draws, kept across frames. Per frame, only tiles that
/// moved or were rebuilt are touched, so extraction cost follows what changed, not how many tiles
/// are in view.
#[derive(Resource, Default)]
pub struct TileTables(pub FxHashMap<Entity, TileTable>);

#[derive(Default)]
pub struct TileTable {
	/// Same order as the camera's `tiles_to_render`, so a slot stays put while membership is unchanged.
	members: Vec<Entity>,
	slots: FxHashMap<Entity, u32>,
	/// Requested tiles whose data doesn't exist yet.
	missing: rustc_hash::FxHashSet<Entity>,
	pub tiles: Vec<TileEntry>,
	/// Bumped whenever slots are reassigned, so direction feedback can tell whether its slot indices
	/// still mean the same tiles.
	pub version: u64,
}

pub struct TileEntry {
	pub entity: Entity,
	tree: PackedBufferAllocation,
	voxels: PackedBufferAllocation,
	generation: u64,
	affine: Affine3A,
	aabb: (Vec3, Vec3),
	pub item: BvhItemData,
	directions: IncomingRayDirections,
}

impl TileEntry {
	fn new(entity: Entity, tile: &RayTileData, tile_global: &GlobalTransform) -> Self {
		let mut entry = Self {
			entity,
			tree: tile.tree.clone(),
			voxels: tile.voxels.clone(),
			generation: tile.generation,
			affine: Affine3A::NAN,
			aabb: (Vec3::ZERO, Vec3::ZERO),
			item: BvhItemData { data_source: BvhDataSource::FeedbackOnly, transform: Transform::IDENTITY, voxel_type: tile.voxel_type },
			directions: IncomingRayDirections::empty(),
		};
		entry.place(tile, tile_global);
		entry
	}

	/// Recomputes placement when the transform or contents changed. Returns the replaced
	/// allocations when the tile was rebuilt.
	fn update(&mut self, tile: &RayTileData, tile_global: &GlobalTransform) -> Option<(PackedBufferAllocation, PackedBufferAllocation)> {
		let mut retired = None;
		if tile.generation != self.generation {
			retired = Some((
				std::mem::replace(&mut self.tree, tile.tree.clone()),
				std::mem::replace(&mut self.voxels, tile.voxels.clone()),
			));
			self.generation = tile.generation;
			self.item.voxel_type = tile.voxel_type;
			self.affine = Affine3A::NAN;
		}
		self.place(tile, tile_global);
		retired
	}

	fn place(&mut self, tile: &RayTileData, tile_global: &GlobalTransform) {
		let affine = tile_global.affine();
		if affine == self.affine { return; }
		let scale = (1u32 << tile.voxel_lod) as f32;
		let transform = tile_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
		let placement = tile.placement;
		self.aabb = aabb_of_transformed_aabb(&transform, placement.bounds_min.as_vec3(), placement.bounds_max.as_vec3() + Vec3::ONE);
		self.item.transform = transform * Transform::from_translation(placement.tree_root_pos.as_vec3());
		self.affine = affine;
	}
}

impl TileTable {
	/// Rebuilds slots when the camera's tile list changed. New tiles are read from the main world;
	/// tiles that stay keep their entries.
	fn sync_members(
		&mut self,
		members: &[Entity],
		tiles: &Query<(&RayTileData, &GlobalTransform)>,
		retired: &mut Vec<(PackedBufferAllocation, PackedBufferAllocation)>,
	) {
		if self.members == members { return; }
		let mut old: FxHashMap<Entity, TileEntry> = self.tiles.drain(..).map(|entry| (entry.entity, entry)).collect();
		self.slots.clear();
		self.members.clear();
		self.missing.clear();
		for &entity in members {
			let entry = match old.remove(&entity) {
				Some(entry) => entry,
				None => {
					let Ok((tile, tile_global)) = tiles.get(entity) else {
						self.missing.insert(entity);
						continue;
					};
					TileEntry::new(entity, tile, tile_global)
				}
			};
			self.slots.insert(entity, self.tiles.len() as u32);
			self.tiles.push(entry);
		}
		// Keep the requested list, not just the tiles that exist yet, so a tile that appears later
		// is picked up through the change query below rather than missed.
		self.members.extend_from_slice(members);
		retired.extend(old.into_values().map(|entry| (entry.tree, entry.voxels)));
		self.version += 1;
	}
}

pub fn extract_voxel_scene(
	mut extracted_scenes: ResMut<ExtractedVoxelScenes>,
	mut tables: ResMut<TileTables>,
	mut residency: ResMut<ResidencyBuffers>,
	direction_feedback: Query<&DirectionFeedback>,
	cameras: Extract<Query<(RenderEntity, &VoxelCamera, &Camera, &GlobalTransform)>>,
	tiles: Extract<Query<(&RayTileData, &GlobalTransform)>>,
	changed_tiles: Extract<Query<(Entity, &RayTileData, &GlobalTransform), Or<(Changed<GlobalTransform>, Changed<RayTileData>)>>>,
	world_gpu: Extract<Res<RayWorldGpuData>>,
	render_stats: Res<crate::direction_feedback::RenderStats>,
) {
	let start = std::time::Instant::now();
	extract_scene(&mut extracted_scenes, &mut tables, &mut residency, &direction_feedback, &cameras, &tiles, &changed_tiles, &world_gpu);
	if render_stats.timing.enabled() {
		render_stats.timing.push_cpu(crate::gpu_timing::RenderCpuSample {
			extract_ms: start.elapsed().as_secs_f32() * 1e3,
			..crate::gpu_timing::RenderCpuSample::new(render_stats.timing.tag())
		});
	}
}

fn extract_scene(
	extracted_scenes: &mut ExtractedVoxelScenes,
	tables: &mut TileTables,
	residency: &mut ResidencyBuffers,
	direction_feedback: &Query<&DirectionFeedback>,
	cameras: &Query<(RenderEntity, &VoxelCamera, &Camera, &GlobalTransform)>,
	tiles: &Query<(&RayTileData, &GlobalTransform)>,
	changed_tiles: &Query<(Entity, &RayTileData, &GlobalTransform), Or<(Changed<GlobalTransform>, Changed<RayTileData>)>>,
	world_gpu: &RayWorldGpuData,
) {
	extracted_scenes.0.clear();

	let mut views = Vec::new();
	for (render_entity, voxel_camera, camera, camera_transform) in cameras.iter() {
		if camera.is_active { views.push((render_entity, voxel_camera, camera_transform.translation())); }
	}
	tables.0.retain(|view, _| views.iter().any(|(entity, _, _)| entity == view));

	let mut retired = Vec::new();
	for (view, voxel_camera, _) in &views {
		let table = tables.0.entry(*view).or_default();
		table.sync_members(&voxel_camera.tiles_to_render, tiles, &mut retired);
		for (entity, tile, tile_global) in changed_tiles.iter() {
			match table.slots.get(&entity) {
				Some(&slot) => retired.extend(table.tiles[slot as usize].update(tile, tile_global)),
				// A requested tile whose data only now exists.
				None if table.missing.remove(&entity) => {
					table.slots.insert(entity, table.tiles.len() as u32);
					table.tiles.push(TileEntry::new(entity, tile, tile_global));
					table.version += 1;
				}
				None => {}
			}
		}
		let feedback = direction_feedback.get(*view).ok();
		apply_direction_feedback(table, feedback);
	}

	let world_gpu = world_gpu.lock();
	select_resident(tables, &views, residency, &world_gpu);
	let offsets = residency.offsets();
	let tree_buffer = residency.tree_buffer().clone();
	let voxel_buffer = residency.voxel_buffer().clone();
	let main_tree_buffer = world_gpu.trees.buffer().clone();
	let main_voxel_buffer = world_gpu.voxels.buffer().clone();

	for (view, _, _) in &views {
		let table = tables.0.get_mut(view).unwrap();
		let mut bvh_items = Vec::with_capacity(table.tiles.len());
		for (slot, entry) in table.tiles.iter_mut().enumerate() {
			entry.item.data_source = if let Some(&(tree_offset, voxel_offset)) = offsets.get(&entry.entity) {
				BvhDataSource::Residency { tree_offset, voxel_offset }
			} else {
				match (world_gpu.trees.held_buffer(entry.tree.id()), world_gpu.voxels.held_buffer(entry.voxels.id())) {
					(Some(tree_held), Some(voxel_held)) => BvhDataSource::MainBuffer {
						tree_offset: tree_held.offset(),
						voxel_offset: voxel_held.offset(),
					},
					_ => BvhDataSource::FeedbackOnly,
				}
			};
			bvh_items.push((slot as u32, entry.aabb));
		}

		extracted_scenes.0.insert(*view, ExtractedVoxelScene {
			bvh: (!bvh_items.is_empty()).then(|| BVH::new(bvh_items)),
			tree_buffer: tree_buffer.clone(),
			voxel_buffer: voxel_buffer.clone(),
			main_tree_buffer: main_tree_buffer.clone(),
			main_voxel_buffer: main_voxel_buffer.clone(),
			_retired_leases: std::mem::take(&mut retired),
		});
	}
}

/// Copies last frame's per-tile ray directions into the table. Feedback is indexed by slot when the
/// table hasn't been reshuffled since; otherwise it is matched by entity.
fn apply_direction_feedback(table: &mut TileTable, feedback: Option<&DirectionFeedback>) {
	let Some(feedback) = feedback.and_then(|feedback| feedback.latest()) else {
		for entry in &mut table.tiles { entry.directions = IncomingRayDirections::empty(); }
		return;
	};
	if feedback.version == table.version {
		for entry in &mut table.tiles { entry.directions = IncomingRayDirections::empty(); }
		for &(slot, directions) in &feedback.slot_directions {
			if let Some(entry) = table.tiles.get_mut(slot as usize) { entry.directions = directions; }
		}
	} else {
		let by_entity: FxHashMap<Entity, IncomingRayDirections> = feedback.entity_directions().collect();
		for entry in &mut table.tiles {
			entry.directions = by_entity.get(&entry.entity).copied().unwrap_or_default();
		}
	}
}

/// Picks the tiles copied into the compact residency buffers: those that rays touched last frame.
/// Closest first, but sorting only happens when the binding limit would actually be exceeded.
fn select_resident(
	tables: &TileTables,
	views: &[(Entity, &VoxelCamera, Vec3)],
	residency: &mut ResidencyBuffers,
	world_gpu: &crate::gpu_data::RayGpuBuffers,
) {
	let tree_alignment = residency.tree_alignment() as u64;
	let voxel_alignment = residency.voxel_alignment() as u64;
	let limit = residency.binding_limit();

	struct Candidate<'a> { entry: &'a TileEntry, tree_size: u64, voxel_size: u64, priority: f32 }
	let mut candidates: Vec<Candidate<'_>> = Vec::new();
	let mut seen = (views.len() > 1).then(FxHashMap::<Entity, usize>::default);
	let (mut tree_total, mut voxel_total) = (0u64, 0u64);
	for (view, _, camera) in views {
		let Some(table) = tables.0.get(view) else { continue };
		for entry in &table.tiles {
			if entry.directions.is_empty() { continue; }
			let priority = camera.distance((entry.aabb.0 + entry.aabb.1) * 0.5);
			if let Some(seen) = seen.as_mut() {
				if let Some(&index) = seen.get(&entry.entity) {
					candidates[index].priority = candidates[index].priority.min(priority);
					continue;
				}
				seen.insert(entry.entity, candidates.len());
			}
			let (Some(tree_held), Some(voxel_held)) = (world_gpu.trees.held_buffer(entry.tree.id()), world_gpu.voxels.held_buffer(entry.voxels.id())) else { continue };
			let tree_size = (tree_held.size() as u64).next_multiple_of(tree_alignment);
			let voxel_size = (voxel_held.size() as u64).next_multiple_of(voxel_alignment);
			tree_total += tree_size;
			voxel_total += voxel_size;
			candidates.push(Candidate { entry, tree_size, voxel_size, priority });
		}
	}

	if tree_total > limit || voxel_total > limit {
		candidates.sort_unstable_by(|a, b| a.priority.total_cmp(&b.priority).then(a.entry.entity.cmp(&b.entry.entity)));
		let (mut tree_total, mut voxel_total, mut kept) = (0u64, 0u64, 0usize);
		let before = candidates.len();
		candidates.retain(|candidate| {
			if tree_total + candidate.tree_size > limit || voxel_total + candidate.voxel_size > limit { return false; }
			tree_total += candidate.tree_size;
			voxel_total += candidate.voxel_size;
			kept += 1;
			true
		});
		log::warn!("residency budget hit: {} feedback-selected sub-grids dropped ({kept} resident)", before - kept);
	}

	let mut resident: Vec<ResidentVoxels> = candidates.iter().map(|candidate| ResidentVoxels {
		entity: candidate.entry.entity,
		tree: candidate.entry.tree.id(),
		voxels: candidate.entry.voxels.id(),
		generation: candidate.entry.generation,
		loaded_directions: ResidencyDirections::Unculled,
	}).collect();
	resident.sort_unstable_by_key(|item| item.entity.to_bits());
	residency.upload(world_gpu, &resident);
}
