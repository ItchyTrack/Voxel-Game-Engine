use std::{
	collections::{HashMap, HashSet},
	sync::{Arc, OnceLock},
};

use bevy::{math::{DMat3, DVec3}, prelude::*};
use basic_voxel::{BasicVoxel, LodVoxel, downsample_region};
use voxel_trees::region::NonZeroVoxelRegion;
use voxel_data::{
	grid::{Grid, GridId},
	voxels::{VoxelType, VoxelTypeId, Voxels},
};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion};
use voxel_physics::{IsStatic, RigidBody, components::VoxelCollider};
use voxel_lightyear::ReplicateVoxels;
use voxel_sources::{ForgottenChunks, ChunkSource, RequestId, SourceCoverage, SourceHandle, SourceManager, VoxelSourcesAppExt, edit::{GridEditIdManager, GridGeneration}};
use voxel_tasks::{AsyncPriorityTaskPool, CancellationToken};
use tile_data::{chunk_of, chunk_origin};
use voxel_streaming::GridStreaming;
use voxel_mass::{CenterOfMass, InertiaTensor, Mass, MassError, MassProperties, RotationalInertia, SourceMassChange, SourceMassState, VoxelMass, VoxelMassReaders, VoxelMassSet, mass_properties_of_voxels};

const WOOD_COLORS: [[u8; 4]; 3] = [[103, 67, 38, 255], [119, 78, 43, 255], [132, 88, 48, 255]];
const LEAF_COLORS: [[u8; 4]; 4] = [[42, 112, 48, 255], [52, 132, 55, 255], [65, 148, 61, 255], [79, 158, 68, 255]];

#[derive(Clone, Copy, Debug)]
pub struct TreeSettings {
	pub seed: u64,
	pub attraction_points: usize,
	pub canopy_center: Vec3,
	pub canopy_radius: Vec3,
	pub trunk_height: f32,
	pub influence_distance: f32,
	pub kill_distance: f32,
	pub segment_length: f32,
	pub max_iterations: usize,
	pub tip_radius: f32,
	pub pipe_exponent: f32,
	pub leaf_radius: f32,
	pub leaf_pipe_threshold: f32,
}

impl Default for TreeSettings {
	fn default() -> Self {
		Self {
			seed: 0x5eed_7aee,
			attraction_points: 1700,
			canopy_center: Vec3::new(0.0, 70.0, 0.0),
			canopy_radius: Vec3::new(71.0, 49.0, 71.0),
			trunk_height: 5.0,
			influence_distance: 45.0,
			kill_distance: 3.5,
			segment_length: 2.25,
			max_iterations: 1000,
			tip_radius: 0.3,
			pipe_exponent: 2.0,
			leaf_radius: 4.25,
			leaf_pipe_threshold: 3.0,
		}
	}
}

pub struct TreeSourcePlugin {
	pub settings: TreeSettings,
	pub position: Vec3,
}

impl Default for TreeSourcePlugin {
	fn default() -> Self { Self { settings: TreeSettings::default(), position: Vec3::new(0.0, 642.0, -2000.0) } }
}

impl Plugin for TreeSourcePlugin {
	fn build(&self, app: &mut App) {
		let grid = Arc::new(OnceLock::new());
		let bounds = estimated_chunk_bounds(self.settings);
		let model = Arc::new(build_tree_model(self.settings, bounds));
		let mass_state = SourceMassState::default();
		let mass_readers = app.world().resource::<VoxelMassReaders>().clone();
		app.register_voxel_source(TreeSource {
			grid: grid.clone(),
			bounds,
			model: model.clone(),
			handle: OnceLock::new(),
			forgotten: ForgottenChunks::default(),
			mass_state: mass_state.clone(),
			mass_readers,
		})
			.insert_resource(TreeGrid { grid, bounds, position: self.position, model, mass_state })
			.add_systems(Startup, spawn_tree)
			.add_systems(FixedUpdate, drain_tree_source_mass_changes.in_set(VoxelMassSet::SourceDrain));
	}
}

#[derive(Resource, Clone)]
struct TreeGrid {
	grid: Arc<OnceLock<GridId>>,
	bounds: NonZeroChunkRegion,
	position: Vec3,
	model: Arc<TreeModel>,
	mass_state: SourceMassState,
}

struct TreeSource {
	grid: Arc<OnceLock<GridId>>,
	bounds: NonZeroChunkRegion,
	model: Arc<TreeModel>,
	handle: OnceLock<SourceHandle>,
	forgotten: ForgottenChunks,
	mass_state: SourceMassState,
	mass_readers: VoxelMassReaders,
}

impl TreeSource {
	fn is_mine(&self, grid: GridId) -> bool { self.grid.get() == Some(&grid) }
}

impl ChunkSource for TreeSource {
	fn init(&mut self, handle: SourceHandle) {
		self.mass_state.set_source_id(handle.id());
		let _ = self.handle.set(handle);
	}

	fn request_voxels(
		&mut self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
	) -> SourceCoverage {
		if cancellation.is_cancelled() || !self.is_mine(grid) { return SourceCoverage::None; }

		let mut owned_chunks = Vec::new();
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					let chunk = IVec3::new(x, y, z);
					if self.bounds.contains(chunk) && !self.forgotten.contains(grid, chunk) {
						owned_chunks.push(chunk);
					}
				}
			}
		}
		let coverage = if owned_chunks.is_empty() {
			return SourceCoverage::None;
		} else if owned_chunks.len() == region.area() as usize {
			SourceCoverage::All
		} else {
			SourceCoverage::Some
		};

		let use_raw = match voxel_type {
			Some(id) if id == BasicVoxel::TYPE_INFO.id && lod == 0 => true,
			Some(id) if id == LodVoxel::TYPE_INFO.id => false,
			_ => lod == 0,
		};

		let model = self.model.clone();
		let mass_state = self.mass_state.clone();
		let mass_readers = self.mass_readers.clone();
		let handle = self.handle.get().expect("tree source was not initialized").clone();
		let cancellation = cancellation.clone();
		AsyncPriorityTaskPool::get().spawn(1.0, async move {
			let _span = bevy::log::info_span!("TreeSource build").entered();
			let voxels = if use_raw {
				let mut merged: Option<Voxels> = None;
				for &chunk in &owned_chunks {
					if cancellation.is_cancelled() { break; }
					let part = rasterize_tree_chunk(&model, chunk, &cancellation);
					if cancellation.is_cancelled() { break; }
					let exact = part.as_ref()
						.and_then(|voxels| mass_properties_of_voxels(&mass_readers, voxels, chunk_origin(chunk)))
						.unwrap_or(MassProperties::ZERO);
					let estimator = model.mass_estimator.chunks.get(&chunk).expect("accepted tree chunk has no mass estimator");
					mass_state.reconcile_generated_chunk(grid, chunk, estimator.estimate, estimator.error, exact);
					if let Some(part) = part {
						let offset = (chunk - region.min()) * CHUNK_SIZE as i32;
						merged.get_or_insert_with(|| Voxels::new::<BasicVoxel>()).merge_from(&part, offset);
					}
				}
				merged
			} else {
				// Skipping the fetch keeps the hole exact at every LOD, unlike punching the result.
				let owned_chunks: HashSet<_> = owned_chunks.into_iter().collect();
				downsample_region(region, lod as f32, |chunk| {
					owned_chunks.contains(&chunk)
						.then(|| rasterize_tree_chunk(&model, chunk, &cancellation))
						.flatten()
				})
			};
			if !cancellation.is_cancelled()
				&& let Some(voxels) = voxels.filter(|voxels| !voxels.is_empty()) {
				handle.voxels(request_id, grid, region, lod, generation, voxels);
			}
			handle.voxels_loaded(request_id, generation);
		});
		coverage
	}

	fn request_presence(&mut self, request_id: RequestId, _cancellation: CancellationToken, grid: GridId) {
		let handle = self.handle.get().expect("tree source was not initialized");
		if self.is_mine(grid) {
			handle.presence(request_id, grid, self.bounds);
		}
		handle.presence_loaded(request_id);
	}

	fn acquire_ownership(&mut self, grid: GridId, region: NonZeroChunkRegion) {
		if !self.is_mine(grid) { return; }
		self.forgotten.remember_area(grid, region);
	}

	fn relinquish_ownership(&mut self, grid: GridId, region: NonZeroChunkRegion) {
		if !self.is_mine(grid) { return; }
		self.forgotten.forget_area(grid, region);
	}
}

fn spawn_tree(mut commands: Commands, tree: Res<TreeGrid>) {
	let body = commands.spawn((RigidBody, IsStatic, Transform::from_translation(tree.position))).id();

	let mut streaming = GridStreaming::default();
	streaming.mark_present_area(tree.bounds);

	let grid = commands.spawn((Transform::IDENTITY, Grid::new::<BasicVoxel>(), GridEditIdManager::default(), ReplicateVoxels, VoxelCollider, VoxelMass, streaming)).id();
	let _ = tree.grid.set(grid);
	tree.mass_state.initialize_grid_once(
		grid,
		tree.model.mass_estimator.total_estimate,
		tree.model.mass_estimator.total_error,
	);
	commands.entity(body).add_child(grid);
}

fn drain_tree_source_mass_changes(
	sources: Res<SourceManager>,
	mut changes: MessageWriter<SourceMassChange>,
) {
	let Some(source) = sources.get_source::<TreeSource>() else { return };
	changes.write_batch(source.mass_state.drain_changes());
}

#[derive(Clone, Copy, Debug)]
struct BranchNode {
	position: Vec3,
	parent: Option<usize>,
}

#[derive(Clone, Copy, Debug)]
struct BranchSegment {
	start: Vec3,
	end: Vec3,
	radius: f32,
}

#[derive(Default)]
struct ChunkPrimitives {
	branches: Vec<usize>,
	leaves: Vec<usize>,
}

struct TreeModel {
	branches: Vec<BranchSegment>,
	leaves: Vec<Vec3>,
	primitive_index: HashMap<IVec3, ChunkPrimitives>,
	mass_estimator: TreeMassEstimator,
	settings: TreeSettings,
}

#[derive(Clone, Copy)]
struct ChunkMassEstimator {
	estimate: MassProperties,
	error: MassError,
}

struct TreeMassEstimator {
	chunks: HashMap<IVec3, ChunkMassEstimator>,
	total_estimate: MassProperties,
	total_error: MassError,
}

#[derive(Default)]
struct ChunkMassBuilder {
	properties: MassProperties,
	first_moment: [i128; 3],
	upper_mass: u64,
	lower_first_moment: [i128; 3],
	upper_first_moment: [i128; 3],
}

struct PrimitiveChunkPart {
	chunk: IVec3,
	weight: u64,
	mass: u64,
	first_moment: [i128; 3],
	upper_mass: u64,
	lower_first_moment: [i128; 3],
	upper_first_moment: [i128; 3],
}

type VoxelBounds = NonZeroVoxelRegion;

fn estimated_chunk_bounds(settings: TreeSettings) -> NonZeroChunkRegion {
	let segment_length = settings.segment_length.abs();
	let influence_distance = settings.influence_distance.abs();
	let canopy_radius = settings.canopy_radius.abs();
	let growth_padding = Vec3::splat(influence_distance + segment_length);
	let canopy_min = settings.canopy_center - canopy_radius - growth_padding;
	let canopy_max = settings.canopy_center + canopy_radius + growth_padding;

	let trunk_segments = (settings.trunk_height / settings.segment_length).ceil().max(0.0) as usize;
	let trunk_end = Vec3::Y * (trunk_segments as f32 * settings.segment_length);
	let tree_min = canopy_min.min(Vec3::ZERO).min(trunk_end);
	let tree_max = canopy_max.max(Vec3::ZERO).max(trunk_end);

	let estimated_pipe_radius = pipe_radius(settings.attraction_points.max(1) as f32, settings);
	let raster_padding = estimated_pipe_radius.max(settings.leaf_radius.abs()).max(1.0) + 2.0;
	let voxel_min = (tree_min - Vec3::splat(raster_padding)).floor().as_ivec3();
	let voxel_max = (tree_max + Vec3::splat(raster_padding)).ceil().as_ivec3();
	let min = chunk_of(voxel_min);
	let max = chunk_of(voxel_max) + IVec3::ONE;
	NonZeroChunkRegion::from_min_end(min, max).unwrap()
}

fn build_tree_model(settings: TreeSettings, bounds: NonZeroChunkRegion) -> TreeModel {
	let nodes = grow_branches(settings);
	let (child_counts, pipe_areas) = branch_pipe_areas(&nodes);
	let mut branches = Vec::with_capacity(nodes.len().saturating_sub(1));

	for (index, node) in nodes.iter().enumerate().skip(1) {
		let Some(parent) = node.parent else { continue };
		branches.push(BranchSegment { start: nodes[parent].position, end: node.position, radius: pipe_radius(pipe_areas[index], settings) });
	}

	let leaf_floor = settings.canopy_center.y - settings.canopy_radius.y * 0.75;
	let leaves = nodes
		.iter()
		.enumerate()
		.filter_map(|(index, node)| {
			let is_leafy_branch = pipe_areas[index] <= settings.leaf_pipe_threshold;
			(node.position.y >= leaf_floor && (is_leafy_branch || child_counts[index] == 0)).then_some(node.position)
		})
		.collect::<Vec<_>>();

	let mut primitive_index = HashMap::<IVec3, ChunkPrimitives>::new();
	for (index, branch) in branches.iter().enumerate() {
		index_primitive(&mut primitive_index, segment_voxel_bounds(*branch), |primitives| primitives.branches.push(index));
	}
	for (index, &center) in leaves.iter().enumerate() {
		index_primitive(&mut primitive_index, sphere_voxel_bounds(center, settings.leaf_radius), |primitives| primitives.leaves.push(index));
	}

	let mass_estimator = estimate_tree_mass(bounds, &branches, &leaves, settings);
	TreeModel { branches, leaves, primitive_index, mass_estimator, settings }
}

fn estimate_tree_mass(
	bounds: NonZeroChunkRegion,
	branches: &[BranchSegment],
	leaves: &[Vec3],
	settings: TreeSettings,
) -> TreeMassEstimator {
	let mut builders = HashMap::new();
	for z in bounds.min().z..bounds.end().z {
		for y in bounds.min().y..bounds.end().y {
			for x in bounds.min().x..bounds.end().x {
				builders.insert(IVec3::new(x, y, z), ChunkMassBuilder::default());
			}
		}
	}

	for &branch in branches {
		let radius = f64::from(branch.radius.abs());
		let length = f64::from(branch.start.distance(branch.end));
		let volume = std::f64::consts::PI * radius * radius * length
			+ 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
		let mass = rounded_mass(volume, 100);
		let center = (branch.start + branch.end) * 0.5;
		add_primitive_estimate(&mut builders, segment_voxel_bounds(branch), 100, MassProperties {
			mass: Mass(mass),
			center_of_mass: CenterOfMass(center.as_dvec3() + DVec3::splat(0.5)),
			rotational_inertia: RotationalInertia(cylinder_inertia(mass, radius, length, branch.end - branch.start)),
		});
	}
	for &center in leaves {
		let radius = f64::from(settings.leaf_radius.abs());
		let volume = 4.0 / 3.0 * std::f64::consts::PI * radius.powi(3);
		let mass = rounded_mass(volume, 10);
		add_primitive_estimate(&mut builders, sphere_voxel_bounds(center, settings.leaf_radius), 10, MassProperties {
			mass: Mass(mass),
			center_of_mass: CenterOfMass(center.as_dvec3() + DVec3::splat(0.5)),
			rotational_inertia: RotationalInertia(InertiaTensor::from_mat3(DMat3::IDENTITY * (0.4 * mass as f64 * radius * radius))),
		});
	}

	let mut builders: Vec<_> = builders.into_iter().collect();
	builders.sort_by_key(|(chunk, _)| (chunk.z, chunk.y, chunk.x));
	let mut chunks = HashMap::with_capacity(builders.len());
	let mut total_estimate = MassProperties::ZERO;
	let mut total_error = MassError::ZERO;
	for (chunk, builder) in builders {
		let estimate = builder.properties;
		let error = MassError::new(
			estimate.mass.0,
			builder.upper_mass.checked_sub(estimate.mass.0).expect("tree estimate exceeds primitive mass bound"),
			std::array::from_fn(|axis| {
				u64::try_from(builder.first_moment[axis] - builder.lower_first_moment[axis])
					.expect("tree first-moment error overflow")
			}),
			std::array::from_fn(|axis| {
				u64::try_from(builder.upper_first_moment[axis] - builder.first_moment[axis])
					.expect("tree first-moment error overflow")
			}),
		);
		total_estimate = total_estimate.checked_add(estimate);
		total_error = total_error.checked_add(error);
		chunks.insert(chunk, ChunkMassEstimator { estimate, error });
	}

	TreeMassEstimator {
		chunks,
		total_estimate,
		total_error,
	}
}

fn add_primitive_estimate(
	builders: &mut HashMap<IVec3, ChunkMassBuilder>,
	bounds: VoxelBounds,
	voxel_mass: u64,
	primitive: MassProperties,
) {
	let nominal_mass = primitive.mass.0;
	let center = primitive.center_of_mass.0 - DVec3::splat(0.5);
	let inertia_at_origin = primitive.rotational_inertia.0.move_from_center_of_mass(&primitive.center_of_mass.0, nominal_mass as f64);
	let mut parts = primitive_chunk_parts(bounds, voxel_mass);
	let total_weight: u64 = parts.iter().map(|part| part.weight).sum();
	let total_upper_mass: u64 = parts.iter().map(|part| part.upper_mass).sum();
	assert!(nominal_mass <= total_upper_mass, "analytic primitive estimate exceeds indexed voxel bound");

	let mut cumulative_weight = 0u128;
	let mut allocated_mass = 0u64;
	for part in &mut parts {
		cumulative_weight += u128::from(part.weight);
		let cumulative_mass = u128::from(nominal_mass) * cumulative_weight / u128::from(total_weight);
		let cumulative_mass = u64::try_from(cumulative_mass).expect("tree primitive mass overflow");
		part.mass = cumulative_mass.checked_sub(allocated_mass).expect("tree primitive mass allocation underflow");
		allocated_mass = cumulative_mass;
		for axis in 0..3 {
			let coordinate = primitive_part_average_coordinate(bounds, part.chunk, axis);
			part.first_moment[axis] = (part.mass as f64 * coordinate).round() as i128;
		}
	}
	assert_eq!(allocated_mass, nominal_mass);

	for axis in 0..3 {
		let target = (nominal_mass as f64 * center[axis]).round() as i128;
		let mut residual = target - parts.iter().map(|part| part.first_moment[axis]).sum::<i128>();
		for part in &mut parts {
			if residual > 0 {
				let adjustment = residual.min(part.upper_first_moment[axis] - part.first_moment[axis]);
				part.first_moment[axis] += adjustment;
				residual -= adjustment;
			} else if residual < 0 {
				let adjustment = (-residual).min(part.first_moment[axis] - part.lower_first_moment[axis]);
				part.first_moment[axis] -= adjustment;
				residual += adjustment;
			}
		}
		assert_eq!(residual, 0, "analytic primitive first moment exceeds indexed voxel bound");
	}

	for part in parts {
		let builder = builders.get_mut(&part.chunk).expect("primitive lies outside estimated tree bounds");
		builder.upper_mass = builder.upper_mass.checked_add(part.upper_mass).expect("tree mass bound overflow");
		for axis in 0..3 {
			builder.first_moment[axis] = builder.first_moment[axis]
				.checked_add(part.first_moment[axis])
				.expect("tree first moment overflow");
			builder.lower_first_moment[axis] = builder.lower_first_moment[axis]
				.checked_add(part.lower_first_moment[axis])
				.expect("tree first-moment bound overflow");
			builder.upper_first_moment[axis] = builder.upper_first_moment[axis]
				.checked_add(part.upper_first_moment[axis])
				.expect("tree first-moment bound overflow");
		}
		if part.mass != 0 {
			let center = DVec3::new(part.first_moment[0] as f64, part.first_moment[1] as f64, part.first_moment[2] as f64)
				/ part.mass as f64 + DVec3::splat(0.5);
			let inertia = InertiaTensor::from_mat3(inertia_at_origin.mat * (part.mass as f64 / nominal_mass as f64));
			builder.properties = builder.properties.checked_add(MassProperties {
				mass: Mass(part.mass),
				center_of_mass: CenterOfMass(center),
				rotational_inertia: RotationalInertia(inertia.move_to_center_of_mass(&center, part.mass as f64)),
			});
		}
	}
}

fn primitive_chunk_parts(bounds: VoxelBounds, voxel_mass: u64) -> Vec<PrimitiveChunkPart> {
	let chunk_min = chunk_of(bounds.min());
	let chunk_max = chunk_of(bounds.end() - IVec3::ONE);
	let mut parts = Vec::new();
	for z in chunk_min.z..=chunk_max.z {
		for y in chunk_min.y..=chunk_max.y {
			for x in chunk_min.x..=chunk_max.x {
				let chunk = IVec3::new(x, y, z);
				let origin = chunk_origin(chunk);
				let min = bounds.min().max(origin);
				let end = bounds.end().min(origin + IVec3::splat(CHUNK_SIZE as i32));
				let size = (end - min).as_uvec3();
				let weight = u64::from(size.x) * u64::from(size.y) * u64::from(size.z);
				let upper_mass = weight.checked_mul(voxel_mass).expect("tree primitive mass bound overflow");
				let mut lower_first_moment = [0i128; 3];
				let mut upper_first_moment = [0i128; 3];
				for axis in 0..3 {
					let plane_count = u64::from(size[(axis + 1) % 3]) * u64::from(size[(axis + 2) % 3]);
					let (negative_sum, positive_sum) = signed_coordinate_sums(min[axis], end[axis]);
					lower_first_moment[axis] = negative_sum * i128::from(plane_count) * i128::from(voxel_mass);
					upper_first_moment[axis] = positive_sum * i128::from(plane_count) * i128::from(voxel_mass);
				}
				parts.push(PrimitiveChunkPart {
					chunk,
					weight,
					mass: 0,
					first_moment: [0; 3],
					upper_mass,
					lower_first_moment,
					upper_first_moment,
				});
			}
		}
	}
	parts
}

fn primitive_part_average_coordinate(bounds: VoxelBounds, chunk: IVec3, axis: usize) -> f64 {
	let origin = chunk_origin(chunk);
	let min = bounds.min().max(origin);
	let end = bounds.end().min(origin + IVec3::splat(CHUNK_SIZE as i32));
	f64::from(min[axis] + end[axis] - 1) * 0.5
}

fn signed_coordinate_sums(min: i32, end: i32) -> (i128, i128) {
	let negative_end = end.min(0);
	let positive_min = min.max(0);
	(
		integer_range_sum(min, negative_end),
		integer_range_sum(positive_min, end),
	)
}

fn integer_range_sum(min: i32, end: i32) -> i128 {
	if end <= min { return 0 }
	let count = i128::from(end - min);
	count * i128::from(min + end - 1) / 2
}

fn rounded_mass(volume: f64, voxel_mass: u64) -> u64 {
	(volume * voxel_mass as f64).round() as u64
}

fn cylinder_inertia(mass: u64, radius: f64, length: f64, direction: Vec3) -> InertiaTensor {
	let mass = mass as f64;
	let axis = direction.normalize_or_zero().as_dvec3();
	let axial = 0.5 * mass * radius * radius;
	let radial = mass * (3.0 * radius * radius + length * length) / 12.0;
	InertiaTensor::from_mat3(DMat3::IDENTITY * radial + outer_product(axis) * (axial - radial))
}

fn outer_product(vector: DVec3) -> DMat3 {
	DMat3::from_cols(vector * vector.x, vector * vector.y, vector * vector.z)
}

fn index_primitive(chunks: &mut HashMap<IVec3, ChunkPrimitives>, bounds: VoxelBounds, mut add: impl FnMut(&mut ChunkPrimitives)) {
	let min = chunk_of(bounds.min());
	let max = chunk_of(bounds.end() - IVec3::ONE);
	for z in min.z..=max.z {
		for y in min.y..=max.y {
			for x in min.x..=max.x {
				add(chunks.entry(IVec3::new(x, y, z)).or_default());
			}
		}
	}
}

fn segment_voxel_bounds(branch: BranchSegment) -> VoxelBounds {
	let extent = branch.radius.max(1.0).ceil() as i32;
	let extent = IVec3::splat(extent);
	VoxelBounds::from_min_end(
		branch.start.min(branch.end).round().as_ivec3() - extent,
		branch.start.max(branch.end).round().as_ivec3() + extent + IVec3::ONE,
	).unwrap()
}

fn sphere_voxel_bounds(center: Vec3, radius: f32) -> VoxelBounds {
	let extent = IVec3::splat(radius.ceil() as i32);
	let center = center.round().as_ivec3();
	VoxelBounds::from_min_end(center - extent, center + extent + IVec3::ONE).unwrap()
}

fn rasterize_tree_chunk(model: &TreeModel, chunk: IVec3, cancellation: &CancellationToken) -> Option<Voxels> {
	let primitives = model.primitive_index.get(&chunk)?;
	let origin = chunk_origin(chunk);
	let bounds = VoxelBounds::from_min_size(origin, UVec3::splat(tile_data::CHUNK_SIZE)).unwrap();
	let mut points = HashMap::new();

	for &index in &primitives.branches {
		if cancellation.is_cancelled() {
			return None;
		}
		let branch = model.branches[index];
		rasterize_segment(&mut points, branch.start, branch.end, branch.radius, Some(bounds));
	}
	for &index in &primitives.leaves {
		if cancellation.is_cancelled() {
			return None;
		}
		rasterize_leaves(&mut points, model.leaves[index], model.settings.leaf_radius, model.settings.seed, Some(bounds));
	}
	if points.is_empty() {
		return None;
	}
	let points: Vec<_> = points.into_iter().map(|(position, voxel)| ((position - origin).as_uvec3(), voxel)).collect();
	let voxel_refs: Vec<_> = points.iter().map(|(pos, voxel)| (*pos, voxel.get_ref())).collect();
	let mut voxels = Voxels::new::<BasicVoxel>();
	voxels.add_voxels(&voxel_refs);
	Some(voxels)
}

#[cfg(test)]
fn generate_tree_voxels(settings: TreeSettings) -> HashMap<IVec3, BasicVoxel> {
	let bounds = estimated_chunk_bounds(settings);
	rasterize_tree_voxels(&build_tree_model(settings, bounds))
}

#[cfg(test)]
fn rasterize_tree_voxels(model: &TreeModel) -> HashMap<IVec3, BasicVoxel> {
	let mut voxels = HashMap::new();
	for branch in &model.branches {
		rasterize_segment(&mut voxels, branch.start, branch.end, branch.radius, None);
	}
	for &center in &model.leaves {
		rasterize_leaves(&mut voxels, center, model.settings.leaf_radius, model.settings.seed, None);
	}
	voxels
}

fn branch_pipe_areas(nodes: &[BranchNode]) -> (Vec<usize>, Vec<f32>) {
	let mut child_counts = vec![0usize; nodes.len()];
	for node in nodes.iter().skip(1) {
		if let Some(parent) = node.parent {
			child_counts[parent] += 1;
		}
	}

	let mut areas: Vec<f32> = child_counts.iter().map(|&children| if children == 0 { 1.0 } else { 0.0 }).collect();
	for index in (1..nodes.len()).rev() {
		if let Some(parent) = nodes[index].parent {
			areas[parent] += areas[index];
		}
	}
	(child_counts, areas)
}

fn pipe_radius(area: f32, settings: TreeSettings) -> f32 { settings.tip_radius * area.max(1.0).powf(1.0 / settings.pipe_exponent.max(0.1)) }

fn grow_branches(settings: TreeSettings) -> Vec<BranchNode> {
	let mut rng = SmallRng::new(settings.seed);
	let mut attractors = Vec::with_capacity(settings.attraction_points);
	while attractors.len() < settings.attraction_points {
		let point = Vec3::new(rng.signed(), rng.signed(), rng.signed());
		if point.length_squared() <= 1.0 {
			attractors.push(settings.canopy_center + point * settings.canopy_radius);
		}
	}

	let trunk_segments = (settings.trunk_height / settings.segment_length).ceil() as usize;
	let mut nodes = Vec::with_capacity(settings.attraction_points * 2);
	for segment in 0..=trunk_segments {
		nodes.push(BranchNode { position: Vec3::Y * (segment as f32 * settings.segment_length), parent: segment.checked_sub(1) });
	}

	let influence_squared = settings.influence_distance.powi(2);
	let kill_squared = settings.kill_distance.powi(2);
	for _ in 0..settings.max_iterations {
		attractors.retain(|attractor| nodes.iter().all(|node| node.position.distance_squared(*attractor) > kill_squared));
		if attractors.is_empty() {
			break;
		}

		let node_count = nodes.len();
		let mut directions = vec![Vec3::ZERO; node_count];
		let mut influences = vec![0u32; node_count];
		for attractor in &attractors {
			let nearest = nodes
				.iter()
				.enumerate()
				.map(|(index, node)| (index, node.position.distance_squared(*attractor)))
				.filter(|(_, distance)| *distance <= influence_squared)
				.min_by(|a, b| a.1.total_cmp(&b.1));
			let Some((index, _)) = nearest else { continue };
			directions[index] += (*attractor - nodes[index].position).normalize_or_zero();
			influences[index] += 1;
		}

		let mut additions = Vec::new();
		for index in 0..node_count {
			if influences[index] == 0 {
				continue;
			}
			let incoming = nodes[index].parent.map(|parent| (nodes[index].position - nodes[parent].position).normalize_or_zero()).unwrap_or(Vec3::Y);
			let direction = (directions[index] / influences[index] as f32 + incoming * 0.3 + Vec3::Y * 0.08).normalize_or_zero();
			let position = nodes[index].position + direction * settings.segment_length;
			let too_close = nodes
				.iter()
				.chain(additions.iter())
				.any(|other: &BranchNode| other.position.distance_squared(position) < (settings.segment_length * 0.55).powi(2));
			if !too_close {
				additions.push(BranchNode { position, parent: Some(index) });
			}
		}
		if additions.is_empty() {
			break;
		}
		nodes.extend(additions);
	}

	nodes
}

fn rasterize_segment(voxels: &mut HashMap<IVec3, BasicVoxel>, start: Vec3, end: Vec3, radius: f32, bounds: Option<VoxelBounds>) {
	let delta = end - start;
	let steps = (delta.length() * 2.0).ceil().max(1.0) as usize;
	for step in 0..=steps {
		let center = start.lerp(end, step as f32 / steps as f32);
		rasterize_sphere(voxels, center, radius.max(1.0), bounds, |position| BasicVoxel {
			color: WOOD_COLORS[(point_hash(position, 0) % WOOD_COLORS.len() as u64) as usize],
			mass: 100,
		});
	}
}

fn rasterize_leaves(voxels: &mut HashMap<IVec3, BasicVoxel>, center: Vec3, radius: f32, seed: u64, bounds: Option<VoxelBounds>) {
	let extent = radius.ceil() as i32;
	let center_voxel = center.round().as_ivec3();
	for z in -extent..=extent {
		for y in -extent..=extent {
			for x in -extent..=extent {
				let position = center_voxel + IVec3::new(x, y, z);
				if bounds.is_some_and(|bounds| !bounds.contains(position)) {
					continue;
				}
				let distance = position.as_vec3().distance(center);
				if distance > radius {
					continue;
				}
				let hash = point_hash(position, seed);
				if distance > radius - 1.0 && hash.is_multiple_of(5) {
					continue;
				}
				voxels.entry(position).or_insert(BasicVoxel { color: LEAF_COLORS[(hash % LEAF_COLORS.len() as u64) as usize], mass: 10 });
			}
		}
	}
}

fn rasterize_sphere(voxels: &mut HashMap<IVec3, BasicVoxel>, center: Vec3, radius: f32, bounds: Option<VoxelBounds>, voxel: impl Fn(IVec3) -> BasicVoxel) {
	let extent = radius.ceil() as i32;
	let center_voxel = center.round().as_ivec3();
	for z in -extent..=extent {
		for y in -extent..=extent {
			for x in -extent..=extent {
				let position = center_voxel + IVec3::new(x, y, z);
				if bounds.is_some_and(|bounds| !bounds.contains(position)) {
					continue;
				}
				if position.as_vec3().distance_squared(center) <= radius * radius {
					voxels.insert(position, voxel(position));
				}
			}
		}
	}
}

fn point_hash(point: IVec3, seed: u64) -> u64 {
	let mut hash = seed
		^ (point.x as u32 as u64).wrapping_mul(0x9e37_79b1)
		^ (point.y as u32 as u64).wrapping_mul(0x85eb_ca77)
		^ (point.z as u32 as u64).wrapping_mul(0xc2b2_ae3d);
	hash ^= hash >> 30;
	hash = hash.wrapping_mul(0xbf58_476d_1ce4_e5b9);
	hash ^= hash >> 27;
	hash = hash.wrapping_mul(0x94d0_49bb_1331_11eb);
	hash ^ (hash >> 31)
}

struct SmallRng(u64);

impl SmallRng {
	fn new(seed: u64) -> Self { Self(seed.max(1)) }

	fn next_u32(&mut self) -> u32 {
		let mut value = self.0;
		value ^= value << 13;
		value ^= value >> 7;
		value ^= value << 17;
		self.0 = value;
		(value >> 32) as u32
	}

	fn signed(&mut self) -> f32 { self.next_u32() as f32 / u32::MAX as f32 * 2.0 - 1.0 }
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn generated_tree_has_dense_foliage() {
		let settings = TreeSettings::default();
		let estimated_bounds = estimated_chunk_bounds(settings);
		let model = build_tree_model(settings, estimated_bounds);
		assert!(model.primitive_index.keys().all(|&chunk| estimated_bounds.contains(chunk)));

		let voxels = rasterize_tree_voxels(&model);
		let leaves = voxels.values().filter(|voxel| LEAF_COLORS.contains(&voxel.color)).count();
		let wood = voxels.values().filter(|voxel| WOOD_COLORS.contains(&voxel.color)).count();
		assert!(voxels.contains_key(&IVec3::ZERO));
		assert!(leaves > wood);
		assert!(voxels.keys().any(|position| position.y > 50));
	}

	#[test]
	fn model_contains_an_estimator_for_every_accepted_chunk() {
		let settings = TreeSettings { attraction_points: 0, max_iterations: 0, ..default() };
		let bounds = estimated_chunk_bounds(settings);
		let model = build_tree_model(settings, bounds);

		for z in bounds.min().z..bounds.end().z {
			for y in bounds.min().y..bounds.end().y {
				for x in bounds.min().x..bounds.end().x {
					assert!(model.mass_estimator.chunks.contains_key(&IVec3::new(x, y, z)));
				}
			}
		}
	}

	#[test]
	fn pipe_area_is_preserved_at_forks() {
		let nodes = [
			BranchNode { position: Vec3::ZERO, parent: None },
			BranchNode { position: Vec3::X, parent: Some(0) },
			BranchNode { position: Vec3::Z, parent: Some(0) },
		];
		let (_, areas) = branch_pipe_areas(&nodes);
		let settings = TreeSettings::default();
		let parent = pipe_radius(areas[0], settings).powf(settings.pipe_exponent);
		let children = pipe_radius(areas[1], settings).powf(settings.pipe_exponent) + pipe_radius(areas[2], settings).powf(settings.pipe_exponent);
		assert!((parent - children).abs() < 0.0001);
	}

	#[test]
	fn generated_tree_is_deterministic() {
		let settings = TreeSettings::default();
		assert_eq!(generate_tree_voxels(settings), generate_tree_voxels(settings));
	}
}
