use std::{
	any::Any,
	collections::HashMap,
	sync::{Arc, Mutex},
};

use bevy::{ecs::system::SystemParam, prelude::*};
use bile_data::{
	DirtyState,
	gather,
	streaming::{self, StreamId},
};
use bile_math::NonZeroRegion;
use crossbeam_channel::{Receiver, Sender};
use tile_data::VoxelRegionResult;
use voxel_data::grid::GridId;
use voxel_tasks::CancellationToken;

use crate::{
	ChunkSource, SourceHandle, SourceId,
	data::{VoxelData, VoxelPresence, VoxelRequest},
	request::{RequestId, SourceResult, SourceResultData},
};

pub(crate) enum RouteKind {
	Voxels(VoxelRequest),
	Presence(GridId),
}

pub(crate) struct BileRoute {
	source: Entity,
	kind: RouteKind,
	stream_id: StreamId,
	started: bool,
}

pub(crate) enum Route {
	/// The results go to Bile as part of a gather stream.
	Bile(BileRoute),
	/// The results are published as [`SourceResult`] messages, ending with a single loaded message.
	Direct { remaining: u32 },
}

/// Where the results of each request go.
#[derive(Default)]
pub(crate) struct Routes(pub(crate) HashMap<RequestId, Route>);

/// Every source sends its results through this channel, from whatever thread it builds them on.
#[derive(Resource, Clone)]
pub(crate) struct SourceChannel {
	sender: Sender<SourceResult>,
	receiver: Receiver<SourceResult>,
	routes: Arc<Mutex<Routes>>,
}

impl Default for SourceChannel {
	fn default() -> Self {
		let (sender, receiver) = crossbeam_channel::unbounded();
		Self { sender, receiver, routes: Arc::default() }
	}
}

/// A registered [`ChunkSource`]. It is a Bile gather node for [`VoxelData`] and [`VoxelPresence`]:
/// every source that covers a request is asked for it, and streams its part.
/// Every source is a node of this one type, so that sources can be registered at any time while the app is built.
#[derive(Component)]
pub struct VoxelSourceNode {
	source: Box<dyn ChunkSource>,
	entity: Entity,
	routes: Arc<Mutex<Routes>>,
	voxels: HashMap<VoxelRequest, (RequestId, CancellationToken)>,
	presence: HashMap<GridId, (RequestId, CancellationToken)>,
}

impl VoxelSourceNode {
	pub fn id(&self) -> SourceId { SourceId(self.entity) }

	pub fn source<S: ChunkSource>(&self) -> Option<&S> {
		(self.source.as_ref() as &dyn Any).downcast_ref()
	}

	pub fn source_mut<S: ChunkSource>(&mut self) -> Option<&mut S> {
		(self.source.as_mut() as &mut dyn Any).downcast_mut()
	}

	/// Starts a request whose results go to Bile, replacing the one this source had for the same params.
	fn route(&mut self, kind: RouteKind, stream_id: StreamId) -> (RequestId, CancellationToken) {
		let request_id = RequestId::next();
		let cancellation = CancellationToken::new();
		let previous = match &kind {
			RouteKind::Voxels(params) => self.voxels.insert(*params, (request_id, cancellation.clone())),
			RouteKind::Presence(grid) => self.presence.insert(*grid, (request_id, cancellation.clone())),
		};
		let mut routes = self.routes.lock().unwrap();
		if let Some((previous, previous_cancellation)) = previous {
			previous_cancellation.cancel();
			routes.0.remove(&previous);
		}
		routes.0.insert(request_id, Route::Bile(BileRoute { source: self.entity, kind, stream_id, started: false }));
		(request_id, cancellation)
	}

	fn stop(&self, stopped: Option<(RequestId, CancellationToken)>) {
		let Some((request_id, cancellation)) = stopped else { return };
		cancellation.cancel();
		self.routes.lock().unwrap().0.remove(&request_id);
	}
}

impl gather::DataProducingNode<VoxelData> for VoxelSourceNode {
	fn cost(&self, request: &streaming::Request<VoxelData>) -> Option<f32> {
		self.source.covers(request.params.grid, request.params.area).then_some(1.0)
	}

	fn request(&mut self, request: streaming::Request<VoxelData>, stream_id: StreamId) {
		let params = request.params;
		let (request_id, cancellation) = self.route(RouteKind::Voxels(params), stream_id);
		self.source.request_voxels(request_id, &cancellation, params.grid, params.area, params.lod, params.voxel_type);
	}

	fn drop(&mut self, params: &VoxelRequest) {
		let stopped = self.voxels.remove(params);
		self.stop(stopped);
	}
}

impl gather::DataProducingNode<VoxelPresence> for VoxelSourceNode {
	fn cost(&self, _request: &streaming::Request<VoxelPresence>) -> Option<f32> { Some(1.0) }

	fn request(&mut self, request: streaming::Request<VoxelPresence>, stream_id: StreamId) {
		let grid = request.params;
		let (request_id, cancellation) = self.route(RouteKind::Presence(grid), stream_id);
		self.source.request_presence(request_id, cancellation, grid);
	}

	fn drop(&mut self, grid: &GridId) {
		let stopped = self.presence.remove(grid);
		self.stop(stopped);
	}
}

pub(crate) fn register_source<S: ChunkSource>(app: &mut App, mut source: S) {
	let world = app.world_mut();
	assert!(
		world.query::<&VoxelSourceNode>().iter(world).all(|node| node.source::<S>().is_none()),
		"voxel source {} is already registered",
		std::any::type_name::<S>(),
	);
	let channel = world.get_resource_or_init::<SourceChannel>().clone();
	let entity = world.spawn(Name::new(std::any::type_name::<S>())).id();
	source.init(SourceHandle { id: SourceId(entity), messages: channel.sender.clone() });
	world.entity_mut(entity).insert(VoxelSourceNode {
		source: Box::new(source),
		entity,
		routes: channel.routes,
		voxels: HashMap::new(),
		presence: HashMap::new(),
	});
}

/// The registered sources, by their type. Also moves ownership of voxels between them, which is not done by Bile.
#[derive(SystemParam)]
pub struct VoxelSources<'w, 's> {
	nodes: Query<'w, 's, &'static mut VoxelSourceNode>,
	channel: Res<'w, SourceChannel>,
}

impl<'w, 's> VoxelSources<'w, 's> {
	pub fn get<S: ChunkSource>(&self) -> Option<&S> {
		self.nodes.iter().find_map(|node| node.source())
	}

	pub fn get_mut<S: ChunkSource>(&mut self) -> Option<&mut S> {
		self.nodes.iter_mut().find_map(|node| node.into_inner().source_mut())
	}

	/// Requests voxels straight from the sources that own them now, outside of Bile, so a new owner can
	/// take the voxels over. The results are published as [`SourceResult`] messages, ending with a single
	/// `VoxelsLoaded` once every source has finished.
	pub fn request_voxels(&mut self, request: VoxelRequest) -> RequestId {
		let request_id = RequestId::next();
		let remaining = self.nodes.iter().filter(|node| node.source.covers(request.grid, request.area)).count() as u32;
		self.channel.routes.lock().unwrap().0.insert(request_id, Route::Direct { remaining: remaining.max(1) });
		if remaining == 0 {
			let _ = self.channel.sender.send(SourceResult { request_id, data: SourceResultData::VoxelsLoaded });
			return request_id;
		}
		let cancellation = CancellationToken::new();
		for mut node in &mut self.nodes {
			if node.source.covers(request.grid, request.area) {
				node.source.request_voxels(request_id, &cancellation, request.grid, request.area, request.lod, request.voxel_type);
			}
		}
		request_id
	}

	pub fn transfer_ownership(&mut self, new_owner: SourceId, grid: GridId, region: NonZeroRegion) {
		for mut node in &mut self.nodes {
			if node.id() == new_owner {
				node.source.acquire_ownership(grid, region);
			} else {
				node.source.relinquish_ownership(grid, region);
			}
		}
	}
}

/// Passes what the sources sent to Bile, or publishes it for requests made outside of Bile.
pub(crate) fn receive_source_results(
	channel: Res<SourceChannel>,
	mut voxel_passer: streaming::DataPasser<VoxelData>,
	mut presence_passer: streaming::DataPasser<VoxelPresence>,
	mut direct: MessageWriter<SourceResult>,
) {
	let mut routes = channel.routes.lock().unwrap();
	for result in channel.receiver.try_iter() {
		let request_id = result.request_id;
		let Some(route) = routes.0.get_mut(&request_id) else { continue }; // dropped or replaced
		let loaded = matches!(result.data, SourceResultData::VoxelsLoaded | SourceResultData::PresenceLoaded);
		match route {
			Route::Direct { remaining } => {
				if loaded {
					*remaining -= 1;
					if *remaining > 0 { continue; }
					routes.0.remove(&request_id);
				}
				direct.write(result);
			}
			Route::Bile(route) => {
				match (&route.kind, result.data) {
					(RouteKind::Voxels(params), data) => {
						if !route.started {
							voxel_passer.start(route.source, params, route.stream_id, DirtyState::UpToDate);
							route.started = true;
						}
						match data {
							SourceResultData::Voxels { region, lod, voxels, .. } => {
								let region = Arc::new(VoxelRegionResult { area: region, lod, voxels });
								voxel_passer.push(route.source, params, route.stream_id, region);
							}
							SourceResultData::VoxelsLoaded => voxel_passer.finish(route.source, params, route.stream_id),
							data => warn!(?request_id, ?data, "ignoring presence sent for a voxel request"),
						}
					}
					(RouteKind::Presence(grid), data) => {
						if !route.started {
							presence_passer.start(route.source, grid, route.stream_id, DirtyState::UpToDate);
							route.started = true;
						}
						match data {
							SourceResultData::Presence { grid: sent, region } if sent == *grid => {
								presence_passer.push(route.source, grid, route.stream_id, region);
							}
							SourceResultData::PresenceLoaded => presence_passer.finish(route.source, grid, route.stream_id),
							data => warn!(?request_id, expected=?grid, ?data, "ignoring mismatched source presence payload"),
						}
					}
				}
				if loaded {
					routes.0.remove(&request_id);
				}
			}
		}
	}
}
