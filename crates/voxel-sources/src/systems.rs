use bevy::prelude::*;

use crate::handle::SourceHandle;
use crate::registry::SourceRegistry;
use crate::source::SourceId;

pub(crate) fn init_sources(registry: ResMut<SourceRegistry>) {
	let events = registry.event_tx.clone();
	let results = registry.result_tx.clone();
	let lod_results = registry.lod_result_tx.clone();
	for (i, source) in registry.sources.iter().enumerate() {
		source.init(SourceHandle {
			id: SourceId(i),
			events: events.clone(),
			results: results.clone(),
			lod_results: lod_results.clone(),
		});
	}
}
