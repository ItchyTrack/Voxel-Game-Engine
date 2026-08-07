use std::{any::TypeId, collections::HashMap};

use crate::data::TileData;

type Reader<Capability> = Box<dyn Fn(&dyn TileData) -> Capability + Send + Sync>;

/// Type-erased readers for one consumer-defined tile capability.
///
/// Each subsystem owns a registry for the capability it consumes. Concrete tile-data types may be
/// registered with any number of such registries; `tile-data` itself does not know ray, raster,
/// physics, or bounds semantics.
pub struct TileCapabilityRegistry<Capability> {
	readers: HashMap<TypeId, Reader<Capability>>,
}

impl<Capability> Default for TileCapabilityRegistry<Capability> {
	fn default() -> Self {
		Self { readers: HashMap::new() }
	}
}

impl<Capability> TileCapabilityRegistry<Capability> {
	pub fn register<T: TileData>(&mut self, reader: impl Fn(&T) -> Capability + Send + Sync + 'static) {
		self.readers.insert(
			TypeId::of::<T>(),
			Box::new(move |data| {
				let data = data.as_any().downcast_ref::<T>().expect("tile capability reader was selected for the wrong tile-data type");
				reader(data)
			}),
		);
	}

	pub fn read(&self, data: &dyn TileData) -> Option<Capability> { self.readers.get(&data.as_any().type_id()).map(|reader| reader(data)) }
}
