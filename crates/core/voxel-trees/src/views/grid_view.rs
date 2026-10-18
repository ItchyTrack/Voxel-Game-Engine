use bevy::math::UVec3;
use std::fmt::Debug;
use bile_math::NonZeroRegion;

pub trait GridView: Debug {
	type Data<'d>: Copy where Self: 'd;

	fn get(&self, pos: UVec3) -> Option<Self::Data<'_>>;

	fn bounds(&self) -> Option<NonZeroRegion>;

	fn for_each_in_region<'grid, F>(&'grid self, region: NonZeroRegion, mut f: F)
	where
		F: FnMut(UVec3, Self::Data<'grid>),
	{
		for x in region.min().x..region.end().x {
			for y in region.min().y..region.end().y {
				for z in region.min().z..region.end().z {
					let pos = UVec3::new(x as u32, y as u32, z as u32);
					if let Some(d) = self.get(pos) { f(pos, d); }
				}
			}
		}
	}

	fn any_in_region(&self, region: NonZeroRegion) -> bool {
		for x in region.min().x..region.end().x {
			for y in region.min().y..region.end().y {
				for z in region.min().z..region.end().z {
					let pos = UVec3::new(x as u32, y as u32, z as u32);
					if self.get(pos).is_some() { return true; }
				}
			}
		}
		false
	}

	fn is_region_filled(&self, region: NonZeroRegion) -> bool {
		for x in region.min().x..region.end().x {
			for y in region.min().y..region.end().y {
				for z in region.min().z..region.end().z {
					let pos = UVec3::new(x as u32, y as u32, z as u32);
					if self.get(pos).is_none() { return false; }
				}
			}
		}
		true
	}
}