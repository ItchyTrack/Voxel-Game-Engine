use bevy::math::IVec3;

/// Half-open axis-aligned voxel region: `[min, end)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GridRegion {
	pub min: IVec3,
	pub end: IVec3,
}

impl GridRegion {
	pub fn new(min: IVec3, end: IVec3) -> Option<Self> {
		if min.cmplt(end).all() {
			Some(Self { min, end })
		} else {
			None
		}
	}

	pub fn from_min_size(min: IVec3, size: IVec3) -> Option<Self> {
		if size.cmple(IVec3::ZERO).any() {
			return None;
		}
		Some(Self { min, end: checked_add(min, size)? })
	}

	pub fn from_min_max_inclusive(min: IVec3, max: IVec3) -> Option<Self> {
		Self::new(min, checked_add(max, IVec3::ONE)?)
	}

	pub fn size(self) -> IVec3 {
		self.end - self.min
	}

	pub fn max_inclusive(self) -> IVec3 {
		self.end - IVec3::ONE
	}

	pub fn contains(self, pos: IVec3) -> bool {
		pos.cmpge(self.min).all() && pos.cmplt(self.end).all()
	}

	pub fn contains_region(self, other: Self) -> bool {
		other.min.cmpge(self.min).all() && other.end.cmple(self.end).all()
	}

	pub fn intersects(self, other: Self) -> bool {
		self.min.cmplt(other.end).all() && other.min.cmplt(self.end).all()
	}

	pub fn intersection(self, other: Self) -> Option<Self> {
		Self::new(self.min.max(other.min), self.end.min(other.end))
	}

	pub fn translated(self, offset: IVec3) -> Self {
		Self { min: self.min + offset, end: self.end + offset }
	}
}

fn checked_add(a: IVec3, b: IVec3) -> Option<IVec3> {
	Some(IVec3::new(a.x.checked_add(b.x)?, a.y.checked_add(b.y)?, a.z.checked_add(b.z)?))
}
