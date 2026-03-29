use glam::Vec3;

#[derive(Clone, Copy)]
pub struct ListenerState {
	pub position: Vec3,
	pub forward: Vec3,
	pub up: Vec3,
}

impl Default for ListenerState {
	fn default() -> Self {
		Self {
			position: Vec3::ZERO,
			forward: Vec3::Z,
			up: Vec3::Y,
		}
	}
}

#[derive(Clone, Copy)]
pub struct SpawnVoiceInstruction {
	pub position: Vec3,
	pub frequency_hz: f32,
	pub gain: f32,
	pub duration_seconds: f32,
	pub decay_rate: f32,
}

#[derive(Clone, Copy)]
pub enum AudioInstruction {
	SpawnVoice(SpawnVoiceInstruction),
	SetListener(ListenerState),
}
