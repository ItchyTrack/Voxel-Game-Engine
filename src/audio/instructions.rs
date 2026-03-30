use glam::Vec3;

#[derive(Clone, Copy)]
pub enum SoundEffect {
	BlockPlace,
	BlockBreak,
	DebugBeep,
}

#[derive(Clone, Copy)]
pub struct ListenerState {
	pub position: Vec3,
	pub forward: Vec3,
	pub right: Vec3,
}

impl Default for ListenerState {
	fn default() -> Self {
		Self {
			position: Vec3::ZERO,
			forward: Vec3::Z,
			right: Vec3::X
		}
	}
}

#[derive(Clone, Copy)]
pub struct SpawnVoiceInstruction {
	pub position: Vec3,
	pub frequency_hz: f32,
	pub gain: f32,
	pub max_volume_distance: f32,
	pub distance_falloff: f32,
	pub duration_seconds: f32,
	pub decay_rate: f32,
}

#[derive(Clone, Copy)]
pub enum AudioInstruction {
	SpawnVoice(SpawnVoiceInstruction),
	SetListener(ListenerState),
}
