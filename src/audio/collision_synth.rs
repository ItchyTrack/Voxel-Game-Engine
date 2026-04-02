use crate::collision_audio::CollisionAudioEvent;

use super::instructions::SpawnVoiceInstruction;

const MIN_AUDIBLE_VELOCITY_CHANGE: f32 = 3.0;
const COLLISION_MAX_VOLUME_DISTANCE: f32 = 1.0;
const COLLISION_DISTANCE_FALLOFF: f32 = 0.10;

pub fn synthesize_collision_audio(
	event: &CollisionAudioEvent,
	mut emit_voice: impl FnMut(SpawnVoiceInstruction),
) {
	let relative_velocity_change = event.pre_relative_velocity.length() - event.post_relative_velocity.length();
	if relative_velocity_change <= MIN_AUDIBLE_VELOCITY_CHANGE {
		return;
	}

	emit_voice(SpawnVoiceInstruction {
		position: event.contact_position,
		frequency_hz: 80.0 + relative_velocity_change * 0.4,
		gain: 0.002 + relative_velocity_change * 0.0008,
		max_volume_distance: COLLISION_MAX_VOLUME_DISTANCE,
		distance_falloff: COLLISION_DISTANCE_FALLOFF,
		duration_seconds: 0.1 + relative_velocity_change * 0.01,
		decay_rate: (40.0 - relative_velocity_change * 0.3).max(10.0),
	});
}
