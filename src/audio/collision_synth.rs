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

    let exciter_frequency = 200.0;
    let half_period = 0.5 / exciter_frequency;
    let energy = relative_velocity_change * relative_velocity_change;

	emit_voice(SpawnVoiceInstruction {
		position: event.contact_position,
		frequency_hz: exciter_frequency,
		gain: energy / 100000.0,
		max_volume_distance: COLLISION_MAX_VOLUME_DISTANCE,
		distance_falloff: COLLISION_DISTANCE_FALLOFF,
		duration_seconds: half_period,
		decay_rate: half_period.recip(),
	});
}
