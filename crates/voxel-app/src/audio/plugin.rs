use bevy::ecs::message::{Message, MessageReader};
use bevy::math::Vec3;
use bevy::prelude::*;
use bevy::transform::components::GlobalTransform;

use crate::audio::audio_engine::{AudioEngine, ListenerState, SoundEffect};

pub struct AudioEngineResource(pub AudioEngine);

impl Default for AudioEngineResource {
	fn default() -> Self { Self(AudioEngine::new()) }
}

#[derive(Message, Debug, Clone, Copy)]
pub struct PlaySfx {
	pub effect: SoundEffect,
	pub position: Vec3,
}

impl PlaySfx {
	pub fn block_place(position: Vec3) -> Self { Self { effect: SoundEffect::BlockPlace, position } }
	pub fn block_break(position: Vec3) -> Self { Self { effect: SoundEffect::BlockBreak, position } }
}

pub struct VoxelAudioPlugin;

impl Plugin for VoxelAudioPlugin {
	fn build(&self, app: &mut App) {
		app.init_non_send::<AudioEngineResource>()
			.add_message::<PlaySfx>()
			.add_systems(Update, (update_listener, drain_sfx_messages));
	}
}

fn update_listener(
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	mut audio: NonSendMut<AudioEngineResource>,
) {
	let Some((_, global_transform)) = cameras.iter().find(|(c, _)| c.is_active) else { return };
	let t = global_transform.compute_transform();
	audio.0.set_listener(ListenerState {
		position: t.translation,
		forward: t.forward().as_vec3(),
		right: t.right().as_vec3(),
	});
}

fn drain_sfx_messages(
	mut messages: MessageReader<PlaySfx>,
	mut audio: NonSendMut<AudioEngineResource>,
) {
	for msg in messages.read() {
		audio.0.play_sound(msg.effect, msg.position);
	}
}
