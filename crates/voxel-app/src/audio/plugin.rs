use bevy::ecs::message::{Message, MessageReader};
use bevy::prelude::*;
use voxel_math::FixedVec3;
use voxel_transform::TransformQuery;

use crate::audio::audio_engine::{AudioEngine, ListenerState, SoundEffect};

pub struct AudioEngineResource(pub AudioEngine);

impl Default for AudioEngineResource {
	fn default() -> Self { Self(AudioEngine::new()) }
}

#[derive(Message, Debug, Clone, Copy)]
pub struct PlaySfx {
	pub effect: SoundEffect,
	pub position: FixedVec3,
}

impl PlaySfx {
	pub fn block_place(position: FixedVec3) -> Self { Self { effect: SoundEffect::BlockPlace, position } }
	pub fn block_break(position: FixedVec3) -> Self { Self { effect: SoundEffect::BlockBreak, position } }
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
	cameras: Query<(Entity, &Camera), With<Camera3d>>,
	transforms: TransformQuery,
	mut audio: NonSendMut<AudioEngineResource>,
) {
	let Some((entity, _)) = cameras.iter().find(|(_, c)| c.is_active) else { return };
	let Some(t) = transforms.get_world(entity) else { return };
	audio.0.set_listener(ListenerState {
		position: t.translation,
		forward: t.forward(),
		right: t.right(),
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
