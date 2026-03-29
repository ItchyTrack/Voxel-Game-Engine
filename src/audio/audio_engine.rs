use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{SampleFormat, Stream, StreamConfig};
use rtrb::{Consumer, Producer, RingBuffer};

use glam::Vec3;

use super::instructions::AudioInstruction;
use super::mixer::build_output_stream_typed;

pub use super::instructions::{ListenerState, SoundEffect, SpawnVoiceInstruction};

const INSTRUCTION_QUEUE_CAPACITY: usize = 256;

pub struct AudioEngine {
	stream: Option<Stream>,
	instruction_producer: Producer<AudioInstruction>,
	instruction_consumer: Option<Consumer<AudioInstruction>>,
}

impl AudioEngine {
	pub fn new() -> Self {
		let (instruction_producer, instruction_consumer) = RingBuffer::new(INSTRUCTION_QUEUE_CAPACITY);
		#[cfg(not(target_arch = "wasm32"))]
		let mut engine = Self {
			stream: None,
			instruction_producer,
			instruction_consumer: Some(instruction_consumer),
		};
		#[cfg(target_arch = "wasm32")]
		let engine = Self {
			stream: None,
			instruction_producer,
			instruction_consumer: Some(instruction_consumer),
		};

		#[cfg(not(target_arch = "wasm32"))]
		engine.resume();

		engine
	}

	pub fn resume(&mut self) {
		if self.stream.is_some() {
			return;
		}

		let Some(device) = cpal::default_host().default_output_device() else {
			log::warn!("No audio output device available");
			return;
		};
		let Ok(supported_config) = device.default_output_config() else {
			log::warn!("No default audio output config available");
			return;
		};
		let Some(instruction_consumer) = self.instruction_consumer.take() else {
			log::warn!("Audio instruction consumer is unavailable");
			return;
		};

		let stream = build_output_stream(
			&device,
			&supported_config.config(),
			supported_config.sample_format(),
			instruction_consumer,
		);
		match stream.play() {
			Ok(()) => {
				self.stream = Some(stream);
			}
			Err(error) => {
				log::error!("Failed to start audio output stream: {error}");
			}
		}
	}

	pub fn spawn_voice(&mut self, instruction: SpawnVoiceInstruction) {
		self.push_instruction(AudioInstruction::SpawnVoice(instruction));
	}

	pub fn play_sound(&mut self, effect: SoundEffect, position: Vec3) {
		for instruction in sound_effect_recipe(effect, position) {
			self.spawn_voice(instruction);
		}
	}

	pub fn set_listener(&mut self, listener: ListenerState) {
		self.push_instruction(AudioInstruction::SetListener(listener));
	}

	fn push_instruction(&mut self, instruction: AudioInstruction) {
		if self.instruction_producer.push(instruction).is_err() {
			log::warn!("Audio instruction queue is full; dropping instruction");
		}
	}
}

fn sound_effect_recipe(effect: SoundEffect, position: Vec3) -> [SpawnVoiceInstruction; 2] {
	match effect {
		SoundEffect::BlockPlace => [
			SpawnVoiceInstruction {
				position,
				frequency_hz: 880.0,
				gain: 0.10,
				duration_seconds: 0.5,
				decay_rate: 18.0,
			},
			SpawnVoiceInstruction {
				position,
				frequency_hz: 1320.0,
				gain: 0.04,
				duration_seconds: 0.3,
				decay_rate: 26.0,
			},
		],
		SoundEffect::BlockBreak => [
			SpawnVoiceInstruction {
				position,
				frequency_hz: 240.0,
				gain: 0.12,
				duration_seconds: 0.9,
				decay_rate: 12.0,
			},
			SpawnVoiceInstruction {
				position,
				frequency_hz: 420.0,
				gain: 0.05,
				duration_seconds: 0.6,
				decay_rate: 17.0,
			},
		],
		SoundEffect::DebugBeep => [
			SpawnVoiceInstruction {
				position,
				frequency_hz: 660.0,
				gain: 0.16,
				duration_seconds: 15.0,
				decay_rate: 0.3,
			},
			SpawnVoiceInstruction {
				position,
				frequency_hz: 990.0,
				gain: 0.08,
				duration_seconds: 15.0,
				decay_rate: 0.6,
			},
		],
	}
}

fn build_output_stream(
	device: &cpal::Device,
	config: &StreamConfig,
	sample_format: SampleFormat,
	instruction_consumer: Consumer<AudioInstruction>,
) -> Stream {
	match sample_format {
		SampleFormat::I8 => build_output_stream_typed::<i8>(device, config, instruction_consumer),
		SampleFormat::I16 => build_output_stream_typed::<i16>(device, config, instruction_consumer),
		SampleFormat::I24 => build_output_stream_typed::<cpal::I24>(device, config, instruction_consumer),
		SampleFormat::I32 => build_output_stream_typed::<i32>(device, config, instruction_consumer),
		SampleFormat::I64 => build_output_stream_typed::<i64>(device, config, instruction_consumer),
		SampleFormat::U8 => build_output_stream_typed::<u8>(device, config, instruction_consumer),
		SampleFormat::U16 => build_output_stream_typed::<u16>(device, config, instruction_consumer),
		SampleFormat::U32 => build_output_stream_typed::<u32>(device, config, instruction_consumer),
		SampleFormat::U64 => build_output_stream_typed::<u64>(device, config, instruction_consumer),
		SampleFormat::F32 => build_output_stream_typed::<f32>(device, config, instruction_consumer),
		SampleFormat::F64 => build_output_stream_typed::<f64>(device, config, instruction_consumer),
		_ => panic!("unsupported audio sample format"),
	}
}
