use cpal::traits::DeviceTrait;
use cpal::{FromSample, Sample, SizedSample, Stream, StreamConfig};
use glam::Vec3;
use rtrb::Consumer;

use super::instructions::{AudioInstruction, ListenerState, SpawnVoiceInstruction};

const MAX_ACTIVE_VOICES: usize = 256;

pub fn build_output_stream_typed<T>(device: &cpal::Device, config: &StreamConfig, mut instruction_consumer: Consumer<AudioInstruction>) -> Stream
where
	T: Sample + SizedSample + FromSample<f32>,
{
	let channels = config.channels as usize;
	let mut mixer = AudioMixer::new(config.sample_rate.0 as f32);

	device
		.build_output_stream(
			config,
			move |output: &mut [T], _| {
				mixer.render(output, channels, &mut instruction_consumer);
			},
			move |error| {
				log::error!("audio stream error: {error}");
			},
			None,
		)
		.expect("failed to build audio output stream")
}

struct AudioMixer {
	sample_rate: f32,
	listener: ListenerState,
	active_voices: Vec<ActiveVoice>,
}

impl AudioMixer {
	fn new(sample_rate: f32) -> Self {
		Self { sample_rate, listener: ListenerState::default(), active_voices: Vec::with_capacity(MAX_ACTIVE_VOICES) }
	}

	fn render<T>(&mut self, output: &mut [T], channels: usize, instruction_consumer: &mut Consumer<AudioInstruction>)
	where
		T: Sample + SizedSample + FromSample<f32>,
	{
		self.drain_instructions(instruction_consumer);

		for frame in output.chunks_mut(channels) {
			let mut left = 0.0;
			let mut right = 0.0;

			for voice in &mut self.active_voices {
				let sample = voice.next_sample(self.sample_rate);
				if sample == 0.0 {
					continue;
				}
				let (voice_left, voice_right) = self.listener.stereo_gains(voice.position, voice.gain, voice.max_volume_distance, voice.distance_falloff);
				left += sample * voice_left;
				right += sample * voice_right;
			}

			self.active_voices.retain(|voice| !voice.finished());

			match channels {
				0 => {}
				1 => {
					frame[0] = T::from_sample((left + right) * 0.5);
				}
				_ => {
					frame[0] = T::from_sample(left);
					frame[1] = T::from_sample(right);
					let mono = T::from_sample((left + right) * 0.5);
					for channel in frame.iter_mut().skip(2) {
						*channel = mono;
					}
				}
			}
		}
	}

	fn drain_instructions(&mut self, instruction_consumer: &mut Consumer<AudioInstruction>) {
		let mut t: usize = 0;
		while let Ok(instruction) = instruction_consumer.pop() {
			match instruction {
				AudioInstruction::SpawnVoice(instruction) => {
					if self.active_voices.len() == MAX_ACTIVE_VOICES {
						self.active_voices.swap_remove(t % self.active_voices.len());
						t += 1;
					}
					self.active_voices.push(ActiveVoice::new(instruction));
				}
				AudioInstruction::SetListener(listener) => {
					self.listener = listener;
				}
			}
		}
	}
}

impl ListenerState {
	fn stereo_gains(&self, emitter_position: Vec3, source_gain: f32, max_volume_distance: f32, distance_falloff: f32) -> (f32, f32) {
		let offset = emitter_position - self.position;
		let distance = offset.length();
		if distance <= f32::EPSILON {
			return (source_gain, source_gain);
		}

		let direction = offset / distance;
		let pan = spatial_pan(direction, self.right, distance, max_volume_distance);
		let angle = (pan + 1.0) * std::f32::consts::FRAC_PI_4;
		let attenuation = attenuated_gain(source_gain, distance, max_volume_distance, distance_falloff);

		(attenuation * angle.cos(), attenuation * angle.sin())
	}
}

struct ActiveVoice {
	position: Vec3,
	gain: f32,
	max_volume_distance: f32,
	distance_falloff: f32,
	synth: SineVoice,
}

impl ActiveVoice {
	fn new(instruction: SpawnVoiceInstruction) -> Self {
		Self {
			position: instruction.position,
			gain: instruction.gain,
			max_volume_distance: instruction.max_volume_distance,
			distance_falloff: instruction.distance_falloff,
			synth: SineVoice::new(instruction.frequency_hz, instruction.duration_seconds, instruction.decay_rate),
		}
	}

	fn next_sample(&mut self, sample_rate: f32) -> f32 {
		self.synth.next_sample(sample_rate)
	}

	fn finished(&self) -> bool {
		self.synth.finished()
	}
}

struct SineVoice {
	frequency_hz: f32,
	duration_seconds: f32,
	decay_rate: f32,
	phase: f32,
	elapsed_seconds: f32,
	finished: bool,
}

impl SineVoice {
	fn new(frequency_hz: f32, duration_seconds: f32, decay_rate: f32) -> Self {
		Self { frequency_hz, duration_seconds, decay_rate, phase: 0.0, elapsed_seconds: 0.0, finished: false }
	}

	fn next_sample(&mut self, sample_rate: f32) -> f32 {
		if self.finished {
			return 0.0;
		}

		let envelope = (-self.decay_rate * self.elapsed_seconds).exp();
		let sample = (self.phase * std::f32::consts::TAU).sin() * envelope;
		self.phase = (self.phase + self.frequency_hz / sample_rate).fract();
		self.elapsed_seconds += 1.0 / sample_rate;

		if self.elapsed_seconds >= self.duration_seconds {
			self.finished = true;
		}

		sample
	}

	fn finished(&self) -> bool {
		self.finished
	}
}

fn attenuated_gain(source_gain: f32, distance: f32, max_volume_distance: f32, distance_falloff: f32) -> f32 {
	let max_volume_distance = max_volume_distance.max(0.0);
	let distance_falloff = distance_falloff.max(0.0);
	let distance_after_radius = (distance - max_volume_distance).max(0.0);
	source_gain / (1.0 + distance_falloff * distance_after_radius).sqrt()
}

fn spatial_pan(direction: Vec3, right: Vec3, distance: f32, max_volume_distance: f32) -> f32 {
	let max_volume_distance = max_volume_distance.max(0.0);
	let surround_blend = if max_volume_distance <= f32::EPSILON {
		1.0
	} else {
		(distance / max_volume_distance).clamp(0.0, 1.0)
	};

	(direction.dot(right) * surround_blend).clamp(-1.0, 1.0)
}
