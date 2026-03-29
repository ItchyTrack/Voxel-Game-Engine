use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, Sample, SampleFormat, SizedSample, Stream, StreamConfig};

const SINE_FREQUENCY_HZ: f32 = 220.0;
const SINE_GAIN: f32 = 0.1;

pub struct AudioEngine {
	stream: Option<Stream>,
}

impl AudioEngine {
	pub fn new() -> Self {
		#[cfg(not(target_arch = "wasm32"))]
		let mut engine = Self { stream: None };
		#[cfg(target_arch = "wasm32")]
		let engine = Self { stream: None };

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

		let stream = build_output_stream(&device, &supported_config.config(), supported_config.sample_format());
		match stream.play() {
			Ok(()) => {
				self.stream = Some(stream);
			}
			Err(error) => {
				log::error!("Failed to start audio output stream: {error}");
			}
		}
	}
}

fn build_output_stream(device: &cpal::Device, config: &StreamConfig, sample_format: SampleFormat) -> Stream {
	match sample_format {
		SampleFormat::I8 => build_output_stream_typed::<i8>(device, config),
		SampleFormat::I16 => build_output_stream_typed::<i16>(device, config),
		SampleFormat::I24 => build_output_stream_typed::<cpal::I24>(device, config),
		SampleFormat::I32 => build_output_stream_typed::<i32>(device, config),
		SampleFormat::I64 => build_output_stream_typed::<i64>(device, config),
		SampleFormat::U8 => build_output_stream_typed::<u8>(device, config),
		SampleFormat::U16 => build_output_stream_typed::<u16>(device, config),
		SampleFormat::U32 => build_output_stream_typed::<u32>(device, config),
		SampleFormat::U64 => build_output_stream_typed::<u64>(device, config),
		SampleFormat::F32 => build_output_stream_typed::<f32>(device, config),
		SampleFormat::F64 => build_output_stream_typed::<f64>(device, config),
		_ => panic!("unsupported audio sample format"),
	}
}

fn build_output_stream_typed<T>(device: &cpal::Device, config: &StreamConfig) -> Stream
where
	T: Sample + SizedSample + FromSample<f32>,
{
	let channels = config.channels as usize;
	let mut synth = SineSynth::new(config.sample_rate.0 as f32, SINE_FREQUENCY_HZ, SINE_GAIN);

	device.build_output_stream(
		config,
		move |output: &mut [T], _| {
			for frame in output.chunks_mut(channels) {
				let sample = T::from_sample(synth.next_sample());
				for channel in frame.iter_mut() {
					*channel = sample;
				}
			}
		},
		move |error| {
			log::error!("audio stream error: {error}");
		},
		None,
	).expect("failed to build audio output stream")
}

struct SineSynth {
	sample_rate: f32,
	frequency: f32,
	gain: f32,
	phase: f32,
}

impl SineSynth {
	fn new(sample_rate: f32, frequency: f32, gain: f32) -> Self {
		Self {
			sample_rate,
			frequency,
			gain,
			phase: 0.0,
		}
	}

	fn next_sample(&mut self) -> f32 {
		let sample = (self.phase * std::f32::consts::TAU).sin() * self.gain;
		self.phase = (self.phase + self.frequency / self.sample_rate).fract();
		sample
	}
}
