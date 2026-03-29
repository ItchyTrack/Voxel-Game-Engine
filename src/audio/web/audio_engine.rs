use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, Sample, SampleFormat, SizedSample, Stream, StreamConfig};

pub struct AudioEngine {
	stream: Option<Stream>,
}

impl AudioEngine {
	pub fn new() -> Self {
		Self { stream: None }
	}

	pub fn resume(&mut self) {
		if self.stream.is_some() {
			return;
		}

		let Some(device) = cpal::default_host().default_output_device() else {
			log::warn!("No wasm audio output device available yet");
			return;
		};
		let Ok(supported_config) = device.default_output_config() else {
			log::warn!("No wasm audio output config available yet");
			return;
		};

		let stream_config: StreamConfig = supported_config.config();
		let stream = match supported_config.sample_format() {
			SampleFormat::I8 => Self::build_output_stream::<i8>(&device, &stream_config),
			SampleFormat::I16 => Self::build_output_stream::<i16>(&device, &stream_config),
			SampleFormat::I24 => Self::build_output_stream::<cpal::I24>(&device, &stream_config),
			SampleFormat::I32 => Self::build_output_stream::<i32>(&device, &stream_config),
			SampleFormat::I64 => Self::build_output_stream::<i64>(&device, &stream_config),
			SampleFormat::U8 => Self::build_output_stream::<u8>(&device, &stream_config),
			SampleFormat::U16 => Self::build_output_stream::<u16>(&device, &stream_config),
			SampleFormat::U32 => Self::build_output_stream::<u32>(&device, &stream_config),
			SampleFormat::U64 => Self::build_output_stream::<u64>(&device, &stream_config),
			SampleFormat::F32 => Self::build_output_stream::<f32>(&device, &stream_config),
			SampleFormat::F64 => Self::build_output_stream::<f64>(&device, &stream_config),
			_ => {
				log::error!("Unsupported wasm audio sample format");
				return;
			}
		};

		if let Err(error) = stream.play() {
			log::error!("Failed to start wasm audio output stream: {error}");
			return;
		}

		self.stream = Some(stream);
	}

	fn build_output_stream<T>(device: &cpal::Device, config: &StreamConfig) -> Stream
	where
		T: Sample + SizedSample + FromSample<f32>,
	{
		let channels = config.channels as usize;
		let mut synth = SineSynth::new(config.sample_rate.0 as f32, 220.0, 0.1);

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
				log::error!("wasm audio stream error: {error}");
			},
			None,
		).expect("failed to build wasm audio output stream")
	}
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
