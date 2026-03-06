fn xorshift(s: &mut u32) -> f32 {
    *s ^= *s << 13;
    *s ^= *s >> 17;
    *s ^= *s << 5;
    (*s as f32 / u32::MAX as f32) * 2.0 - 1.0
}

fn generate_victory_jingle(sr: f32) -> Vec<f32> {
    let dur = 0.9;
    let len = (dur * sr) as usize;
    let mut buf = vec![0.0f32; len];
    let tau = std::f32::consts::TAU;

    let notes: &[(f32, f32, f32)] = &[
        (0.00, 523.25, 0.15),
        (0.10, 659.25, 0.15),
        (0.20, 783.99, 0.15),
        (0.35, 1046.50, 0.35),
        (0.55, 1318.51, 0.30),
    ];

    for &(onset, freq, note_dur) in notes {
        let start_i = (onset * sr) as usize;
        let end_i = ((onset + note_dur) * sr).min(len as f32) as usize;
        for i in start_i..end_i {
            let t = (i - start_i) as f32 / sr;
            let p = t / note_dur;
            let attack = (p * 20.0).min(1.0);
            let release = (1.0 - p).powf(1.5);
            let env = attack * release;

            let s = (t * freq * tau).sin() * 0.5
                + (t * freq * 2.0 * tau).sin() * 0.2
                + (t * freq * 3.0 * tau).sin() * 0.08;

            buf[i] += s * env * 0.12;
        }
    }

    for s in buf.iter_mut() {
        *s = s.clamp(-1.0, 1.0);
    }
    buf
}

fn generate_snap_samples(sr: f32) -> Vec<f32> {
    let len = (0.08 * sr) as usize;
    let mut buf = vec![0.0f32; len];
    let mut seed: u32 = 0xDEAD_BEEF;
    for i in 0..len {
        let t = i as f32 / sr;
        let env = (1.0 - t / 0.08).powi(4);
        let thud = (t * 55.0 * std::f32::consts::TAU).sin() * (-t * 60.0).exp() * 0.5;
        let body = (t * 140.0 * std::f32::consts::TAU).sin() * (-t * 45.0).exp() * 0.25;
        let noise = xorshift(&mut seed) * 0.04 * (-t * 80.0).exp();
        buf[i] = (thud + body + noise) * env * 0.06;
    }
    buf
}

fn generate_vanish_samples(sr: f32) -> Vec<f32> {
    let dur = 0.15;
    let len = (dur * sr) as usize;
    let mut buf = vec![0.0f32; len];
    let mut seed: u32 = 0xCAFE_BABE;
    for i in 0..len {
        let t = i as f32 / sr;
        let p = t / dur;
        let env = (1.0 - p).powi(3) * (p * 12.0).min(1.0);
        let tone = (t * 80.0 * std::f32::consts::TAU).sin() * (-t * 30.0).exp() * 0.4;
        let body = (t * 160.0 * std::f32::consts::TAU).sin() * (-t * 40.0).exp() * 0.2;
        let noise = xorshift(&mut seed) * 0.06 * (1.0 - p).powi(4);
        buf[i] = (tone + body + noise) * env * 0.07;
    }
    buf
}

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::{generate_snap_samples, generate_vanish_samples, generate_victory_jingle};
    use rodio::source::Source;
    use rodio::stream::{DeviceSinkBuilder, MixerDeviceSink};
    use std::num::NonZero;
    use std::time::Duration;

    struct SampleSource {
        samples: Vec<f32>,
        pos: usize,
        sample_rate: u32,
    }

    impl SampleSource {
        fn from_samples(samples: Vec<f32>, sample_rate: u32) -> Self {
            Self {
                samples,
                pos: 0,
                sample_rate,
            }
        }
    }

    impl Source for SampleSource {
        fn current_span_len(&self) -> Option<usize> {
            None
        }
        fn channels(&self) -> NonZero<u16> {
            NonZero::new(1).unwrap()
        }
        fn sample_rate(&self) -> NonZero<u32> {
            NonZero::new(self.sample_rate).unwrap()
        }
        fn total_duration(&self) -> Option<Duration> {
            Some(Duration::from_secs_f32(
                self.samples.len() as f32 / self.sample_rate as f32,
            ))
        }
    }

    impl Iterator for SampleSource {
        type Item = f32;
        fn next(&mut self) -> Option<f32> {
            if self.pos >= self.samples.len() {
                return None;
            }
            let v = self.samples[self.pos];
            self.pos += 1;
            Some(v)
        }
    }

    pub struct AudioPlayer {
        sink: MixerDeviceSink,
    }

    impl AudioPlayer {
        pub fn new() -> Option<Self> {
            let sink = DeviceSinkBuilder::open_default_sink().ok()?;
            Some(Self { sink })
        }

        pub fn ensure_context(&mut self) {}

        pub fn resume(&self) {}

        pub fn play_firework(&self, _current_time: f32) {
            let samples = generate_victory_jingle(44100.0);
            self.sink
                .mixer()
                .add(SampleSource::from_samples(samples, 44100));
        }

        pub fn play_snap(&self) {
            let samples = generate_snap_samples(44100.0);
            self.sink
                .mixer()
                .add(SampleSource::from_samples(samples, 44100));
        }

        pub fn play_vanish(&self) {
            let samples = generate_vanish_samples(44100.0);
            self.sink
                .mixer()
                .add(SampleSource::from_samples(samples, 44100));
        }
    }
}

#[cfg(target_arch = "wasm32")]
mod web {
    use super::{generate_snap_samples, generate_vanish_samples, generate_victory_jingle};
    use web_sys::AudioContext;

    pub struct AudioPlayer {
        ctx: Option<AudioContext>,
    }

    impl AudioPlayer {
        pub fn new() -> Option<Self> {
            Some(Self { ctx: None })
        }

        pub fn ensure_context(&mut self) {
            if self.ctx.is_some() {
                return;
            }
            self.ctx = AudioContext::new().ok();
        }

        pub fn resume(&self) {
            if let Some(ctx) = &self.ctx {
                if ctx.state() == web_sys::AudioContextState::Suspended {
                    let _ = ctx.resume();
                }
            }
        }

        fn play_buffer(&self, data: &[f32]) {
            let ctx = match &self.ctx {
                Some(c) => c,
                None => return,
            };
            if ctx.state() == web_sys::AudioContextState::Suspended {
                let _ = ctx.resume();
            }
            let sr = ctx.sample_rate();
            let len = data.len() as u32;
            let buffer = match ctx.create_buffer(1, len, sr) {
                Ok(b) => b,
                Err(_) => return,
            };
            let _ = buffer.copy_to_channel(data, 0);
            if let Ok(source) = ctx.create_buffer_source() {
                source.set_buffer(Some(&buffer));
                let _ = source.connect_with_audio_node(&ctx.destination());
                let _ = source.start();
            }
        }

        pub fn play_firework(&self, _current_time: f32) {
            let sr = match &self.ctx {
                Some(c) => c.sample_rate(),
                None => return,
            };
            let data = generate_victory_jingle(sr);
            self.play_buffer(&data);
        }

        pub fn play_snap(&self) {
            let sr = match &self.ctx {
                Some(c) => c.sample_rate(),
                None => return,
            };
            let data = generate_snap_samples(sr);
            self.play_buffer(&data);
        }

        pub fn play_vanish(&self) {
            let sr = match &self.ctx {
                Some(c) => c.sample_rate(),
                None => return,
            };
            let data = generate_vanish_samples(sr);
            self.play_buffer(&data);
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use native::AudioPlayer;

#[cfg(target_arch = "wasm32")]
pub use web::AudioPlayer;
