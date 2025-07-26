use std::{fs::File, io::BufReader, ops::Sub};

use anyhow::{Context, Result};
use nalgebra::Complex;

use self::protocol::{Message, MessageDecoder};

mod decoder;
mod kalman;
mod protocol;
mod pzf;

fn main() -> Result<()> {
    let path = "/dev/ttyAMA0";

    let device = File::open(path).context("open device")?;
    let device = BufReader::new(device);
    let mut message_decoder = MessageDecoder::new(device);

    let mut expected_next_time = SampleTime::default();

    let mut decoder = decoder::Decoder::new();

    loop {
        let message = message_decoder.read()?;
        let message = match Message::read(message) {
            Ok(message) => message,
            Err(err) => {
                eprintln!("parse message: {err:#?}");
                continue;
            }
        };

        let sample_time = expected_next_time.parse_message_id(message.message_id);

        if !expected_next_time.is_zero() {
            let skipped_time = sample_time.checked_sub(expected_next_time).unwrap();
            if !skipped_time.is_zero() {
                decoder.missing_samples(skipped_time);
            }
        }

        for sample in message.samples {
            let measurement = Complex::new(
                (((sample.0 as i32) << 20 >> 20) << message.sample_shift as u32) as f64,
                (((sample.1 as i32) << 20 >> 20) << message.sample_shift as u32) as f64,
            );
            decoder.sample(measurement);
        }

        expected_next_time = sample_time.next_sample();
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct SampleTime {
    seconds: u64,

    /// 0..2500
    subsecond: u16,
}

impl SampleTime {
    fn parse_message_id(self, id: u32) -> Self {
        let mut seconds = (self.seconds & !0xF_FFFF) | (id >> 12) as u64;
        let subsecond = (id & 0xFFF) as u16;

        if seconds + 0x8_0000 < self.seconds {
            seconds += 0x10_0000;
        }

        Self { seconds, subsecond }
    }

    fn as_secs_f64(self) -> f64 {
        self.seconds as f64 + self.subsecond as f64 / 2500.0
    }

    fn checked_sub(self, rhs: Self) -> Option<Self> {
        let mut seconds = self.seconds.checked_sub(rhs.seconds)?;
        let subsecond = if let Some(subsecond) = self.subsecond.checked_sub(rhs.subsecond) {
            subsecond
        } else {
            seconds -= 1;
            self.subsecond
                .checked_add(2500)?
                .checked_sub(rhs.subsecond)?
        };
        Some(Self { seconds, subsecond })
    }

    fn next_sample(mut self) -> Self {
        self.subsecond += 1;
        if self.subsecond == 2500 {
            self.subsecond = 0;
            self.seconds += 1;
        }
        self
    }

    fn is_zero(self) -> bool {
        self == Self {
            seconds: 0,
            subsecond: 0,
        }
    }

    fn sample_index(self) -> u64 {
        self.seconds * 77500 + self.subsecond as u64 * 31
    }
}

impl Sub for SampleTime {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        let mut seconds = self.seconds - rhs.seconds;
        let subsecond = if let Some(subsecond) = self.subsecond.checked_sub(rhs.subsecond) {
            subsecond
        } else {
            seconds -= 1;
            self.subsecond + 2500 - rhs.subsecond
        };
        Self { seconds, subsecond }
    }
}
