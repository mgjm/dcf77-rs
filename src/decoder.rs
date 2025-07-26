use std::{f64::consts::PI, mem, num::Wrapping};

use nalgebra::Complex;

use crate::{SampleTime, kalman::KalmanFilter, pzf};

pub struct Decoder {
    simple_phase: Complex<f64>,
    kf: KalmanFilter,
    phase_decoder: PhaseDecoder,
    sample_counter: usize,
    prev_phase: f64,
}

impl Decoder {
    pub fn new() -> Self {
        Self {
            simple_phase: Complex::default(),
            kf: KalmanFilter::new(),
            phase_decoder: PhaseDecoder::new(),
            sample_counter: 0,
            prev_phase: 0.0,
        }
    }

    pub fn missing_samples(&mut self, duration: SampleTime) {
        let mut kf = self.kf.clone();
        for _ in 0..duration.sample_index() {
            // send simulated phase values to decoder,
            // those samples will have no phase offset
            self.phase_decoder.sample(Complex::ZERO);
            kf.advance_time(1.0 / 77500.0);

            self.increment_sample_counter();
        }

        // update kalman filter in one step to reduce rounding errors
        self.kf.advance_time(duration.as_secs_f64());
    }

    pub fn sample(&mut self, measurement: Complex<f64>) {
        self.simple_phase += measurement;

        self.kf
            .measurement(measurement.arg(), const { (PI / 3.0) * (PI / 3.0) });

        self.phase_decoder
            .sample(measurement * Complex::cis(-self.kf.state.x));

        self.increment_sample_counter();

        self.kf.advance_time(1.0 / (2500.0 * 31.0));
    }

    fn increment_sample_counter(&mut self) {
        const MESSAGES_PER_SEC: usize = 1;

        self.sample_counter += 1;
        if self.sample_counter != 77500 / MESSAGES_PER_SEC {
            return;
        }
        self.sample_counter = 0;

        let mut diff = (self.prev_phase - self.kf.state.y)
            .to_degrees()
            .rem_euclid(360.0);

        if diff > 180.0 {
            diff -= 360.0;
        }

        diff *= MESSAGES_PER_SEC as f64;

        // println!(
        //     "{:7.3}° | {:7.3}° {:9.5}°/s | {:7.5}° {:7.5}°/s | {:6.3}°/s^2 | {:6.3} Hz {:8.5} Hz/s",
        //     // min_max.1 - min_max.0,
        //     self.simple_phase.arg().to_degrees().rem_euclid(360.0),
        //     self.kf.state.x.to_degrees().rem_euclid(360.0),
        //     self.kf.state.y.to_degrees(),
        //     self.kf.uncertainty.m11.sqrt(),
        //     self.kf.uncertainty.m22.sqrt(),
        //     diff,
        //     freq,
        //     (freq - self.prev_freq) * MESSAGES_PER_SEC as f64,
        // );

        println!();
        println!(
            "current phase angle: {:7.3}°",
            self.simple_phase.arg().to_degrees().rem_euclid(360.0),
        );
        println!(
            "kalman filter:       {:7.3}° {:9.5}°/s",
            self.kf.state.x.to_degrees().rem_euclid(360.0),
            self.kf.state.y.to_degrees(),
        );
        println!(
            "uncertainty:         {:7.5}° {:9.5}°/s",
            self.kf.uncertainty.m11.sqrt(),
            self.kf.uncertainty.m22.sqrt(),
        );
        println!("acceleration:         {diff:6.3}°/s^2");

        self.simple_phase = Complex::default();
        self.prev_phase = self.kf.state.y;
    }
}

const NUM_RAW_SAMPLES: usize = 1 << 18;

const GROUP_SIZE: usize = 60;

const GROUPS_PER_SECOND: usize = 77500_usize.div_ceil(GROUP_SIZE);

/// GROUPS_PER_SECOND start positions for 512 chip long sequence (2 groups per chip)
const NUM_GROUPS: usize = GROUPS_PER_SECOND + 512 * 2 - 1;

pub struct PhaseDecoder {
    raw_samples: [Complex<f64>; NUM_RAW_SAMPLES],
    raw_index: Wrapping<usize>,

    /// 60 samples per group
    current_groups: [Complex<f64>; NUM_GROUPS],
    next_groups: [Complex<f64>; NUM_GROUPS],

    /// current index into the groups
    sample_counter: usize,

    /// current bit sequence
    bits: u64,
}

impl PhaseDecoder {
    pub fn new() -> Self {
        Self {
            raw_samples: [Complex::ZERO; NUM_RAW_SAMPLES],
            raw_index: Wrapping(0),
            current_groups: [Complex::ZERO; NUM_GROUPS],
            next_groups: [Complex::ZERO; NUM_GROUPS],
            sample_counter: 0,
            bits: 0,
            // group_scores: [0.0; GROUPS_PER_SECOND],
        }
    }

    pub fn sample(&mut self, phase_offset: Complex<f64>) {
        self.raw_samples[self.raw_index.0 % NUM_RAW_SAMPLES] = phase_offset;
        self.raw_index += 1;

        self.current_groups[self.sample_counter / GROUP_SIZE] += phase_offset;
        if let Some(counter) = self.sample_counter.checked_sub(77500) {
            self.next_groups[counter / GROUP_SIZE] += phase_offset;
        }

        self.sample_counter += 1;
        if self.sample_counter != NUM_GROUPS * GROUP_SIZE {
            return;
        }

        let mut min = f64::INFINITY;
        let mut index = 0;
        // let mut bit = false;

        for i in 0..GROUPS_PER_SECOND {
            let mut one = 0.0;
            let mut zero = 0.0;

            for (group, bit) in self.current_groups[i..i + 1024]
                .chunks_exact(2)
                .zip(pzf::PZF)
            {
                let sum = group.iter().sum::<Complex<_>>();

                let process = |phase_shift: Complex<f64>| {
                    let diff = (sum * phase_shift).arg();
                    diff * diff
                };

                one += process(pzf::phase_shift_one_bit(bit));
                zero += process(pzf::phase_shift_zero_bit(bit));
            }

            if one < min {
                min = one;
                index = i;
                // bit = true;
            }

            if zero < min {
                min = zero;
                index = i;
                // bit = false;
            }
        }

        // min = min.sqrt().to_degrees();
        index *= 60;

        // println!("{}: {index} {bit} {min}", self.sample_counter);

        let first_index = index;
        let raw_offset = NUM_GROUPS * GROUP_SIZE - index;
        const SEARCH_RANGE: usize = GROUP_SIZE;
        let range = raw_offset - SEARCH_RANGE..=raw_offset + SEARCH_RANGE;

        let mut min = f64::INFINITY;
        let mut index = 0;
        let mut bit = false;

        for i in range {
            let mut one = 0.0;
            let mut zero = 0.0;

            for (j, bit) in pzf::PZF.into_iter().enumerate() {
                let mut sum = Complex::<f64>::ZERO;
                for k in 0..120 {
                    sum += self.raw_samples[(self.raw_index - Wrapping(i)
                        + Wrapping(j * 120)
                        + Wrapping(k))
                    .0 % NUM_RAW_SAMPLES];
                }

                let process = |phase_shift: Complex<f64>| {
                    let diff = (sum * phase_shift).arg();
                    diff * diff
                };

                one += process(pzf::phase_shift_one_bit(bit));
                zero += process(pzf::phase_shift_zero_bit(bit));
            }

            if one < min {
                min = one;
                index = i;
                bit = true;
            }

            if zero < min {
                min = zero;
                index = i;
                bit = false;
            }
        }

        min = min.sqrt().to_degrees();
        index = NUM_GROUPS * GROUP_SIZE - index;

        println!(
            "{}: {index} {} {bit} {min}",
            self.sample_counter,
            index as isize - first_index as isize,
        );
        // println!();

        self.sample_counter -= 77500;
        self.current_groups = mem::replace(&mut self.next_groups, [Complex::ZERO; NUM_GROUPS]);

        self.bits >>= 1;
        self.bits |= (bit as u64) << 59;

        println!("{:060b}", self.bits);

        const START: u64 = (1 << 10) - 1;
        if self.bits & START != START {
            return;
        }

        println!("MINUTE DETECTED:");
        println!("{:060b}", self.bits);

        let bits = self.bits;

        let minute = ((bits >> 21) & 0xF) + ((bits >> 25) & 7) * 10;
        let hour = ((bits >> 29) & 0xF) + ((bits >> 33) & 3) * 10;

        let day = ((bits >> 36) & 0xF) + ((bits >> 40) & 2) * 10;
        let weekday = (bits >> 42) & 7;
        let month = ((bits >> 45) & 0xF) + ((bits >> 49) & 1) * 10;
        let year = ((bits >> 50) & 0xF) + ((bits >> 54) & 0xF) * 10;

        const WEEKDAYS: [&str; 7] = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"];
        println!(
            "{hour:02}:{minute:02} {} {day:02}-{month:02}-{year:02}",
            WEEKDAYS
                .get(weekday.wrapping_sub(1) as usize)
                .copied()
                .unwrap_or("---"),
        );
        println!("===============");
    }
}
