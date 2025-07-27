use core::{mem, ptr::NonNull};

use crate::CHUNKS_PER_BUF;

const _: [(); 1] = [(); mem::align_of::<Message>()];

pub struct Writer<const N: usize>(&'static mut Message);

impl Writer<0> {
    pub fn set_header(self, header: MessageHeader) -> Writer<1> {
        self.0.header = header;
        Writer(self.0)
    }
}

impl Writer<1> {
    pub fn set_samples(self, samples: [U12x2; CHUNKS_PER_BUF]) -> Writer<2> {
        self.0.samples = samples;
        Writer(self.0)
    }
}

impl Writer<2> {
    pub fn set_crc(self, crc: u32) -> Writer<3> {
        self.0.crc = crc.into();
        Writer(self.0)
    }
}

impl Writer<3> {
    pub fn encode(self) -> &'static mut Message {
        let (mut start, mut tail) = self.0.as_bytes().split_first_mut().unwrap();

        loop {
            let pos = tail.iter().position(|b| *b == 0).unwrap();
            *start = (pos + 1).try_into().unwrap();
            (start, tail) = tail[pos..].split_first_mut().unwrap();
            if tail.is_empty() {
                break;
            }
        }

        assert_eq!(*start, 0);

        self.0
    }
}

#[repr(C)]
pub struct Message {
    cobs_start: u8,

    crc: U32,

    header: MessageHeader,

    /// iq data samples
    ///
    /// encoded as 24 bit (two 12 bit numbers):
    /// i << 12 | q
    samples: [U12x2; CHUNKS_PER_BUF],

    cobs_zero: u8,
}

impl Message {
    pub const fn zeroed() -> Self {
        // Safety: this struct only contains byte values
        unsafe { mem::zeroed() }
    }

    const SIZE: usize = mem::size_of::<Self>();

    pub fn writer(&'static mut self) -> Writer<0> {
        Writer(self)
    }

    fn as_bytes(&mut self) -> &mut [u8; Self::SIZE] {
        unsafe { NonNull::from(self).cast().as_mut() }
    }
}

unsafe impl embedded_dma::ReadTarget for Message {
    type Word = u8;
}

#[repr(C)]
pub struct MessageHeader {
    /// 20 bit seconds counter + 12 bit subsecond counter
    ///
    /// subsecond = 0..2500
    /// `(seconds << 12) | subsecond`
    pub message_id: U32,

    /// current absolute raw value (after low-pass filter)
    ///
    /// 12.4 fixed point
    pub average: U16,

    /// unfiltered amplitude in this period
    /// (sum of absolute difference to average per sample)
    ///
    /// top 16-bit of sum (sum >> 4)
    pub amplitude: U16,

    /// minimum and maximum raw value during this period
    pub min_max: U12x2,

    /// status flags
    pub flags: Flag,

    /// number of bits that all samples in this message have been shifted by
    ///
    /// iq data starts as a full range i32 number, but only the most significant 12 bit get sent
    ///
    /// this specifies how many bits were truncated to not overflow the 12 bit range
    ///
    /// shift the received 12 bit number by this number of bits to restore the original i32
    pub sample_shift: u8,
}

macro_rules! bitflags2 {
    ({
        $vis:vis struct $ident:ident($ty:ty);

        $(
            $name:ident = $expr:expr;
        )*
    }) => {
        ::bitflags::bitflags! {
            #[derive(Clone)]
            #[repr(transparent)]
            $vis struct $ident: $ty {
                $(
                    const $name = $expr;
                )*
            }
        }
    };
}

bitflags2!({
    pub struct Flag(u8);

    FIFO_OVERRUN_NOW = 1 << 0;
    FIFO_OVERRUN_STICKY = 1 << 1;
});

#[repr(transparent)]
pub struct U32([u8; 4]);

impl From<u32> for U32 {
    fn from(value: u32) -> Self {
        Self(value.to_le_bytes())
    }
}

#[repr(transparent)]
pub struct U16([u8; 2]);

impl From<u16> for U16 {
    fn from(value: u16) -> Self {
        Self(value.to_le_bytes())
    }
}

#[repr(transparent)]
pub struct U12x2([u8; 3]);

impl From<(u16, u16)> for U12x2 {
    fn from((a, b): (u16, u16)) -> Self {
        let n = (a as u32) << 12 | b as u32;
        Self(n.to_le_bytes()[..3].try_into().unwrap())
    }
}
