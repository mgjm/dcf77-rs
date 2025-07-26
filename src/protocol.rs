use std::{io::BufRead, mem, ops::Deref};

use anyhow::{Context, Result};

pub struct MessageDecoder<R> {
    reader: R,
    prev_message: Vec<u8>,
    prev_crc: u32,
    message: Vec<u8>,
}

impl<R> MessageDecoder<R>
where
    R: BufRead,
{
    pub fn new(reader: R) -> Self {
        Self {
            reader,
            prev_message: Vec::new(),
            prev_crc: 0,
            message: Vec::new(),
        }
    }

    pub fn read(&mut self) -> Result<&[u8]> {
        loop {
            self.message.clear();
            self.reader
                .read_until(0, &mut self.message)
                .context("read from device")?;

            match self.try_after_read() {
                Ok(()) => break Ok(&self.message[5..]),
                Err(err) => eprintln!("decode message: {err:#?}"),
            }
        }
    }

    fn try_after_read(&mut self) -> Result<()> {
        anyhow::ensure!(self.message.len() >= 2);

        let message_crc = crc32fast::hash(&self.message);
        self.message.pop();

        {
            let mut tail = &mut *self.message;
            while let Some(&pos) = tail.first() {
                tail[0] = 0;
                tail = tail
                    .get_mut(pos as usize..)
                    .context("invalid cobs encoding")?;
            }
        }

        let crc_field = read_u32(&mut &self.message[1..])?;

        mem::swap(&mut self.prev_message, &mut self.message);
        let prev_crc = mem::replace(&mut self.prev_crc, message_crc);

        anyhow::ensure!(prev_crc == crc_field, "CRC does not match");

        Ok(())
    }
}

pub struct Message {
    header: MessageHeader,
    pub samples: [(u16, u16); 31],
}

impl Message {
    pub fn read(mut data: &[u8]) -> Result<Self> {
        let data = &mut data;
        let header = MessageHeader::read(data)?;
        let mut samples = [(0, 0); 31];
        for sample in &mut samples {
            *sample = read_u12x2(data)?;
        }
        anyhow::ensure!(data.is_empty(), "message too large: {} bytes", data.len());
        Ok(Self { header, samples })
    }
}

impl Deref for Message {
    type Target = MessageHeader;

    fn deref(&self) -> &Self::Target {
        &self.header
    }
}

#[expect(dead_code)]
pub struct MessageHeader {
    /// 20 bit seconds counter + 12 bit subsecond counter
    ///
    /// subsecond = 0..2500
    /// `(seconds << 12) | subsecond`
    pub message_id: u32,

    /// current absolute raw value (after low-pass filter)
    ///
    /// 12.4 fixed point
    pub average: u16,

    /// unfiltered amplitude in this period
    /// (sum of absolute difference to average per sample)
    ///
    /// top 16-bit of sum (sum >> 4)
    pub amplitude: u16,

    /// minimum and maximum raw value during this period
    pub min_max: (u16, u16),

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

impl MessageHeader {
    fn read(data: &mut &[u8]) -> Result<Self> {
        let message_id = read_u32(data)?;
        let average = read_u16(data)?;
        let amplitude = read_u16(data)?;
        let min_max = read_u12x2(data)?;
        let flags = read_u8(data).map(Flag::from_bits_retain)?;
        let sample_shift = read_u8(data)?;
        Ok(Self {
            message_id,
            average,
            amplitude,
            min_max,
            flags,
            sample_shift,
        })
    }
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

fn read_array<const N: usize>(data: &mut &[u8]) -> Result<[u8; N]> {
    let value;
    (value, *data) = data.split_first_chunk().context("message too short")?;
    Ok(*value)
}

fn read_u32(data: &mut &[u8]) -> Result<u32> {
    read_array(data).map(u32::from_le_bytes)
}
fn read_u16(data: &mut &[u8]) -> Result<u16> {
    read_array(data).map(u16::from_le_bytes)
}
fn read_u8(data: &mut &[u8]) -> Result<u8> {
    read_array(data).map(u8::from_le_bytes)
}
fn read_u12x2(data: &mut &[u8]) -> Result<(u16, u16)> {
    let mut buf = [0; 4];
    buf[..3].copy_from_slice(&read_array::<3>(data)?);
    let value = u32::from_le_bytes(buf);
    Ok(((value >> 12) as u16, (value & 0xFFF) as u16))
}
