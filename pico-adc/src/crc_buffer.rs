//! Single-buffered or peripheral-peripheral DMA Transfers with CRC support

use core::{
    mem,
    sync::atomic::{Ordering, compiler_fence},
};

use rp2040_hal::{
    dma::{Channels, DMAExt, Pace, ReadTarget, SingleChannel, WriteTarget},
    pac::{DMA, RESETS},
};

pub struct DmaSniff(());

impl DmaSniff {
    pub fn new(dma: DMA, resets: &mut RESETS) -> (Self, Channels) {
        let channels = dma.split(resets);
        let mut this = Self(());
        this.ctrl().reset();
        (this, channels)
    }

    fn ctrl(&mut self) -> &rp2040_hal::pac::dma::SNIFF_CTRL {
        // Safety: We have exclusive control over the sniffer hardware.
        unsafe { &*rp2040_hal::pac::DMA::ptr() }.sniff_ctrl()
    }

    fn data(&mut self) -> &rp2040_hal::pac::dma::SNIFF_DATA {
        // Safety: We have exclusive control over the sniffer hardware.
        unsafe { &*rp2040_hal::pac::DMA::ptr() }.sniff_data()
    }

    fn configure(mut self, ch: u8) -> RunningDmaSniff {
        self.ctrl().write(|w| {
            w.en().set_bit();
            unsafe { w.dmach().bits(ch) };
            w.calc().crc32r();
            w
        });

        self.write(u32::MAX);

        RunningDmaSniff(self)
    }

    fn disable(&mut self) {
        self.ctrl().write(|w| w.en().clear_bit());
    }

    fn write(&mut self, seed: u32) {
        self.data().write(|w| unsafe { w.bits(seed) });
    }

    fn read(&mut self) -> u32 {
        !self.data().read().bits().reverse_bits()
    }
}

struct RunningDmaSniff(DmaSniff);

impl RunningDmaSniff {
    fn abort(mut self) -> DmaSniff {
        self.0.disable();
        self.0
    }

    fn finish(mut self) -> (DmaSniff, u32) {
        self.0.disable();
        let value = self.0.read();
        (self.0, value)
    }
}

/// Configuration for single-buffered DMA transfer
pub struct Config<CH: SingleChannel, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
    sniff: DmaSniff,
    pace: Pace,
    bswap: bool,
}

impl<CH, FROM, TO, WORD> Config<CH, FROM, TO>
where
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Create a new configuration for single-buffered DMA transfer
    pub fn new(ch: CH, from: FROM, to: TO, sniff: DmaSniff) -> Config<CH, FROM, TO> {
        Config {
            ch,
            from,
            to,
            sniff,
            pace: Pace::PreferSource,
            bswap: false,
        }
    }

    /// Sets the (preferred) pace for the DMA transfers.
    ///
    /// Usually, the code will automatically configure the correct pace, but
    /// peripheral-to-peripheral transfers require the user to manually select whether the source
    /// or the sink shall be queried for the pace signal.
    #[expect(dead_code)]
    pub fn pace(&mut self, pace: Pace) {
        self.pace = pace;
    }

    /// Enable/disable byteswapping for the DMA transfers, default value is false.
    ///
    /// For byte data, this has no effect. For halfword data, the two bytes of
    /// each halfword are swapped. For word data, the four bytes of each word
    /// are swapped to reverse order.
    ///
    /// This is a convenient way to change the (half-)words' byte endianness on the fly.
    #[expect(dead_code)]
    pub fn bswap(&mut self, bswap: bool) {
        self.bswap = bswap;
    }

    /// Start the DMA transfer
    pub fn start(mut self) -> Transfer<CH, FROM, TO> {
        // TODO: Do we want to call any callbacks to configure source/sink?

        // Make sure that memory contents reflect what the user intended.
        // TODO: How much of the following is necessary?
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        let sniff = self.sniff.configure(self.ch.id());

        // Configure the DMA channel and start it.
        self.ch.config(
            &self.from,
            &mut self.to,
            self.pace,
            self.bswap,
            true,
            None,
            true,
        );

        Transfer {
            ch: self.ch,
            from: self.from,
            to: self.to,
            sniff,
        }
    }
}

// TODO: Drop for most of these structs
/// Instance of a single-buffered DMA transfer
pub struct Transfer<CH: SingleChannel, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
    sniff: RunningDmaSniff,
}

impl<CH, FROM, TO, WORD> Transfer<CH, FROM, TO>
where
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Check if an interrupt is pending for this channel and clear the corresponding pending bit
    #[expect(dead_code)]
    pub fn check_irq0(&mut self) -> bool {
        self.ch.check_irq0()
    }

    /// Check if an interrupt is pending for this channel and clear the corresponding pending bit
    #[expect(dead_code)]
    pub fn check_irq1(&mut self) -> bool {
        self.ch.check_irq1()
    }

    /// Check if the transfer has completed.
    pub fn is_done(&self) -> bool {
        !self.ch.ch().ch_ctrl_trig().read().busy().bit_is_set()
    }

    /// Block until the transfer is complete, returning the channel and targets
    pub fn wait(self) -> (CH, FROM, TO, DmaSniff, u32) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        let (sniff, value) = self.sniff.finish();

        (self.ch, self.from, self.to, sniff, value)
    }

    /// Aborts the current transfer, returning the channel and targets
    #[expect(dead_code)]
    pub fn abort(mut self) -> (CH, FROM, TO, DmaSniff) {
        let irq0_was_enabled = self.ch.is_enabled_irq0();
        let irq1_was_enabled = self.ch.is_enabled_irq1();
        self.ch.disable_irq0();
        self.ch.disable_irq1();

        let chan_abort = unsafe { &*rp2040_hal::pac::DMA::ptr() }.chan_abort();
        let abort_mask = (1 << self.ch.id()) as u16;

        chan_abort.write(|w| unsafe { w.chan_abort().bits(abort_mask) });

        while chan_abort.read().chan_abort().bits() != 0 {}

        while !self.is_done() {}

        self.ch.check_irq0();
        self.ch.check_irq1();

        if irq0_was_enabled {
            self.ch.enable_irq0();
        }

        if irq1_was_enabled {
            self.ch.enable_irq1();
        }

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        (self.ch, self.from, self.to, self.sniff.abort())
    }
}

// RP2040's DMA engine only works with certain word sizes. Make sure that other
// word sizes will fail to compile.
struct IsValidWordSize<WORD> {
    w: core::marker::PhantomData<WORD>,
}

impl<WORD> IsValidWordSize<WORD> {
    const OK: usize = {
        match mem::size_of::<WORD>() {
            1 | 2 | 4 => 0, // ok
            _ => panic!("Unsupported DMA word size"),
        }
    };
}

trait ChannelConfig {
    #[expect(clippy::too_many_arguments)]
    fn config<WORD, FROM, TO>(
        &mut self,
        from: &FROM,
        to: &mut TO,
        pace: Pace,
        bswap: bool,
        sniff: bool,
        chain_to: Option<u8>,
        start: bool,
    ) where
        FROM: ReadTarget<ReceivedWord = WORD>,
        TO: WriteTarget<TransmittedWord = WORD>;

    #[expect(dead_code)]
    fn set_chain_to_enabled<CH: SingleChannel>(&mut self, other: &mut CH);

    #[expect(dead_code)]
    fn start(&mut self);

    #[expect(dead_code)]
    fn start_both<CH: SingleChannel>(&mut self, other: &mut CH);
}

impl<CH: SingleChannel> ChannelConfig for CH {
    fn config<WORD, FROM, TO>(
        &mut self,
        from: &FROM,
        to: &mut TO,
        pace: Pace,
        bswap: bool,
        sniff: bool,
        chain_to: Option<u8>,
        start: bool,
    ) where
        FROM: ReadTarget<ReceivedWord = WORD>,
        TO: WriteTarget<TransmittedWord = WORD>,
    {
        // Configure the DMA channel.
        let _ = IsValidWordSize::<WORD>::OK;

        let (src, src_count) = from.rx_address_count();
        let src_incr = from.rx_increment();
        let (dest, dest_count) = to.tx_address_count();
        let dest_incr = to.tx_increment();
        const TREQ_UNPACED: u8 = 0x3f;
        let treq = match pace {
            Pace::PreferSource => FROM::rx_treq().or_else(TO::tx_treq).unwrap_or(TREQ_UNPACED),
            Pace::PreferSink => TO::tx_treq().or_else(FROM::rx_treq).unwrap_or(TREQ_UNPACED),
        };
        let len = u32::min(src_count, dest_count);
        self.ch().ch_al1_ctrl().write(|w| unsafe {
            w.data_size().bits(mem::size_of::<WORD>() as u8 >> 1);
            w.incr_read().bit(src_incr);
            w.incr_write().bit(dest_incr);
            w.treq_sel().bits(treq);
            w.bswap().bit(bswap);
            w.sniff_en().bit(sniff);
            w.chain_to().bits(chain_to.unwrap_or_else(|| self.id()));
            w.en().bit(true);
            w
        });
        self.ch().ch_read_addr().write(|w| unsafe { w.bits(src) });
        self.ch().ch_trans_count().write(|w| unsafe { w.bits(len) });
        if start {
            self.ch()
                .ch_al2_write_addr_trig()
                .write(|w| unsafe { w.bits(dest) });
        } else {
            self.ch().ch_write_addr().write(|w| unsafe { w.bits(dest) });
        }
    }

    fn set_chain_to_enabled<CH2: SingleChannel>(&mut self, other: &mut CH2) {
        // We temporarily pause the channel when setting CHAIN_TO, to prevent any race condition
        // that could occur, as we need to check afterwards whether the channel was successfully
        // chained to this channel or whether this channel was already completed. If we did not
        // pause this channel, we could get into a situation where both channels completed in quick
        // succession, yet we did not notice, as the situation is not distinguishable from one
        // where the second channel was not started at all.

        self.ch().ch_al1_ctrl().modify(|_, w| unsafe {
            w.chain_to().bits(other.id());
            w.en().clear_bit();
            w
        });
        if self.ch().ch_al1_ctrl().read().busy().bit_is_set() {
            // This channel is still active, so just continue.
            self.ch().ch_al1_ctrl().modify(|_, w| w.en().set_bit());
        } else {
            // This channel has already finished, so just start the other channel directly.
            other.start();
        }
    }

    fn start(&mut self) {
        // Safety: The write does not interfere with any other writes, it only affects this
        // channel.
        unsafe { &*rp2040_hal::pac::DMA::ptr() }
            .multi_chan_trigger()
            .write(|w| unsafe { w.bits(1 << self.id()) });
    }

    fn start_both<CH2: SingleChannel>(&mut self, other: &mut CH2) {
        // Safety: The write does not interfere with any other writes, it only affects this
        // channel and other (which we have an exclusive borrow of).
        let channel_flags = 1 << self.id() | 1 << other.id();
        unsafe { &*rp2040_hal::pac::DMA::ptr() }
            .multi_chan_trigger()
            .write(|w| unsafe { w.bits(channel_flags) });
    }
}
