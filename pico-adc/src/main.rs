#![cfg_attr(target_os = "none", no_std)]
#![no_main]

use embedded_hal::{delay::DelayNs, digital::StatefulOutputPin};
use protocol::{Flag, Message, MessageHeader};
use rp2040_hal::{
    Adc, Clock, Sio, Timer, Watchdog,
    adc::AdcPin,
    clocks::{ClocksManager, GpioOutput0Clock, StoppableClock, ValidSrc},
    dma::double_buffer,
    fugit::{HertzU32, MillisDurationU64, RateExtU32},
    gpio::{FunctionClock, Pin, PinId, Pins, PullNone, bank0::Gpio21},
    pac::Peripherals,
    pio::{PIO, PIOBuilder, PIOExt, PinDir, StateMachineIndex, UninitStateMachine},
    pll, xosc,
};

mod crc_buffer;
mod pio_uart;
mod protocol;

#[expect(dead_code)]
#[cfg_attr(target_os = "none", panic_handler)]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = info;
    cortex_m::asm::udf();
}

#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub const XOSC_CRYSTAL_FREQ: HertzU32 = HertzU32::MHz(12);

#[expect(clippy::excessive_precision)]
mod float {
    pub const SIN_15: f64 = 0.25881904510252076;
    pub const SIN_45: f64 = 0.70710678118654752;
    pub const SIN_75: f64 = 0.96592582628906829;
}

/// number of bits for pre-calculated sin values
///
/// 32 bit signed integer (31 data bits available):
/// - 12 bit ADC samples
/// - 6 samples per period multiplied by sin is less then 4 = 2 bit
/// - 17 remaining bits for sin
const SIN_BITS: u32 = 31 - 12 - 2;

/// sin multiplication factor (for conversion to integer)
const SIN_FACTOR: f64 = (1 << SIN_BITS) as f64;

/// sin(15 deg)
const SIN_15: i32 = (float::SIN_15 * SIN_FACTOR + 0.5) as i32;

/// sin(45 deg)
const SIN_45: i32 = (float::SIN_45 * SIN_FACTOR + 0.5) as i32;

/// sin(75 deg)
const SIN_75: i32 = (float::SIN_75 * SIN_FACTOR + 0.5) as i32;

/// precalculated sin wave with 6 samples, starting at 45 deg
const SIN: [i32; 6] = [
    SIN_45,  // sin(45)
    SIN_75,  // sin(105) = sin(180 - 105) = sin(75)
    SIN_15,  // sin(165) = sin(180 - 165) = sin(15)
    -SIN_45, // sin(225) = sin(180 - 225) = sin(-45)
    -SIN_75, // sin(285) = sin(285 - 360) = sin(-75)
    -SIN_15, // sin(345) = sin(345 - 360) = sin(-15)
];

/// precalculated cos wave with 6 samples, starting at 45 deg
const COS: [i32; 6] = [
    SIN_45,  // cos(45)  = sin(90 - 45)   = sin(45)
    -SIN_15, // cos(105) = sin(90 - 105)  = sin(-15)
    -SIN_75, // cos(165) = sin(90 - 165)  = sin(-75)
    -SIN_45, // cos(225) = sin(225 - 270) = sin(-45)
    SIN_15,  // cos(285) = sin(285 - 270) = sin(15)
    SIN_75,  // cos(345) = sin(345 - 270) = sin(75)
];

const CHUNKS_PER_BUF: usize = 31;

#[rp2040_hal::entry]
fn main() -> ! {
    ////////////////////////////////////////////////////////////////////////////////
    // initalize peripherals
    // PLL_USB 44.64 MHz / 96 ADC sample-duration / 6 samples = 77500 kHz
    ////////////////////////////////////////////////////////////////////////////////

    let mut p = Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(p.WATCHDOG);

    let pll_usb;
    let clocks = {
        let xosc = xosc::setup_xosc_blocking(p.XOSC, XOSC_CRYSTAL_FREQ).unwrap();

        // Configure watchdog tick generation to tick over every microsecond
        watchdog.enable_tick_generation(XOSC_CRYSTAL_FREQ.to_MHz() as u8);

        let mut clocks = ClocksManager::new(p.CLOCKS);

        let pll_sys = pll::setup_pll_blocking(
            p.PLL_SYS,
            xosc.operating_frequency(),
            pll::common_configs::PLL_SYS_125MHZ,
            &mut clocks,
            &mut p.RESETS,
        )
        .unwrap();

        pll_usb = pll::setup_pll_blocking(
            p.PLL_USB,
            xosc.operating_frequency(),
            pll::PLLConfig {
                vco_freq: 1116.MHz(),
                refdiv: 1,
                post_div1: 5,
                post_div2: 5,
            },
            &mut clocks,
            &mut p.RESETS,
        )
        .unwrap();

        clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

        clocks
    };

    const ADC_FREQ: HertzU32 = HertzU32::Hz(77500 * 6 * 96);
    assert_eq!(clocks.adc_clock.freq(), ADC_FREQ);

    let mut timer = Timer::new(p.TIMER, &mut p.RESETS, &clocks);

    let sio = Sio::new(p.SIO);

    let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    ////////////////////////////////////////////////////////////////////////////////
    // logger (via PIO UART)
    ////////////////////////////////////////////////////////////////////////////////

    let (mut pio, sm0, sm1, ..) = p.PIO0.split(&mut p.RESETS);

    let uart = pio_uart::UartTx::new(&mut pio, sm0, pins.gpio28.reconfigure());

    ////////////////////////////////////////////////////////////////////////////////
    // status LED
    ////////////////////////////////////////////////////////////////////////////////

    let mut led = pins.gpio25.into_push_pull_output();

    ////////////////////////////////////////////////////////////////////////////////
    // CLK output
    ////////////////////////////////////////////////////////////////////////////////

    let enable_pps = pps_from_clk(
        clocks.gpio_output0_clock,
        &pll_usb,
        &mut timer,
        &mut pio,
        sm1,
        pins.gpio21.reconfigure(),
        pins.gpio20.reconfigure(),
        pins.gpio15.reconfigure(),
    );

    ////////////////////////////////////////////////////////////////////////////////
    // ADC
    ////////////////////////////////////////////////////////////////////////////////

    let mut adc = Adc::new(p.ADC, &mut p.RESETS);
    let mut ch = AdcPin::new(pins.gpio26).unwrap();
    let mut fifo = adc
        .build_fifo()
        .set_channel(&mut ch)
        .enable_dma()
        .start_paused();

    let (sniff, dma) = crc_buffer::DmaSniff::new(p.DMA, &mut p.RESETS);

    const BUF_SIZE: usize = 6 * CHUNKS_PER_BUF;

    #[allow(static_mut_refs)]
    let (buf0, buf1, message_buf0, message_buf1) = {
        type Buf = [u16; BUF_SIZE];
        static mut BUFS: (Buf, Buf, Message, Message) = (
            [0; BUF_SIZE],
            [0; BUF_SIZE],
            //
            Message::zeroed(),
            Message::zeroed(),
        );
        unsafe { &mut BUFS }
    };

    let mut adc_transfer =
        double_buffer::Config::new((dma.ch0, dma.ch1), fifo.dma_read_target(), buf0)
            .start()
            .write_next(buf1);

    ////////////////////////////////////////////////////////////////////////////////
    // UART DMA (with CRC)
    ////////////////////////////////////////////////////////////////////////////////

    let mut message_next = message_buf0;

    let mut uart_transfer: Result<crc_buffer::Transfer<_, _, _>, _> =
        Err((dma.ch2, message_buf1, uart.into_write_target(), sniff));

    ////////////////////////////////////////////////////////////////////////////////
    // prepare main loop
    ////////////////////////////////////////////////////////////////////////////////

    let mut blink = timer
        .get_counter()
        .checked_add_duration(MillisDurationU64::millis(300))
        .unwrap();

    // we add 12 bit values to the 32 bit avrg_sum, max 20 bits remaining
    //
    // cut-off frequencies and rise time (change from 0 to 4095):
    // bits | cut-off frequency | rise time
    // ---- | ----------------- | ---------
    //    1 |       322.313 kHz |   28.0 us
    //    2 |       133.772 kHz |   68.8 us
    //    3 |        62.092 kHz |    144 us
    //    4 |        30.010 kHz |    299 us
    //    5 |        14.763 kHz |    604 us
    //    6 |         7.323 kHz |   1.22 ms
    //    7 |         3.647 kHz |   2.44 ms
    //    8 |         1.820 kHz |   4.89 ms
    //    9 |           909  Hz |   9.78 ms
    //   10 |           454  Hz |   19.6 ms
    //   11 |           227  Hz |   39.2 ms
    //   12 |           114  Hz |   78.3 ms
    //   13 |          56.8  Hz |    157 ms
    //   14 |          28.4  Hz |    313 ms
    //   15 |          14.2  Hz |    627 ms
    //   16 |          7.10  Hz |   1.25  s
    //   17 |          3.55  Hz |   2.51  s
    //   18 |          1.77  Hz |   5.01  s
    //   19 |         0.887  Hz |   10.0  s
    //   20 |         0.443  Hz |   20.1  s
    //
    // n = samples per second = 77500 * 6
    // b = mixing factor = (1 << AVRG_BITS - 1) / (1 << AVRG_BITS)
    // w = n * ln(1/b)
    const AVRG_BITS: u32 = 16;

    let mut avrg_sum: u32 = 0;

    let mut seconds = 0;
    let mut subsecond = 0;
    let mut sticky_flags = Flag::empty();

    // the i and q data from the current buffer
    let mut iq_data = [(0, 0); CHUNKS_PER_BUF];

    // start ADC fifo and clock output at the same time
    fifo.resume();
    enable_pps();

    loop {
        ////////////////////////////////////////////////////////////////////////////////
        // main loop
        ////////////////////////////////////////////////////////////////////////////////
        let now = timer.get_counter();
        if blink <= now {
            led.toggle().unwrap();

            blink = blink
                .checked_add_duration(MillisDurationU64::millis(300))
                .unwrap();
        }

        ////////////////////////////////////////////////////////////////////////////////
        // calculate i and q data from the sample buffer
        ////////////////////////////////////////////////////////////////////////////////

        let (buf, adc_dma) = adc_transfer.wait();

        let mut amplitude = 0;
        let mut min_raw_value = 4095;
        let mut max_raw_value = 0;
        let mut max_value = 0;

        for (chunk, iq_data) in buf.chunks_exact(6).zip(&mut iq_data) {
            let chunk: &[u16; 6] = chunk.try_into().unwrap();

            let mut i_data = 0;
            let mut q_data = 0;

            let avrg = avrg_sum >> AVRG_BITS;
            avrg_sum -= avrg * 6;

            for i in 0..6 {
                let mut value = chunk[i];

                if value > 0xFFF {
                    value = avrg as u16;
                }

                if value < min_raw_value {
                    min_raw_value = value;
                }

                if value > max_raw_value {
                    max_raw_value = value;
                }

                avrg_sum += value as u32;

                let ampl = value as i32 - avrg as i32;

                amplitude += ampl.unsigned_abs();

                i_data += ampl * SIN[i];
                q_data += ampl * COS[i];
            }

            *iq_data = (i_data, q_data);
            max_value = max_value
                .max(i_data.unsigned_abs())
                .max(q_data.unsigned_abs());
        }

        adc_transfer = adc_dma.write_next(buf);

        ////////////////////////////////////////////////////////////////////////////////
        // generate a UART message from the samples
        ////////////////////////////////////////////////////////////////////////////////

        let shift = max_value
            .checked_ilog2()
            .unwrap_or_default()
            .saturating_sub(10);

        let overrun = fifo.is_over();

        if overrun {
            sticky_flags.insert(Flag::FIFO_OVERRUN_STICKY);
        }

        let message_id = seconds | subsecond;
        subsecond += 1;
        if subsecond >= 2500 {
            seconds += 0x1000;
            subsecond = 0;
        }

        let message = message_next.writer();

        let message = message.set_header(MessageHeader {
            message_id: message_id.into(),
            average: ((avrg_sum >> const { AVRG_BITS + 12 - u16::BITS }) as u16).into(),
            amplitude: ((amplitude >> 4) as u16).into(),
            min_max: (min_raw_value, max_raw_value).into(),
            flags: {
                let mut flags = sticky_flags.clone();
                if overrun {
                    flags.insert(Flag::FIFO_OVERRUN_NOW);
                }
                flags
            },
            sample_shift: shift as u8,
        });

        let message = message.set_samples(iq_data.map(|(i_data, q_data)| {
            let i_data = ((i_data >> shift).cast_unsigned() & 0xFFF) as u16;
            let q_data = ((q_data >> shift).cast_unsigned() & 0xFFF) as u16;

            (i_data, q_data).into()
        }));

        ////////////////////////////////////////////////////////////////////////////////
        // send the UART message
        ////////////////////////////////////////////////////////////////////////////////

        let uart_ch;
        let uart_tx;
        let sniff;

        let crc;
        match uart_transfer {
            Ok(transfer) => {
                (uart_ch, message_next, uart_tx, sniff, crc) = transfer.wait();
            }
            Err(no_transfer) => {
                (uart_ch, message_next, uart_tx, sniff) = no_transfer;
                crc = 0;
            }
        };

        let message = message.set_crc(crc).encode();

        uart_transfer = Ok(crc_buffer::Config::new(uart_ch, message, uart_tx, sniff).start());

        ////////////////////////////////////////////////////////////////////////////////
        // end of main loop
        ////////////////////////////////////////////////////////////////////////////////
    }
}

#[expect(clippy::too_many_arguments)]
fn pps_from_clk<P, SM>(
    mut clock: GpioOutput0Clock,
    clk_src: &impl ValidSrc<GpioOutput0Clock>,
    timer: &mut Timer,
    pio: &mut PIO<P>,
    sm: UninitStateMachine<(P, SM)>,
    clk_out: Pin<Gpio21, FunctionClock, PullNone>,
    clk_in: Pin<impl PinId, P::PinFunction, PullNone>,
    pps_out: Pin<impl PinId, P::PinFunction, PullNone>,
) -> impl FnOnce() + 'static
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    let _ = clk_out;

    // ensure the clock is divisible by 3
    assert!(clk_src.get_freq().to_Hz().is_multiple_of(3));

    clock.configure_clock(clk_src, 3.Hz()).unwrap();
    clock.disable();

    // wait for one clock cycle, after the disable (sync only at end of cycle)
    timer.delay_ms(400);

    let prog = pio_proc::pio_asm!(
        ".side_set 1 opt",
        //
        "wait 1 pin, 0 side 1",
        "wait 0 pin, 0",
        "wait 1 pin, 0 side 0",
        "wait 0 pin, 0",
        "wait 1 pin, 0",
        "wait 0 pin, 0",
    );

    let prog = pio.install(&prog.program).unwrap();

    let (mut sm, ..) = PIOBuilder::from_installed_program(prog)
        .in_pin_base(clk_in.id().num)
        .side_set_pin_base(pps_out.id().num)
        .build(sm);

    sm.set_pindirs([
        (clk_in.id().num, PinDir::Input),
        (pps_out.id().num, PinDir::Output),
    ]);

    sm.start();

    move || clock.enable()
}
