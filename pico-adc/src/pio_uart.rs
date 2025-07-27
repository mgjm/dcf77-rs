use core::fmt;

use rp2040_hal::{
    dma::Byte,
    gpio::{Pin, PinId, PullNone},
    pio::{
        Buffers, PIO, PIOBuilder, PIOExt, PinDir, PinState, ShiftDirection, StateMachineIndex, Tx,
        UninitStateMachine,
    },
};

pub struct UartTx<P: PIOExt, SM: StateMachineIndex> {
    tx: Tx<(P, SM), Byte>,
}

impl<P, SM> UartTx<P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    pub fn new(
        pio: &mut PIO<P>,
        sm: UninitStateMachine<(P, SM)>,
        pin: Pin<impl PinId, P::PinFunction, PullNone>,
    ) -> Self {
        let prog = pio_proc::pio_asm!(
            ".side_set 1 opt",
            //
            "pull      side 1  [1]",
            "set x, 7  side 0  [1]",
            "bitloop:",
            "  out pins, 1",
            "  jmp x-- bitloop",
        );

        let prog = pio.install(&prog.program).unwrap();

        // const BAUD_RATE: u64 = 115_200;
        // const BAUD_RATE: u64 = 1_000_000;
        const BAUD_RATE: u64 = 3_000_000;

        const CLOCK_DIV: u64 = 125_000_000 * 256 / 2 / BAUD_RATE;

        let (mut sm, _, tx) = PIOBuilder::from_installed_program(prog)
            .clock_divisor_fixed_point((CLOCK_DIV >> 8) as u16, CLOCK_DIV as u8)
            .buffers(Buffers::OnlyTx)
            .out_shift_direction(ShiftDirection::Right)
            .out_pins(pin.id().num, 1)
            .side_set_pin_base(pin.id().num)
            .build(sm);

        sm.set_pins([(pin.id().num, PinState::High)]);
        sm.set_pindirs([(pin.id().num, PinDir::Output)]);

        let sm = sm.start();
        let tx = tx.transfer_size(Byte);

        let _ = sm;

        Self { tx }
    }

    pub fn write_blocking(&mut self, byte: u8) {
        while !self.tx.write(byte as u32) {}
    }

    #[expect(dead_code)]
    pub fn write_fmt(&mut self, args: fmt::Arguments) {
        fmt::Write::write_fmt(self, args).unwrap();
    }

    pub fn into_write_target(self) -> Tx<(P, SM), Byte> {
        self.tx
    }
}

impl<P, SM> fmt::Write for UartTx<P, SM>
where
    P: PIOExt,
    SM: StateMachineIndex,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for &b in s.as_bytes() {
            self.write_blocking(b);
        }
        Ok(())
    }
}
