# DCF77

> Decode the pseudo-random phase noise that is modulated onto the carrier of DCF77. Without relying on the amplitude modulation.

## Hardware setup

- ferrite rod antenna + tuning capacitor (tuned to 77.5 kHz)
- quad op-amp
  - virtual ground from voltage divider (0.825 V)
  - antenna signal directly connected to amplifier circuit
  - (... maybe 1 or 2 additional amplifier stages)
- Raspberry Pi Pico (used as ADC)
- Raspberry Pi Zero 2 (or other model)

## [Pico ADC](pico-adc)

The raspberry pi pico is used as an ADC to sample the received signal directly (after amplification).
The antenna is connected to the cirtual ground (3.3V / 4 = 0,825 V).
Therefore the antenna signal swings arround that voltage (plus any DC-offset).

The pico is confugred with a 44.64 MHz ADC-clock. This results in 44.64 MHz / 96 (cycles per sample) = 465 ksps (6 samples per DCF77 signal period).
Chunks of 6 samples (one period) are combined into their i and q components.

31 periods are sent together in a message via UART at 3_000_000 baud rate.
This results in 2500 messages per second.

## Raspberry Pi

The big raspberry pi (e.g. zero 2) does the heavy lifting and uses a lot of floating point math (f64).

First the phase of the signal is decoded and filtered to estimate the frequency difference between the local clock and the DCF77 source.
Then the phase corrected samples are fed into a phase signal decoder that compares the individual samples to the expected pseudo-random noise pattern (PZF = "Pseudozufallsfolge").
This detects at which period of the signal the PZF started (200ms offset from second start) and if the transmitted signal was a 0 or 1 bit.
The bit pattern of the PZF starts with 10 times a 1 bit and then continues with the regular bit pattern (from the AM signal).

## License

Not yet. It will be licensed under either "GPL" or "MIT OR Apache-2.0". I haven't decided yet.

Keep this in mind if you want to contribute to the project.
