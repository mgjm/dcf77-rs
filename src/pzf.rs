use nalgebra::Complex;

pub const PZF: [bool; 512] = {
    let mut out = [false; 512];
    let mut i = 0;

    let mut bits = 1;

    while i < 512 {
        let value = ((bits) & 16 != 0) ^ (bits & 256 != 0);

        bits <<= 1;
        bits |= value as u16;

        out[i] = value;
        i += 1;
    }

    out
};

const SHIFT_FORWARD: Complex<f64> = Complex::new(0.9631625667976582, 0.2689198206152657);
const SHIFT_BACKWARDS: Complex<f64> = Complex::new(0.9631625667976582, -0.2689198206152657);

pub fn phase_shift_one_bit(bit: bool) -> Complex<f64> {
    if bit { SHIFT_FORWARD } else { SHIFT_BACKWARDS }
}

pub fn phase_shift_zero_bit(bit: bool) -> Complex<f64> {
    if bit { SHIFT_BACKWARDS } else { SHIFT_FORWARD }
}

#[test]
fn test() {
    pub const BITS: [u64; 512 / 64] = [
        0b0000100011000010011100101010110000110111101001101110010001010000,
        0b1010110100111111011001001001011011111100100110101001100110000000,
        0b1100011001010001101001011111110100010110001110101100101100111100,
        0b0111110111010000011010110110111011000001011010111110101010100000,
        0b0101001010111100101110111000000111001110100100111101011101010001,
        0b0010000110011100001011110110110011010000111011110000111111111000,
        0b0011110111110001011100110010000010010100111011010001111001111100,
        0b1101100010101001000111000110110101011100010011000100010000000010,
    ];

    for (i, bit) in PZF.into_iter().enumerate() {
        assert_eq!(bit, BITS[i / 64] & (1 << (63 - i % 64)) != 0, "chip {i}");
    }
}

#[test]
fn shift() {
    let forward = Complex::from_polar(1.0, f64::to_radians(15.6));
    assert_eq!(SHIFT_FORWARD, forward);

    let backwards = Complex::from_polar(1.0, f64::to_radians(-15.6));
    assert_eq!(SHIFT_BACKWARDS, backwards);
}
