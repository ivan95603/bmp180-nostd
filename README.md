# Rust Bosch BMP180 Driver [no-std, embedded-hal]

<!-- [![crates.io](https://img.shields.io/crates/v/bmp180-no-std.svg)](https://crates.io/crates/bmp180-no-std)
[![Docs](https://docs.rs/bmp180-no-std/badge.svg)](https://docs.rs/bmp180-no-std) -->

This is a platform agnostic Rust driver for the BMP180 pressure sensor using the [`embedded-hal`] traits.

This driver allows you to:
- Measure the pressure in pascals. See: `pressure_pa()`.
- Measure the pressure in hectopascals. See: `pressure_hpa()`.
- Measure the pressure in kilopascals. See: `pressure_kpa()`.
- Measure the temperature in celsius. See: `temperature_celsius()`.

<!-- TODO
[Introductory blog post]()
-->

The BMP180 is the function compatible successor of the BMP085, a new generation of high
precision digital pressure sensors for consumer applications.

The ultra-low power, low voltage electronics of the BMP180 is optimized for use in mobile phones,
PDAs, GPS navigation devices and outdoor equipment. With a low altitude noise of merely 0.25m at
fast conversion time, the BMP180 offers superior performance. The I2C interface allows for easy
system integration with a microcontroller.

The BMP180 is based on piezo-resistive technology for EMC robustness, high accuracy and linearity as
well as long term stability.

Documentation:
- Datasheets: [BMP180](https://media.digikey.com/pdf/Data%20Sheets/Bosch/BMP180.pdf)

## Usage

To use this driver, import this crate and an `embedded_hal` implementation,
then instantiate the appropriate device.

<!-- TODO: Please find additional examples using hardware in this repository: [driver-examples]

[driver-examples]: https://github.com/ivan95603/driver-examples -->

```rust
use linux_embedded_hal::I2cdev;
use mlx9061x::{Mlx9061x, SlaveAddr};

fn main() {
    // Create a delay abstraction based on SysTick
    let mut delayObj = hal::delay::Delay::new(cp.SYST, &clocks);

    let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &clocks); //100 kHz I2C Bus speed
    let mut sensor = bmp180::BMP180BarometerThermometer::new(i2c, boxy , bmp180::BMP180PressureMode::BMP180Standard);
    loop {
            let pressure_in_hpa: f32 = barometer.pressure_hpa();
            let pressure_temp_celsius: f32 = barometer.temperature_celsius();
    }
}
```

## Support

For questions, issues, feature requests, and other changes, please file an
[issue in the github project](https://github.com/ivan95.603/bmp180-no-std/issues).

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](http://www.apache.org/licenses/LICENSE-2.0))
 * MIT license ([LICENSE-MIT](http://opensource.org/licenses/MIT))
 * BSD-3-Clause license ([LICENSE-BSD-3-Clause](http://opensource.org/licenses/BSD-3-Clause))

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be triple licensed as above, without any additional terms or conditions.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal