# Rust SDP800 Driver

![No Std][no-std-badge]

This is a platform agnostic Rust driver for the Sensirion SDP800 differential pressure sensor.
using the [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.

## The Device

The Sensirion SDP800 is a differential pressure sensor. It has an I²C interface.

## Status

- [x] Get product identifier
- [x] Get differential pressure (triggered sample)
- [x] Get mass flow (triggered sample)
- [x] Stop sampling continuous measurement
- [x] Continuously sample differential pressure
- [x] Continuously sample mass flow
- [x] Value conversion
- [x] Type-state based initialization and mode selection (triggered and continuous mode, idle mode)
- [ ] Polling and timeouts for states instead of delay-based implementation
- [ ] Soft reset
- [ ] Sample pressure/mass flow only in sampling state, use saved values for conversion factor and temperature

## License

Licensed under

 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

<!-- Badges -->
[no-std-badge]: https://img.shields.io/badge/no__std-yes-blue
