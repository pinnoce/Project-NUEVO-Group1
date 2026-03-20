# Rev. A to Rev. B Changes

This document summarizes the board-level pin remapping from Rev. A to Rev. B and how the current firmware treats those changes.

## Current status

- Rev. A remains the baseline reference board.
- Rev. B is supported by the current firmware pin map.
- The firmware abstracts most of the difference through [`../arduino/src/pins.h`](../arduino/src/pins.h).

For the full current pin maps, use:

- [`pin_table_rev_A.md`](pin_table_rev_A.md)
- [`pin_table_rev_B.md`](pin_table_rev_B.md)

## Main reasons Rev. B changed

Rev. B primarily reshuffled pins to improve encoder support and free better interrupt resources for the wheel motors.

Key goals:

- give M1 and M2 full hardware interrupt coverage for both encoder phases
- support M3 and M4 quadrature through PCINT
- preserve existing actuator and communication features

## High-level remap summary

### Wheel motors (M1 / M2)

- M1 and M2 gained full dual-channel interrupt-capable encoder wiring
- M2 direction and encoder pins were relocated to free the needed interrupt pins

### Manipulator motors (M3 / M4)

- M3 and M4 encoder channels moved onto PCINT-capable pins
- this preserves 4x support without consuming the highest-priority external interrupt pins

### LEDs / user pins

- `LED_RED` moved from pin `11` on Rev. A to pin `5` on Rev. B
- some former motor pins became user GPIOs on Rev. B

## Firmware implications

The current firmware handles the revision difference in three places:

1. `pins.h`
   - logical pin names
   - OCR register aliases
   - revision-specific comments and register bindings

2. `ISRScheduler`
   - external interrupt and PCINT attachment helpers

3. `UserIO`
   - direct PWM register writes for `LED_RED`

## Important current note about LEDs

Older notes may describe `LED_RED` as an automatic fault or battery indicator.

That is no longer the current firmware policy.

In `v0.9.0`:

- the discrete LEDs are reserved for user/TLV control
- the NeoPixel is the automatic system-state indicator

## When to use this document

Use this document when you need to understand:

- why the Rev. B pin map differs from Rev. A
- why M3/M4 use PCINT while M1/M2 use external interrupts
- why `LED_RED` is special on Rev. B

For current implementation details, use:

- [`architecture.md`](architecture.md)
- [`technical_notes.md`](technical_notes.md)
- [`pin_table_rev_B.md`](pin_table_rev_B.md)
