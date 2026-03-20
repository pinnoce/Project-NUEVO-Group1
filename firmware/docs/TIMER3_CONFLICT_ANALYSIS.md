# Timer3 Conflict Analysis (Historical)

This note is kept as historical context for a board-level design issue that affected the firmware during Rev. B bring-up.

## What the conflict was

On Rev. B, `LED_RED` moved to pin `5`, which is `OC3A` on the ATmega2560.

At the same time, Timer3 is the timer used for stepper pulse timing. In the older design, Timer3 was configured in a way that made `OCR3A` unavailable for independent PWM output, so:

- stepper timing wanted Timer3
- red-LED PWM also wanted Timer3

That created a real hardware/software ownership conflict.

## Current resolution

The production firmware no longer treats this as an active issue.

Current `v0.9.0` behavior:

- Timer3 remains the stepper timer
- the timer configuration keeps the relevant hardware PWM path available
- `LED_RED` uses direct OCR writes instead of `analogWrite()`
- the discrete LEDs are reserved for user/TLV control, not automatic system status
- the automatic system-state indicator is the NeoPixel

Relevant files:

- [`../arduino/src/ISRScheduler.cpp`](../arduino/src/ISRScheduler.cpp)
- [`../arduino/src/pins.h`](../arduino/src/pins.h)
- [`../arduino/src/modules/UserIO.cpp`](../arduino/src/modules/UserIO.cpp)

## Why this note still matters

It explains two current design rules that remain important:

1. Do not use `analogWrite()` on timer-owned pins whose timer mode matters to the runtime.
2. Treat timer ownership as part of the architecture, not as a local implementation detail.

## Current recommendation

If you need automatic status indication, use the NeoPixel path.

If you need user-controlled LED behavior, use the discrete LED APIs in `UserIO`.

If you need to revisit Timer3 ownership, review:

- [`architecture.md`](architecture.md)
- [`technical_notes.md`](technical_notes.md)
- [`pin_table_rev_B.md`](pin_table_rev_B.md)
