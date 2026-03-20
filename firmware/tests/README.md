# Firmware Test Sketches

Each `test_*` directory is a standalone Arduino sketch used to bring up or isolate one subsystem. These sketches are not the production firmware; many are intentionally narrow and may use simpler flows than the main firmware.

## Layout

- `test_scheduler/`: scheduler and timing bring-up
- `test_uart_tlv/`: TLV communication path checks
- `test_uart_loop_drain/`: UART loop-drain experiments
- `test_uart_rx_10khz/`: older UART timing experiment
- `test_encoder/`: encoder counting and direction
- `test_dc_motor_pwm/`: open-loop DC motor drive
- `test_dc_motor_pid/`: closed-loop DC motor control experiments
- `test_current_sensing/`: current feedback
- `test_servo/`: PCA9685 servo control
- `test_stepper/`: stepper driver bring-up
- `test_user_io/`: button / LED / NeoPixel bring-up
- `test_voltage/`: rail and battery sensing
- `test_eeprom/`: persistent storage checks
- `test_i2c_scanner/`: bus scan utility

## Shared Source Tree

Most tests include a `src` symlink pointing back to `firmware/arduino/src/` so they can reuse the current production drivers and modules.

## Build

From the repository root:

```bash
arduino-cli compile --fqbn arduino:avr:mega firmware/tests/test_scheduler
```

Replace `test_scheduler` with the sketch you want to build.

## Usage Notes

- Treat the production firmware documentation in [`../README.md`](../README.md) and [`../docs/`](../docs/) as the source of truth for the shipped architecture.
- Treat each test sketch as a targeted bring-up tool; read the sketch header and source comments for its exact assumptions.
- Some older tests remain useful as experiments even if they do not mirror the exact current production runtime structure.
