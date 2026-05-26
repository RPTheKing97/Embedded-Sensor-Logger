# Project Update Summary

Integrated updates:

- STM32 NUCLEO-F401RE firmware for MPU6050 + BMP280 over I2C1.
- TIM2 interrupt-driven sampling at 100 Hz.
- UART telemetry over USART2/ST-LINK VCP at 10 Hz.
- Sensor disconnect fault detection using validity mask `V`.
- Retry/error counters included in telemetry: `RET`, `ERR`.
- 8-sample moving average filter on acceleration axes.
- Python serial dashboard for live plotting.
- README updated with wiring, telemetry format, dashboard steps, and demo plan.
- README schematic image is stored as `docs/hardware_schematic.png`.

Local sanity checks performed:
- Firmware source reviewed locally; run a CubeIDE or ARM GCC build before flashing.
- Python dashboard syntax checked with bundled Python.

Note:
- A full firmware build requires the STM32Cube HAL project, startup file, linker script, and generated project metadata, which are not included here.
