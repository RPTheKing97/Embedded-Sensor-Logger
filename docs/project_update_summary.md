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
- README schematic image generated under `docs/embedded_sensor_logger_schematic.png`.

Local sanity checks performed:
- C source syntax checked with a lightweight STM32 HAL stub.
- Python dashboard syntax checked with `python -m py_compile`.

Note:
- The C syntax check is not a substitute for a CubeIDE build because the full STM32Cube HAL project, startup file, linker script, and generated project metadata are not included here.
