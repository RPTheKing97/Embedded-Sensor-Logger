# Embedded-Sensor-Logger

Firmware that samples sensor data with interrupt-driven timing and streams telemetry over UART with robust error handling.

## Features
- TIM2 interrupt-driven sampling @ 100 Hz (deterministic timing)
- Telemetry over USART2 (ST-LINK VCP) @ 10 Hz
- I2C sensor comms (MPU6050 + BMP280) with retry + timeout handling
- Lightweight DSP: 8-sample moving average on accel axes
- UART fault recovery (abort + deinit/init on error)
- Validity mask in telemetry to show which sensors succeeded

## Target Hardware
- Board: NUCLEO-F401RE (STM32F401RE)
- UART: USART2 via ST-LINK Virtual COM Port
- I2C: I2C1 (PB8 SCL, PB9 SDA)

## Sensors
- MPU6050 I2C address: 0x68 (default)
- BMP280 I2C address: 0x76 (some boards use 0x77; update SL_BMP280_ADDR7)

## CubeMX Configuration (Required)
Create a CubeMX project for NUCLEO-F401RE and enable:
1) USART2 (Asynchronous)
   - 115200 baud, 8N1
2) I2C1
   - 400 kHz (Fast Mode) recommended
3) TIM2 (Base Timer)
   - Enable Update Interrupt (NVIC)
   - Configure to 100 Hz update rate

TIM2 suggestion:
- Set TIM2 counter clock to 10 kHz
- Set period to 100 - 1 (=> 100 Hz)

Generate code, then copy these files into your project:
- Core/Inc/sensor_logger.h
- Core/Src/sensor_logger.c
- Replace Core/Src/main.c with the provided one (or copy the USER CODE blocks)

## Wiring
- MPU6050: VCC=3.3V, GND, SDA=PB9, SCL=PB8
- BMP280: VCC=3.3V, GND, SDA=PB9, SCL=PB8
Notes:
- Ensure SDA/SCL pull-ups exist (most breakouts include them)

## Telemetry Output
Lines look like:
TS:<ms>,V:<mask>,AX:<g>,AY:<g>,AZ:<g>,GX:<dps>,GY:<dps>,GZ:<dps>,T:<C>,P:<hPa>

Validity mask (V):
- bit0 (1): MPU6050 OK
- bit1 (2): BMP280 OK
Example: V=3 means both OK.

## Build Note (printf float)
If float formatting prints incorrectly, add linker flag:
- -u _printf_float
CubeIDE: Project Properties → C/C++ Build → Settings → MCU GCC Linker → Miscellaneous → Linker flags.

## Verification (Scope / Logic Analyzer)
Timing:
- Confirm TIM2 tick frequency at 100 Hz by toggling a GPIO in the timer callback if desired.
I2C integrity:
- Probe PB8/PB9 for clean edges and stable rise times.
UART integrity:
- Confirm 115200 8N1, no framing errors, stable line rate.

## Test Cases
1) Normal operation: steady telemetry at 10 Hz
2) Disconnect BMP280: V becomes 1, firmware continues
3) Disconnect MPU6050: V becomes 2, firmware continues
4) Add noisy wiring / longer leads: observe retries and error count rising while system continues

## Recommended GitHub Contents
- Core/Inc/sensor_logger.h
- Core/Src/sensor_logger.c
- Core/Src/main.c
- README.md
- .gitignore
Optionally include the .ioc for easy reproduction if you want “clone + regenerate”.
