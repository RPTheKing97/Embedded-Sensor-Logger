# Embedded-Sensor-Logger

This project implements an optimized embedded sensor logger firmware demonstrating:
- Interrupt-driven sampling (TIM2) for consistent timing
- Sensor acquisition over I2C (MPU6050 + BMP280) with retry-based robustness
- SPI communication layer + demo WHOAMI read (optional)
- Basic DSP filtering (moving average) for noise reduction
- UART telemetry streaming with timeout + recovery behavior
- Bring-up steps and verification using oscilloscope/logic analyzer

## Target Platform
- Board: NUCLEO-F401RE (STM32F401RE, 84 MHz)
- UART: USART2 via ST-LINK Virtual COM Port (115200 8N1)
- I2C: I2C1 (400 kHz)

## Wiring
### UART
- Uses onboard ST-LINK VCP (no extra wiring)
- Open serial terminal at 115200 baud

### I2C Sensors (I2C1)
- SCL: PB8
- SDA: PB9
- VCC: 3.3V
- GND: GND
Notes:
- Ensure pull-ups exist on SDA/SCL (many breakout boards include them)
- BMP280 address may be 0x76 or 0x77. If your board is 0x77, change I2C_ADDR_BMP in app.c.

### SPI Demo (Optional)
- SPI1 pins: PA5(SCK), PA6(MISO), PA7(MOSI)
- CS: PA6 is used as GPIO in this template for CS (adjust if needed)
- Enable by setting ENABLE_SPI_DEMO=1 in app.c and matching WHOAMI register/value for your SPI sensor.

## Telemetry Format
UART prints one line per telemetry event:
TS:<ms>,V:<mask>,AX:<g>,AY:<g>,AZ:<g>,GX:<dps>,GY:<dps>,GZ:<dps>,T:<C>,P:<hPa>

Validity mask V:
- bit0 (1): MPU6050 read OK
- bit1 (2): BMP280 read OK
Example: V=3 means both sensors OK.

## Code Notes (Design)
- app.c: main scheduling logic; uses flags from TIM2 tick to run sample/process/telemetry.
- moving_average.c: O(1) moving average using ring buffer + running sum.
- comm_i2c.c: HAL I2C memory read/write with retry logic and small inter-retry delay.
- telemetry.c: UART output using snprintf and UART timeout; abort/deinit on error so board layer can re-init.
- bmp280.c: reads calibration and applies temperature/pressure compensation for real units.

## Bring-Up Steps
1. Create CubeIDE project for NUCLEO-F401RE.
2. Enable peripherals (or leave if already default):
   - USART2 Asynchronous
   - I2C1
   - TIM2 base timer with interrupt enabled
   - SPI1 (only needed if you enable the SPI demo)
3. Copy the provided files into Core/Inc and Core/Src.
4. Build + flash.
5. Open serial monitor (115200 8N1). Confirm telemetry lines.

## Verification (Oscilloscope / Logic Analyzer)
### Timing verification (TIM2 tick)
- Option A: Toggle LD2 (PA5) inside App_TimerTick() temporarily and measure:
  - Period should be 10 ms for 100 Hz
  - Jitter should be minimal
- Option B: Use LA on a spare GPIO toggle around sample routine to confirm sample execution time < 10 ms.

### I2C signal integrity
- Probe PB8/PB9:
  - Confirm clean rise times and no excessive ringing
  - If edges are slow: reduce I2C speed to 100 kHz or adjust pull-ups

### UART integrity
- Confirm no framing errors at 115200.
- If missing floats, enable printf float support (see below).

## Common Build Gotcha: printf float support
If floats print as 0.00 or not at all, add linker flag:
- -u _printf_float
CubeIDE: Project Properties -> C/C++ Build -> Settings -> MCU GCC Linker -> Miscellaneous.

## Test Cases
1. Baseline output:
   - Telemetry prints continuously
2. MPU6050 sanity:
   - Flat on table: AZ ~ 1 g
   - Tilt: AX/AY change as expected
3. BMP280 sanity:
   - Temp near room temperature
   - Pressure around ~1000–1025 hPa depending on altitude/weather
4. Fault injection:
   - Disconnect BMP280: V mask becomes 1; firmware continues running
   - Disconnect MPU6050: V mask becomes 2; firmware continues running
   - Shorten/lengthen I2C wires: observe retry behavior and error counters (extend telemetry if desired)

## Future of the Project
- Convert UART to DMA + ring buffer for non-blocking telemetry
- Add packet framing + CRC for binary streaming
- Add watchdog (IWDG) for autonomous recovery
- Add SD card logging (FATFS) for offline capture
