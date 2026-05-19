# Embedded Sensor Logger

STM32 NUCLEO-F401RE firmware that samples an MPU6050 and BMP280 over I2C, filters accelerometer data, detects sensor disconnects, and streams live UART telemetry for a Python dashboard.

## Project Highlights

- **STM32 NUCLEO-F401RE / STM32F401RE** firmware written in C using STM32 HAL.
- **TIM2 interrupt sampling at 100 Hz** for deterministic sensor acquisition timing.
- **UART telemetry at 10 Hz** over USART2 through the ST-LINK Virtual COM Port.
- **I2C1 sensor bus on PB8/PB9** for MPU6050 and BMP280.
- **Fault detection** when either sensor disconnects while firmware continues running.
- **Retry/error counters** sent in telemetry for debugging noisy wiring or missing sensors.
- **8-sample moving average filter** on accelerometer axes.
- **Python serial dashboard** for live plotting acceleration, gyro, temperature, pressure, errors, and retries.

## Repository Layout

```text
Embedded-Sensor-Logger/
├── Core/
│   ├── Inc/
│   │   └── sensor_logger.h
│   └── Src/
│       ├── main.c
│       ├── sensor_logger.c
│       ├── stm32f4xx_hal_msp.c
│       └── stm32f4xx_it.c
├── dashboard/
│   └── serial_dashboard.py
├── docs/
│   ├── demo_test_plan.md
│   ├── telemetry_format.md
│   ├── project_update_summary.md
│   └── hardware_schematic.png
├── requirements.txt
├── .gitignore
└── README.md
```

## Target Hardware

| Item | Configuration |
|---|---|
| Board | NUCLEO-F401RE |
| MCU | STM32F401RE |
| UART | USART2 through ST-LINK VCP |
| UART settings | 115200 baud, 8N1 |
| I2C | I2C1 Fast Mode, 400 kHz |
| I2C pins | PB8 = SCL, PB9 = SDA |
| Timer | TIM2 base timer interrupt at 100 Hz |

## Sensors

| Sensor | Purpose | I2C Address |
|---|---|---|
| MPU6050 | Accelerometer + gyroscope | `0x68` |
| BMP280 | Temperature + pressure | `0x76` by default; some modules use `0x77` |

If testing with a BMP280 board use `0x77`, change this line in `Core/Inc/sensor_logger.h`:

```c
#define SL_BMP280_ADDR7 0x77U
```

## Wiring

| NUCLEO-F401RE | MPU6050 | BMP280 | Notes |
|---|---|---|---|
| 3.3V | VCC | VCC | Use 3.3V only |
| GND | GND | GND | Shared ground required |
| PB8 | SCL | SCL | I2C1 clock |
| PB9 | SDA | SDA | I2C1 data |

Most breakout boards include SDA/SCL pull-ups. If the I2C bus is unstable, use shorter wires first, then verify pull-ups.

## STM32CubeMX Setup

CubeMX project for **NUCLEO-F401RE** and enabled:

1. **USART2**
   - Mode: Asynchronous
   - Baud: 115200
   - 8 data bits, no parity, 1 stop bit
2. **I2C1**
   - Pins: PB8 = SCL, PB9 = SDA
   - Speed: 400 kHz Fast Mode
3. **TIM2**
   - Base timer
   - Enable update interrupt in NVIC
   - Configure for 100 Hz update rate

Recommended TIM2 setup for 84 MHz SYSCLK:

```text
TIM2 clock = 84 MHz
Prescaler = 8399    => 10 kHz timer counter
Period    = 99      => 100 Hz update interrupt
```

Then copy the provided files into the CubeIDE project:

```text
Core/Inc/sensor_logger.h
Core/Src/sensor_logger.c
Core/Src/main.c
Core/Src/stm32f4xx_hal_msp.c
Core/Src/stm32f4xx_it.c
```

## Telemetry Output

The firmware sends integer-scaled CSV-style UART lines at 10 Hz:

```text
TS:12345,V:3,AX_mg:12,AY_mg:-5,AZ_mg:1001,GX_cdps:2,GY_cdps:-3,GZ_cdps:1,T_cC:2451,P_chPa:101325,ERR:0,RET:0,S:1234
```

Validity mask `V`:

| Bit | Meaning |
|---|---|
| bit0 / value `1` | MPU6050 OK |
| bit1 / value `2` | BMP280 OK |

Examples:

| V value | Meaning |
|---|---|
| `3` | Both sensors OK |
| `1` | MPU6050 OK, BMP280 fault/disconnected |
| `2` | BMP280 OK, MPU6050 fault/disconnected |
| `0` | Both sensors fault/disconnected |

Scaled units:

| Field | Unit |
|---|---|
| `AX_mg`, `AY_mg`, `AZ_mg` | milli-g |
| `GX_cdps`, `GY_cdps`, `GZ_cdps` | centi-degrees/second |
| `T_cC` | centi-degrees Celsius |
| `P_chPa` | centi-hPa |
| `ERR` | communication/fault counter |
| `RET` | I2C retry counter |
| `S` | samples taken |

This avoids expensive float formatting on the STM32, so the project does **not** require the `-u _printf_float` linker flag.

## Verification / Demo Test Cases

Use a serial monitor or the Python dashboard while running these tests:

1. **Normal operation**
   - Expected: steady telemetry at 10 Hz.
   - Expected validity: `V:3`.

2. **Disconnect BMP280**
   - Expected: firmware continues running.
   - Expected validity: `V:1`.
   - Expected: `ERR` and/or `RET` increase.

3. **Disconnect MPU6050**
   - Expected: firmware continues running.
   - Expected validity: `V:2`.
   - Expected: `ERR` and/or `RET` increase.

4. **Add noisy wiring / longer leads**
   - Expected: dashboard continues updating.
   - Expected: retry counter rises while firmware remains alive.
