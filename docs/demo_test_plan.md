# Demo Video Test Plan

## Goal

Show that the STM32 NUCLEO-F401RE sensor logger continues running during normal operation, sensor disconnects, and noisy wiring conditions.

## Equipment

- NUCLEO-F401RE board
- MPU6050 breakout
- BMP280 breakout
- Jumper wires
- USB cable for ST-LINK VCP
- PC running the Python serial dashboard
- Optional: oscilloscope or logic analyzer for PB8/PB9 and USART2

## Suggested Recording Setup

- No voiceover needed.
- Use close-up shots of the wiring.
- Use one hand to disconnect/reconnect sensor wires.
- Pan to the laptop/monitor after each test so the dashboard is visible.

## Shot List

### 1. Project overview

Show the NUCLEO-F401RE connected to the MPU6050 and BMP280. Show shared 3.3V, GND, PB8/SCL, and PB9/SDA wiring.

### 2. Normal operation

Start with both sensors connected.

Expected result:

```text
V:3
ERR stable or low
RET stable or low
Telemetry continues at 10 Hz
```

Dashboard should show live accel, gyro, temperature, and pressure plots.

### 3. Disconnect BMP280

Disconnect BMP280 VCC, GND, SDA, or SCL.

Expected result:

```text
V:1
Firmware continues
ERR and/or RET increase
MPU6050 data continues plotting
```

### 4. Reconnect BMP280

Reconnect BMP280 wiring.

Expected result:

```text
V:3
BMP280 data resumes after recovery/re-init
Telemetry continues
```

### 5. Disconnect MPU6050

Disconnect MPU6050 VCC, GND, SDA, or SCL.

Expected result:

```text
V:2
Firmware continues
ERR and/or RET increase
BMP280 data continues plotting
```

### 6. Noisy wiring / longer leads

Add longer jumper wires or lightly disturb the I2C wiring.

Expected result:

```text
Telemetry continues
RET increases during bus issues
ERR may increase
System does not lock up
```

## Final README Clip

End with a shot of the GitHub README showing the schematic and project features.
