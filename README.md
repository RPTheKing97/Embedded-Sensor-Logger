# Embedded-Sensor-Logger
Embedded Sensor Logger is STM32F401RE firmware (NUCLEO-F401RE) that samples MPU6050 + BMP280 over I2C at 100 Hz using TIM2 interrupts, applies an 8-sample moving average to accel data, and streams timestamped telemetry over UART2 at 10 Hz with retries and fault recovery for robust sensor communication.
