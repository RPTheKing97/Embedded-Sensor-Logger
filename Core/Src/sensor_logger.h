/* =================== INCLUDES =================== */
#pragma once
#ifndef SENSOR_LOGGER_H
#define SENSOR_LOGGER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* =================== CONFIG =================== */
#define SL_SAMPLE_RATE_HZ        100U
#define SL_TELEMETRY_RATE_HZ     10U
#define SL_MA_WINDOW             8U
#define SL_MAX_RETRIES           3U

#define SL_I2C_TIMEOUT_MS        10U
#define SL_UART_TIMEOUT_MS       50U

#define SL_MPU6050_ADDR7         0x68U
#define SL_BMP280_ADDR7          0x76U  /* change to 0x77U if your module uses 0x77 */

/* =================== TYPES =================== */
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float temperature_c;
    float pressure_hpa;
    uint32_t timestamp_ms;
    uint8_t valid_mask;
} SL_SensorData_t;

typedef struct {
    uint32_t samples_taken;
    uint32_t samples_sent;
    uint32_t comm_errors;
    uint32_t retries;
    uint32_t last_uart_error;
} SL_Status_t;

/* =================== API =================== */
HAL_StatusTypeDef SL_Init(UART_HandleTypeDef *huart2,
                          I2C_HandleTypeDef  *hi2c1);

void SL_Loop(void);
void SL_OnTim2Tick(void);

const SL_SensorData_t* SL_GetLatest(void);
const SL_Status_t* SL_GetStatus(void);

#endif /* SENSOR_LOGGER_H */
