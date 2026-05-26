/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sensor_logger.c
  * @brief   Interrupt-timed sensor logger with retry, fault detection, filtering,
  *          and UART telemetry for a NUCLEO-F401RE.
  ******************************************************************************
  */
/* USER CODE END Header */

/* =================== INCLUDES =================== */
#include "sensor_logger.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* =================== LOCALS =================== */
typedef struct {
    float buf[SL_MA_WINDOW];
    uint16_t idx;
    uint16_t count;
    float sum;
} MA_t;

typedef struct {
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3; int16_t dig_P4; int16_t dig_P5;
    int16_t dig_P6; int16_t dig_P7; int16_t dig_P8; int16_t dig_P9;
    int32_t t_fine;
} BMP280_Cal_t;

static UART_HandleTypeDef *g_uart = NULL;
static I2C_HandleTypeDef  *g_i2c  = NULL;

/* Counters prevent silently losing ticks if foreground work ever runs long. */
static volatile uint32_t g_sample_pending = 0;
static volatile uint32_t g_telem_pending  = 0;

static SL_SensorData_t g_raw  = {0};
static SL_SensorData_t g_filt = {0};
static SL_Status_t     g_stat = {0};

static MA_t g_ma_ax = {0};
static MA_t g_ma_ay = {0};
static MA_t g_ma_az = {0};

static BMP280_Cal_t g_bmp = {0};

static uint8_t g_mpu_ready = 0;
static uint8_t g_bmp_ready = 0;

/* =================== FORWARD =================== */
static void ma_update(MA_t *ma, float x);
static float ma_get(const MA_t *ma);

static uint8_t i2c_read_retry(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);
static uint8_t i2c_write_retry(uint8_t addr7, uint8_t reg, uint8_t val);

static void i2c_soft_recover(void);
static void i2c_bus_clear_pb8_pb9(void);

static HAL_StatusTypeDef mpu_init(void);
static uint8_t mpu_read(SL_SensorData_t *out);

static HAL_StatusTypeDef bmp_init(void);
static uint8_t bmp_read(SL_SensorData_t *out);

static int32_t bmp_comp_t(int32_t adc_t);
static uint32_t bmp_comp_p(int32_t adc_p);

static void sample_all(void);
static void process_all(void);
static void send_uart(void);
static int32_t scale_i32(float value, float scale);
static void clear_mpu_fields(SL_SensorData_t *out);
static void clear_bmp_fields(SL_SensorData_t *out);

/* =================== API =================== */
HAL_StatusTypeDef SL_Init(UART_HandleTypeDef *huart2,
                          I2C_HandleTypeDef  *hi2c1)
{
    if (huart2 == NULL || hi2c1 == NULL) return HAL_ERROR;

    g_uart = huart2;
    g_i2c  = hi2c1;

    memset(&g_raw,  0, sizeof(g_raw));
    memset(&g_filt, 0, sizeof(g_filt));
    memset(&g_stat, 0, sizeof(g_stat));
    memset(&g_ma_ax, 0, sizeof(g_ma_ax));
    memset(&g_ma_ay, 0, sizeof(g_ma_ay));
    memset(&g_ma_az, 0, sizeof(g_ma_az));
    memset(&g_bmp,   0, sizeof(g_bmp));

    g_mpu_ready = (mpu_init() == HAL_OK) ? 1U : 0U;
    g_bmp_ready = (bmp_init() == HAL_OK) ? 1U : 0U;

    /*
     * Return HAL_OK even when a sensor is disconnected.
     * The logger keeps running and reports sensor state through V, ERR, and RET.
     */
    return HAL_OK;
}

void SL_Loop(void)
{
    for (;;) {
        __disable_irq();
        if (g_sample_pending == 0U) {
            __enable_irq();
            break;
        }
        g_sample_pending--;
        __enable_irq();

        sample_all();
        process_all();
    }

    if (g_telem_pending > 0U) {
        __disable_irq();
        g_telem_pending = 0U;  /* coalesce telemetry into the latest sample */
        __enable_irq();

        send_uart();
    }
}

void SL_OnTim2Tick(void)
{
    static uint16_t telem_ctr = 0;

    if (g_sample_pending < 1000U) {
        g_sample_pending++;
    } else {
        g_stat.sample_overruns++;
    }

    telem_ctr++;
    const uint16_t div = (uint16_t)(SL_SAMPLE_RATE_HZ / SL_TELEMETRY_RATE_HZ);
    if (div > 0U && telem_ctr >= div) {
        telem_ctr = 0;
        if (g_telem_pending < 1000U) {
            g_telem_pending++;
        }
    }
}

uint8_t SL_HasPendingWork(void)
{
    return (g_sample_pending > 0U || g_telem_pending > 0U) ? 1U : 0U;
}

const SL_SensorData_t* SL_GetLatest(void) { return &g_filt; }
const SL_Status_t* SL_GetStatus(void) { return &g_stat; }

/* =================== MOVING AVERAGE =================== */
static void ma_update(MA_t *ma, float x)
{
    if (ma->count < SL_MA_WINDOW) ma->count++;
    ma->sum -= ma->buf[ma->idx];
    ma->buf[ma->idx] = x;
    ma->sum += x;
    ma->idx = (uint16_t)((ma->idx + 1U) % SL_MA_WINDOW);
}

static float ma_get(const MA_t *ma)
{
    if (ma->count == 0U) return 0.0f;
    return ma->sum / (float)ma->count;
}

/* =================== I2C RETRY =================== */
static uint8_t i2c_read_retry(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (g_i2c == NULL || data == NULL || len == 0U) return 0U;

    for (uint8_t r = 0; r < SL_MAX_RETRIES; r++) {
        if (HAL_I2C_Mem_Read(g_i2c, (uint16_t)(addr7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, SL_I2C_TIMEOUT_MS) == HAL_OK) {
            return 1U;
        }
        g_stat.retries++;
        HAL_Delay(1);
    }

    i2c_soft_recover();
    return 0U;
}

static uint8_t i2c_write_retry(uint8_t addr7, uint8_t reg, uint8_t val)
{
    if (g_i2c == NULL) return 0U;

    for (uint8_t r = 0; r < SL_MAX_RETRIES; r++) {
        if (HAL_I2C_Mem_Write(g_i2c, (uint16_t)(addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, SL_I2C_TIMEOUT_MS) == HAL_OK) {
            return 1U;
        }
        g_stat.retries++;
        HAL_Delay(1);
    }

    i2c_soft_recover();
    return 0U;
}

/* =================== I2C RECOVERY =================== */
static void i2c_soft_recover(void)
{
    if (g_i2c == NULL) return;

    /*
     * A simple peripheral re-init is not always enough when SDA is held low.
     * Clear PB8/PB9 manually first, then restore the I2C peripheral.
     */
    i2c_bus_clear_pb8_pb9();
    (void)HAL_I2C_DeInit(g_i2c);
    (void)HAL_I2C_Init(g_i2c);
}

static void i2c_bus_clear_pb8_pb9(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    (void)HAL_I2C_DeInit(g_i2c);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    /* Pulse SCL up to 9 times to release a stuck slave. */
    for (uint8_t i = 0; i < 9U; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* Generate a STOP condition: SDA low while SCL high, then SDA high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* =================== MPU6050 =================== */
static HAL_StatusTypeDef mpu_init(void)
{
    if (!i2c_write_retry(SL_MPU6050_ADDR7, 0x6B, 0x00)) return HAL_ERROR; /* wake */
    if (!i2c_write_retry(SL_MPU6050_ADDR7, 0x1B, 0x00)) return HAL_ERROR; /* gyro +/-250 dps */
    if (!i2c_write_retry(SL_MPU6050_ADDR7, 0x1C, 0x00)) return HAL_ERROR; /* accel +/-2g */
    return HAL_OK;
}

static uint8_t mpu_read(SL_SensorData_t *out)
{
    uint8_t b[14];
    if (!i2c_read_retry(SL_MPU6050_ADDR7, 0x3B, b, (uint16_t)sizeof(b))) return 0U;

    int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
    int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
    int16_t az = (int16_t)((b[4]  << 8) | b[5]);
    int16_t t  = (int16_t)((b[6]  << 8) | b[7]);
    int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
    int16_t gy = (int16_t)((b[10] << 8) | b[11]);
    int16_t gz = (int16_t)((b[12] << 8) | b[13]);

    out->accel_x = (float)ax / 16384.0f;
    out->accel_y = (float)ay / 16384.0f;
    out->accel_z = (float)az / 16384.0f;

    out->gyro_x  = (float)gx / 131.0f;
    out->gyro_y  = (float)gy / 131.0f;
    out->gyro_z  = (float)gz / 131.0f;

    /* MPU6050 internal die temperature */
    out->temperature_c = ((float)t / 340.0f) + 36.53f;

    return 1U;
}

/* =================== BMP280 =================== */
static HAL_StatusTypeDef bmp_init(void)
{
    uint8_t id = 0;
    if (!i2c_read_retry(SL_BMP280_ADDR7, 0xD0, &id, 1)) return HAL_ERROR;
    if (id != 0x58 && id != 0x56 && id != 0x57) return HAL_ERROR;

    uint8_t c[24];
    if (!i2c_read_retry(SL_BMP280_ADDR7, 0x88, c, (uint16_t)sizeof(c))) return HAL_ERROR;

    g_bmp.dig_T1 = (uint16_t)(c[1] << 8 | c[0]);
    g_bmp.dig_T2 = (int16_t)(c[3] << 8 | c[2]);
    g_bmp.dig_T3 = (int16_t)(c[5] << 8 | c[4]);

    g_bmp.dig_P1 = (uint16_t)(c[7] << 8 | c[6]);
    g_bmp.dig_P2 = (int16_t)(c[9] << 8 | c[8]);
    g_bmp.dig_P3 = (int16_t)(c[11] << 8 | c[10]);
    g_bmp.dig_P4 = (int16_t)(c[13] << 8 | c[12]);
    g_bmp.dig_P5 = (int16_t)(c[15] << 8 | c[14]);
    g_bmp.dig_P6 = (int16_t)(c[17] << 8 | c[16]);
    g_bmp.dig_P7 = (int16_t)(c[19] << 8 | c[18]);
    g_bmp.dig_P8 = (int16_t)(c[21] << 8 | c[20]);
    g_bmp.dig_P9 = (int16_t)(c[23] << 8 | c[22]);

    /* ctrl_meas: temp x1, pressure x1, normal mode */
    if (!i2c_write_retry(SL_BMP280_ADDR7, 0xF4, 0x27)) return HAL_ERROR;

    /* config: standby 1000 ms, filter x4 */
    if (!i2c_write_retry(SL_BMP280_ADDR7, 0xF5, 0xA0)) return HAL_ERROR;

    return HAL_OK;
}

static uint8_t bmp_read(SL_SensorData_t *out)
{
    uint8_t b[6];

    /* One burst read gets pressure and temperature registers. */
    if (!i2c_read_retry(SL_BMP280_ADDR7, 0xF7, b, (uint16_t)sizeof(b))) return 0U;

    int32_t adc_p = (int32_t)(((uint32_t)b[0] << 12) | ((uint32_t)b[1] << 4) | ((uint32_t)b[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | ((uint32_t)b[5] >> 4));

    int32_t t_x100 = bmp_comp_t(adc_t);
    uint32_t p_q24_8 = bmp_comp_p(adc_p);

    out->temperature_c = (float)t_x100 / 100.0f;
    out->pressure_hpa  = ((float)p_q24_8 / 256.0f) / 100.0f;

    return 1U;
}

static int32_t bmp_comp_t(int32_t adc_t)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)g_bmp.dig_T1 << 1))) * ((int32_t)g_bmp.dig_T2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)g_bmp.dig_T1)) * ((adc_t >> 4) - ((int32_t)g_bmp.dig_T1))) >> 12) *
                     ((int32_t)g_bmp.dig_T3)) >> 14;
    g_bmp.t_fine = var1 + var2;
    return (g_bmp.t_fine * 5 + 128) >> 8;
}

static uint32_t bmp_comp_p(int32_t adc_p)
{
    int64_t var1 = (int64_t)g_bmp.t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)g_bmp.dig_P6;
    var2 = var2 + ((var1 * (int64_t)g_bmp.dig_P5) << 17);
    var2 = var2 + (((int64_t)g_bmp.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)g_bmp.dig_P3) >> 8) + ((var1 * (int64_t)g_bmp.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)g_bmp.dig_P1) >> 33;
    if (var1 == 0) return 0U;

    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)g_bmp.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)g_bmp.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)g_bmp.dig_P7) << 4);
    return (uint32_t)p;
}

/* =================== PIPELINE =================== */
static void sample_all(void)
{
    static uint16_t mpu_reinit_ctr = 0;
    static uint16_t bmp_reinit_ctr = 0;

    g_raw.timestamp_ms = HAL_GetTick();
    g_raw.valid_mask = 0U;

    uint8_t ok_mpu = 0U;
    uint8_t ok_bmp = 0U;
    uint8_t attempted_mpu = 0U;
    uint8_t attempted_bmp = 0U;

    if (g_mpu_ready) {
        attempted_mpu = 1U;
        ok_mpu = mpu_read(&g_raw);
    } else if (++mpu_reinit_ctr >= SL_SAMPLE_RATE_HZ) {
        mpu_reinit_ctr = 0U;
        attempted_mpu = 1U;
        g_mpu_ready = (mpu_init() == HAL_OK) ? 1U : 0U;
        if (g_mpu_ready) ok_mpu = mpu_read(&g_raw);
    }

    if (ok_mpu) {
        mpu_reinit_ctr = 0U;
        g_raw.valid_mask |= SL_VALID_MPU6050;
    } else {
        g_mpu_ready = 0U;
        if (attempted_mpu) {
            g_stat.comm_errors++;
            g_stat.mpu_failures++;
        }
        clear_mpu_fields(&g_raw);
    }

    if (g_bmp_ready) {
        attempted_bmp = 1U;
        ok_bmp = bmp_read(&g_raw);
    } else if (++bmp_reinit_ctr >= SL_SAMPLE_RATE_HZ) {
        bmp_reinit_ctr = 0U;
        attempted_bmp = 1U;
        g_bmp_ready = (bmp_init() == HAL_OK) ? 1U : 0U;
        if (g_bmp_ready) ok_bmp = bmp_read(&g_raw);
    }

    if (ok_bmp) {
        bmp_reinit_ctr = 0U;
        g_raw.valid_mask |= SL_VALID_BMP280;
    } else {
        g_bmp_ready = 0U;
        if (attempted_bmp) {
            g_stat.comm_errors++;
            g_stat.bmp_failures++;
        }
        clear_bmp_fields(&g_raw);
    }

    g_stat.samples_taken++;
}

static void process_all(void)
{
    if ((g_raw.valid_mask & SL_VALID_MPU6050) != 0U) {
        ma_update(&g_ma_ax, g_raw.accel_x);
        ma_update(&g_ma_ay, g_raw.accel_y);
        ma_update(&g_ma_az, g_raw.accel_z);

        g_filt = g_raw;
        g_filt.accel_x = ma_get(&g_ma_ax);
        g_filt.accel_y = ma_get(&g_ma_ay);
        g_filt.accel_z = ma_get(&g_ma_az);
    } else {
        g_filt = g_raw;
    }
}

/* =================== UART TELEMETRY =================== */
static void send_uart(void)
{
    uint8_t buf[240];

    const int32_t ax_mg = scale_i32(g_filt.accel_x, 1000.0f);
    const int32_t ay_mg = scale_i32(g_filt.accel_y, 1000.0f);
    const int32_t az_mg = scale_i32(g_filt.accel_z, 1000.0f);

    const int32_t gx_cdps = scale_i32(g_filt.gyro_x, 100.0f);
    const int32_t gy_cdps = scale_i32(g_filt.gyro_y, 100.0f);
    const int32_t gz_cdps = scale_i32(g_filt.gyro_z, 100.0f);

    const int32_t t_cC = scale_i32(g_filt.temperature_c, 100.0f);
    const int32_t p_chPa = scale_i32(g_filt.pressure_hpa, 100.0f);

    /*
     * Integer-scaled telemetry avoids expensive float printf and does not need
     * the -u _printf_float linker flag.
     */
    int n = snprintf((char*)buf, sizeof(buf),
        "TS:%" PRIu32 ",V:%u,AX_mg:%" PRId32 ",AY_mg:%" PRId32 ",AZ_mg:%" PRId32
        ",GX_cdps:%" PRId32 ",GY_cdps:%" PRId32 ",GZ_cdps:%" PRId32
        ",T_cC:%" PRId32 ",P_chPa:%" PRId32
        ",ERR:%" PRIu32 ",RET:%" PRIu32 ",S:%" PRIu32 "\r\n",
        g_filt.timestamp_ms, (unsigned)g_filt.valid_mask,
        ax_mg, ay_mg, az_mg,
        gx_cdps, gy_cdps, gz_cdps,
        t_cC, p_chPa,
        g_stat.comm_errors, g_stat.retries, g_stat.samples_taken);

    if (n < 0) {
        g_stat.comm_errors++;
        return;
    }

    uint16_t len = (uint16_t)((n >= (int)sizeof(buf)) ? (sizeof(buf) - 1U) : (uint16_t)n);
    HAL_StatusTypeDef st = HAL_UART_Transmit(g_uart, buf, len, SL_UART_TIMEOUT_MS);

    if (st == HAL_OK) {
        g_stat.samples_sent++;
    } else {
        g_stat.comm_errors++;
        g_stat.last_uart_error = g_uart->ErrorCode;
        (void)HAL_UART_Abort(g_uart);
        (void)HAL_UART_DeInit(g_uart);
        (void)HAL_UART_Init(g_uart);
    }
}

static int32_t scale_i32(float value, float scale)
{
    float scaled = value * scale;
    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    return (int32_t)scaled;
}

static void clear_mpu_fields(SL_SensorData_t *out)
{
    if (out == NULL) return;

    out->accel_x = 0.0f;
    out->accel_y = 0.0f;
    out->accel_z = 0.0f;
    out->gyro_x = 0.0f;
    out->gyro_y = 0.0f;
    out->gyro_z = 0.0f;
}

static void clear_bmp_fields(SL_SensorData_t *out)
{
    if (out == NULL) return;

    out->temperature_c = 0.0f;
    out->pressure_hpa = 0.0f;
}
