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

static volatile uint8_t g_sample_flag = 0;
static volatile uint8_t g_telem_flag  = 0;

static SL_SensorData_t g_raw  = {0};
static SL_SensorData_t g_filt = {0};
static SL_Status_t     g_stat = {0};

static MA_t g_ma_ax = {0};
static MA_t g_ma_ay = {0};
static MA_t g_ma_az = {0};

static BMP280_Cal_t g_bmp = {0};

/* =================== FORWARD =================== */
static void ma_update(MA_t *ma, float x);
static float ma_get(const MA_t *ma);

static uint8_t i2c_read_retry(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);
static uint8_t i2c_write_retry(uint8_t addr7, uint8_t reg, uint8_t val);

static void i2c_soft_recover(void);

static HAL_StatusTypeDef mpu_init(void);
static uint8_t mpu_read(SL_SensorData_t *out);

static HAL_StatusTypeDef bmp_init(void);
static uint8_t bmp_read(SL_SensorData_t *out);

static int32_t bmp_comp_t(int32_t adc_t);
static uint32_t bmp_comp_p(int32_t adc_p);

static void sample_all(void);
static void process_all(void);
static void send_uart(void);

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

    uint8_t tries = 0;
    while (tries < SL_MAX_RETRIES) {
        if (mpu_init() == HAL_OK && bmp_init() == HAL_OK) return HAL_OK;
        tries++;
        HAL_Delay(100);
    }
    return HAL_ERROR;
}

void SL_Loop(void)
{
    if (g_sample_flag) {
        g_sample_flag = 0;
        sample_all();
        process_all();
    }

    if (g_telem_flag) {
        g_telem_flag = 0;
        send_uart();
    }
}

void SL_OnTim2Tick(void)
{
    static uint16_t telem_ctr = 0;

    g_sample_flag = 1;

    telem_ctr++;
    const uint16_t div = (uint16_t)(SL_SAMPLE_RATE_HZ / SL_TELEMETRY_RATE_HZ);
    if (div > 0U && telem_ctr >= div) {
        telem_ctr = 0;
        g_telem_flag = 1;
    }
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
    if (g_i2c == NULL) return 0;

    for (uint8_t r = 0; r < SL_MAX_RETRIES; r++) {
        if (HAL_I2C_Mem_Read(g_i2c, (uint16_t)(addr7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, SL_I2C_TIMEOUT_MS) == HAL_OK) {
            return 1;
        }
        g_stat.retries++;
        HAL_Delay(1);
    }

    i2c_soft_recover();
    return 0;
}

static uint8_t i2c_write_retry(uint8_t addr7, uint8_t reg, uint8_t val)
{
    if (g_i2c == NULL) return 0;

    for (uint8_t r = 0; r < SL_MAX_RETRIES; r++) {
        if (HAL_I2C_Mem_Write(g_i2c, (uint16_t)(addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, SL_I2C_TIMEOUT_MS) == HAL_OK) {
            return 1;
        }
        g_stat.retries++;
        HAL_Delay(1);
    }

    i2c_soft_recover();
    return 0;
}

/* =================== I2C RECOVERY =================== */
static void i2c_soft_recover(void)
{
    if (g_i2c == NULL) return;
    (void)HAL_I2C_DeInit(g_i2c);
    (void)HAL_I2C_Init(g_i2c);
}

/* =================== MPU6050 =================== */
static HAL_StatusTypeDef mpu_init(void)
{
    if (!i2c_write_retry(SL_MPU6050_ADDR7, 0x6B, 0x00)) return HAL_ERROR;
    if (!i2c_write_retry(SL_MPU6050_ADDR7, 0x1B, 0x00)) return HAL_ERROR;
    if (!i2c_write_retry(SL_MPU6050_ADDR7, 0x1C, 0x00)) return HAL_ERROR;
    return HAL_OK;
}

static uint8_t mpu_read(SL_SensorData_t *out)
{
    uint8_t b[14];
    if (!i2c_read_retry(SL_MPU6050_ADDR7, 0x3B, b, (uint16_t)sizeof(b))) return 0;

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

    out->temperature_c = ((float)t / 340.0f) + 36.53f;

    return 1;
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

    if (!i2c_write_retry(SL_BMP280_ADDR7, 0xF4, 0x27)) return HAL_ERROR;
    if (!i2c_write_retry(SL_BMP280_ADDR7, 0xF5, 0xA0)) return HAL_ERROR;

    return HAL_OK;
}

static uint8_t bmp_read(SL_SensorData_t *out)
{
    uint8_t pbuf[3], tbuf[3];
    if (!i2c_read_retry(SL_BMP280_ADDR7, 0xF7, pbuf, 3)) return 0;
    if (!i2c_read_retry(SL_BMP280_ADDR7, 0xFA, tbuf, 3)) return 0;

    int32_t adc_p = (int32_t)(((uint32_t)pbuf[0] << 12) | ((uint32_t)pbuf[1] << 4) | ((uint32_t)pbuf[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)tbuf[0] << 12) | ((uint32_t)tbuf[1] << 4) | ((uint32_t)tbuf[2] >> 4));

    int32_t t_x100 = bmp_comp_t(adc_t);
    uint32_t p_q24_8 = bmp_comp_p(adc_p);

    out->temperature_c = (float)t_x100 / 100.0f;
    out->pressure_hpa  = ((float)p_q24_8 / 256.0f) / 100.0f;

    return 1;
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
    if (var1 == 0) return 0;

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
    g_raw.timestamp_ms = HAL_GetTick();
    g_raw.valid_mask = 0;

    uint8_t ok_mpu = mpu_read(&g_raw);
    if (ok_mpu) g_raw.valid_mask |= 0x01; else g_stat.comm_errors++;

    uint8_t ok_bmp = bmp_read(&g_raw);
    if (ok_bmp) g_raw.valid_mask |= 0x02; else g_stat.comm_errors++;

    g_stat.samples_taken++;
}

static void process_all(void)
{
    ma_update(&g_ma_ax, g_raw.accel_x);
    ma_update(&g_ma_ay, g_raw.accel_y);
    ma_update(&g_ma_az, g_raw.accel_z);

    g_filt = g_raw;
    g_filt.accel_x = ma_get(&g_ma_ax);
    g_filt.accel_y = ma_get(&g_ma_ay);
    g_filt.accel_z = ma_get(&g_ma_az);
}

/* =================== UART TELEMETRY =================== */
static void send_uart(void)
{
    uint8_t buf[220];

    int n = snprintf((char*)buf, sizeof(buf),
        "TS:%" PRIu32 ",V:%u,AX:%.3f,AY:%.3f,AZ:%.3f,GX:%.2f,GY:%.2f,GZ:%.2f,T:%.2fC,P:%.2fhPa\r\n",
        g_filt.timestamp_ms, (unsigned)g_filt.valid_mask,
        g_filt.accel_x, g_filt.accel_y, g_filt.accel_z,
        g_filt.gyro_x,  g_filt.gyro_y,  g_filt.gyro_z,
        g_filt.temperature_c, g_filt.pressure_hpa);

    if (n < 0) { g_stat.comm_errors++; return; }

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
