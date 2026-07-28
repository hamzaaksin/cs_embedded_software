#ifndef MPU9250_H
#define MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifndef MPU9250_I2C_ADDRESS_7BIT
#define MPU9250_I2C_ADDRESS_7BIT 0x68U
#endif

#ifndef MPU9250_I2C_TIMEOUT_MS
#define MPU9250_I2C_TIMEOUT_MS 100U
#endif

#define MPU9250_I2C_ADDRESS ((uint16_t)(MPU9250_I2C_ADDRESS_7BIT << 1U))
#define MPU9250_WHO_AM_I_VALUE 0x71U

typedef enum
{
    MPU9250_ACCEL_RANGE_2G = 0,
    MPU9250_ACCEL_RANGE_4G = 1,
    MPU9250_ACCEL_RANGE_8G = 2,
    MPU9250_ACCEL_RANGE_16G = 3
} MPU9250_AccelRange_t;

typedef enum
{
    MPU9250_GYRO_RANGE_250DPS = 0,
    MPU9250_GYRO_RANGE_500DPS = 1,
    MPU9250_GYRO_RANGE_1000DPS = 2,
    MPU9250_GYRO_RANGE_2000DPS = 3
} MPU9250_GyroRange_t;

typedef struct
{
    MPU9250_AccelRange_t accel_range;
    MPU9250_GyroRange_t gyro_range;
    uint8_t sample_rate_divider;
    uint8_t digital_low_pass_filter;
} MPU9250_Config_t;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} MPU9250_RawData_t;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float mag_x_uT;
    float mag_y_uT;
    float mag_z_uT;
} MPU9250_Data_t;

HAL_StatusTypeDef MPU9250_Init(void);
HAL_StatusTypeDef MPU9250_InitWithConfig(const MPU9250_Config_t *config);
HAL_StatusTypeDef MPU9250_IsReady(void);
HAL_StatusTypeDef MPU9250_ReadRaw(MPU9250_RawData_t *data);
HAL_StatusTypeDef MPU9250_Read(MPU9250_Data_t *data);
HAL_StatusTypeDef MPU9250_StartReadDMA(void);
HAL_StatusTypeDef MPU9250_ProcessDMA(MPU9250_Data_t *data);
float MPU9250_GetAccelSensitivity(void);
float MPU9250_GetGyroSensitivity(void);

#ifdef __cplusplus
}
#endif

#endif
