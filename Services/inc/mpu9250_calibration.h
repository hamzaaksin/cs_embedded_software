#ifndef MPU9250_CALIBRATION_H
#define MPU9250_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu9250.h"
#include <stdint.h>

#ifndef MPU9250_CALIBRATION_FLASH_ADDRESS
#define MPU9250_CALIBRATION_FLASH_ADDRESS 0x080E0000UL
#endif

#ifndef MPU9250_CALIBRATION_FLASH_SECTOR
#define MPU9250_CALIBRATION_FLASH_SECTOR FLASH_SECTOR_11
#endif

typedef enum
{
    MPU9250_GRAVITY_POSITIVE_X = 0,
    MPU9250_GRAVITY_NEGATIVE_X,
    MPU9250_GRAVITY_POSITIVE_Y,
    MPU9250_GRAVITY_NEGATIVE_Y,
    MPU9250_GRAVITY_POSITIVE_Z,
    MPU9250_GRAVITY_NEGATIVE_Z
} MPU9250_GravityAxis_t;

typedef enum
{
    MPU9250_CALIBRATION_OK = 0,
    MPU9250_CALIBRATION_NOT_FOUND,
    MPU9250_CALIBRATION_SENSOR_ERROR,
    MPU9250_CALIBRATION_FLASH_ERROR,
    MPU9250_CALIBRATION_INVALID_ARGUMENT,
    MPU9250_CALIBRATION_INVALID_DATA
} MPU9250_CalibrationStatus_t;

typedef struct
{
    float accel_x_offset_g;
    float accel_y_offset_g;
    float accel_z_offset_g;
    float gyro_x_offset_dps;
    float gyro_y_offset_dps;
    float gyro_z_offset_dps;
    float mag_x_offset_uT;
    float mag_y_offset_uT;
    float mag_z_offset_uT;
    float mag_x_scale;
    float mag_y_scale;
    float mag_z_scale;
} MPU9250_CalibrationData_t;

MPU9250_CalibrationStatus_t MPU9250_Calibration_Init(void);
MPU9250_CalibrationStatus_t MPU9250_Calibration_Load(void);
MPU9250_CalibrationStatus_t MPU9250_Calibration_Save(void);
MPU9250_CalibrationStatus_t MPU9250_Calibration_Erase(void);
MPU9250_CalibrationStatus_t MPU9250_Calibration_Calculate(
    uint16_t sample_count,
    uint32_t sample_delay_ms,
    MPU9250_GravityAxis_t gravity_axis);
MPU9250_CalibrationStatus_t MPU9250_Calibration_CalculateAndSave(
    uint16_t sample_count,
    uint32_t sample_delay_ms,
    MPU9250_GravityAxis_t gravity_axis);
MPU9250_CalibrationStatus_t MPU9250_Calibration_CalculateMagnetometer(
    uint16_t sample_count,
    uint32_t sample_delay_ms);
MPU9250_CalibrationStatus_t
MPU9250_Calibration_CalculateMagnetometerAndSave(
    uint16_t sample_count,
    uint32_t sample_delay_ms);
HAL_StatusTypeDef MPU9250_Calibration_Read(MPU9250_Data_t *data);
void MPU9250_Calibration_Apply(MPU9250_Data_t *data);
void MPU9250_Calibration_Get(MPU9250_CalibrationData_t *data);
MPU9250_CalibrationStatus_t MPU9250_Calibration_Set(
    const MPU9250_CalibrationData_t *data);
uint8_t MPU9250_Calibration_HasValidData(void);

#ifdef __cplusplus
}
#endif

#endif
