#include "mpu9250_calibration.h"
#include <float.h>
#include <string.h>

#define MPU9250_CALIBRATION_MAGIC 0x4D503943UL
#define MPU9250_CALIBRATION_VERSION 2U
#define MPU9250_CALIBRATION_MIN_MAG_SPAN_UT 5.0f

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    MPU9250_CalibrationData_t data;
    uint32_t crc32;
} MPU9250_CalibrationBlob_t;

static MPU9250_CalibrationData_t mpu9250_calibration_data =
{
    .mag_x_scale = 1.0f,
    .mag_y_scale = 1.0f,
    .mag_z_scale = 1.0f
};
static uint8_t mpu9250_calibration_valid = 0U;

static uint32_t MPU9250_Calibration_Crc32(
    const uint8_t *data,
    uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];

        for (bit = 0U; bit < 8U; ++bit)
        {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }

    return ~crc;
}

static uint8_t MPU9250_Calibration_FloatIsValid(float value, float limit)
{
    return (uint8_t)(
        value == value &&
        value <= limit &&
        value >= -limit);
}

static uint8_t MPU9250_Calibration_PositiveFloatIsValid(
    float value,
    float minimum,
    float maximum)
{
    return (uint8_t)(
        value == value &&
        value >= minimum &&
        value <= maximum);
}

static uint8_t MPU9250_Calibration_DataIsValid(
    const MPU9250_CalibrationData_t *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    return (uint8_t)(
        MPU9250_Calibration_FloatIsValid(data->accel_x_offset_g, 4.0f) &&
        MPU9250_Calibration_FloatIsValid(data->accel_y_offset_g, 4.0f) &&
        MPU9250_Calibration_FloatIsValid(data->accel_z_offset_g, 4.0f) &&
        MPU9250_Calibration_FloatIsValid(data->gyro_x_offset_dps, 500.0f) &&
        MPU9250_Calibration_FloatIsValid(data->gyro_y_offset_dps, 500.0f) &&
        MPU9250_Calibration_FloatIsValid(data->gyro_z_offset_dps, 500.0f) &&
        MPU9250_Calibration_FloatIsValid(data->mag_x_offset_uT, 2000.0f) &&
        MPU9250_Calibration_FloatIsValid(data->mag_y_offset_uT, 2000.0f) &&
        MPU9250_Calibration_FloatIsValid(data->mag_z_offset_uT, 2000.0f) &&
        MPU9250_Calibration_PositiveFloatIsValid(
            data->mag_x_scale, 0.1f, 10.0f) &&
        MPU9250_Calibration_PositiveFloatIsValid(
            data->mag_y_scale, 0.1f, 10.0f) &&
        MPU9250_Calibration_PositiveFloatIsValid(
            data->mag_z_scale, 0.1f, 10.0f));
}

static void MPU9250_Calibration_ResetActive(void)
{
    memset(&mpu9250_calibration_data, 0, sizeof(mpu9250_calibration_data));
    mpu9250_calibration_data.mag_x_scale = 1.0f;
    mpu9250_calibration_data.mag_y_scale = 1.0f;
    mpu9250_calibration_data.mag_z_scale = 1.0f;
    mpu9250_calibration_valid = 0U;
}

static MPU9250_CalibrationStatus_t MPU9250_Calibration_EraseSector(void)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0U;
    HAL_StatusTypeDef status;

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = MPU9250_CALIBRATION_FLASH_SECTOR;
    erase_init.NbSectors = 1U;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    return (status == HAL_OK)
        ? MPU9250_CALIBRATION_OK
        : MPU9250_CALIBRATION_FLASH_ERROR;
}

static void MPU9250_Calibration_SetExpectedGravity(
    MPU9250_GravityAxis_t gravity_axis,
    float *x,
    float *y,
    float *z)
{
    *x = 0.0f;
    *y = 0.0f;
    *z = 0.0f;

    switch (gravity_axis)
    {
        case MPU9250_GRAVITY_POSITIVE_X:
            *x = 1.0f;
            break;
        case MPU9250_GRAVITY_NEGATIVE_X:
            *x = -1.0f;
            break;
        case MPU9250_GRAVITY_POSITIVE_Y:
            *y = 1.0f;
            break;
        case MPU9250_GRAVITY_NEGATIVE_Y:
            *y = -1.0f;
            break;
        case MPU9250_GRAVITY_POSITIVE_Z:
            *z = 1.0f;
            break;
        case MPU9250_GRAVITY_NEGATIVE_Z:
            *z = -1.0f;
            break;
        default:
            break;
    }
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_Init(void)
{
    MPU9250_Calibration_ResetActive();
    return MPU9250_Calibration_Load();
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_Load(void)
{
    MPU9250_CalibrationBlob_t blob;
    const void *flash_data =
        (const void *)(uintptr_t)MPU9250_CALIBRATION_FLASH_ADDRESS;
    uint32_t expected_crc;

    memcpy(&blob, flash_data, sizeof(blob));

    if (blob.magic == 0xFFFFFFFFUL)
    {
        MPU9250_Calibration_ResetActive();
        return MPU9250_CALIBRATION_NOT_FOUND;
    }

    expected_crc = MPU9250_Calibration_Crc32(
        (const uint8_t *)&blob,
        (uint32_t)(sizeof(blob) - sizeof(blob.crc32)));

    if (blob.magic != MPU9250_CALIBRATION_MAGIC ||
        blob.version != MPU9250_CALIBRATION_VERSION ||
        blob.size != sizeof(MPU9250_CalibrationData_t) ||
        blob.crc32 != expected_crc ||
        !MPU9250_Calibration_DataIsValid(&blob.data))
    {
        MPU9250_Calibration_ResetActive();
        return MPU9250_CALIBRATION_INVALID_DATA;
    }

    mpu9250_calibration_data = blob.data;
    mpu9250_calibration_valid = 1U;

    return MPU9250_CALIBRATION_OK;
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_Save(void)
{
    MPU9250_CalibrationBlob_t blob;
    MPU9250_CalibrationStatus_t calibration_status;
    HAL_StatusTypeDef hal_status;
    uint32_t offset;

    if (!mpu9250_calibration_valid ||
        !MPU9250_Calibration_DataIsValid(&mpu9250_calibration_data))
    {
        return MPU9250_CALIBRATION_INVALID_DATA;
    }

    blob.magic = MPU9250_CALIBRATION_MAGIC;
    blob.version = MPU9250_CALIBRATION_VERSION;
    blob.size = sizeof(MPU9250_CalibrationData_t);
    blob.data = mpu9250_calibration_data;
    blob.crc32 = MPU9250_Calibration_Crc32(
        (const uint8_t *)&blob,
        (uint32_t)(sizeof(blob) - sizeof(blob.crc32)));

    hal_status = HAL_FLASH_Unlock();

    if (hal_status != HAL_OK)
    {
        return MPU9250_CALIBRATION_FLASH_ERROR;
    }

    __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP |
        FLASH_FLAG_OPERR |
        FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR |
        FLASH_FLAG_PGPERR |
        FLASH_FLAG_PGSERR);

    calibration_status = MPU9250_Calibration_EraseSector();

    if (calibration_status == MPU9250_CALIBRATION_OK)
    {
        for (offset = 0U; offset < sizeof(blob); offset += sizeof(uint32_t))
        {
            uint32_t word;

            memcpy(
                &word,
                ((const uint8_t *)&blob) + offset,
                sizeof(word));

            hal_status = HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_WORD,
                MPU9250_CALIBRATION_FLASH_ADDRESS + offset,
                word);

            if (hal_status != HAL_OK)
            {
                calibration_status = MPU9250_CALIBRATION_FLASH_ERROR;
                break;
            }
        }
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        calibration_status = MPU9250_CALIBRATION_FLASH_ERROR;
    }

    if (calibration_status != MPU9250_CALIBRATION_OK)
    {
        return calibration_status;
    }

    return MPU9250_Calibration_Load();
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_Erase(void)
{
    MPU9250_CalibrationStatus_t calibration_status;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return MPU9250_CALIBRATION_FLASH_ERROR;
    }

    __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP |
        FLASH_FLAG_OPERR |
        FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR |
        FLASH_FLAG_PGPERR |
        FLASH_FLAG_PGSERR);

    calibration_status = MPU9250_Calibration_EraseSector();

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        calibration_status = MPU9250_CALIBRATION_FLASH_ERROR;
    }

    if (calibration_status == MPU9250_CALIBRATION_OK)
    {
        MPU9250_Calibration_ResetActive();
    }

    return calibration_status;
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_Calculate(
    uint16_t sample_count,
    uint32_t sample_delay_ms,
    MPU9250_GravityAxis_t gravity_axis)
{
    int64_t accel_x_sum = 0;
    int64_t accel_y_sum = 0;
    int64_t accel_z_sum = 0;
    int64_t gyro_x_sum = 0;
    int64_t gyro_y_sum = 0;
    int64_t gyro_z_sum = 0;
    float expected_x;
    float expected_y;
    float expected_z;
    float accel_sensitivity;
    float gyro_sensitivity;
    uint16_t sample;

    if (sample_count == 0U ||
        gravity_axis < MPU9250_GRAVITY_POSITIVE_X ||
        gravity_axis > MPU9250_GRAVITY_NEGATIVE_Z)
    {
        return MPU9250_CALIBRATION_INVALID_ARGUMENT;
    }

    accel_sensitivity = MPU9250_GetAccelSensitivity();
    gyro_sensitivity = MPU9250_GetGyroSensitivity();

    if (accel_sensitivity <= 0.0f || gyro_sensitivity <= 0.0f)
    {
        return MPU9250_CALIBRATION_INVALID_DATA;
    }

    for (sample = 0U; sample < sample_count; ++sample)
    {
        MPU9250_RawData_t raw_data;

        if (MPU9250_ReadRaw(&raw_data) != HAL_OK)
        {
            return MPU9250_CALIBRATION_SENSOR_ERROR;
        }

        accel_x_sum += raw_data.accel_x;
        accel_y_sum += raw_data.accel_y;
        accel_z_sum += raw_data.accel_z;
        gyro_x_sum += raw_data.gyro_x;
        gyro_y_sum += raw_data.gyro_y;
        gyro_z_sum += raw_data.gyro_z;

        if (sample_delay_ms > 0U)
        {
            HAL_Delay(sample_delay_ms);
        }
    }

    MPU9250_Calibration_SetExpectedGravity(
        gravity_axis,
        &expected_x,
        &expected_y,
        &expected_z);

    mpu9250_calibration_data.accel_x_offset_g =
        ((float)accel_x_sum / (float)sample_count / accel_sensitivity) -
        expected_x;
    mpu9250_calibration_data.accel_y_offset_g =
        ((float)accel_y_sum / (float)sample_count / accel_sensitivity) -
        expected_y;
    mpu9250_calibration_data.accel_z_offset_g =
        ((float)accel_z_sum / (float)sample_count / accel_sensitivity) -
        expected_z;
    mpu9250_calibration_data.gyro_x_offset_dps =
        (float)gyro_x_sum / (float)sample_count / gyro_sensitivity;
    mpu9250_calibration_data.gyro_y_offset_dps =
        (float)gyro_y_sum / (float)sample_count / gyro_sensitivity;
    mpu9250_calibration_data.gyro_z_offset_dps =
        (float)gyro_z_sum / (float)sample_count / gyro_sensitivity;

    if (!MPU9250_Calibration_DataIsValid(&mpu9250_calibration_data))
    {
        MPU9250_Calibration_ResetActive();
        return MPU9250_CALIBRATION_INVALID_DATA;
    }

    mpu9250_calibration_valid = 1U;

    return MPU9250_CALIBRATION_OK;
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_CalculateAndSave(
    uint16_t sample_count,
    uint32_t sample_delay_ms,
    MPU9250_GravityAxis_t gravity_axis)
{
    MPU9250_CalibrationStatus_t status;

    status = MPU9250_Calibration_Calculate(
        sample_count,
        sample_delay_ms,
        gravity_axis);

    if (status != MPU9250_CALIBRATION_OK)
    {
        return status;
    }

    return MPU9250_Calibration_Save();
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_CalculateMagnetometer(
    uint16_t sample_count,
    uint32_t sample_delay_ms)
{
    float mag_x_min = FLT_MAX;
    float mag_y_min = FLT_MAX;
    float mag_z_min = FLT_MAX;
    float mag_x_max = -FLT_MAX;
    float mag_y_max = -FLT_MAX;
    float mag_z_max = -FLT_MAX;
    float mag_x_radius;
    float mag_y_radius;
    float mag_z_radius;
    float average_radius;
    uint16_t sample;

    if (sample_count < 2U)
    {
        return MPU9250_CALIBRATION_INVALID_ARGUMENT;
    }

    for (sample = 0U; sample < sample_count; ++sample)
    {
        MPU9250_Data_t data;

        if (MPU9250_Read(&data) != HAL_OK)
        {
            return MPU9250_CALIBRATION_SENSOR_ERROR;
        }

        if (data.mag_x_uT < mag_x_min)
        {
            mag_x_min = data.mag_x_uT;
        }
        if (data.mag_x_uT > mag_x_max)
        {
            mag_x_max = data.mag_x_uT;
        }
        if (data.mag_y_uT < mag_y_min)
        {
            mag_y_min = data.mag_y_uT;
        }
        if (data.mag_y_uT > mag_y_max)
        {
            mag_y_max = data.mag_y_uT;
        }
        if (data.mag_z_uT < mag_z_min)
        {
            mag_z_min = data.mag_z_uT;
        }
        if (data.mag_z_uT > mag_z_max)
        {
            mag_z_max = data.mag_z_uT;
        }

        if (sample_delay_ms > 0U)
        {
            HAL_Delay(sample_delay_ms);
        }
    }

    mag_x_radius = (mag_x_max - mag_x_min) * 0.5f;
    mag_y_radius = (mag_y_max - mag_y_min) * 0.5f;
    mag_z_radius = (mag_z_max - mag_z_min) * 0.5f;

    if (mag_x_radius < (MPU9250_CALIBRATION_MIN_MAG_SPAN_UT * 0.5f) ||
        mag_y_radius < (MPU9250_CALIBRATION_MIN_MAG_SPAN_UT * 0.5f) ||
        mag_z_radius < (MPU9250_CALIBRATION_MIN_MAG_SPAN_UT * 0.5f))
    {
        return MPU9250_CALIBRATION_INVALID_DATA;
    }

    average_radius = (mag_x_radius + mag_y_radius + mag_z_radius) / 3.0f;

    mpu9250_calibration_data.mag_x_offset_uT =
        (mag_x_max + mag_x_min) * 0.5f;
    mpu9250_calibration_data.mag_y_offset_uT =
        (mag_y_max + mag_y_min) * 0.5f;
    mpu9250_calibration_data.mag_z_offset_uT =
        (mag_z_max + mag_z_min) * 0.5f;
    mpu9250_calibration_data.mag_x_scale =
        average_radius / mag_x_radius;
    mpu9250_calibration_data.mag_y_scale =
        average_radius / mag_y_radius;
    mpu9250_calibration_data.mag_z_scale =
        average_radius / mag_z_radius;

    if (!MPU9250_Calibration_DataIsValid(&mpu9250_calibration_data))
    {
        MPU9250_Calibration_ResetActive();
        return MPU9250_CALIBRATION_INVALID_DATA;
    }

    mpu9250_calibration_valid = 1U;

    return MPU9250_CALIBRATION_OK;
}

MPU9250_CalibrationStatus_t
MPU9250_Calibration_CalculateMagnetometerAndSave(
    uint16_t sample_count,
    uint32_t sample_delay_ms)
{
    MPU9250_CalibrationStatus_t status;

    status = MPU9250_Calibration_CalculateMagnetometer(
        sample_count,
        sample_delay_ms);

    if (status != MPU9250_CALIBRATION_OK)
    {
        return status;
    }

    return MPU9250_Calibration_Save();
}

HAL_StatusTypeDef MPU9250_Calibration_Read(MPU9250_Data_t *data)
{
    HAL_StatusTypeDef status;

    status = MPU9250_Read(data);

    if (status == HAL_OK)
    {
        MPU9250_Calibration_Apply(data);
    }

    return status;
}

void MPU9250_Calibration_Apply(MPU9250_Data_t *data)
{
    if (data == NULL || !mpu9250_calibration_valid)
    {
        return;
    }

    data->accel_x_g -= mpu9250_calibration_data.accel_x_offset_g;
    data->accel_y_g -= mpu9250_calibration_data.accel_y_offset_g;
    data->accel_z_g -= mpu9250_calibration_data.accel_z_offset_g;
    data->gyro_x_dps -= mpu9250_calibration_data.gyro_x_offset_dps;
    data->gyro_y_dps -= mpu9250_calibration_data.gyro_y_offset_dps;
    data->gyro_z_dps -= mpu9250_calibration_data.gyro_z_offset_dps;
    data->mag_x_uT =
        (data->mag_x_uT - mpu9250_calibration_data.mag_x_offset_uT) *
        mpu9250_calibration_data.mag_x_scale;
    data->mag_y_uT =
        (data->mag_y_uT - mpu9250_calibration_data.mag_y_offset_uT) *
        mpu9250_calibration_data.mag_y_scale;
    data->mag_z_uT =
        (data->mag_z_uT - mpu9250_calibration_data.mag_z_offset_uT) *
        mpu9250_calibration_data.mag_z_scale;
}

void MPU9250_Calibration_Get(MPU9250_CalibrationData_t *data)
{
    if (data != NULL)
    {
        *data = mpu9250_calibration_data;
    }
}

MPU9250_CalibrationStatus_t MPU9250_Calibration_Set(
    const MPU9250_CalibrationData_t *data)
{
    if (!MPU9250_Calibration_DataIsValid(data))
    {
        return MPU9250_CALIBRATION_INVALID_ARGUMENT;
    }

    mpu9250_calibration_data = *data;
    mpu9250_calibration_valid = 1U;

    return MPU9250_CALIBRATION_OK;
}

uint8_t MPU9250_Calibration_HasValidData(void)
{
    return mpu9250_calibration_valid;
}
