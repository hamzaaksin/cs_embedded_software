#include "mpu9250.h"

/*
 * 0: MPU6500 / magnetometresiz deneme karti
 * 1: Gercek MPU9250 + AK8963
 *
 * Mevcut kart icin 0 birak. Gercek MPU9250 geldiginde yalnizca 1 yap.
 */
#ifndef MPU9250_USE_MAGNETOMETER
#define MPU9250_USE_MAGNETOMETER 0
#endif

#if ((MPU9250_USE_MAGNETOMETER != 0) && \
     (MPU9250_USE_MAGNETOMETER != 1))
#error "MPU9250_USE_MAGNETOMETER must be 0 or 1"
#endif

#define MPU9250_REG_SMPLRT_DIV 0x19U
#define MPU9250_REG_CONFIG 0x1AU
#define MPU9250_REG_GYRO_CONFIG 0x1BU
#define MPU9250_REG_ACCEL_CONFIG 0x1CU
#define MPU9250_REG_ACCEL_CONFIG_2 0x1DU
#define MPU9250_REG_INT_PIN_CFG 0x37U
#define MPU9250_REG_ACCEL_XOUT_H 0x3BU
#define MPU9250_REG_USER_CTRL 0x6AU
#define MPU9250_REG_PWR_MGMT_1 0x6BU
#define MPU9250_REG_WHO_AM_I 0x75U

#define AK8963_I2C_ADDRESS (0x0CU << 1U)
#define AK8963_REG_WHO_AM_I 0x00U
#define AK8963_REG_STATUS_1 0x02U
#define AK8963_REG_DATA_X_LOW 0x03U
#define AK8963_REG_STATUS_2 0x09U
#define AK8963_REG_CONTROL_1 0x0AU
#define AK8963_REG_SENSITIVITY_X 0x10U

#define AK8963_WHO_AM_I_VALUE 0x48U
#define AK8963_STATUS_1_DATA_READY 0x01U
#define AK8963_STATUS_2_OVERFLOW 0x08U
#define AK8963_MODE_POWER_DOWN 0x00U
#define AK8963_MODE_FUSE_ROM 0x0FU
#define AK8963_MODE_CONTINUOUS_100HZ_16BIT 0x16U
#define AK8963_16BIT_SENSITIVITY_UT_PER_LSB 0.15f

#define MPU9250_DMA_PHASE_IDLE 0U
#define MPU9250_DMA_PHASE_MPU9250 1U
#define MPU9250_DMA_PHASE_AK8963 2U

extern I2C_HandleTypeDef hi2c1;

static float mpu9250_accel_sensitivity = 16384.0f;
static float mpu9250_gyro_sensitivity = 131.0f;
static float ak8963_sensitivity_adjustment[3] = { 1.0f, 1.0f, 1.0f };
static uint8_t mpu9250_rx_buffer[14];
static uint8_t ak8963_rx_buffer[8];
static int16_t ak8963_last_mag_x = 0;
static int16_t ak8963_last_mag_y = 0;
static int16_t ak8963_last_mag_z = 0;
static volatile uint8_t mpu9250_rx_busy = 0U;
static volatile uint8_t mpu9250_data_ready = 0U;
static volatile uint8_t mpu9250_dma_phase = MPU9250_DMA_PHASE_IDLE;
static volatile uint32_t mpu9250_callback_count = 0U;

static HAL_StatusTypeDef MPU9250_WriteRegister(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1,
	MPU9250_I2C_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, &value, 1U,
	MPU9250_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU9250_ReadRegister(uint8_t reg, uint8_t *value) {
	return HAL_I2C_Mem_Read(&hi2c1,
	MPU9250_I2C_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, value, 1U,
	MPU9250_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef AK8963_WriteRegister(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1,
	AK8963_I2C_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, &value, 1U,
	MPU9250_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef AK8963_ReadRegisters(uint8_t reg, uint8_t *buffer,
		uint16_t size) {
	return HAL_I2C_Mem_Read(&hi2c1,
	AK8963_I2C_ADDRESS, reg,
	I2C_MEMADD_SIZE_8BIT, buffer, size,
	MPU9250_I2C_TIMEOUT_MS);
}

static int16_t MPU9250_CombineBytes(uint8_t high_byte, uint8_t low_byte) {
	uint16_t combined = ((uint16_t) high_byte << 8U) | (uint16_t) low_byte;
	return (int16_t) combined;
}

static int16_t AK8963_CombineBytes(uint8_t low_byte, uint8_t high_byte) {
	uint16_t combined = ((uint16_t) high_byte << 8U) | (uint16_t) low_byte;
	return (int16_t) combined;
}

static float MPU9250_AccelSensitivity(MPU9250_AccelRange_t range) {
	switch (range) {
	case MPU9250_ACCEL_RANGE_2G:
		return 16384.0f;
	case MPU9250_ACCEL_RANGE_4G:
		return 8192.0f;
	case MPU9250_ACCEL_RANGE_8G:
		return 4096.0f;
	case MPU9250_ACCEL_RANGE_16G:
		return 2048.0f;
	default:
		return 0.0f;
	}
}

static float MPU9250_GyroSensitivity(MPU9250_GyroRange_t range) {
	switch (range) {
	case MPU9250_GYRO_RANGE_250DPS:
		return 131.0f;
	case MPU9250_GYRO_RANGE_500DPS:
		return 65.5f;
	case MPU9250_GYRO_RANGE_1000DPS:
		return 32.8f;
	case MPU9250_GYRO_RANGE_2000DPS:
		return 16.4f;
	default:
		return 0.0f;
	}
}

static HAL_StatusTypeDef AK8963_Init(void) {
	uint8_t who_am_i = 0U;
	uint8_t sensitivity[3];
	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(&hi2c1,
	AK8963_I2C_ADDRESS, 3U,
	MPU9250_I2C_TIMEOUT_MS);

	if (status != HAL_OK) {
		return status;
	}

	status = AK8963_ReadRegisters(AK8963_REG_WHO_AM_I, &who_am_i, 1U);

	if (status != HAL_OK) {
		return status;
	}

	if (who_am_i != AK8963_WHO_AM_I_VALUE) {
		return HAL_ERROR;
	}

	status = AK8963_WriteRegister(
	AK8963_REG_CONTROL_1,
	AK8963_MODE_POWER_DOWN);

	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(10U);

	status = AK8963_WriteRegister(
	AK8963_REG_CONTROL_1,
	AK8963_MODE_FUSE_ROM);

	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(10U);

	status = AK8963_ReadRegisters(
	AK8963_REG_SENSITIVITY_X, sensitivity, sizeof(sensitivity));

	if (status != HAL_OK) {
		return status;
	}

	ak8963_sensitivity_adjustment[0] = (((float) sensitivity[0] - 128.0f)
			/ 256.0f) + 1.0f;

	ak8963_sensitivity_adjustment[1] = (((float) sensitivity[1] - 128.0f)
			/ 256.0f) + 1.0f;

	ak8963_sensitivity_adjustment[2] = (((float) sensitivity[2] - 128.0f)
			/ 256.0f) + 1.0f;

	status = AK8963_WriteRegister(
	AK8963_REG_CONTROL_1,
	AK8963_MODE_POWER_DOWN);

	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(10U);

	status = AK8963_WriteRegister(
	AK8963_REG_CONTROL_1,
	AK8963_MODE_CONTINUOUS_100HZ_16BIT);

	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(10U);

	return HAL_OK;
}

static HAL_StatusTypeDef AK8963_ReadRaw(int16_t *mag_x, int16_t *mag_y,
		int16_t *mag_z) {
	uint8_t status_1 = 0U;
	uint8_t buffer[7];
	uint32_t start_tick = HAL_GetTick();
	HAL_StatusTypeDef status;

	do {
		status = AK8963_ReadRegisters(
		AK8963_REG_STATUS_1, &status_1, 1U);

		if (status != HAL_OK) {
			return status;
		}

		if ((status_1 & AK8963_STATUS_1_DATA_READY) != 0U) {
			break;
		}

		if ((HAL_GetTick() - start_tick) >= MPU9250_I2C_TIMEOUT_MS) {
			return HAL_TIMEOUT;
		}

		HAL_Delay(1U);
	} while (1);

	status = AK8963_ReadRegisters(
	AK8963_REG_DATA_X_LOW, buffer, sizeof(buffer));

	if (status != HAL_OK) {
		return status;
	}

	if ((buffer[6] & AK8963_STATUS_2_OVERFLOW) != 0U) {
		return HAL_ERROR;
	}

	*mag_x = AK8963_CombineBytes(buffer[0], buffer[1]);
	*mag_y = AK8963_CombineBytes(buffer[2], buffer[3]);
	*mag_z = AK8963_CombineBytes(buffer[4], buffer[5]);

	return HAL_OK;
}

HAL_StatusTypeDef MPU9250_IsReady(void) {
	uint8_t who_am_i = 0U;
	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(&hi2c1,
	MPU9250_I2C_ADDRESS, 3U,
	MPU9250_I2C_TIMEOUT_MS);

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_ReadRegister(MPU9250_REG_WHO_AM_I, &who_am_i);

	if (status != HAL_OK) {
		return status;
	}

#if MPU9250_USE_MAGNETOMETER
    /*
     * Magnetometreli modda ana sensorun gercek MPU9250 olmasini iste.
     * MPU9250 WHO_AM_I degeri 0x71'dir.
     */
    if (who_am_i == MPU9250_WHO_AM_I_VALUE)
#else
	/*
	 * Magnetometresiz modda mevcut 0x70 kimlikli karti da kabul et.
	 */
	if ((who_am_i == MPU9250_WHO_AM_I_VALUE) || (who_am_i == 0x70U))
#endif
			{
		return HAL_OK;
	}

	return HAL_ERROR;
}

HAL_StatusTypeDef MPU9250_Init(void) {
	MPU9250_Config_t config;

	config.accel_range = MPU9250_ACCEL_RANGE_2G;
	config.gyro_range = MPU9250_GYRO_RANGE_250DPS;
	config.sample_rate_divider = 9U;
	config.digital_low_pass_filter = 3U;

	return MPU9250_InitWithConfig(&config);
}

HAL_StatusTypeDef MPU9250_InitWithConfig(const MPU9250_Config_t *config) {
	HAL_StatusTypeDef status;
	float accel_sensitivity;
	float gyro_sensitivity;

	if (config == NULL || config->digital_low_pass_filter > 7U) {
		return HAL_ERROR;
	}

	accel_sensitivity = MPU9250_AccelSensitivity(config->accel_range);
	gyro_sensitivity = MPU9250_GyroSensitivity(config->gyro_range);

	if (accel_sensitivity == 0.0f || gyro_sensitivity == 0.0f) {
		return HAL_ERROR;
	}

	status = MPU9250_IsReady();

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_WriteRegister(MPU9250_REG_PWR_MGMT_1, 0x80U);

	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(100U);

	status = MPU9250_WriteRegister(MPU9250_REG_PWR_MGMT_1, 0x01U);

	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(10U);

	status = MPU9250_WriteRegister(
	MPU9250_REG_SMPLRT_DIV, config->sample_rate_divider);

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_WriteRegister(
	MPU9250_REG_CONFIG, config->digital_low_pass_filter & 0x07U);

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_WriteRegister(
	MPU9250_REG_ACCEL_CONFIG_2, config->digital_low_pass_filter & 0x07U);

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_WriteRegister(
	MPU9250_REG_ACCEL_CONFIG, ((uint8_t) config->accel_range & 0x03U) << 3U);

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_WriteRegister(
	MPU9250_REG_GYRO_CONFIG, ((uint8_t) config->gyro_range & 0x03U) << 3U);

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_WriteRegister(MPU9250_REG_USER_CTRL, 0x00U);

	if (status != HAL_OK) {
		return status;
	}

	status = MPU9250_WriteRegister(MPU9250_REG_INT_PIN_CFG, 0x02U);

	if (status != HAL_OK) {
		return status;
	}

	HAL_Delay(100U);

#if MPU9250_USE_MAGNETOMETER
    status = AK8963_Init();

    if (status != HAL_OK)
    {
        return status;
    }
#endif

	mpu9250_accel_sensitivity = accel_sensitivity;
	mpu9250_gyro_sensitivity = gyro_sensitivity;

	mpu9250_rx_busy = 0U;
	mpu9250_data_ready = 0U;
	mpu9250_dma_phase = MPU9250_DMA_PHASE_IDLE;

	ak8963_last_mag_x = 0;
	ak8963_last_mag_y = 0;
	ak8963_last_mag_z = 0;

	return HAL_OK;
}

HAL_StatusTypeDef MPU9250_ReadRaw(MPU9250_RawData_t *data) {
	uint8_t buffer[14];
	HAL_StatusTypeDef status;

	if (data == NULL) {
		return HAL_ERROR;
	}

	status = HAL_I2C_Mem_Read(&hi2c1,
	MPU9250_I2C_ADDRESS,
	MPU9250_REG_ACCEL_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, buffer, sizeof(buffer),
	MPU9250_I2C_TIMEOUT_MS);

	if (status != HAL_OK) {
		return status;
	}

	data->accel_x = MPU9250_CombineBytes(buffer[0], buffer[1]);
	data->accel_y = MPU9250_CombineBytes(buffer[2], buffer[3]);
	data->accel_z = MPU9250_CombineBytes(buffer[4], buffer[5]);
	data->temperature = MPU9250_CombineBytes(buffer[6], buffer[7]);
	data->gyro_x = MPU9250_CombineBytes(buffer[8], buffer[9]);
	data->gyro_y = MPU9250_CombineBytes(buffer[10], buffer[11]);
	data->gyro_z = MPU9250_CombineBytes(buffer[12], buffer[13]);

#if MPU9250_USE_MAGNETOMETER
    status = AK8963_ReadRaw(&data->mag_x, &data->mag_y, &data->mag_z);

    if (status != HAL_OK)
    {
        return status;
    }
#else
	data->mag_x = 0;
	data->mag_y = 0;
	data->mag_z = 0;
#endif

	return HAL_OK;
}

HAL_StatusTypeDef MPU9250_Read(MPU9250_Data_t *data) {
	MPU9250_RawData_t raw_data;
	HAL_StatusTypeDef status;

	if (data == NULL) {
		return HAL_ERROR;
	}

	status = MPU9250_ReadRaw(&raw_data);

	if (status != HAL_OK) {
		return status;
	}

	data->accel_x_g = (float) raw_data.accel_x / mpu9250_accel_sensitivity;
	data->accel_y_g = (float) raw_data.accel_y / mpu9250_accel_sensitivity;
	data->accel_z_g = (float) raw_data.accel_z / mpu9250_accel_sensitivity;
	data->temperature_c = ((float) raw_data.temperature / 333.87f) + 21.0f;
	data->gyro_x_dps = (float) raw_data.gyro_x / mpu9250_gyro_sensitivity;
	data->gyro_y_dps = (float) raw_data.gyro_y / mpu9250_gyro_sensitivity;
	data->gyro_z_dps = (float) raw_data.gyro_z / mpu9250_gyro_sensitivity;

	data->mag_x_uT = (float) raw_data.mag_x *
	AK8963_16BIT_SENSITIVITY_UT_PER_LSB * ak8963_sensitivity_adjustment[0];

	data->mag_y_uT = (float) raw_data.mag_y *
	AK8963_16BIT_SENSITIVITY_UT_PER_LSB * ak8963_sensitivity_adjustment[1];

	data->mag_z_uT = (float) raw_data.mag_z *
	AK8963_16BIT_SENSITIVITY_UT_PER_LSB * ak8963_sensitivity_adjustment[2];

	return HAL_OK;
}

HAL_StatusTypeDef MPU9250_StartReadDMA(void) {
	HAL_StatusTypeDef status;

	if (mpu9250_rx_busy || mpu9250_data_ready) {
		return HAL_BUSY;
	}

	mpu9250_rx_busy = 1U;
	mpu9250_dma_phase = MPU9250_DMA_PHASE_MPU9250;

	status = HAL_I2C_Mem_Read_DMA(&hi2c1,
	MPU9250_I2C_ADDRESS,
	MPU9250_REG_ACCEL_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, mpu9250_rx_buffer, sizeof(mpu9250_rx_buffer));

	if (status != HAL_OK) {
		mpu9250_rx_busy = 0U;
		mpu9250_dma_phase = MPU9250_DMA_PHASE_IDLE;
	}

	return status;
}

HAL_StatusTypeDef MPU9250_ProcessDMA(MPU9250_Data_t *data) {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temperature;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	if (data == NULL) {
		return HAL_ERROR;
	}

	if (!mpu9250_data_ready) {
		return HAL_BUSY;
	}

	accel_x = MPU9250_CombineBytes(mpu9250_rx_buffer[0], mpu9250_rx_buffer[1]);

	accel_y = MPU9250_CombineBytes(mpu9250_rx_buffer[2], mpu9250_rx_buffer[3]);

	accel_z = MPU9250_CombineBytes(mpu9250_rx_buffer[4], mpu9250_rx_buffer[5]);

	temperature = MPU9250_CombineBytes(mpu9250_rx_buffer[6],
			mpu9250_rx_buffer[7]);

	gyro_x = MPU9250_CombineBytes(mpu9250_rx_buffer[8], mpu9250_rx_buffer[9]);

	gyro_y = MPU9250_CombineBytes(mpu9250_rx_buffer[10], mpu9250_rx_buffer[11]);

	gyro_z = MPU9250_CombineBytes(mpu9250_rx_buffer[12], mpu9250_rx_buffer[13]);

	data->accel_x_g = (float) accel_x / mpu9250_accel_sensitivity;

	data->accel_y_g = (float) accel_y / mpu9250_accel_sensitivity;

	data->accel_z_g = (float) accel_z / mpu9250_accel_sensitivity;

	data->temperature_c = ((float) temperature / 333.87f) + 21.0f;

	data->gyro_x_dps = (float) gyro_x / mpu9250_gyro_sensitivity;

	data->gyro_y_dps = (float) gyro_y / mpu9250_gyro_sensitivity;

	data->gyro_z_dps = (float) gyro_z / mpu9250_gyro_sensitivity;

#if MPU9250_USE_MAGNETOMETER
    /*
     * Buffer sirasi:
     * ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2
     */
    if (((ak8963_rx_buffer[0] & AK8963_STATUS_1_DATA_READY) != 0U) &&
        ((ak8963_rx_buffer[7] & AK8963_STATUS_2_OVERFLOW) == 0U))
    {
        ak8963_last_mag_x = AK8963_CombineBytes(
            ak8963_rx_buffer[1],
            ak8963_rx_buffer[2]);

        ak8963_last_mag_y = AK8963_CombineBytes(
            ak8963_rx_buffer[3],
            ak8963_rx_buffer[4]);

        ak8963_last_mag_z = AK8963_CombineBytes(
            ak8963_rx_buffer[5],
            ak8963_rx_buffer[6]);
    }

    /*
     * AK8963 100 Hz calisir. Yeni veri yoksa son gecerli olcumu koru.
     */
    data->mag_x_uT =
        (float)ak8963_last_mag_x *
        AK8963_16BIT_SENSITIVITY_UT_PER_LSB *
        ak8963_sensitivity_adjustment[0];

    data->mag_y_uT =
        (float)ak8963_last_mag_y *
        AK8963_16BIT_SENSITIVITY_UT_PER_LSB *
        ak8963_sensitivity_adjustment[1];

    data->mag_z_uT =
        (float)ak8963_last_mag_z *
        AK8963_16BIT_SENSITIVITY_UT_PER_LSB *
        ak8963_sensitivity_adjustment[2];
#else
	/* Mevcut 0x70 kart icin eski 6 eksen davranisi. */
	data->mag_x_uT = 0.0f;
	data->mag_y_uT = 0.0f;
	data->mag_z_uT = 0.0f;
#endif

	mpu9250_data_ready = 0U;

	return HAL_OK;
}

float MPU9250_GetAccelSensitivity(void) {
	return mpu9250_accel_sensitivity;
}

float MPU9250_GetGyroSensitivity(void) {
	return mpu9250_gyro_sensitivity;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if ((hi2c == &hi2c1) && mpu9250_rx_busy) {
#if MPU9250_USE_MAGNETOMETER
        if (mpu9250_dma_phase == MPU9250_DMA_PHASE_MPU9250)
        {
            HAL_StatusTypeDef status;

            mpu9250_dma_phase = MPU9250_DMA_PHASE_AK8963;

            status = HAL_I2C_Mem_Read_DMA(
                &hi2c1,
                AK8963_I2C_ADDRESS,
                AK8963_REG_STATUS_1,
                I2C_MEMADD_SIZE_8BIT,
                ak8963_rx_buffer,
                sizeof(ak8963_rx_buffer));

            if (status != HAL_OK)
            {
                mpu9250_rx_busy = 0U;
                mpu9250_data_ready = 0U;
                mpu9250_dma_phase = MPU9250_DMA_PHASE_IDLE;
            }

            return;
        }

        if (mpu9250_dma_phase == MPU9250_DMA_PHASE_AK8963)
        {
            mpu9250_rx_busy = 0U;
            mpu9250_data_ready = 1U;
            mpu9250_dma_phase = MPU9250_DMA_PHASE_IDLE;
            mpu9250_callback_count++;
        }
#else
		/*
		 * Magnetometresiz mod: MPU'nun 14 byte DMA okumasi tamamlaninca
		 * veri hemen hazirdir.
		 */
		mpu9250_rx_busy = 0U;
		mpu9250_data_ready = 1U;
		mpu9250_dma_phase = MPU9250_DMA_PHASE_IDLE;
		mpu9250_callback_count++;
#endif
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		mpu9250_rx_busy = 0U;
		mpu9250_data_ready = 0U;
		mpu9250_dma_phase = MPU9250_DMA_PHASE_IDLE;
	}
}
