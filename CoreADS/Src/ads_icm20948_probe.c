#include "ads_icm20948_probe.h"

#define ADS_ICM20948_ADDR_PRIMARY_7BIT      (0x69u)
#define ADS_ICM20948_ADDR_SECONDARY_7BIT    (0x68u)

#define ADS_ICM20948_I2C_TRIALS             (3u)
#define ADS_ICM20948_I2C_TIMEOUT_MS         (100u)

#define ADS_ICM20948_REG_WHO_AM_I           (0x00u)
#define ADS_ICM20948_WHO_AM_I_EXPECTED      (0xEAu)

#define ADS_ICM20948_REG_BANK_SEL           (0x7Fu)
#define ADS_ICM20948_BANK_0                 (0x00u)

#define ADS_ICM20948_REG_PWR_MGMT_1         (0x06u)
#define ADS_ICM20948_PWR_MGMT_1_CLK_AUTO    (0x01u)

#define ADS_ICM20948_REG_ACCEL_XOUT_H       (0x2Du)
#define ADS_ICM20948_ACCEL_GYRO_RAW_LEN     (12u)

#define ADS_ICM20948_BANK_2                 (0x20u)

#define ADS_ICM20948_REG_GYRO_CONFIG_1      (0x01u)
#define ADS_ICM20948_REG_ACCEL_CONFIG       (0x14u)

#define ADS_ICM20948_GYRO_FS_500DPS         (0x02u)
#define ADS_ICM20948_ACCEL_FS_4G            (0x02u)

static ADS_ICM20948_ProbeDiagnostics g_icm20948_probe_diag =
{
    .result = ADS_ICM20948_PROBE_NOT_RUN,
    .detected_address_7bit = 0u,
    .hal_status_0x69 = HAL_ERROR,
    .hal_status_0x68 = HAL_ERROR,
    .probe_count = 0u
};

static ADS_ICM20948_WhoAmIDiagnostics g_icm20948_whoami_diag =
{
    .result = ADS_ICM20948_WHOAMI_NOT_RUN,
    .address_7bit = 0u,
    .who_am_i_value = 0u,
    .hal_status = HAL_ERROR,
    .read_count = 0u
};

static ADS_ICM20948_WakeDiagnostics g_icm20948_wake_diag =
{
    .result = ADS_ICM20948_WAKE_NOT_RUN,
    .address_7bit = 0u,
    .hal_status = HAL_ERROR,
    .wake_count = 0u
};

static ADS_ICM20948_RawDiagnostics g_icm20948_raw_diag =
{
    .result = ADS_ICM20948_RAW_NOT_RUN,
    .address_7bit = 0u,

    .accel_x_raw = 0,
    .accel_y_raw = 0,
    .accel_z_raw = 0,

    .gyro_x_raw = 0,
    .gyro_y_raw = 0,
    .gyro_z_raw = 0,

    .hal_status = HAL_ERROR,
    .read_count = 0u
};

static ADS_ICM20948_ConfigDiagnostics g_icm20948_config_diag =
{
    .result = ADS_ICM20948_CONFIG_NOT_RUN,
    .address_7bit = 0u,
    .hal_status = HAL_ERROR,
    .config_count = 0u
};

ADS_ICM20948_ProbeDiagnostics ADS_ICM20948_Probe_I2C(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status_0x69;
    HAL_StatusTypeDef status_0x68;

    g_icm20948_probe_diag.probe_count++;

    if (hi2c == NULL)
    {
        g_icm20948_probe_diag.result = ADS_ICM20948_PROBE_HAL_ERROR;
        g_icm20948_probe_diag.detected_address_7bit = 0u;
        return g_icm20948_probe_diag;
    }

    /*
     * STM32 HAL expects 7-bit I2C addresses shifted left by 1.
     */
    status_0x69 = HAL_I2C_IsDeviceReady(
        hi2c,
        (uint16_t)(ADS_ICM20948_ADDR_PRIMARY_7BIT << 1u),
        ADS_ICM20948_I2C_TRIALS,
        ADS_ICM20948_I2C_TIMEOUT_MS
    );

    g_icm20948_probe_diag.hal_status_0x69 = status_0x69;

    if (status_0x69 == HAL_OK)
    {
        g_icm20948_probe_diag.result = ADS_ICM20948_PROBE_FOUND_0X69;
        g_icm20948_probe_diag.detected_address_7bit = ADS_ICM20948_ADDR_PRIMARY_7BIT;
        return g_icm20948_probe_diag;
    }

    status_0x68 = HAL_I2C_IsDeviceReady(
        hi2c,
        (uint16_t)(ADS_ICM20948_ADDR_SECONDARY_7BIT << 1u),
        ADS_ICM20948_I2C_TRIALS,
        ADS_ICM20948_I2C_TIMEOUT_MS
    );

    g_icm20948_probe_diag.hal_status_0x68 = status_0x68;

    if (status_0x68 == HAL_OK)
    {
        g_icm20948_probe_diag.result = ADS_ICM20948_PROBE_FOUND_0X68;
        g_icm20948_probe_diag.detected_address_7bit = ADS_ICM20948_ADDR_SECONDARY_7BIT;
        return g_icm20948_probe_diag;
    }

    g_icm20948_probe_diag.result = ADS_ICM20948_PROBE_NOT_FOUND;
    g_icm20948_probe_diag.detected_address_7bit = 0u;

    return g_icm20948_probe_diag;
}

ADS_ICM20948_WhoAmIDiagnostics ADS_ICM20948_ReadWhoAmI(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
)
{
    uint8_t who_am_i_value = 0u;
    HAL_StatusTypeDef status;

    g_icm20948_whoami_diag.read_count++;
    g_icm20948_whoami_diag.address_7bit = address_7bit;
    g_icm20948_whoami_diag.who_am_i_value = 0u;

    if ((hi2c == NULL) || (address_7bit == 0u))
    {
        g_icm20948_whoami_diag.result = ADS_ICM20948_WHOAMI_HAL_ERROR;
        g_icm20948_whoami_diag.hal_status = HAL_ERROR;
        return g_icm20948_whoami_diag;
    }

    /*
     * STM32 HAL expects the 7-bit I2C address shifted left by 1.
     * ICM-20948 WHO_AM_I is in user bank 0 at register 0x00.
     */
    status = HAL_I2C_Mem_Read(
        hi2c,
        (uint16_t)(address_7bit << 1u),
        ADS_ICM20948_REG_WHO_AM_I,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i_value,
        1u,
        ADS_ICM20948_I2C_TIMEOUT_MS
    );

    g_icm20948_whoami_diag.hal_status = status;
    g_icm20948_whoami_diag.who_am_i_value = who_am_i_value;

    if (status != HAL_OK)
    {
        g_icm20948_whoami_diag.result = ADS_ICM20948_WHOAMI_HAL_ERROR;
        return g_icm20948_whoami_diag;
    }

    if (who_am_i_value == ADS_ICM20948_WHO_AM_I_EXPECTED)
    {
        g_icm20948_whoami_diag.result = ADS_ICM20948_WHOAMI_OK;
    }
    else
    {
        g_icm20948_whoami_diag.result = ADS_ICM20948_WHOAMI_BAD_VALUE;
    }

    return g_icm20948_whoami_diag;
}

static HAL_StatusTypeDef ADS_ICM20948_WriteReg(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit,
    uint8_t reg,
    uint8_t value
)
{
    return HAL_I2C_Mem_Write(
        hi2c,
        (uint16_t)(address_7bit << 1u),
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1u,
        ADS_ICM20948_I2C_TIMEOUT_MS
    );
}

static HAL_StatusTypeDef ADS_ICM20948_SelectBank0(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
)
{
    return ADS_ICM20948_WriteReg(
        hi2c,
        address_7bit,
        ADS_ICM20948_REG_BANK_SEL,
        ADS_ICM20948_BANK_0
    );
}

static HAL_StatusTypeDef ADS_ICM20948_SelectBank2(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
)
{
    return ADS_ICM20948_WriteReg(
        hi2c,
        address_7bit,
        ADS_ICM20948_REG_BANK_SEL,
        ADS_ICM20948_BANK_2
    );
}

static int16_t ADS_ICM20948_CombineI16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((uint16_t)((uint16_t)msb << 8u) | (uint16_t)lsb);
}

ADS_ICM20948_WakeDiagnostics ADS_ICM20948_Wake(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
)
{
    HAL_StatusTypeDef status;

    g_icm20948_wake_diag.wake_count++;
    g_icm20948_wake_diag.address_7bit = address_7bit;

    if ((hi2c == NULL) || (address_7bit == 0u))
    {
        g_icm20948_wake_diag.result = ADS_ICM20948_WAKE_HAL_ERROR;
        g_icm20948_wake_diag.hal_status = HAL_ERROR;
        return g_icm20948_wake_diag;
    }

    status = ADS_ICM20948_SelectBank0(hi2c, address_7bit);

    if (status != HAL_OK)
    {
        g_icm20948_wake_diag.result = ADS_ICM20948_WAKE_HAL_ERROR;
        g_icm20948_wake_diag.hal_status = status;
        return g_icm20948_wake_diag;
    }

    status = ADS_ICM20948_WriteReg(
        hi2c,
        address_7bit,
        ADS_ICM20948_REG_PWR_MGMT_1,
        ADS_ICM20948_PWR_MGMT_1_CLK_AUTO
    );

    g_icm20948_wake_diag.hal_status = status;

    if (status == HAL_OK)
    {
        g_icm20948_wake_diag.result = ADS_ICM20948_WAKE_OK;
    }
    else
    {
        g_icm20948_wake_diag.result = ADS_ICM20948_WAKE_HAL_ERROR;
    }

    return g_icm20948_wake_diag;
}

ADS_ICM20948_RawDiagnostics ADS_ICM20948_ReadAccelGyroRaw(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
)
{
    uint8_t raw[ADS_ICM20948_ACCEL_GYRO_RAW_LEN] = {0u};
    HAL_StatusTypeDef status;

    g_icm20948_raw_diag.read_count++;
    g_icm20948_raw_diag.address_7bit = address_7bit;

    if ((hi2c == NULL) || (address_7bit == 0u))
    {
        g_icm20948_raw_diag.result = ADS_ICM20948_RAW_BAD_ARG;
        g_icm20948_raw_diag.hal_status = HAL_ERROR;
        return g_icm20948_raw_diag;
    }

    status = ADS_ICM20948_SelectBank0(hi2c, address_7bit);

    if (status != HAL_OK)
    {
        g_icm20948_raw_diag.result = ADS_ICM20948_RAW_HAL_ERROR;
        g_icm20948_raw_diag.hal_status = status;
        return g_icm20948_raw_diag;
    }

    status = HAL_I2C_Mem_Read(
        hi2c,
        (uint16_t)(address_7bit << 1u),
        ADS_ICM20948_REG_ACCEL_XOUT_H,
        I2C_MEMADD_SIZE_8BIT,
        raw,
        ADS_ICM20948_ACCEL_GYRO_RAW_LEN,
        ADS_ICM20948_I2C_TIMEOUT_MS
    );

    g_icm20948_raw_diag.hal_status = status;

    if (status != HAL_OK)
    {
        g_icm20948_raw_diag.result = ADS_ICM20948_RAW_HAL_ERROR;
        return g_icm20948_raw_diag;
    }

    g_icm20948_raw_diag.accel_x_raw = ADS_ICM20948_CombineI16(raw[0], raw[1]);
    g_icm20948_raw_diag.accel_y_raw = ADS_ICM20948_CombineI16(raw[2], raw[3]);
    g_icm20948_raw_diag.accel_z_raw = ADS_ICM20948_CombineI16(raw[4], raw[5]);

    g_icm20948_raw_diag.gyro_x_raw = ADS_ICM20948_CombineI16(raw[6], raw[7]);
    g_icm20948_raw_diag.gyro_y_raw = ADS_ICM20948_CombineI16(raw[8], raw[9]);
    g_icm20948_raw_diag.gyro_z_raw = ADS_ICM20948_CombineI16(raw[10], raw[11]);

    g_icm20948_raw_diag.result = ADS_ICM20948_RAW_OK;

    return g_icm20948_raw_diag;
}

ADS_ICM20948_ConfigDiagnostics ADS_ICM20948_ConfigureAccelGyro(
    I2C_HandleTypeDef *hi2c,
    uint8_t address_7bit
)
{
    HAL_StatusTypeDef status;

    g_icm20948_config_diag.config_count++;
    g_icm20948_config_diag.address_7bit = address_7bit;

    if ((hi2c == NULL) || (address_7bit == 0u))
    {
        g_icm20948_config_diag.result = ADS_ICM20948_CONFIG_HAL_ERROR;
        g_icm20948_config_diag.hal_status = HAL_ERROR;
        return g_icm20948_config_diag;
    }

    status = ADS_ICM20948_SelectBank2(hi2c, address_7bit);

    if (status != HAL_OK)
    {
        g_icm20948_config_diag.result = ADS_ICM20948_CONFIG_HAL_ERROR;
        g_icm20948_config_diag.hal_status = status;
        return g_icm20948_config_diag;
    }

    status = ADS_ICM20948_WriteReg(
        hi2c,
        address_7bit,
        ADS_ICM20948_REG_GYRO_CONFIG_1,
        ADS_ICM20948_GYRO_FS_500DPS
    );

    if (status != HAL_OK)
    {
        (void)ADS_ICM20948_SelectBank0(hi2c, address_7bit);

        g_icm20948_config_diag.result = ADS_ICM20948_CONFIG_HAL_ERROR;
        g_icm20948_config_diag.hal_status = status;
        return g_icm20948_config_diag;
    }

    status = ADS_ICM20948_WriteReg(
        hi2c,
        address_7bit,
        ADS_ICM20948_REG_ACCEL_CONFIG,
        ADS_ICM20948_ACCEL_FS_4G
    );

    if (status != HAL_OK)
    {
        (void)ADS_ICM20948_SelectBank0(hi2c, address_7bit);

        g_icm20948_config_diag.result = ADS_ICM20948_CONFIG_HAL_ERROR;
        g_icm20948_config_diag.hal_status = status;
        return g_icm20948_config_diag;
    }

    status = ADS_ICM20948_SelectBank0(hi2c, address_7bit);

    g_icm20948_config_diag.hal_status = status;

    if (status == HAL_OK)
    {
        g_icm20948_config_diag.result = ADS_ICM20948_CONFIG_OK;
    }
    else
    {
        g_icm20948_config_diag.result = ADS_ICM20948_CONFIG_HAL_ERROR;
    }

    return g_icm20948_config_diag;
}