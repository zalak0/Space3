#include "icm20948.h"
#include <math.h>
#include <string.h>

/* ── Active device address (found during Init) ─────────────────── */
static uint16_t s_dev_addr = 0;

/* ── Timeout for all HAL calls (ms) ───────────────────────────── */
#define I2C_TIMEOUT 10

/* ════════════════════════════════════════════════════════════════
   Low-level helpers
   ════════════════════════════════════════════════════════════════ */

/**
 * Write one byte to a register in the currently selected user bank.
 */
static bool reg_write(I2C_HandleTypeDef *hi2c, uint16_t dev,
                      uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(hi2c, dev, buf, 2, I2C_TIMEOUT) == HAL_OK;
}

/**
 * Read `len` bytes starting at `reg`.
 */
static bool reg_read(I2C_HandleTypeDef *hi2c, uint16_t dev,
                     uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(hi2c, dev, &reg, 1, I2C_TIMEOUT) != HAL_OK)
        return false;
    return HAL_I2C_Master_Receive(hi2c, dev, buf, len, I2C_TIMEOUT) == HAL_OK;
}

/**
 * Switch between ICM user banks (0-3).
 * REG_BANK_SEL is at 0x7F in every bank.
 */
static bool select_bank(I2C_HandleTypeDef *hi2c, uint8_t bank)
{
    return reg_write(hi2c, s_dev_addr, 0x7F, (uint8_t)(bank << 4));
}

/* ════════════════════════════════════════════════════════════════
   Magnetometer helpers  (AK09916 is on its own I2C address)
   ════════════════════════════════════════════════════════════════ */

static bool mag_write(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return reg_write(hi2c, AK09916_ADDR, reg, val);
}

static bool mag_read(I2C_HandleTypeDef *hi2c,
                     uint8_t reg, uint8_t *buf, uint16_t len)
{
    return reg_read(hi2c, AK09916_ADDR, reg, buf, len);
}

/* ════════════════════════════════════════════════════════════════
   ICM20948_Init
   ════════════════════════════════════════════════════════════════ */

bool ICM20948_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0;

    /* ── 1. Find the device ────────────────────────────────────── */
    uint16_t candidates[2] = { ICM20948_ADDR_HIGH, ICM20948_ADDR_LOW };
    bool found = false;

    for (int i = 0; i < 2; i++) {
        s_dev_addr = candidates[i];
        /* Must be in bank 0 to read WHO_AM_I */
        reg_write(hi2c, s_dev_addr, 0x7F, 0x00); /* select bank 0 */
        if (reg_read(hi2c, s_dev_addr, ICM_WHO_AM_I, &who, 1) &&
            who == ICM_WHO_AM_I_VAL)
        {
            found = true;
            break;
        }
    }

    if (!found) return false;

    /* ── 2. Reset & wake ───────────────────────────────────────── */
    select_bank(hi2c, 0);
    reg_write(hi2c, s_dev_addr, ICM_PWR_MGMT_1, 0x80); /* device reset */
    HAL_Delay(100);
    /* Auto-select best clock, clear sleep */
    reg_write(hi2c, s_dev_addr, ICM_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    /* Enable accel + gyro */
    reg_write(hi2c, s_dev_addr, ICM_PWR_MGMT_2, 0x00);

    /* ── 3. Gyro config (Bank 2): ±500 dps, DLPF on ───────────── */
    select_bank(hi2c, 2);
    /*  GYRO_CONFIG_1: GYRO_FS_SEL=01 (500 dps), GYRO_FCHOICE=1, DLPFCFG=3 */
    reg_write(hi2c, s_dev_addr, ICM_GYRO_CONFIG_1, 0x0F);

    /* ── 4. Accel config (Bank 2): ±4 g, DLPF on ──────────────── */
    /*  ACCEL_CONFIG: ACCEL_FS_SEL=01 (4g), ACCEL_FCHOICE=1, DLPFCFG=3 */
    reg_write(hi2c, s_dev_addr, ICM_ACCEL_CONFIG, 0x09);

    /* ── 5. Back to bank 0 ─────────────────────────────────────── */
    select_bank(hi2c, 0);

    /* ── 6. Enable I2C master for magnetometer ─────────────────── */
    /*
     * The AK09916 is connected internally to the ICM's auxiliary I2C bus,
     * BUT on most breakout boards (SparkFun, Adafruit) it's also exposed
     * on the main I2C bus at 0x0C.
     *
     * Strategy used here: talk to AK09916 directly on the main bus
     * (same approach as Adafruit's driver). If your board routes it
     * only through the ICM master, you'll need I2C master passthrough
     * instead — let me know if that's the case.
     */
    uint8_t ak_who = 0;
    if (!mag_read(hi2c, AK09916_WIA2, &ak_who, 1) || ak_who != 0x09) {
        /*
         * Magnetometer not visible on main bus.
         * Return true anyway so accel/gyro/temp still work —
         * mag fields will read 0.
         */
        return true;
    }

    /* Soft-reset the magnetometer */
    mag_write(hi2c, AK09916_CNTL3, 0x01);
    HAL_Delay(10);

    /* Continuous measurement mode 4 = 100 Hz */
    mag_write(hi2c, AK09916_CNTL2, AK09916_CONT_MODE_100HZ);
    HAL_Delay(10);

    return true;
}

/* ════════════════════════════════════════════════════════════════
   ICM20948_Read
   ════════════════════════════════════════════════════════════════ */

bool ICM20948_Read(I2C_HandleTypeDef *hi2c, ICM20948_Data *out)
{
    memset(out, 0, sizeof(*out));
    out->valid = false;

    select_bank(hi2c, 0);

    /* ── Accel (6 bytes) ───────────────────────────────────────── */
    uint8_t raw[6];
    if (!reg_read(hi2c, s_dev_addr, ICM_ACCEL_XOUT_H, raw, 6))
        return false;

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

    /* ±4 g range → sensitivity = 8192 LSB/g */
    const float ACCEL_SCALE = 9.80665f / 8192.0f;
    out->accel_x = ax * ACCEL_SCALE;
    out->accel_y = ay * ACCEL_SCALE;
    out->accel_z = az * ACCEL_SCALE;

    /* ── Gyro (6 bytes) ────────────────────────────────────────── */
    if (!reg_read(hi2c, s_dev_addr, ICM_GYRO_XOUT_H, raw, 6))
        return false;

    int16_t gx = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t gy = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t gz = (int16_t)((raw[4] << 8) | raw[5]);

    /* ±500 dps range → sensitivity = 65.5 LSB/dps → convert to rad/s */
    const float GYRO_SCALE = (1.0f / 65.5f) * (3.14159265f / 180.0f);
    out->gyro_x = gx * GYRO_SCALE;
    out->gyro_y = gy * GYRO_SCALE;
    out->gyro_z = gz * GYRO_SCALE;

    /* ── Temperature (2 bytes) ─────────────────────────────────── */
    if (!reg_read(hi2c, s_dev_addr, ICM_TEMP_OUT_H, raw, 2))
        return false;

    int16_t t_raw = (int16_t)((raw[0] << 8) | raw[1]);
    /* From datasheet: Temp_degC = ((TEMP_OUT – RoomTemp_Offset) / Temp_Sensitivity) + 21 */
    out->temp_c = ((float)t_raw / 333.87f) + 21.0f;

    /* ── Magnetometer ──────────────────────────────────────────── */
    uint8_t st1 = 0;
    if (mag_read(hi2c, AK09916_ST1, &st1, 1) && (st1 & 0x01)) {
        uint8_t mag_raw[6];
        if (mag_read(hi2c, AK09916_HXL, mag_raw, 6)) {
            int16_t mx = (int16_t)((mag_raw[1] << 8) | mag_raw[0]); /* little-endian */
            int16_t my = (int16_t)((mag_raw[3] << 8) | mag_raw[2]);
            int16_t mz = (int16_t)((mag_raw[5] << 8) | mag_raw[4]);

            /* AK09916 sensitivity: 0.15 µT/LSB */
            const float MAG_SCALE = 0.15f;
            out->mag_x = mx * MAG_SCALE;
            out->mag_y = my * MAG_SCALE;
            out->mag_z = mz * MAG_SCALE;
        }
    }

    out->valid = true;
    return true;
}
