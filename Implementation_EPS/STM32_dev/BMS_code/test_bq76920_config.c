#include <stdio.h>
#include <stdint.h>
#include "bq76920_config.h"

/*
 * hi2c1 is defined in:
 * STM32_dev/Common/fake_hal_i2c.c
 */
extern I2C_HandleTypeDef hi2c1;

static BQ76920_HandleTypeDef bq76920;

/* ============================================================
 * TEST RESULT HELPERS
 * ============================================================ */

static void print_pass(const char *message)
{
    printf("[PASS] %s\n", message);
}

static void print_fail(const char *message)
{
    printf("[FAIL] %s\n", message);
}

static int check_hal_status(HAL_StatusTypeDef status, const char *message)
{
    if (status == HAL_OK)
    {
        print_pass(message);
        return 1;
    }

    print_fail(message);
    return 0;
}

static int check_register(uint8_t reg, uint8_t expected, const char *name)
{
    uint8_t value = 0;

    if (BQ76920_ReadReg(&bq76920, reg, &value) != HAL_OK)
    {
        printf("[FAIL] Could not read %s register 0x%02X\n", name, reg);
        return 0;
    }

    if (value == expected)
    {
        printf("[PASS] %s = 0x%02X\n", name, value);
        return 1;
    }

    printf("[FAIL] %s expected 0x%02X but got 0x%02X\n",
           name,
           expected,
           value);

    return 0;
}

/* ============================================================
 * MAIN TEST
 * ============================================================ */

int main(void)
{
    int tests_passed = 0;
    int tests_failed = 0;

    uint8_t reg_value = 0;
    int32_t cell_mV[5] = {0};
    uint16_t raw = 0;

    printf("\n========================================\n");
    printf("BQ76920 CONFIG TEST\n");
    printf("========================================\n\n");

    /*
     * Configure BQ76920 handle.
     * GPIOA / GPIO_PIN_5 is fake in the PC test environment.
     */
    bq76920.hi2c = &hi2c1;
    bq76920.ts1_boot_gpio_port = GPIOA;
    bq76920.ts1_boot_gpio_pin  = GPIO_PIN_5;
    bq76920.adc_gain_uV_per_lsb = 0;
    bq76920.adc_offset_mV = 0;

    /* ========================================================
     * TEST 1: I2C DEVICE READY
     * ======================================================== */

    printf("Test 1: I2C device ready\n");

    if (HAL_I2C_IsDeviceReady(&hi2c1,
                              BQ76920_I2C_ADDR_HAL,
                              3,
                              HAL_MAX_DELAY) == HAL_OK)
    {
        print_pass("BQ76920 responded on I2C address");
        tests_passed++;
    }
    else
    {
        print_fail("BQ76920 did not respond on I2C address");
        tests_failed++;
    }

    printf("\n");

    /* ========================================================
     * TEST 2: BOOT USING TS1 GPIO
     * ======================================================== */

    printf("Test 2: TS1 boot pulse\n");

    if (check_hal_status(BQ76920_Boot_TS1(&bq76920),
                         "BQ76920 TS1 boot pulse completed"))
    {
        tests_passed++;
    }
    else
    {
        tests_failed++;
    }

    printf("\n");

    /* ========================================================
     * TEST 3: REGISTER WRITE / READ
     * ======================================================== */

    printf("Test 3: Register write/read\n");

    if (BQ76920_WriteReg(&bq76920, BQ76920_REG_CC_CFG, 0x19) == HAL_OK &&
        BQ76920_ReadReg(&bq76920, BQ76920_REG_CC_CFG, &reg_value) == HAL_OK &&
        reg_value == 0x19)
    {
        print_pass("CC_CFG write/read works");
        tests_passed++;
    }
    else
    {
        print_fail("CC_CFG write/read failed");
        tests_failed++;
    }

    printf("\n");

    /* ========================================================
     * TEST 4: MONITORING-ONLY INITIALISATION
     * ======================================================== */

    printf("Test 4: Monitoring-only initialisation\n");

    if (BQ76920_Init_MonitoringOnly(&bq76920) == HAL_OK)
    {
        print_pass("BQ76920 monitoring-only init completed");
        tests_passed++;
    }
    else
    {
        print_fail("BQ76920 monitoring-only init failed");
        tests_failed++;
    }

    printf("\n");

    /* ========================================================
     * TEST 5: CHECK EXPECTED CONFIG REGISTERS
     * ======================================================== */

    printf("Test 5: Check config registers\n");

    if (check_register(BQ76920_REG_CC_CFG, 0x19, "CC_CFG"))
    {
        tests_passed++;
    }
    else
    {
        tests_failed++;
    }

    if (check_register(BQ76920_REG_SYS_CTRL1,
                       BQ76920_SYS_CTRL1_ADC_EN,
                       "SYS_CTRL1"))
    {
        tests_passed++;
    }
    else
    {
        tests_failed++;
    }

    if (check_register(BQ76920_REG_SYS_CTRL2, 0x00, "SYS_CTRL2"))
    {
        tests_passed++;
    }
    else
    {
        tests_failed++;
    }

    if (check_register(BQ76920_REG_CELLBAL1, 0x00, "CELLBAL1"))
    {
        tests_passed++;
    }
    else
    {
        tests_failed++;
    }

    printf("\n");

    /* ========================================================
     * TEST 6: READ CALIBRATION
     * ======================================================== */

    printf("Test 6: ADC calibration\n");

    if (BQ76920_ReadCalibration(&bq76920) == HAL_OK)
    {
        printf("[PASS] ADC calibration read\n");
        printf("       ADC gain   = %u uV/LSB\n", bq76920.adc_gain_uV_per_lsb);
        printf("       ADC offset = %d mV\n", bq76920.adc_offset_mV);
        tests_passed++;
    }
    else
    {
        print_fail("ADC calibration read failed");
        tests_failed++;
    }

    printf("\n");

    /* ========================================================
     * TEST 7: READ RAW CELL VALUES
     * ======================================================== */

    printf("Test 7: Raw cell ADC readings\n");

    for (uint8_t cell = 1; cell <= 5; cell++)
    {
        if (BQ76920_ReadCellRaw(&bq76920, cell, &raw) == HAL_OK)
        {
            printf("[PASS] Cell %u raw = %u\n", cell, raw);
            tests_passed++;
        }
        else
        {
            printf("[FAIL] Could not read Cell %u raw value\n", cell);
            tests_failed++;
        }
    }

    printf("\n");

    /* ========================================================
     * TEST 8: READ CELL VOLTAGES
     * ======================================================== */

    printf("Test 8: Cell voltage readings\n");

    if (BQ76920_ReadAllCells_mV(&bq76920, cell_mV) == HAL_OK)
    {
        printf("[PASS] Read all cell voltages\n");

        for (uint8_t i = 0; i < 5; i++)
        {
            printf("       Cell %u = %ld mV\n", i + 1, cell_mV[i]);
        }

        tests_passed++;
    }
    else
    {
        print_fail("Read all cell voltages failed");
        tests_failed++;
    }

    printf("\n");

    /* ========================================================
     * TEST SUMMARY
     * ======================================================== */

    printf("========================================\n");
    printf("BQ76920 CONFIG TEST SUMMARY\n");
    printf("========================================\n");
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_failed);

    if (tests_failed == 0)
    {
        printf("\nRESULT: ALL TESTS PASSED\n");
    }
    else
    {
        printf("\nRESULT: SOME TESTS FAILED\n");
    }

    printf("========================================\n\n");

    return tests_failed;
}