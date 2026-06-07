#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "main.h"
#include "fsw_ctx.h"
#include "eps.h"
#include "STM32_dev/Charger_Code/bq25798.h"
#include "STM32_dev/BMS_code/bq76920.h"

void FakeGPIO_SetPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState state);
void FakeBQ25798_Reset(void);
void FakeBQ25798_Set8(uint8_t reg, uint8_t value);
void FakeBQ25798_Set16(uint8_t reg, uint16_t value);
void FakeBQ76920_SetCellRaw(uint8_t cell_index, uint16_t raw14);
void FakeBQ76920_SetSysStat(uint8_t value);

static int tests_passed = 0;
static int tests_failed = 0;
static char popup_report[8192];

static void append_report(const char *line)
{
    snprintf(
        popup_report + strlen(popup_report),
        sizeof(popup_report) - strlen(popup_report),
        "%s\n",
        line
    );
}

static void assert_true(bool condition, const char *message)
{
    char line[256];

    snprintf(
        line,
        sizeof(line),
        "%-45s : %s",
        message,
        condition ? "PASS" : "FAIL"
    );

    printf("%s\n", line);
    append_report(line);

    if (condition) {
        tests_passed++;
    } else {
        tests_failed++;
    }
}

static void set_all_rails_good(void)
{
    FakeGPIO_SetPin(EPS_3V3_PG_GPIO_Port, EPS_3V3_PG_Pin, GPIO_PIN_SET);
    FakeGPIO_SetPin(EPS_3V3_FLT_GPIO_Port, EPS_3V3_FLT_Pin, GPIO_PIN_SET);

    FakeGPIO_SetPin(EPS_5V_PG_GPIO_Port, EPS_5V_PG_Pin, GPIO_PIN_SET);
    FakeGPIO_SetPin(EPS_5V_FLT_GPIO_Port, EPS_5V_FLT_Pin, GPIO_PIN_SET);

    FakeGPIO_SetPin(EPS_VBUS_PG_GPIO_Port, EPS_VBUS_PG_Pin, GPIO_PIN_SET);
    FakeGPIO_SetPin(EPS_VBUS_FLT_GPIO_Port, EPS_VBUS_FLT_Pin, GPIO_PIN_SET);
}

static void setup_charger_nominal(void)
{
    FakeBQ25798_Reset();

    FakeBQ25798_Set8(BQ25798_REG_PART_INFORMATION, 0x98);

    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_0, 0x00);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_1, 0x00);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_2, 0x00);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_3, 0x00);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_4, 0x00);

    FakeBQ25798_Set8(BQ25798_REG_FAULT_STATUS_0, 0x00);
    FakeBQ25798_Set8(BQ25798_REG_FAULT_STATUS_1, 0x00);

    FakeBQ25798_Set16(BQ25798_REG_VBUS_ADC, 5000);
    FakeBQ25798_Set16(BQ25798_REG_VBAT_ADC, 14800);
    FakeBQ25798_Set16(BQ25798_REG_VSYS_ADC, 5000);
    FakeBQ25798_Set16(BQ25798_REG_IBUS_ADC, 500);
    FakeBQ25798_Set16(BQ25798_REG_IBAT_ADC, 250);
    FakeBQ25798_Set16(BQ25798_REG_TDIE_ADC, 50);
}

static void setup_battery_nominal(void)
{
    FakeBQ76920_SetSysStat(0x00);

    for (uint8_t i = 0; i < 5; i++) {
        FakeBQ76920_SetCellRaw(i, 10137);
    }
}

static void setup_nominal_system(void)
{
    set_all_rails_good();
    setup_charger_nominal();
    setup_battery_nominal();
}

static void test_bq25798_telemetry(void)
{
    BQ25798_Telemetry t;
    bool ok;

    printf("\nTEST: BQ25798 telemetry\n");
    append_report("\nTEST: BQ25798 telemetry");

    setup_charger_nominal();

    ok = BQ25798_ReadTelemetry(&t);

    assert_true(ok, "BQ25798_ReadTelemetry returned true");
    assert_true(t.i2c_ok, "BQ25798 I2C OK");
    assert_true(t.vbus_v > 4.9f && t.vbus_v < 5.1f, "VBUS approx 5V");
    assert_true(t.vbat_v > 14.7f && t.vbat_v < 14.9f, "VBAT approx 14.8V");
}

static void test_bq76920_telemetry(void)
{
    BQ76920_Telemetry t;
    bool ok;

    printf("\nTEST: BQ76920 telemetry\n");
    append_report("\nTEST: BQ76920 telemetry");

    setup_battery_nominal();

    ok = BQ76920_Init();
    assert_true(ok, "BQ76920_Init returned true");

    ok = BQ76920_ReadTelemetry(&t);

    assert_true(ok, "BQ76920_ReadTelemetry returned true");
    assert_true(t.i2c_ok, "BQ76920 I2C OK");
    assert_true(t.cell_min_v > 3.6f && t.cell_min_v < 3.8f, "Cell min approx 3.7V");
}

static void test_eps_init(void)
{
    fsw_ctx_t ctx = {0};
    bool ok;

    printf("\nTEST: EPS init\n");
    append_report("\nTEST: EPS init");

    setup_nominal_system();

    ok = eps_init(&ctx);

    assert_true(ok, "eps_init returned true");
    assert_true(ctx.eps_ok, "EPS OK after init");
    assert_true(!ctx.eps_fault, "EPS fault clear after init");
}

static void test_eps_nominal(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: EPS nominal mode\n");
    append_report("\nTEST: EPS nominal mode");

    setup_nominal_system();

    eps_init(&ctx);
    eps_task(EPS_MODE_NOMINAL, &ctx);

    assert_true(ctx.eps_ok, "EPS OK in nominal mode");
    assert_true(!ctx.eps_fault, "No EPS fault in nominal mode");
    assert_true(!ctx.soc_low, "SOC not low");
}

static void test_eps_safe(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: EPS safe mode\n");
    append_report("\nTEST: EPS safe mode");

    setup_nominal_system();

    eps_init(&ctx);
    eps_task(EPS_MODE_SAFE, &ctx);

    assert_true(ctx.eps_ok, "EPS OK in safe mode");
    assert_true(!ctx.eps_fault, "No EPS fault just from safe mode");
}

static void test_eps_science(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: EPS science mode\n");
    append_report("\nTEST: EPS science mode");

    setup_nominal_system();

    eps_init(&ctx);
    eps_task(EPS_MODE_SCIENCE, &ctx);

    assert_true(ctx.eps_ok, "EPS OK in science mode");
    assert_true(!ctx.eps_fault, "No EPS fault in science mode");
}

static void test_3v3_fault(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: 3V3 FLT\n");
    append_report("\nTEST: 3V3 FLT");

    setup_nominal_system();

    eps_init(&ctx);

    FakeGPIO_SetPin(EPS_3V3_FLT_GPIO_Port, EPS_3V3_FLT_Pin, GPIO_PIN_RESET);

    eps_task(EPS_MODE_NOMINAL, &ctx);

    assert_true(ctx.rail_3v3_flt, "3V3 FLT detected");
    assert_true(ctx.eps_fault, "EPS fault set");
}

static void test_5v_fault(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: 5V FLT\n");
    append_report("\nTEST: 5V FLT");

    setup_nominal_system();

    eps_init(&ctx);

    FakeGPIO_SetPin(EPS_5V_FLT_GPIO_Port, EPS_5V_FLT_Pin, GPIO_PIN_RESET);

    eps_task(EPS_MODE_NOMINAL, &ctx);

    assert_true(ctx.rail_5v_flt, "5V FLT detected");
    assert_true(ctx.eps_fault, "EPS fault set");
}

static void test_vbus_fault(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: VBUS FLT\n");
    append_report("\nTEST: VBUS FLT");

    setup_nominal_system();

    eps_init(&ctx);

    FakeGPIO_SetPin(EPS_VBUS_FLT_GPIO_Port, EPS_VBUS_FLT_Pin, GPIO_PIN_RESET);

    eps_task(EPS_MODE_NOMINAL, &ctx);

    assert_true(ctx.vbus_flt, "VBUS FLT detected");
    assert_true(ctx.eps_fault, "EPS fault set");
}

static void test_low_cell_voltage(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: Low cell voltage\n");
    append_report("\nTEST: Low cell voltage");

    setup_nominal_system();

    eps_init(&ctx);

    for (uint8_t i = 0; i < 5; i++) {
        FakeBQ76920_SetCellRaw(i, 8000);
    }

    eps_task(EPS_MODE_NOMINAL, &ctx);

    assert_true(ctx.battery_low, "Battery low set");
    assert_true(ctx.soc_low, "SOC low set");
}

static void test_high_cell_voltage(void)
{
    fsw_ctx_t ctx = {0};

    printf("\nTEST: High cell voltage\n");
    append_report("\nTEST: High cell voltage");

    setup_nominal_system();

    eps_init(&ctx);

    for (uint8_t i = 0; i < 5; i++) {
        FakeBQ76920_SetCellRaw(i, 11600);
    }

    eps_task(EPS_MODE_NOMINAL, &ctx);

    assert_true(ctx.battery_high, "Battery high set");
}

static void show_popup_summary(void)
{
    char summary[1024];

    snprintf(
        summary,
        sizeof(summary),
        "\n========================================\n"
        "TEST SUMMARY\n"
        "========================================\n"
        "Passed: %d\n"
        "Failed: %d\n"
        "Status: %s\n",
        tests_passed,
        tests_failed,
        tests_failed == 0 ? "ALL TESTS PASSED" : "TEST FAILURES DETECTED"
    );

    printf("%s", summary);
    append_report(summary);

#ifdef _WIN32
    MessageBoxA(
        NULL,
        popup_report,
        "EPS PC Test Results",
        tests_failed == 0 ? MB_OK | MB_ICONINFORMATION : MB_OK | MB_ICONERROR
    );
#else
    printf("\nPopup only works on Windows. Results shown in terminal.\n");
#endif
}

int main(void)
{
    printf("\n========================================\n");
    printf("EPS PC TEST HARNESS\n");
    printf("========================================\n");

    snprintf(
        popup_report,
        sizeof(popup_report),
        "EPS / BQ25798 / BQ76920 TEST RESULTS\n"
        "=====================================\n"
    );

    test_bq25798_telemetry();
    test_bq76920_telemetry();

    test_eps_init();
    test_eps_nominal();
    test_eps_safe();
    test_eps_science();

    test_3v3_fault();
    test_5v_fault();
    test_vbus_fault();

    test_low_cell_voltage();
    test_high_cell_voltage();

    show_popup_summary();

    return tests_failed;
}