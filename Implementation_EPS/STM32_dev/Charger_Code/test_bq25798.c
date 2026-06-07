#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "bq25798.h"

/* ============================================================
 * FAKE TEST HELPERS FROM fake_hal_i2c.c
 * ============================================================ */

void FakeBQ25798_Reset(void);
void FakeBQ25798_Set8(uint8_t reg, uint8_t value);
void FakeBQ25798_Set16(uint8_t reg, uint16_t value);

/* ============================================================
 * GPIO INTERRUPT PIN MAPPING
 *
 * GPIO1 -> PD0 -> EXTI0
 * GPIO2 -> PE9 -> EXTI9
 * ============================================================ */

#define GPIO1_INT_PIN GPIO_PIN_0
#define GPIO2_INT_PIN GPIO_PIN_9

static volatile uint8_t gpio1_int_flag = 0;
static volatile uint8_t gpio2_int_flag = 0;

/* ============================================================
 * FAKE STM32 GPIO INTERRUPT CALLBACK
 * ============================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO1_INT_PIN)
    {
        gpio1_int_flag = 1;
    }

    if (GPIO_Pin == GPIO2_INT_PIN)
    {
        gpio2_int_flag = 1;
    }
}

/* ============================================================
 * TEST 1: TELEMETRY
 * ============================================================ */

static void test_telemetry(void)
{
    BQ25798_Telemetry t = {0};

    printf("\n========================================\n");
    printf("BQ25798 TELEMETRY TEST\n");
    printf("========================================\n");

    FakeBQ25798_Reset();

    FakeBQ25798_Set16(BQ25798_REG_VBUS_ADC, 18000);
    FakeBQ25798_Set16(BQ25798_REG_VBAT_ADC, 14800);
    FakeBQ25798_Set16(BQ25798_REG_VSYS_ADC, 14750);
    FakeBQ25798_Set16(BQ25798_REG_IBUS_ADC, 450);
    FakeBQ25798_Set16(BQ25798_REG_IBAT_ADC, 380);
    FakeBQ25798_Set16(BQ25798_REG_TDIE_ADC, 50);

    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_0, 0x11);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_1, 0x22);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_2, 0x33);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_3, 0x44);
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_STATUS_4, 0x55);

    FakeBQ25798_Set8(BQ25798_REG_FAULT_STATUS_0, 0x66);
    FakeBQ25798_Set8(BQ25798_REG_FAULT_STATUS_1, 0x77);
    FakeBQ25798_Set8(BQ25798_REG_PART_INFORMATION, 0x98);

    assert(BQ25798_ReadTelemetry(&t) == true);

    printf("VBUS = %.3f V\n", t.vbus_v);
    printf("VBAT = %.3f V\n", t.vbat_v);
    printf("VSYS = %.3f V\n", t.vsys_v);
    printf("IBUS = %.3f A\n", t.ibus_a);
    printf("IBAT = %.3f A\n", t.ibat_a);
    printf("TDIE = %.3f C\n", t.die_temp_c);

    printf("CHARGER_STATUS_0 = 0x%02X\n", t.charger_status_0);
    printf("CHARGER_STATUS_1 = 0x%02X\n", t.charger_status_1);
    printf("CHARGER_STATUS_2 = 0x%02X\n", t.charger_status_2);
    printf("CHARGER_STATUS_3 = 0x%02X\n", t.charger_status_3);
    printf("CHARGER_STATUS_4 = 0x%02X\n", t.charger_status_4);
    printf("FAULT_STATUS_0   = 0x%02X\n", t.fault_status_0);
    printf("FAULT_STATUS_1   = 0x%02X\n", t.fault_status_1);
    printf("PART_INFO        = 0x%02X\n", t.part_info);

    assert(t.vbus_v > 17.99f && t.vbus_v < 18.01f);
    assert(t.vbat_v > 14.79f && t.vbat_v < 14.81f);
    assert(t.vsys_v > 14.74f && t.vsys_v < 14.76f);
    assert(t.ibus_a > 0.449f && t.ibus_a < 0.451f);
    assert(t.ibat_a > 0.379f && t.ibat_a < 0.381f);

    assert(t.charger_status_0 == 0x11);
    assert(t.charger_status_1 == 0x22);
    assert(t.charger_status_2 == 0x33);
    assert(t.charger_status_3 == 0x44);
    assert(t.charger_status_4 == 0x55);

    assert(t.fault_status_0 == 0x66);
    assert(t.fault_status_1 == 0x77);
    assert(t.part_info == 0x98);

    printf("[PASS] Telemetry test passed\n");
}

/* ============================================================
 * TEST 2: DIRECT INTERRUPT FLAG READ
 * ============================================================ */

static void test_direct_interrupt_info(void)
{
    BQ25798_InterruptInfo info = {0};

    printf("\n========================================\n");
    printf("BQ25798 DIRECT INTERRUPT INFO TEST\n");
    printf("========================================\n");

    FakeBQ25798_Reset();

    FakeBQ25798_Set8(BQ25798_REG_CHARGER_FLAG_0,
                     BQ25798_FLAG0_VBUS_PRESENT |
                     BQ25798_FLAG0_PG |
                     BQ25798_FLAG0_POORSRC);

    FakeBQ25798_Set8(BQ25798_REG_CHARGER_FLAG_1,
                     BQ25798_FLAG1_CHG |
                     BQ25798_FLAG1_TREG);

    FakeBQ25798_Set8(BQ25798_REG_FAULT_FLAG_0,
                     BQ25798_FAULT0_VBUS_OVP |
                     BQ25798_FAULT0_IBUS_OCP);

    assert(BQ25798_ReadInterruptInfo(&info) == true);

    BQ25798_PrintInterruptInfo(&info);

    assert(info.charger_flag_0 & BQ25798_FLAG0_VBUS_PRESENT);
    assert(info.charger_flag_0 & BQ25798_FLAG0_PG);
    assert(info.charger_flag_0 & BQ25798_FLAG0_POORSRC);

    assert(info.charger_flag_1 & BQ25798_FLAG1_CHG);
    assert(info.charger_flag_1 & BQ25798_FLAG1_TREG);

    assert(info.fault_flag_0 & BQ25798_FAULT0_VBUS_OVP);
    assert(info.fault_flag_0 & BQ25798_FAULT0_IBUS_OCP);

    printf("[PASS] Direct interrupt info test passed\n");
}

/* ============================================================
 * TEST 3: SIMULATED GPIO1 INTERRUPT
 * ============================================================ */

static void test_gpio1_interrupt_simulation(void)
{
    BQ25798_InterruptInfo info = {0};

    printf("\n========================================\n");
    printf("BQ25798 GPIO1 INTERRUPT SIMULATION TEST\n");
    printf("========================================\n");

    FakeBQ25798_Reset();

    gpio1_int_flag = 0;
    gpio2_int_flag = 0;

    /*
     * Simulate BQ25798 setting interrupt flags before pulling INT low.
     */
    FakeBQ25798_Set8(BQ25798_REG_CHARGER_FLAG_0,
                     BQ25798_FLAG0_VBUS_PRESENT |
                     BQ25798_FLAG0_PG);

    /*
     * Simulate STM32 receiving GPIO1 interrupt.
     */
    HAL_GPIO_EXTI_Callback(GPIO1_INT_PIN);

    assert(gpio1_int_flag == 1);
    assert(gpio2_int_flag == 0);

    if (gpio1_int_flag)
    {
        gpio1_int_flag = 0;

        assert(BQ25798_ReadInterruptInfo(&info) == true);

        printf("GPIO1 interrupt detected\n");
        BQ25798_PrintInterruptInfo(&info);

        assert(info.charger_flag_0 & BQ25798_FLAG0_VBUS_PRESENT);
        assert(info.charger_flag_0 & BQ25798_FLAG0_PG);
    }

    assert(gpio1_int_flag == 0);

    printf("[PASS] GPIO1 interrupt simulation passed\n");
}

/* ============================================================
 * TEST 4: SIMULATED GPIO2 INTERRUPT
 * ============================================================ */

static void test_gpio2_interrupt_simulation(void)
{
    BQ25798_InterruptInfo info = {0};

    printf("\n========================================\n");
    printf("BQ25798 GPIO2 INTERRUPT SIMULATION TEST\n");
    printf("========================================\n");

    FakeBQ25798_Reset();

    gpio1_int_flag = 0;
    gpio2_int_flag = 0;

    /*
     * Simulate charger fault event.
     */
    FakeBQ25798_Set8(BQ25798_REG_FAULT_FLAG_0,
                     BQ25798_FAULT0_VBUS_OVP |
                     BQ25798_FAULT0_IBUS_OCP |
                     BQ25798_FAULT0_IBAT_OCP);

    FakeBQ25798_Set8(BQ25798_REG_FAULT_FLAG_1,
                     BQ25798_FAULT1_VSYS_SHORT |
                     BQ25798_FAULT1_VSYS_OVP);

    /*
     * Simulate STM32 receiving GPIO2 interrupt.
     */
    HAL_GPIO_EXTI_Callback(GPIO2_INT_PIN);

    assert(gpio1_int_flag == 0);
    assert(gpio2_int_flag == 1);

    if (gpio2_int_flag)
    {
        gpio2_int_flag = 0;

        assert(BQ25798_ReadInterruptInfo(&info) == true);

        printf("GPIO2 interrupt detected\n");
        BQ25798_PrintInterruptInfo(&info);

        assert(info.fault_flag_0 & BQ25798_FAULT0_VBUS_OVP);
        assert(info.fault_flag_0 & BQ25798_FAULT0_IBUS_OCP);
        assert(info.fault_flag_0 & BQ25798_FAULT0_IBAT_OCP);

        assert(info.fault_flag_1 & BQ25798_FAULT1_VSYS_SHORT);
        assert(info.fault_flag_1 & BQ25798_FAULT1_VSYS_OVP);
    }

    assert(gpio2_int_flag == 0);

    printf("[PASS] GPIO2 interrupt simulation passed\n");
}

/* ============================================================
 * TEST 5: NO FLAGS
 * ============================================================ */

static void test_no_interrupt_flags(void)
{
    BQ25798_InterruptInfo info = {0};

    printf("\n========================================\n");
    printf("BQ25798 NO INTERRUPT FLAGS TEST\n");
    printf("========================================\n");

    FakeBQ25798_Reset();

    assert(BQ25798_ReadInterruptInfo(&info) == true);

    BQ25798_PrintInterruptInfo(&info);

    assert(info.charger_flag_0 == 0x00);
    assert(info.charger_flag_1 == 0x00);
    assert(info.charger_flag_2 == 0x00);
    assert(info.charger_flag_3 == 0x00);
    assert(info.fault_flag_0 == 0x00);
    assert(info.fault_flag_1 == 0x00);

    printf("[PASS] No interrupt flags test passed\n");
}

/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    printf("\n========================================\n");
    printf("BQ25798 SOFTWARE TEST START\n");
    printf("========================================\n");

    test_telemetry();
    test_direct_interrupt_info();
    test_gpio1_interrupt_simulation();
    test_gpio2_interrupt_simulation();
    test_no_interrupt_flags();

    printf("\n========================================\n");
    printf("ALL BQ25798 SOFTWARE TESTS PASSED\n");
    printf("========================================\n");

    return 0;
}