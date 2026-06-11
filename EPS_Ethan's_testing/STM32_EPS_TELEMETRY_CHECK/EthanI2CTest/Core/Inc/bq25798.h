#ifndef BQ25798_H
#define BQ25798_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BQ25798_I2C_ADDR_7BIT   0x6B
#define BQ25798_I2C_ADDR        (BQ25798_I2C_ADDR_7BIT << 1)

/* Core configuration registers */
#define BQ25798_REG_VSYSMIN             0x00
#define BQ25798_REG_CHARGE_VOLTAGE      0x01
#define BQ25798_REG_CHARGE_CURRENT      0x03
#define BQ25798_REG_INPUT_VOLTAGE_LIMIT 0x05
#define BQ25798_REG_INPUT_CURRENT_LIMIT 0x06
#define BQ25798_REG_CHARGER_CONTROL_0   0x0F
#define BQ25798_REG_CHARGER_CONTROL_1   0x10
#define BQ25798_REG_ADC_CONTROL         0x2E

/* Status registers */
#define BQ25798_REG_CHARGER_STATUS_0    0x1B
#define BQ25798_REG_CHARGER_STATUS_1    0x1C
#define BQ25798_REG_CHARGER_STATUS_2    0x1D
#define BQ25798_REG_CHARGER_STATUS_3    0x1E
#define BQ25798_REG_CHARGER_STATUS_4    0x1F
#define BQ25798_REG_FAULT_STATUS_0      0x20
#define BQ25798_REG_FAULT_STATUS_1      0x21

/* Interrupt flag registers */
#define BQ25798_REG_CHARGER_FLAG_0      0x22
#define BQ25798_REG_CHARGER_FLAG_1      0x23
#define BQ25798_REG_CHARGER_FLAG_2      0x24
#define BQ25798_REG_CHARGER_FLAG_3      0x25
#define BQ25798_REG_FAULT_FLAG_0        0x26
#define BQ25798_REG_FAULT_FLAG_1        0x27

/* Interrupt mask registers */
#define BQ25798_REG_CHARGER_MASK_0      0x28
#define BQ25798_REG_CHARGER_MASK_1      0x29
#define BQ25798_REG_CHARGER_MASK_2      0x2A
#define BQ25798_REG_CHARGER_MASK_3      0x2B
#define BQ25798_REG_FAULT_MASK_0        0x2C
#define BQ25798_REG_FAULT_MASK_1        0x2D

/* Telemetry registers */
#define BQ25798_REG_IBUS_ADC            0x31
#define BQ25798_REG_IBAT_ADC            0x33
#define BQ25798_REG_VBUS_ADC            0x35
#define BQ25798_REG_VBAT_ADC            0x3B
#define BQ25798_REG_VSYS_ADC            0x3D
#define BQ25798_REG_TDIE_ADC            0x41
#define BQ25798_REG_PART_INFORMATION    0x48

/* Charger Control 0 bits */
#define BQ25798_EN_CHG_BIT              (1U << 5)
#define BQ25798_EN_HIZ_BIT              (1U << 2)
#define BQ25798_EN_TERM_BIT             (1U << 1)

/* Charger Control 1 bits */
#define BQ25798_WD_RST_BIT              (1U << 3)
#define BQ25798_WATCHDOG_MASK           0x07

/* ============================================================
 * CHARGER FLAG 0 - REG 0x22
 * ============================================================ */

#define BQ25798_FLAG0_IINDPM            (1U << 7)
#define BQ25798_FLAG0_VINDPM            (1U << 6)
#define BQ25798_FLAG0_WD                (1U << 5)
#define BQ25798_FLAG0_POORSRC           (1U << 4)
#define BQ25798_FLAG0_PG                (1U << 3)
#define BQ25798_FLAG0_AC2_PRESENT       (1U << 2)
#define BQ25798_FLAG0_AC1_PRESENT       (1U << 1)
#define BQ25798_FLAG0_VBUS_PRESENT      (1U << 0)

/* ============================================================
 * CHARGER FLAG 1 - REG 0x23
 * ============================================================ */

#define BQ25798_FLAG1_CHG               (1U << 7)
#define BQ25798_FLAG1_ICO               (1U << 6)
#define BQ25798_FLAG1_VBUS              (1U << 4)
#define BQ25798_FLAG1_TREG              (1U << 2)
#define BQ25798_FLAG1_VBAT_PRESENT      (1U << 1)
#define BQ25798_FLAG1_BC12_DONE         (1U << 0)

/* ============================================================
 * CHARGER FLAG 2 - REG 0x24
 * ============================================================ */

#define BQ25798_FLAG2_DPDM_DONE         (1U << 6)
#define BQ25798_FLAG2_ADC_DONE          (1U << 5)
#define BQ25798_FLAG2_VSYS              (1U << 4)
#define BQ25798_FLAG2_CHG_TMR           (1U << 3)
#define BQ25798_FLAG2_TRICHG_TMR        (1U << 2)
#define BQ25798_FLAG2_PRECHG_TMR        (1U << 1)
#define BQ25798_FLAG2_TOPOFF_TMR        (1U << 0)

/* ============================================================
 * CHARGER FLAG 3 - REG 0x25
 * ============================================================ */

#define BQ25798_FLAG3_VBATOTG_LOW       (1U << 4)
#define BQ25798_FLAG3_TS_COLD           (1U << 3)
#define BQ25798_FLAG3_TS_COOL           (1U << 2)
#define BQ25798_FLAG3_TS_WARM           (1U << 1)
#define BQ25798_FLAG3_TS_HOT            (1U << 0)

/* ============================================================
 * FAULT FLAG 0 - REG 0x26
 * ============================================================ */

#define BQ25798_FAULT0_IBAT_REG         (1U << 7)
#define BQ25798_FAULT0_VBUS_OVP         (1U << 6)
#define BQ25798_FAULT0_VBAT_OVP         (1U << 5)
#define BQ25798_FAULT0_IBUS_OCP         (1U << 4)
#define BQ25798_FAULT0_IBAT_OCP         (1U << 3)
#define BQ25798_FAULT0_CONV_OCP         (1U << 2)
#define BQ25798_FAULT0_VAC2_OVP         (1U << 1)
#define BQ25798_FAULT0_VAC1_OVP         (1U << 0)

/* ============================================================
 * FAULT FLAG 1 - REG 0x27
 * ============================================================ */

#define BQ25798_FAULT1_VSYS_SHORT       (1U << 7)
#define BQ25798_FAULT1_VSYS_OVP         (1U << 6)
#define BQ25798_FAULT1_OTG_OVP          (1U << 5)
#define BQ25798_FAULT1_OTG_UVP          (1U << 4)
#define BQ25798_FAULT1_TSHUT            (1U << 2)

typedef enum
{
    BQ25798_WATCHDOG_DISABLED = 0,
    BQ25798_WATCHDOG_0P5S     = 1,
    BQ25798_WATCHDOG_1S       = 2,
    BQ25798_WATCHDOG_2S       = 3,
    BQ25798_WATCHDOG_20S      = 4,
    BQ25798_WATCHDOG_40S      = 5,
    BQ25798_WATCHDOG_80S      = 6,
    BQ25798_WATCHDOG_160S     = 7
} BQ25798_WatchdogSetting;

typedef struct
{
    bool i2c_ok;

    float vbus_v;
    float vbat_v;
    float vsys_v;
    float ibus_a;
    float ibat_a;
    float die_temp_c;

    uint8_t charger_status_0;
    uint8_t charger_status_1;
    uint8_t charger_status_2;
    uint8_t charger_status_3;
    uint8_t charger_status_4;

    uint8_t fault_status_0;
    uint8_t fault_status_1;
    uint8_t part_info;

} BQ25798_Telemetry;

typedef struct
{
    uint8_t charger_status_0;
    uint8_t charger_status_1;
    uint8_t charger_status_2;
    uint8_t charger_status_3;
    uint8_t charger_status_4;

    uint8_t fault_status_0;
    uint8_t fault_status_1;

    uint8_t charger_flag_0;
    uint8_t charger_flag_1;
    uint8_t charger_flag_2;
    uint8_t charger_flag_3;

    uint8_t fault_flag_0;
    uint8_t fault_flag_1;

} BQ25798_InterruptInfo;

/* Low-level I2C */
bool BQ25798_IsConnected(void);
HAL_StatusTypeDef BQ25798_Read8(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef BQ25798_Write8(uint8_t reg, uint8_t value);
HAL_StatusTypeDef BQ25798_Read16(uint8_t reg, uint16_t *value);
HAL_StatusTypeDef BQ25798_Write16(uint8_t reg, uint16_t value);

/* Configuration helpers */
bool BQ25798_SetMinSystemVoltage_mV(uint16_t mv);
bool BQ25798_SetChargeVoltage_mV(uint16_t mv);
bool BQ25798_SetChargeCurrent_mA(uint16_t ma);
bool BQ25798_SetInputVoltageLimit_mV(uint16_t mv);
bool BQ25798_SetInputCurrentLimit_mA(uint16_t ma);

bool BQ25798_EnableCharging(bool enable);
bool BQ25798_EnableTermination(bool enable);
bool BQ25798_SetWatchdog(BQ25798_WatchdogSetting setting);
bool BQ25798_ResetWatchdog(void);
bool BQ25798_EnableADC(bool enable);

/* Telemetry */
bool BQ25798_ReadTelemetry(BQ25798_Telemetry *t);

/* Interrupt / INT information */
bool BQ25798_ReadInterruptInfo(BQ25798_InterruptInfo *info);
void BQ25798_PrintInterruptInfo(const BQ25798_InterruptInfo *info);

#endif
