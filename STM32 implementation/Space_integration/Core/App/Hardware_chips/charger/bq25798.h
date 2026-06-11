#ifndef BQ25798_H
#define BQ25798_H

#include "stm32h7xx_hal.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct fsw_ctx_t fsw_ctx_t;

#define BQ25798_I2C_ADDR_7BIT   0x6B
#define BQ25798_I2C_ADDR        (BQ25798_I2C_ADDR_7BIT << 1)

#define BQ25798_REG_VSYSMIN             0x00
#define BQ25798_REG_CHARGE_VOLTAGE      0x01
#define BQ25798_REG_CHARGE_CURRENT      0x03
#define BQ25798_REG_INPUT_VOLTAGE_LIMIT 0x05
#define BQ25798_REG_INPUT_CURRENT_LIMIT 0x06
#define BQ25798_REG_CHARGER_CONTROL_0   0x0F
#define BQ25798_REG_CHARGER_CONTROL_1   0x10
#define BQ25798_REG_ADC_CONTROL         0x2E

#define BQ25798_REG_CHARGER_STATUS_0    0x1B
#define BQ25798_REG_CHARGER_STATUS_1    0x1C
#define BQ25798_REG_CHARGER_STATUS_2    0x1D
#define BQ25798_REG_CHARGER_STATUS_3    0x1E
#define BQ25798_REG_CHARGER_STATUS_4    0x1F
#define BQ25798_REG_FAULT_STATUS_0      0x20
#define BQ25798_REG_FAULT_STATUS_1      0x21

#define BQ25798_REG_VBUS_ADC            0x35
#define BQ25798_REG_VBAT_ADC            0x3B
#define BQ25798_REG_VSYS_ADC            0x3D
#define BQ25798_REG_IBUS_ADC            0x31
#define BQ25798_REG_IBAT_ADC            0x33
#define BQ25798_REG_TDIE_ADC            0x41
#define BQ25798_REG_PART_INFORMATION    0x48

#define BQ25798_EN_CHG_BIT              (1U << 5)
#define BQ25798_EN_TERM_BIT             (1U << 1)
#define BQ25798_WD_RST_BIT              (1U << 3)
#define BQ25798_WATCHDOG_MASK           0x07

typedef enum {
    BQ25798_WATCHDOG_DISABLED = 0,
    BQ25798_WATCHDOG_0P5S     = 1,
    BQ25798_WATCHDOG_1S       = 2,
    BQ25798_WATCHDOG_2S       = 3,
    BQ25798_WATCHDOG_20S      = 4,
    BQ25798_WATCHDOG_40S      = 5,
    BQ25798_WATCHDOG_80S      = 6,
    BQ25798_WATCHDOG_160S     = 7
} BQ25798_WatchdogSetting;

typedef struct {
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

bool BQ25798_IsConnected(void);
HAL_StatusTypeDef BQ25798_Read8(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef BQ25798_Write8(uint8_t reg, uint8_t value);
HAL_StatusTypeDef BQ25798_Read16(uint8_t reg, uint16_t *value);
HAL_StatusTypeDef BQ25798_Write16(uint8_t reg, uint16_t value);

bool BQ25798_SetMinSystemVoltage_mV(uint16_t mv);
bool BQ25798_SetChargeVoltage_mV(uint16_t mv);
bool BQ25798_SetChargeCurrent_mA(uint16_t ma);
bool BQ25798_SetInputVoltageLimit_mV(uint16_t mv);
bool BQ25798_SetInputCurrentLimit_mA(uint16_t ma);

bool BQ25798_EnableCharging(bool enable);
bool BQ25798_EnableChargerHardware(bool enable);
bool BQ25798_InitCharger4S(void);
bool BQ25798_EnableTermination(bool enable);
bool BQ25798_SetWatchdog(BQ25798_WatchdogSetting setting);
bool BQ25798_ResetWatchdog(void);
bool BQ25798_EnableADC(bool enable);

void BQ25798_ReadRailGPIOStatus(fsw_ctx_t *ctx);
bool BQ25798_ReadTelemetry(BQ25798_Telemetry *t);

#endif