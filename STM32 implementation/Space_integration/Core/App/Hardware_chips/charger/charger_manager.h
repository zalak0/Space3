#ifndef CHARGER_MANAGER_H
#define CHARGER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware_chips/charger/bq25798.h"

typedef struct
{
    uint16_t charge_voltage_mV;
    uint16_t charge_current_mA;
    uint16_t input_voltage_limit_mV;
    uint16_t input_current_limit_mA;
    uint16_t min_system_voltage_mV;

    bool enable_charging;
    bool enable_termination;

    BQ25798_WatchdogSetting watchdog;

} ChargerManager_Config;

bool ChargerManager_Init(void);
bool ChargerManager_ApplyConfig(const ChargerManager_Config *config);
bool ChargerManager_ServiceWatchdog(void);
bool ChargerManager_ReadTelemetry(BQ25798_Telemetry *telemetry);

const ChargerManager_Config *ChargerManager_GetDefaultConfig(void);

#endif
