#include "../../Hardware_chips/charger/charger_manager.h"

static const ChargerManager_Config default_config =
{
    /*
        Conservative 4S Li-ion bring-up settings.
        Adjust after validating battery datasheet, thermal limits,
        solar input capability, and EPS power budget.
    */
    .charge_voltage_mV       = 16800,
    .charge_current_mA       = 1000,
    .input_voltage_limit_mV  = 12000,
    .input_current_limit_mA  = 500,
    .min_system_voltage_mV   = 12000,

    .enable_charging         = true,
    .enable_termination      = true,

    /*
        For development, use DISABLED.
        For integrated/flight-like operation, use 40s/80s/160s
        and call ChargerManager_ServiceWatchdog() periodically.
    */
    .watchdog                = BQ25798_WATCHDOG_DISABLED
};

const ChargerManager_Config *ChargerManager_GetDefaultConfig(void)
{
    return &default_config;
}

bool ChargerManager_Init(void)
{
    if (!BQ25798_IsConnected())
        return false;

    return ChargerManager_ApplyConfig(&default_config);
}

bool ChargerManager_ApplyConfig(const ChargerManager_Config *config)
{
    if (config == 0)
        return false;

    /*
        Order is intentional:
        1. Set watchdog behaviour first.
        2. Set safe limits.
        3. Enable ADC for telemetry.
        4. Enable/disable charging last.
    */

    if (!BQ25798_SetWatchdog(config->watchdog))
        return false;

    if (!BQ25798_SetMinSystemVoltage_mV(config->min_system_voltage_mV))
        return false;

    if (!BQ25798_SetInputVoltageLimit_mV(config->input_voltage_limit_mV))
        return false;

    if (!BQ25798_SetInputCurrentLimit_mA(config->input_current_limit_mA))
        return false;

    if (!BQ25798_SetChargeVoltage_mV(config->charge_voltage_mV))
        return false;

    if (!BQ25798_SetChargeCurrent_mA(config->charge_current_mA))
        return false;

    if (!BQ25798_EnableTermination(config->enable_termination))
        return false;

    if (!BQ25798_EnableADC(true))
        return false;

    if (!BQ25798_EnableCharging(config->enable_charging))
        return false;

    return true;
}

bool ChargerManager_ServiceWatchdog(void)
{
    return BQ25798_ResetWatchdog();
}

bool ChargerManager_ReadTelemetry(BQ25798_Telemetry *telemetry)
{
    return BQ25798_ReadTelemetry(telemetry);
}