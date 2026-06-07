#ifndef ADS_H743_SENSORS_H
#define ADS_H743_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    uint32_t init_count;
    uint32_t update_count;
    uint32_t read_count;
    uint8_t initialized;
    uint8_t sensor_backend_ready;
} ADS_H743_SensorDiagnostics;

void ADS_H743_Sensors_Init(void);
void ADS_H743_Sensors_Update(float dt_s);

uint8_t ADS_H743_Sensors_IsInitialized(void);
uint8_t ADS_H743_Sensors_IsBackendReady(void);

ADS_H743_SensorDiagnostics ADS_H743_Sensors_GetDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS_H743_SENSORS_H */