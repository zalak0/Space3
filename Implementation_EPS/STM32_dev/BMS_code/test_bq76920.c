#include <stdio.h>
#include <assert.h>
#include <stdint.h>

#include "bq76920.h"

void FakeBQ76920_Reset(void);
void FakeBQ76920_Set8(uint8_t reg, uint8_t value);
void FakeBQ76920_SetCellAdc(uint8_t cell_index, uint16_t adc);

int main(void)
{
    BQ76920_Telemetry bms;

    FakeBQ76920_Reset();

    FakeBQ76920_Set8(BQ76920_ADCGAIN1, 0x00);
    FakeBQ76920_Set8(BQ76920_ADCGAIN2, 0x00);
    FakeBQ76920_Set8(BQ76920_ADCOFFSET, 0x00);

    FakeBQ76920_SetCellAdc(0, 10137); // about 3.700 V
    FakeBQ76920_SetCellAdc(1, 10164); // about 3.710 V
    FakeBQ76920_SetCellAdc(2, 10110); // about 3.690 V
    FakeBQ76920_SetCellAdc(3, 10137); // about 3.700 V

    assert(BQ76920_Init() == true);
    assert(BQ76920_ReadTelemetry(&bms) == true);

    printf("Cell 1 = %.3f V\n", bms.cell_v[0]);
    printf("Cell 2 = %.3f V\n", bms.cell_v[1]);
    printf("Cell 3 = %.3f V\n", bms.cell_v[2]);
    printf("Cell 4 = %.3f V\n", bms.cell_v[3]);

    printf("Pack   = %.3f V\n", bms.pack_voltage_v);
    printf("Delta  = %.0f mV\n", bms.cell_delta_v * 1000.0f);

    printf("SYS_STAT = 0x%02X\n", bms.sys_stat);
    printf("ADC Gain = %d uV/LSB\n", bms.cal.adc_gain_uv_per_lsb);
    printf("ADC Offset = %d mV\n", bms.cal.adc_offset_mv);

    assert(bms.cell_v[0] > 3.69f && bms.cell_v[0] < 3.71f);
    assert(bms.pack_voltage_v > 14.79f && bms.pack_voltage_v < 14.81f);

    printf("BQ76920 software test passed\n");

    return 0;
}