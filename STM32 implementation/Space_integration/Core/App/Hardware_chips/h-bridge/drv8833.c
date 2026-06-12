/* magnetorquer.c */
#include <hardware_chips/h-bridge/drv8833.h>
#include <math.h>

/* Map each axis to its two PWM channels.
 * Fill these in to match your CubeMX timer assignment.
 * Example: X = TIM1 CH1/CH2, Y = TIM1 CH3/CH4, Z = TIM2 CH1/CH2 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

static const float k_torquer[3] = { 0.046f, 0.046f,  0.046f };
static const float duty_max[3] = { 0.5f, 0.5f, 0.5f };  /* set per axis from measured R_coil */

static mtq_bridge_t bridge[3] = {
    { &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2 },  /* X */
    { &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2 },  /* Y */
    { &htim8, TIM_CHANNEL_3, TIM_CHANNEL_4 },  /* Z */
};
void drv8833_init(void)
{
    for (int i=0;i<3;i++){
        HAL_TIM_PWM_Start(bridge[i].htim, bridge[i].ch_in1);
        HAL_TIM_PWM_Start(bridge[i].htim, bridge[i].ch_in2);
        __HAL_TIM_SET_COMPARE(bridge[i].htim, bridge[i].ch_in1, 0);
        __HAL_TIM_SET_COMPARE(bridge[i].htim, bridge[i].ch_in2, 0);
    }
}

/* duty in [-1, +1] */
void drv8833_set_axis(int axis, float duty)
{
    mtq_bridge_t *b = &bridge[axis];
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(b->htim);      /* timer period */
    if (duty >  1.f) duty =  1.f;
    if (duty < -1.f) duty = -1.f;
    uint32_t ccr = (uint32_t)(fabsf(duty) * (float)arr);

    if (duty >= 0.f) {                 /* forward: PWM IN1, IN2 low */
        __HAL_TIM_SET_COMPARE(b->htim, b->ch_in1, ccr);
        __HAL_TIM_SET_COMPARE(b->htim, b->ch_in2, 0);
    } else {                           /* reverse: IN1 low, PWM IN2 */
        __HAL_TIM_SET_COMPARE(b->htim, b->ch_in1, 0);
        __HAL_TIM_SET_COMPARE(b->htim, b->ch_in2, ccr);
    }
}

void drv8833_coast_all(void)           /* both inputs low = high-Z, coils de-energised */
{
    for (int i=0;i<3;i++){
        __HAL_TIM_SET_COMPARE(bridge[i].htim, bridge[i].ch_in1, 0);
        __HAL_TIM_SET_COMPARE(bridge[i].htim, bridge[i].ch_in2, 0);
    }
}

void torquer_apply(const adcs_dipole_t *m)
{
    float duty[3] = { m->mx/k_torquer[0], m->my/k_torquer[1], m->mz/k_torquer[2] };
    for (int i=0;i<3;i++){
        if (duty[i] >  duty_max[i]) duty[i] =  duty_max[i];
        if (duty[i] < -duty_max[i]) duty[i] = -duty_max[i];
        drv8833_set_axis(i, duty[i]);
    }
}
