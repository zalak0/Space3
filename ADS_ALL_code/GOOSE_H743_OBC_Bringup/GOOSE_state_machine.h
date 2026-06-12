/* USER CODE BEGIN WHILE region */
static fsw_ctx_t ctx = {0};
adcs_mode_t mode = MODE_LAUNCH;
uint32_t next = HAL_GetTick();

ads_init(); acs_init(); eps_init(); payload_init(); comms_init();

while (1)
{
    mode = sm_update(&ctx, mode);   /* 1. decide mode (flags from last cycle) */

    ads_task(mode, &ctx);           /* 2. producers first ...                 */
    payload_task(mode, &ctx);       /*    science_window before ACS reads it  */
    acs_task(mode, &ctx);           /*    consumer                            */
    eps_task(mode, &ctx);           /*    soc_low -> gate next cycle          */
    comms_task(mode, &ctx);         /*    ground_contact -> SM next cycle      */

    HAL_IWDG_Refresh(&hiwdg);       /* 3. kick only on a healthy full cycle   */
    next += DT_CTRL_MS;             /*    fixed-rate anchor, not HAL_Delay     */
    while ((int32_t)(HAL_GetTick() - next) < 0) { /* idle */ }
}