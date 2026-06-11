#ifndef ACS_TASK_H
#define ACS_TASK_H
#include <stdint.h>
#include "modes.h"
#include "adcs_types.h"
#include "fsw_ctx.h"

void acs_task(sat_mode_t mode, fsw_ctx_t *ctx);

#endif
