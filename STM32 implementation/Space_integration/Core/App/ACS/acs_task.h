#ifndef ACS_TASK_H
#define ACS_TASK_H
#include <acs/adcs_types.h>
#include <stdint.h>

#include "common/fsw_ctx.h"
#include "stm/modes.h"

void acs_task(sat_mode_t mode, fsw_ctx_t *ctx);

#endif
