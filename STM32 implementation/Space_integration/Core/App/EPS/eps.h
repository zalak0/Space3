#ifndef EPS_H
#define EPS_H

#include <stdbool.h>

#include "common/fsw_ctx.h"
#include "stm/modes.h"

void eps_init(fsw_ctx_t *ctx);
void eps_request_telemetry(fsw_ctx_t *ctx);
void eps_task(fsw_ctx_t *ctx);

#endif
