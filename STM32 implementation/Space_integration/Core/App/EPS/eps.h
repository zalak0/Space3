#ifndef EPS_H
#define EPS_H

#include <stdbool.h>

#include "fsw_ctx.h"
#include "modes.h"

bool eps_init(fsw_ctx_t *ctx);
void eps_task(sat_mode_t mode, fsw_ctx_t *ctx);
void eps_shed_nonessential(void);
void eps_nominal_rails(void);
void eps_enable_payload_rail(void);

#endif
