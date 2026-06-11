#ifndef MODES_H
#define MODES_H

typedef enum {
    MODE_NOMINAL = 0,   /* power-on / deploy-inhibit; zero value = safe boot state */
    MODE_DETUMBLE,
    MODE_SAFE,
    MODE_POINTING,
    MODE_SCIENCE,
    MODE_DOWNLINK,
    MODE_UPLINK,
    MODE_COUNT         /* not a real mode — count/bounds-check sentinel */
} sat_mode_t;

#endif
