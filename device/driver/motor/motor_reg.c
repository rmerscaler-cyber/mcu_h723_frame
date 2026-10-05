#include "motor_reg.h"

/*
 * Robot-specific motor topology belongs in the concrete project, not in this
 * frame. Add one entry per installed motor and bind DJI/DM/LK driver ops there.
 */
const motor_reg_t motor_regs[] = {
    {0},
};

const uint8_t motor_reg_count = 0u;
