#ifndef ROBOT_APPLICATION_H
#define ROBOT_APPLICATION_H

#include <stdint.h>

typedef struct
{
    uint32_t cycle_count;
    uint32_t last_step_ms;
} robot_application_state_t;

void robot_application_init(void);
void robot_application_step(uint32_t now_ms);
const robot_application_state_t *robot_application_state_get(void);

#endif /* ROBOT_APPLICATION_H */
