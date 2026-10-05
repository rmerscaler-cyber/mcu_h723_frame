#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include <stdint.h>

#define ROBOT_TASK_INIT_TIME_TICKS  10u
#define ROBOT_TASK_PERIOD_TICKS      1u

typedef struct
{
    uint32_t cycle_count;
    uint32_t last_start_tick;
    uint32_t last_run_ticks;
    uint32_t max_run_ticks;
    uint32_t overrun_count;
} robot_task_statistics_t;

extern volatile robot_task_statistics_t robot_task_statistics;
void robot_task(void const *pvParameters);

#endif /* ROBOT_TASK_H */
