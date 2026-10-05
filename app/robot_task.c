#include "robot_task.h"
#include "robot_application.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

volatile robot_task_statistics_t robot_task_statistics;

void robot_task(void const *pvParameters)
{
    TickType_t last_wake_time;
    TickType_t start_tick;
    TickType_t run_ticks;
    const TickType_t period_ticks = (TickType_t)ROBOT_TASK_PERIOD_TICKS;

    (void)pvParameters;
    robot_application_init();
    vTaskDelay((TickType_t)ROBOT_TASK_INIT_TIME_TICKS);
    last_wake_time = xTaskGetTickCount();

    for (;;) {
        start_tick = xTaskGetTickCount();
        robot_task_statistics.last_start_tick = (uint32_t)start_tick;
        robot_application_step(HAL_GetTick());

        run_ticks = xTaskGetTickCount() - start_tick;
        robot_task_statistics.last_run_ticks = (uint32_t)run_ticks;
        if ((uint32_t)run_ticks > robot_task_statistics.max_run_ticks)
            robot_task_statistics.max_run_ticks = (uint32_t)run_ticks;
        if (run_ticks >= period_ticks)
            robot_task_statistics.overrun_count++;
        robot_task_statistics.cycle_count++;
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}
