/**
 * Robot application framework.
 *
 * This layer deliberately contains no robot-class policy. A concrete project
 * may add chassis, gimbal, flight, manipulator or shooter modules and call
 * them from robot_application_step(), while the frame keeps the scheduler
 * entry point stable.
 */
#include "robot_application.h"

#include <string.h>

static robot_application_state_t robot_application_state;

void robot_application_init(void)
{
    memset(&robot_application_state, 0, sizeof(robot_application_state));
}

void robot_application_step(uint32_t now_ms)
{
    robot_application_state.last_step_ms = now_ms;
    robot_application_state.cycle_count++;
}

const robot_application_state_t *robot_application_state_get(void)
{
    return &robot_application_state;
}
