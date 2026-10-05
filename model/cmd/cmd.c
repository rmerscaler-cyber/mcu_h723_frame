#include "cmd.h"

#include <stddef.h>
#include <string.h>

void robot_cmd_init(robot_cmd_t *instance)
{
    if (instance == NULL)
        return;
    memset(instance, 0, sizeof(*instance));
    instance->output.control_lost = 1u;
}

const cmd_t *robot_cmd_output(const robot_cmd_t *instance)
{
    return instance != NULL ? &instance->output : NULL;
}

void robot_cmd_step(robot_cmd_t *instance, const robot_cmd_input_t *input)
{
    if (instance == NULL || input == NULL)
        return;

    if (input->system_force_safe) {
        memset(&instance->output, 0, sizeof(instance->output));
        instance->output.control_lost = 1u;
        return;
    }

    instance->output = input->command;
}
