#include "cmd_port.h"

#include <stddef.h>
#include <string.h>

void cmd_port_collect_input(robot_cmd_input_t *input)
{
    if (input == NULL)
        return;
    memset(input, 0, sizeof(*input));
    input->system_force_safe = 1u;
}
