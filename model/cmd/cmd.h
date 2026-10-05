#ifndef CMD_H
#define CMD_H

#include "struct_typedef.h"

/* 通用命令容器：字段只是框架能力，具体机器人按需解释和扩展。 */
typedef struct
{
    uint8_t valid;
    uint8_t control_lost;
    fp32 vx;
    fp32 vy;
    fp32 vz;
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
    fp32 auxiliary[8];
    uint32_t flags;
} cmd_t;

typedef struct
{
    cmd_t command;
    uint8_t system_force_safe;
} robot_cmd_input_t;

typedef struct
{
    cmd_t output;
} robot_cmd_t;

void robot_cmd_init(robot_cmd_t *instance);
const cmd_t *robot_cmd_output(const robot_cmd_t *instance);
void robot_cmd_step(robot_cmd_t *instance, const robot_cmd_input_t *input);

#endif /* CMD_H */
