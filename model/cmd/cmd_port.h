#ifndef CMD_PORT_H
#define CMD_PORT_H

#include "cmd.h"

/* 具体机器人在设备层实现输入适配；框架默认提供安全空实现。 */
void cmd_port_collect_input(robot_cmd_input_t *input);

#endif /* CMD_PORT_H */
