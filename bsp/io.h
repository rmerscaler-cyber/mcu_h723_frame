/**
 * [H7-019] 迁移: GPIO IO 控制头文件
 */
#ifndef IO_H
#define IO_H
#include "main.h"
#include "struct_typedef.h"

#define monitor_touchspot_bullet HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)

extern bool_t touchspot_bullet_monitor(void);
extern void io_configuration(void);
extern void magnet_withdraw(void);
extern void magnet_push(void);

#endif
