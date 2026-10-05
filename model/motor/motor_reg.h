#ifndef MOTOR_REG_H
#define MOTOR_REG_H

#include "motor_typedef.h"   /* motor_reg_t */

/* 板级拓扑、协议绑定及 PID 标定由各板 device/driver/motor/motor_reg.c 提供。 */
extern const motor_reg_t motor_regs[];
extern const uint8_t motor_reg_count;

#endif /* MOTOR_REG_H */
