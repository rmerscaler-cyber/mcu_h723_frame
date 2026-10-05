#ifndef IMU_PORT_H
#define IMU_PORT_H

#include "struct_typedef.h"

/* 模型算法触发的硬件动作：IMU 加热片 PWM（设备层 imu_io.c 实现）。 */
void imu_port_pwm_set(uint16_t pwm);

#endif
