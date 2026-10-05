/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             bmi088ist8310
  *             bmi088data ready
  *             DMASPICPU.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_Task_H
#define INS_Task_H
#include "struct_typedef.h"


#define TEMPERATURE_PID_KP 1600.0f //PIDkp
#define TEMPERATURE_PID_KI 0.2f    //PIDki
#define TEMPERATURE_PID_KD 0.0f    //PIDkd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //PIDmax_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //PIDmax_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500TIMPWM MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 // delay

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu,  bmi088, ist8310,
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void INS_task(void const *pvParameters);

#endif
