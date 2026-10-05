/**
 * [H7-008] 迁移: IMU加热PWM驱动
 *   - F4用 TIM10_CH1(PF6), H7改用 TIM12_CH2(PB15)
 */
#include "bsp_imu_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim12;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, pwm);
}
