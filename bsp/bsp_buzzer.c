/**
 * [H7-007] 迁移: 蜂鸣器驱动
 *   - F4用 TIM4_CH3(PD14), H7用 TIM3_CH4(PB1)
 */
#include "bsp_buzzer.h"
#include "main.h"
extern TIM_HandleTypeDef htim3;
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim3, psc);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
}
