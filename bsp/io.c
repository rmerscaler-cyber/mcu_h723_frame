/**
 * [H7-019] 迁移: GPIO IO 控制
 *   - GPIOI 在 H723VGTx(LQFP100)上不存在, 改用可用引脚
 *   - PB14=磁铁控制, PD4/PD5/PE4=子弹检测, PE5=碰触检测
 *   - !!! 请根据实际硬件修改引脚 !!!
 */
#include "io.h"
#include "stm32h7xx_hal.h"

#define MAGNET_PIN      GPIO_PIN_14
#define MAGNET_PORT     GPIOB
#define BULLET1_PIN     GPIO_PIN_4
#define BULLET1_PORT    GPIOD
#define BULLET2_PIN     GPIO_PIN_5
#define BULLET2_PORT    GPIOD
#define BULLET3_PIN     GPIO_PIN_4
#define BULLET3_PORT    GPIOE

void io_configuration(void)
{
    GPIO_InitTypeDef s = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    s.Mode = GPIO_MODE_INPUT;
    s.Pull = GPIO_NOPULL;
    s.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    s.Pin = BULLET1_PIN; HAL_GPIO_Init(BULLET1_PORT, &s);
    s.Pin = BULLET2_PIN; HAL_GPIO_Init(BULLET2_PORT, &s);
    s.Pin = BULLET3_PIN; HAL_GPIO_Init(BULLET3_PORT, &s);

    s.Pin = MAGNET_PIN;
    s.Mode = GPIO_MODE_OUTPUT_PP;
    s.Pull = GPIO_NOPULL;
    s.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(MAGNET_PORT, &s);

    magnet_withdraw();
}

void magnet_push(void)
{
    HAL_GPIO_WritePin(MAGNET_PORT, MAGNET_PIN, GPIO_PIN_SET);
}

void magnet_withdraw(void)
{
    HAL_GPIO_WritePin(MAGNET_PORT, MAGNET_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BULLET1_PORT, BULLET1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BULLET2_PORT, BULLET2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BULLET3_PORT, BULLET3_PIN, GPIO_PIN_RESET);
}

bool_t touchspot_bullet_monitor(void)
{
    if (HAL_GPIO_ReadPin(BULLET1_PORT, BULLET1_PIN) == GPIO_PIN_RESET ||
        HAL_GPIO_ReadPin(BULLET2_PORT, BULLET2_PIN) == GPIO_PIN_RESET ||
        HAL_GPIO_ReadPin(BULLET3_PORT, BULLET3_PIN) == GPIO_PIN_RESET)
        return 1;
    return 0;
}
