/**
 * [H7-016] 迁移: BMI088 中间件
 *   - SPI: hspi1 → hspi2
 *   - CS引脚: CS1_ACCEL→ACC_CS, CS1_GYRO→GYRO_CS
 *   - 延时: us*168→us*480 (H7@480MHz)
 */
#include "BMI088Middleware.h"
#include "main.h"

static const bsp_spi_endpoint_t *bmi088_gyro_endpoint;
static const bsp_spi_endpoint_t *bmi088_accel_endpoint;
static const bsp_spi_endpoint_t *bmi088_selected_endpoint;

void BMI088_GPIO_init(void) {}
void BMI088_com_init(void) {}

void BMI088_delay_ms(uint16_t ms)
{
    while(ms--) { BMI088_delay_us(1000); }
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t ticks = 0, told = 0, tnow = 0, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    ticks = us * 480;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told) tcnt += told - tnow;
            else tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks) break;
        }
    }
}

void BMI088_ACCEL_NS_L(void)
{
    if (bsp_spi_transaction_begin(bmi088_accel_endpoint))
        bmi088_selected_endpoint = bmi088_accel_endpoint;
}
void BMI088_ACCEL_NS_H(void)
{
    bsp_spi_transaction_end(bmi088_accel_endpoint);
    bmi088_selected_endpoint = NULL;
}
void BMI088_GYRO_NS_L(void)
{
    if (bsp_spi_transaction_begin(bmi088_gyro_endpoint))
        bmi088_selected_endpoint = bmi088_gyro_endpoint;
}
void BMI088_GYRO_NS_H(void)
{
    bsp_spi_transaction_end(bmi088_gyro_endpoint);
    bmi088_selected_endpoint = NULL;
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0u;

    bsp_spi_transfer_byte(bmi088_selected_endpoint,
                          txdata,
                          &rx_data,
                          1000u);
    return rx_data;
}

void BMI088_middleware_bind_spi(
    const bsp_spi_endpoint_t *gyro_endpoint,
    const bsp_spi_endpoint_t *accel_endpoint)
{
    bmi088_gyro_endpoint = gyro_endpoint;
    bmi088_accel_endpoint = accel_endpoint;
    bmi088_selected_endpoint = NULL;
}
