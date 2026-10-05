#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"
#include "bsp_spi.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

extern void BMI088_middleware_bind_spi(
    const bsp_spi_endpoint_t *gyro_endpoint,
    const bsp_spi_endpoint_t *accel_endpoint);

#elif defined(BMI088_USE_IIC)

#endif

#endif
