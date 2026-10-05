#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include "struct_typedef.h"

#define BSP_EXTI_ENDPOINT_MAX 8u

typedef void (*bsp_exti_callback_t)(void *owner, uint16_t pin);

typedef struct {
    uint16_t pin;
    void *owner;
    bsp_exti_callback_t callback;
} bsp_exti_endpoint_config_t;

typedef struct {
    uint16_t pin;
    void *owner;
    bsp_exti_callback_t callback;
    volatile uint32_t event_count;
    uint8_t registered;
} bsp_exti_endpoint_t;

typedef enum {
    BSP_EXTI_ENDPOINT_OK = 0,
    BSP_EXTI_ENDPOINT_ERR_ARGUMENT,            //endpoint/config/pin/owner/callback 存在非法值
    BSP_EXTI_ENDPOINT_ERR_ALREADY_REGISTERED,  //这个 endpoint 自己已经注册过
    BSP_EXTI_ENDPOINT_ERR_DUPLICATE,           //路由表中已有其他 endpoint 使用同一个 pin
    BSP_EXTI_ENDPOINT_ERR_FULL,                //8 个路由位置全部用完
} bsp_exti_endpoint_status_e;

bsp_exti_endpoint_status_e bsp_exti_endpoint_register(
    bsp_exti_endpoint_t *endpoint,
    const bsp_exti_endpoint_config_t *config);

#endif
