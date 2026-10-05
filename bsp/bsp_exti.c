#include "bsp_exti.h"

#include "main.h"

/* Endpoints are owned by device instances; BSP only keeps stable pointers. */
static bsp_exti_endpoint_t * volatile exti_endpoints[BSP_EXTI_ENDPOINT_MAX];

static void bsp_exti_irq_restore(uint32_t primask)
{
    if (primask == 0u)
        __enable_irq();
}

bsp_exti_endpoint_status_e bsp_exti_endpoint_register(
    bsp_exti_endpoint_t *endpoint,
    const bsp_exti_endpoint_config_t *config)
{
    uint8_t i;
    uint8_t free_slot = BSP_EXTI_ENDPOINT_MAX;
    uint32_t primask;

    if (endpoint == NULL || config == NULL || config->pin == 0u ||
        config->owner == NULL || config->callback == NULL)
        return BSP_EXTI_ENDPOINT_ERR_ARGUMENT;
    if (endpoint->registered)
        return BSP_EXTI_ENDPOINT_ERR_ALREADY_REGISTERED;

    endpoint->pin = config->pin;
    endpoint->owner = config->owner;
    endpoint->callback = config->callback;
    endpoint->event_count = 0u;

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0u; i < BSP_EXTI_ENDPOINT_MAX; i++) {
        bsp_exti_endpoint_t *registered = exti_endpoints[i];

        if (registered == NULL) {
            if (free_slot == BSP_EXTI_ENDPOINT_MAX)
                free_slot = i;
            continue;
        }
        if (registered->pin == endpoint->pin) {
            bsp_exti_irq_restore(primask);
            return BSP_EXTI_ENDPOINT_ERR_DUPLICATE;
        }
    }

    if (free_slot == BSP_EXTI_ENDPOINT_MAX) {
        bsp_exti_irq_restore(primask);
        return BSP_EXTI_ENDPOINT_ERR_FULL;
    }

    endpoint->registered = 1u;
    __DMB();
    exti_endpoints[free_slot] = endpoint;
    bsp_exti_irq_restore(primask);
    return BSP_EXTI_ENDPOINT_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t i;

    for (i = 0u; i < BSP_EXTI_ENDPOINT_MAX; i++) {
        bsp_exti_endpoint_t *endpoint = exti_endpoints[i];

        if (endpoint == NULL || endpoint->pin != GPIO_Pin)
            continue;

        endpoint->event_count++;
        endpoint->callback(endpoint->owner, GPIO_Pin);
        return;
    }
}
