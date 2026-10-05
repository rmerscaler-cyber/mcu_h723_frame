#include "motor_port.h"
#include "motor.h"
#include "bsp_can.h"
#include "main.h"

/* The CAN routing table and physical bus numbers belong to the device layer. */
static bsp_can_endpoint_t motor_endpoints[MOTOR_COUNT];

static uint8_t motor_bus_valid(motor_bus_e bus)
{
    return bus <= MOTOR_BUS_2;
}

static uint8_t motor_physical_bus(motor_bus_e bus)
{
    switch (bus) {
    case MOTOR_BUS_0: return BSP_CAN1;
    case MOTOR_BUS_1: return BSP_CAN2;
    case MOTOR_BUS_2: return BSP_CAN3;
    default:          return BSP_CAN1;
    }
}

static void motor_port_rx(void *owner, const can_raw_msg_t *msg)
{
    if (msg != NULL)
        motor_receive_feedback((motor_handle_t)owner, msg->data,
                               msg->timestamp_ms);
}

uint8_t motor_port_register(motor_handle_t motor, motor_bus_e bus, uint16_t rx_id)
{
    bsp_can_endpoint_config_t config;

    if (motor == NULL || motor->id >= MOTOR_COUNT || !motor_bus_valid(bus))
        return 0u;

    config.bus = motor_physical_bus(bus);
    config.rx_id = rx_id;
    config.expected_dlc = 8u;
    config.is_extend = 0u;
    config.owner = motor;
    config.rx_callback = motor_port_rx;
    return bsp_can_endpoint_register(&motor_endpoints[motor->id], &config) ==
           BSP_CAN_ENDPOINT_OK;
}

uint8_t motor_port_send(motor_bus_e bus, uint16_t tx_id,
                        const uint8_t *data, uint8_t length)
{
    if (!motor_bus_valid(bus) || data == NULL)
        return 0u;
    return bsp_can_send(motor_physical_bus(bus), tx_id, data, length);
}

uint32_t motor_port_tick_ms(void)
{
    return HAL_GetTick();
}
