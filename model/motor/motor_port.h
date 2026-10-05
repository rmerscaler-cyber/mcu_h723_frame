#ifndef MOTOR_PORT_H
#define MOTOR_PORT_H

#include "motor_typedef.h"

/* The board adapter owns CAN endpoints and maps logical buses to physical CAN. */
uint8_t motor_port_register(motor_handle_t motor, motor_bus_e bus, uint16_t rx_id);
uint8_t motor_port_send(motor_bus_e bus, uint16_t tx_id,
                        const uint8_t *data, uint8_t length);
uint32_t motor_port_tick_ms(void);

#endif
