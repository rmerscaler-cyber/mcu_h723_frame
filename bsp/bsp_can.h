#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"

/* 对外 endpoint 接口与 Upboard 一致；H7 只保留三路 FDCAN 板级差异。 */
typedef enum {
    BSP_CAN1 = 0,   /* hfdcan1: 4 路 3508 轮向电机 */
    BSP_CAN2,       /* hfdcan2: 4 路 6020 舵向电机 */
    BSP_CAN3,       /* hfdcan3: 板间通信、超级电容和 LK 拨盘 */
    BSP_CAN_NUM,
} bsp_can_id_e;

#define BSP_CAN_ENDPOINT_MAX  16u

uint8_t bsp_can_send(bsp_can_id_e can_bus,
                     uint16_t id,
                     const uint8_t *data,
                     uint8_t len);
uint8_t can_filter_init(void);

typedef struct {
    bsp_can_id_e bus;
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    uint8_t is_extend;
    uint32_t timestamp_ms;
} can_raw_msg_t;

typedef void (*bsp_can_endpoint_rx_cb_t)(void *owner,
                                         const can_raw_msg_t *msg);

typedef struct {
    bsp_can_id_e bus;
    uint32_t rx_id;
    uint8_t expected_dlc;
    uint8_t is_extend;
    void *owner;
    bsp_can_endpoint_rx_cb_t rx_callback;
} bsp_can_endpoint_config_t;

typedef struct {
    bsp_can_id_e bus;
    uint32_t rx_id;
    uint8_t expected_dlc;
    uint8_t is_extend;
    void *owner;
    bsp_can_endpoint_rx_cb_t rx_callback;
    volatile uint32_t rx_count;
    volatile uint32_t last_rx_ms;
    uint8_t registered;
} bsp_can_endpoint_t;

typedef enum {
    BSP_CAN_ENDPOINT_OK = 0,
    BSP_CAN_ENDPOINT_ERR_ARGUMENT,
    BSP_CAN_ENDPOINT_ERR_BUS,
    BSP_CAN_ENDPOINT_ERR_DLC,
    BSP_CAN_ENDPOINT_ERR_ALREADY_REGISTERED,
    BSP_CAN_ENDPOINT_ERR_DUPLICATE,
    BSP_CAN_ENDPOINT_ERR_FULL,
} bsp_can_endpoint_status_e;

bsp_can_endpoint_status_e bsp_can_endpoint_register(
    bsp_can_endpoint_t *endpoint,
    const bsp_can_endpoint_config_t *config);

uint8_t bsp_can_endpoint_send(const bsp_can_endpoint_t *endpoint,
                              uint16_t tx_id,
                              uint8_t *data,
                              uint8_t len);

#endif /* BSP_CAN_H */
