#include "bsp_can.h"

#include "fdcan.h"
#include "main.h"
#include <string.h>

static bsp_can_endpoint_t * volatile rx_endpoints[BSP_CAN_ENDPOINT_MAX];

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/* FDCAN3 的 0x105 发送现场快照：仅供 Keil Watch/JScope 调试。 */
volatile uint32_t jscope_can105_hal_status;
volatile uint32_t jscope_fdcan3_state;
volatile uint32_t jscope_fdcan3_error_code;
volatile uint32_t jscope_fdcan3_cccr;
volatile uint32_t jscope_fdcan3_psr;
volatile uint32_t jscope_fdcan3_txfqs;

/* FDCAN1 原始接收诊断：在 endpoint 分发前计数，用于区分总线与注册问题。 */
volatile uint8_t debug_can_filter_init_ok;
volatile uint32_t debug_can1_rx_total;
volatile uint32_t debug_can1_rx_get_fail;
volatile uint32_t debug_can1_last_id;
volatile uint32_t debug_can1_rx_201_hits;
volatile uint32_t debug_can1_rx_202_hits;
volatile uint32_t debug_can1_rx_203_hits;
volatile uint32_t debug_can1_rx_204_hits;

static FDCAN_HandleTypeDef *bsp_can_handle(bsp_can_id_e bus)
{
    switch (bus) {
    case BSP_CAN1: return &hfdcan1;
    case BSP_CAN2: return &hfdcan2;
    case BSP_CAN3: return &hfdcan3;
    default:       return NULL;
    }
}

static uint8_t bsp_can_configure_bus(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0u;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0u;
    filter.FilterID2 = 0u;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
        return 0u;
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
        return 0u;
    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0u) != HAL_OK)
        return 0u;
    return HAL_FDCAN_Start(hfdcan) == HAL_OK;
}

uint8_t can_filter_init(void)
{
    uint8_t init_ok = 1u;

    if (!bsp_can_configure_bus(&hfdcan1)) init_ok = 0u;
    if (!bsp_can_configure_bus(&hfdcan2)) init_ok = 0u;
    if (!bsp_can_configure_bus(&hfdcan3)) init_ok = 0u;
    debug_can_filter_init_ok = init_ok;
    return init_ok;
}

static void bsp_can_irq_restore(uint32_t primask)
{
    if (primask == 0u)
        __enable_irq();
}

bsp_can_endpoint_status_e bsp_can_endpoint_register(
    bsp_can_endpoint_t *endpoint,
    const bsp_can_endpoint_config_t *config)
{
    uint8_t i;
    uint8_t free_slot = BSP_CAN_ENDPOINT_MAX;
    uint32_t primask;

    if (endpoint == NULL || config == NULL)
        return BSP_CAN_ENDPOINT_ERR_ARGUMENT;
    if (config->bus >= BSP_CAN_NUM)
        return BSP_CAN_ENDPOINT_ERR_BUS;
    if (config->expected_dlc > 8u)
        return BSP_CAN_ENDPOINT_ERR_DLC;
    if (config->rx_callback != NULL && config->owner == NULL)
        return BSP_CAN_ENDPOINT_ERR_ARGUMENT;
    if (endpoint->registered)
        return BSP_CAN_ENDPOINT_ERR_ALREADY_REGISTERED;

    endpoint->bus = config->bus;
    endpoint->rx_id = config->rx_id;
    endpoint->expected_dlc = config->expected_dlc;
    endpoint->is_extend = config->is_extend ? 1u : 0u;
    endpoint->owner = config->owner;
    endpoint->rx_callback = config->rx_callback;
    endpoint->rx_count = 0u;
    endpoint->last_rx_ms = 0u;

    if (endpoint->rx_callback == NULL) {
        endpoint->registered = 1u;
        return BSP_CAN_ENDPOINT_OK;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0u; i < BSP_CAN_ENDPOINT_MAX; i++) {
        bsp_can_endpoint_t *registered = rx_endpoints[i];

        if (registered == NULL) {
            if (free_slot == BSP_CAN_ENDPOINT_MAX)
                free_slot = i;
            continue;
        }
        if (registered->bus == endpoint->bus &&
            registered->rx_id == endpoint->rx_id &&
            registered->is_extend == endpoint->is_extend) {
            bsp_can_irq_restore(primask);
            return BSP_CAN_ENDPOINT_ERR_DUPLICATE;
        }
    }

    if (free_slot == BSP_CAN_ENDPOINT_MAX) {
        bsp_can_irq_restore(primask);
        return BSP_CAN_ENDPOINT_ERR_FULL;
    }

    endpoint->registered = 1u;
    __DMB();
    rx_endpoints[free_slot] = endpoint;
    bsp_can_irq_restore(primask);
    return BSP_CAN_ENDPOINT_OK;
}

uint8_t bsp_can_endpoint_send(const bsp_can_endpoint_t *endpoint,
                              uint16_t tx_id,
                              uint8_t *data,
                              uint8_t len)
{
    if (endpoint == NULL || !endpoint->registered)
        return 0u;
    return bsp_can_send(endpoint->bus, tx_id, data, len);
}

static void bsp_can_endpoint_dispatch(const can_raw_msg_t *msg)
{
    uint8_t i;

    for (i = 0u; i < BSP_CAN_ENDPOINT_MAX; i++) {
        bsp_can_endpoint_t *endpoint = rx_endpoints[i];

        if (endpoint == NULL || endpoint->bus != msg->bus ||
            endpoint->rx_id != msg->id ||
            endpoint->is_extend != msg->is_extend)
            continue;
        if (endpoint->expected_dlc != 0u &&
            endpoint->expected_dlc != msg->dlc)
            return;

        endpoint->rx_count++;
        endpoint->last_rx_ms = msg->timestamp_ms;
        endpoint->rx_callback(endpoint->owner, msg);
        return;
    }
}

static uint8_t bsp_can_rx_bus(FDCAN_HandleTypeDef *hfdcan,
                              bsp_can_id_e *bus)
{
    if (hfdcan == &hfdcan1) *bus = BSP_CAN1;
    else if (hfdcan == &hfdcan2) *bus = BSP_CAN2;
    else if (hfdcan == &hfdcan3) *bus = BSP_CAN3;
    else return 0u;
    return 1u;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    can_raw_msg_t msg;

    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u)
        return;
    if (!bsp_can_rx_bus(hfdcan, &msg.bus))
        return;
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &rx_header,
                               rx_data) != HAL_OK) {
        if (hfdcan == &hfdcan1)
            debug_can1_rx_get_fail++;
        return;
    }

    msg.is_extend = (rx_header.IdType == FDCAN_EXTENDED_ID);
    msg.id = rx_header.Identifier;
    if (hfdcan == &hfdcan1) {
        debug_can1_rx_total++;
        debug_can1_last_id = msg.id;
        switch (msg.id) {
        case 0x201u: debug_can1_rx_201_hits++; break;
        case 0x202u: debug_can1_rx_202_hits++; break;
        case 0x203u: debug_can1_rx_203_hits++; break;
        case 0x204u: debug_can1_rx_204_hits++; break;
        default: break;
        }
    }
    /* H723 HAL 的 FDCAN_DLC_BYTES_x 直接使用 0..15 编码，不是 F4 风格位域。 */
    msg.dlc = (uint8_t)(rx_header.DataLength & 0x0fu);
    if (msg.dlc > 8u) msg.dlc = 8u;
    msg.timestamp_ms = HAL_GetTick();
    memset(msg.data, 0, sizeof(msg.data));
    memcpy(msg.data, rx_data, msg.dlc);
    bsp_can_endpoint_dispatch(&msg);
}

static uint32_t bsp_can_fdcan_dlc(uint8_t len)
{
    static const uint32_t dlc[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };
    return dlc[len];
}

uint8_t bsp_can_send(bsp_can_id_e can_bus,
                     uint16_t id,
                     const uint8_t *data,
                     uint8_t len)
{
    FDCAN_HandleTypeDef *hfdcan = bsp_can_handle(can_bus);
    FDCAN_TxHeaderTypeDef tx_header;
    HAL_StatusTypeDef hal_status;

    if (hfdcan == NULL || data == NULL || len == 0u || len > 8u)
        return 0u;

    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = bsp_can_fdcan_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    hal_status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);

    if (can_bus == BSP_CAN3 && id == 0x105u) {
        jscope_can105_hal_status = (uint32_t)hal_status;
        jscope_fdcan3_state = (uint32_t)hfdcan->State;
        jscope_fdcan3_error_code = hfdcan->ErrorCode;
        jscope_fdcan3_cccr = hfdcan->Instance->CCCR;
        jscope_fdcan3_psr = hfdcan->Instance->PSR;
        jscope_fdcan3_txfqs = hfdcan->Instance->TXFQS;
    }

    return hal_status == HAL_OK;
}
