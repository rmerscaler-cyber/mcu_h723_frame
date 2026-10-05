#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "main.h"

typedef struct
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;
    DMA_HandleTypeDef  *hdma_tx;   /* 无发送需求可留 NULL */
} bsp_uart_dma_map_t;

/* 逻辑UART编号: 上层代码只认这个枚举, 不接触 HAL 句柄 */
typedef enum
{
    BSP_UART5 = 0,
    BSP_UART6,
    BSP_UART10,
    BSP_UART_NUM,   /* 哨兵: 注册表长度 + 边界检查 */
} bsp_uart_id_t;

/* 收到一帧数据的回调 (在 UART IDLE 中断上下文执行, 处理要短小) */
typedef void (*bsp_uart_rx_cb_t)(bsp_uart_id_t uartx, uint8_t *data, uint16_t size);

/* 上层注册配置: 只描述"用哪个串口、DMA双缓冲放哪、数据怎么处理",
   huart/hdma 句柄绑定由 BSP 内部查 uart_dma_map 注册表完成 */
typedef struct
{
    bsp_uart_id_t   uartx;        /* 逻辑串口号 */
    uint8_t        *buffer_a;     /* DMA 双缓冲 A */
    uint8_t        *buffer_b;     /* DMA 双缓冲 B */
    uint16_t        buffer_size;  /* 单缓冲长度 */
    bsp_uart_rx_cb_t rx_cb;       /* 收到一帧回调, 可 NULL */
} BSP_UART_DMA_RX_Config;

typedef struct
{
    const bsp_uart_dma_map_t *map;
    uint8_t          *buffer_a;
    uint8_t          *buffer_b;
    uint16_t          buffer_size;
    bsp_uart_rx_cb_t  rx_cb;
    uint8_t           active;
} bsp_uart_dma_rx_handle_t;

void bsp_uart_dma_rx_init(const BSP_UART_DMA_RX_Config *cfg);
void bsp_uart_dma_rx_register_cb(bsp_uart_id_t uartx, bsp_uart_rx_cb_t cb);
void bsp_uart_dma_rx_restart(bsp_uart_id_t uartx);   /* 掉线/出错后重拉DMA */
void bsp_uart_dma_rx_stop(bsp_uart_id_t uartx);
void bsp_uart_dma_tx_send(bsp_uart_id_t uartx, uint8_t *data, uint16_t len);

#endif
