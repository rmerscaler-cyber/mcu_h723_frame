/**
 * bsp_rc: 遥控器(SBUS over UART5) 薄封装
 *   - 底层 DMA 双缓冲接收由 bsp_usart 统一管理, 这里只做参数装配
 *   - 帧解析回调由 remote_control 通过 bsp_uart_dma_rx_register_cb 注册
 */
#include "bsp_rc.h"
#include "bsp_usart.h"

void RC_Init(bsp_uart_id_t uartx, uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    BSP_UART_DMA_RX_Config cfg = {
        .uartx       = uartx,
        .buffer_a    = rx1_buf,
        .buffer_b    = rx2_buf,
        .buffer_size = dma_buf_num,
        .rx_cb       = NULL,
    };
    bsp_uart_dma_rx_init(&cfg);
}

void RC_unable(void)
{
    bsp_uart_dma_rx_stop(BSP_UART5);
}

void RC_restart(uint16_t dma_buf_num)
{
    (void)dma_buf_num;   /* 缓冲长度已注册在 bsp_usart 句柄里, 重启时沿用 */
    bsp_uart_dma_rx_restart(BSP_UART5);
}
