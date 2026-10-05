#ifndef BSP_RC_H
#define BSP_RC_H
#include "struct_typedef.h"
#include "bsp_usart.h"

/* 遥控器走 UART5 DMA 双缓冲, 句柄绑定由 bsp_usart 内部注册表完成 */
void RC_Init(bsp_uart_id_t uartx, uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);
#endif
