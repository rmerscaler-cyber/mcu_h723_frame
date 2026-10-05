/**
 * bsp_usart: UART DMA 双缓冲接收 + DMA 发送 (H7)
 *
 *   - 逻辑串口号(BSP_UART5/6/10) -> HAL句柄 通过 const 注册表 uart_dma_map 绑定,
 *     上层只传枚举, 不接触 huart/hdma。
 *   - HAL_UARTEx_RxEventCallback 在本文件实现(整个工程唯一一份), 按 huart 查表
 *     分发到各串口注册的 rx_cb, 完成双缓冲切换与重装。
 *   - 旧接口 usart5_tx_dma_init/usart6_init/usart10_* 已被本文件取代(原为死代码)。
 */
#include "bsp_usart.h"

/* CubeMX 生成的句柄 (Core/Src/usart.c) */
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;

/* ============ 逻辑UART -> HAL句柄 注册表 (唯一事实来源) ============ */
static const bsp_uart_dma_map_t uart_dma_map[BSP_UART_NUM] =
{
    [BSP_UART5]  = { .huart = &huart5,        .hdma_rx = &hdma_uart5_rx,   .hdma_tx = NULL },
    [BSP_UART6]  = { .huart = &huart6,        .hdma_rx = &hdma_usart6_rx,  .hdma_tx = &hdma_usart6_tx },
    [BSP_UART10] = { .huart = &huart10,       .hdma_rx = &hdma_usart10_rx, .hdma_tx = &hdma_usart10_tx },
};

/* 每个串口的运行时状态 (缓冲 + 回调), 由 bsp_uart_dma_rx_init 填充 */
static bsp_uart_dma_rx_handle_t uart_rx_handles[BSP_UART_NUM] = {0};

static const bsp_uart_dma_map_t *bsp_uart_get_map(bsp_uart_id_t uartx)
{
    if (uartx >= BSP_UART_NUM)
    {
        return NULL;
    }
    return &uart_dma_map[uartx];
}

void bsp_uart_dma_rx_init(const BSP_UART_DMA_RX_Config *cfg)
{
    if (cfg == NULL || cfg->uartx >= BSP_UART_NUM)
    {
        return;
    }

    const bsp_uart_dma_map_t *map = bsp_uart_get_map(cfg->uartx);
    if (map == NULL || map->huart == NULL || map->hdma_rx == NULL ||
        cfg->buffer_a == NULL || cfg->buffer_b == NULL || cfg->buffer_size == 0)
    {
        return;
    }

    bsp_uart_dma_rx_handle_t *h   = &uart_rx_handles[cfg->uartx];
    UART_HandleTypeDef       *huart = map->huart;
    DMA_HandleTypeDef        *hdma  = map->hdma_rx;
    DMA_Stream_TypeDef       *dma   = (DMA_Stream_TypeDef *)hdma->Instance;

    h->map         = map;
    h->buffer_a    = cfg->buffer_a;
    h->buffer_b    = cfg->buffer_b;
    h->buffer_size = cfg->buffer_size;
    h->rx_cb       = cfg->rx_cb;
    h->active      = 1;

    /* 复用 HAL 空闲接收事件路径: HAL_UART_IRQHandler(IDLE) -> HAL_UARTEx_RxEventCallback */
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType   = HAL_UART_RXEVENT_IDLE;
    huart->RxXferSize    = cfg->buffer_size;

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    /* DMA 双缓冲配置 */
    __HAL_DMA_DISABLE(hdma);
    while (dma->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma);
    }
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

    dma->PAR  = (uint32_t)&huart->Instance->RDR;
    dma->M0AR = (uint32_t)cfg->buffer_a;
    dma->M1AR = (uint32_t)cfg->buffer_b;
    __HAL_DMA_SET_COUNTER(hdma, cfg->buffer_size);
    SET_BIT(dma->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(hdma);
}

void bsp_uart_dma_rx_register_cb(bsp_uart_id_t uartx, bsp_uart_rx_cb_t cb)
{
    if (uartx >= BSP_UART_NUM)
    {
        return;
    }
    uart_rx_handles[uartx].rx_cb = cb;
}

void bsp_uart_dma_rx_restart(bsp_uart_id_t uartx)
{
    if (uartx >= BSP_UART_NUM)
    {
        return;
    }
    bsp_uart_dma_rx_handle_t *h = &uart_rx_handles[uartx];
    if (!h->active)
    {
        return;
    }

    UART_HandleTypeDef *huart = h->map->huart;
    DMA_HandleTypeDef  *hdma  = h->map->hdma_rx;
    DMA_Stream_TypeDef *dma   = (DMA_Stream_TypeDef *)hdma->Instance;

    __HAL_UART_DISABLE(huart);
    __HAL_DMA_DISABLE(hdma);
    while (dma->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma);
    }

    /* 重新装载双缓冲, 恢复接收 (旧 RC_restart 写的是假句柄 hdma_uart5_stream_rx, 实际没生效) */
    dma->M0AR = (uint32_t)h->buffer_a;
    dma->M1AR = (uint32_t)h->buffer_b;
    __HAL_DMA_SET_COUNTER(hdma, h->buffer_size);
    SET_BIT(dma->CR, DMA_SxCR_DBM);
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

    __HAL_DMA_ENABLE(hdma);
    __HAL_UART_ENABLE(huart);
}

void bsp_uart_dma_rx_stop(bsp_uart_id_t uartx)
{
    if (uartx >= BSP_UART_NUM)
    {
        return;
    }
    bsp_uart_dma_rx_handle_t *h = &uart_rx_handles[uartx];
    if (!h->active)
    {
        return;
    }
    __HAL_UART_DISABLE(h->map->huart);
    __HAL_DMA_DISABLE(h->map->hdma_rx);
    h->active = 0;
}

void bsp_uart_dma_tx_send(bsp_uart_id_t uartx, uint8_t *data, uint16_t len)
{
    const bsp_uart_dma_map_t *map = bsp_uart_get_map(uartx);
    if (map == NULL || map->huart == NULL || map->hdma_tx == NULL || data == NULL || len == 0)
    {
        return;
    }

    DMA_HandleTypeDef  *hdma = map->hdma_tx;
    DMA_Stream_TypeDef *dma  = (DMA_Stream_TypeDef *)hdma->Instance;

    SET_BIT(map->huart->Instance->CR3, USART_CR3_DMAT);

    __HAL_DMA_DISABLE(hdma);
    while (dma->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma);
    }
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

    dma->PAR  = (uint32_t)&map->huart->Instance->TDR;
    dma->M0AR = (uint32_t)data;
    __HAL_DMA_SET_COUNTER(hdma, len);

    __HAL_DMA_ENABLE(hdma);
}

/* HAL 全局空闲接收事件回调 (整个工程唯一一份, 由本文件实现) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < BSP_UART_NUM; i++)
    {
        bsp_uart_dma_rx_handle_t *h = &uart_rx_handles[i];
        if (!h->active || h->map->huart != huart)
        {
            continue;
        }

        DMA_HandleTypeDef  *hdma = h->map->hdma_rx;
        DMA_Stream_TypeDef *dma  = (DMA_Stream_TypeDef *)hdma->Instance;

        __HAL_DMA_DISABLE(hdma);
        if ((dma->CR & DMA_SxCR_CT) == RESET)
        {
            /* 刚写满 buffer A: 切到 B 继续接收 */
            SET_BIT(dma->CR, DMA_SxCR_CT);
            __HAL_DMA_SET_COUNTER(hdma, h->buffer_size);
            if (h->rx_cb != NULL)
            {
                h->rx_cb((bsp_uart_id_t)i, h->buffer_a, Size);
            }
        }
        else
        {
            /* 刚写满 buffer B: 切回 A */
            CLEAR_BIT(dma->CR, DMA_SxCR_CT);
            __HAL_DMA_SET_COUNTER(hdma, h->buffer_size);
            if (h->rx_cb != NULL)
            {
                h->rx_cb((bsp_uart_id_t)i, h->buffer_b, Size);
            }
        }
        __HAL_DMA_ENABLE(hdma);
        break;
    }
}
