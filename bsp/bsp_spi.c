#include "bsp_spi.h"

#include "main.h"
#include "spi.h"

/* H7 的 IMU 物理总线是 SPI2，DMA 由 CubeMX 绑定到 DMA1 Stream0/1。 */
extern SPI_HandleTypeDef hspi2;

static bsp_spi_endpoint_t * volatile active_endpoint;
static const bsp_spi_endpoint_t * volatile blocking_endpoint;
static uint8_t spi2_dma_initialized;

static uint32_t bsp_spi_prescaler_value(bsp_spi_prescaler_e prescaler)
{
    switch (prescaler) {
    case BSP_SPI_PRESCALER_2:   return SPI_BAUDRATEPRESCALER_2;
    case BSP_SPI_PRESCALER_4:   return SPI_BAUDRATEPRESCALER_4;
    case BSP_SPI_PRESCALER_8:   return SPI_BAUDRATEPRESCALER_8;
    case BSP_SPI_PRESCALER_16:  return SPI_BAUDRATEPRESCALER_16;
    case BSP_SPI_PRESCALER_32:  return SPI_BAUDRATEPRESCALER_32;
    case BSP_SPI_PRESCALER_64:  return SPI_BAUDRATEPRESCALER_64;
    case BSP_SPI_PRESCALER_128: return SPI_BAUDRATEPRESCALER_128;
    case BSP_SPI_PRESCALER_256: return SPI_BAUDRATEPRESCALER_256;
    default:                    return 0u;
    }
}

bsp_spi_endpoint_status_e bsp_spi_endpoint_register(
    bsp_spi_endpoint_t *endpoint,
    const bsp_spi_endpoint_config_t *config)
{
    if (endpoint == NULL || config == NULL ||
        config->cs_port == NULL || config->cs_pin == 0u ||
        config->owner == NULL || config->complete_callback == NULL)
        return BSP_SPI_ENDPOINT_ERR_ARGUMENT;
    if (config->bus != BSP_SPI1)
        return BSP_SPI_ENDPOINT_ERR_BUS;
    if (endpoint->registered)
        return BSP_SPI_ENDPOINT_ERR_ALREADY_REGISTERED;

    endpoint->bus = config->bus;
    endpoint->cs_port = config->cs_port;
    endpoint->cs_pin = config->cs_pin;
    endpoint->owner = config->owner;
    endpoint->complete_callback = config->complete_callback;
    endpoint->transfer_count = 0u;
    endpoint->busy = 0u;
    endpoint->registered = 1u;
    return BSP_SPI_ENDPOINT_OK;
}

uint8_t bsp_spi_dma_bus_init(uint8_t bus, bsp_spi_prescaler_e prescaler)
{
    uint32_t hal_prescaler;

    if (bus != BSP_SPI1)
        return 0u;
    hal_prescaler = bsp_spi_prescaler_value(prescaler);
    if (hal_prescaler == 0u)
        return 0u;

    hspi2.Init.BaudRatePrescaler = hal_prescaler;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
        return 0u;

    active_endpoint = NULL;
    blocking_endpoint = NULL;
    spi2_dma_initialized = 1u;
    return 1u;
}

uint8_t bsp_spi_dma_is_busy(uint8_t bus)
{
    if (bus != BSP_SPI1 || !spi2_dma_initialized)
        return 0u;
    return active_endpoint != NULL || blocking_endpoint != NULL ||
           HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY;
}

uint8_t bsp_spi_transaction_begin(const bsp_spi_endpoint_t *endpoint)
{
    if (endpoint == NULL || !endpoint->registered ||
        endpoint->bus != BSP_SPI1)
        return 0u;
    if (active_endpoint != NULL || blocking_endpoint != NULL)
        return 0u;

    blocking_endpoint = endpoint;
    __DMB();
    HAL_GPIO_WritePin((GPIO_TypeDef *)endpoint->cs_port,
                      endpoint->cs_pin,
                      GPIO_PIN_RESET);
    return 1u;
}

void bsp_spi_transaction_end(const bsp_spi_endpoint_t *endpoint)
{
    if (endpoint == NULL || !endpoint->registered ||
        endpoint->bus != BSP_SPI1 || blocking_endpoint != endpoint)
        return;

    HAL_GPIO_WritePin((GPIO_TypeDef *)endpoint->cs_port,
                      endpoint->cs_pin,
                      GPIO_PIN_SET);
    blocking_endpoint = NULL;
    __DMB();
}

uint8_t bsp_spi_transfer_byte(const bsp_spi_endpoint_t *endpoint,
                              uint8_t tx_data,
                              uint8_t *rx_data,
                              uint32_t timeout_ms)
{
    if (endpoint == NULL || !endpoint->registered ||
        endpoint->bus != BSP_SPI1 || rx_data == NULL ||
        blocking_endpoint != endpoint)
        return 0u;

    return HAL_SPI_TransmitReceive(&hspi2,
                                   &tx_data,
                                   rx_data,
                                   1u,
                                   timeout_ms) == HAL_OK;
}

uint8_t bsp_spi_dma_start(bsp_spi_endpoint_t *endpoint,
                          const uint8_t *tx_buf,
                          uint8_t *rx_buf,
                          uint16_t length)
{
    if (endpoint == NULL || !endpoint->registered ||
        endpoint->bus != BSP_SPI1 || !spi2_dma_initialized ||
        tx_buf == NULL || rx_buf == NULL || length == 0u)
        return 0u;
    if (bsp_spi_dma_is_busy(endpoint->bus))
        return 0u;

    endpoint->busy = 1u;
    active_endpoint = endpoint;
    __DMB();
    HAL_GPIO_WritePin((GPIO_TypeDef *)endpoint->cs_port,
                      endpoint->cs_pin,
                      GPIO_PIN_RESET);

    /* DMA1 Stream0/1 与 SPI2 的关联和 IRQ 仍由 H7 CubeMX/HAL 管理。 */
    if (HAL_SPI_TransmitReceive_DMA(&hspi2,
                                    tx_buf,
                                    rx_buf,
                                    length) != HAL_OK) {
        HAL_GPIO_WritePin((GPIO_TypeDef *)endpoint->cs_port,
                          endpoint->cs_pin,
                          GPIO_PIN_SET);
        active_endpoint = NULL;
        endpoint->busy = 0u;
        return 0u;
    }
    return 1u;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    bsp_spi_endpoint_t *completed;

    if (hspi != &hspi2)
        return;

    completed = active_endpoint;
    if (completed == NULL)
        return;

    completed->busy = 0u;
    completed->transfer_count++;
    HAL_GPIO_WritePin((GPIO_TypeDef *)completed->cs_port,
                      completed->cs_pin,
                      GPIO_PIN_SET);
    active_endpoint = NULL;
    __DMB();
    completed->complete_callback(completed->owner, completed);
}
