#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "struct_typedef.h"

#define BSP_SPI1 0u

typedef enum {
    BSP_SPI_PRESCALER_2 = 2,
    BSP_SPI_PRESCALER_4 = 4,
    BSP_SPI_PRESCALER_8 = 8,
    BSP_SPI_PRESCALER_16 = 16,
    BSP_SPI_PRESCALER_32 = 32,
    BSP_SPI_PRESCALER_64 = 64,
    BSP_SPI_PRESCALER_128 = 128,
    BSP_SPI_PRESCALER_256 = 256,
} bsp_spi_prescaler_e;

typedef struct bsp_spi_endpoint bsp_spi_endpoint_t;

/* Necessary BSP -> device asynchronous callback; bound once during init. */
typedef void (*bsp_spi_dma_complete_cb_t)(
    void *owner,
    const bsp_spi_endpoint_t *endpoint);

typedef struct {
    uint8_t bus;
    void *cs_port;
    uint16_t cs_pin;
    void *owner;
    bsp_spi_dma_complete_cb_t complete_callback;
} bsp_spi_endpoint_config_t;

struct bsp_spi_endpoint {
    uint8_t bus;
    void *cs_port;
    uint16_t cs_pin;
    void *owner;
    bsp_spi_dma_complete_cb_t complete_callback;

    volatile uint32_t transfer_count;
    volatile uint8_t busy;
    uint8_t registered;
};

typedef enum {
    BSP_SPI_ENDPOINT_OK = 0,
    BSP_SPI_ENDPOINT_ERR_ARGUMENT,
    BSP_SPI_ENDPOINT_ERR_BUS,
    BSP_SPI_ENDPOINT_ERR_ALREADY_REGISTERED,
} bsp_spi_endpoint_status_e;

bsp_spi_endpoint_status_e bsp_spi_endpoint_register(
    bsp_spi_endpoint_t *endpoint,
    const bsp_spi_endpoint_config_t *config);

/* Configure the physical SPI bus and its DMA streams once before first use. */
uint8_t bsp_spi_dma_bus_init(uint8_t bus, bsp_spi_prescaler_e prescaler);

/*
 * Start one full-duplex DMA transaction. BSP owns CS and the active transfer
 * until completion; tx/rx buffers must remain valid until the callback.
 */
uint8_t bsp_spi_dma_start(bsp_spi_endpoint_t *endpoint,
                          const uint8_t *tx_buf,
                          uint8_t *rx_buf,
                          uint16_t length);

uint8_t bsp_spi_dma_is_busy(uint8_t bus);

/* Blocking protocol helpers; device drivers still define transaction bounds. */
uint8_t bsp_spi_transaction_begin(const bsp_spi_endpoint_t *endpoint);
void bsp_spi_transaction_end(const bsp_spi_endpoint_t *endpoint);
uint8_t bsp_spi_transfer_byte(const bsp_spi_endpoint_t *endpoint,
                              uint8_t tx_data,
                              uint8_t *rx_data,
                              uint32_t timeout_ms);

#endif
