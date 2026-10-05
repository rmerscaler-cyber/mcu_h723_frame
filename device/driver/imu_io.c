#include "imu_io.h"

#include <string.h>

#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "ist8310driver.h"
#include "bsp_exti.h"
#include "bsp_spi.h"
#include "bsp_imu_pwm.h"
#include "imu_port.h"
#include "main.h"
#include "cmsis_os.h"

/*
 * 设备侧 IMU IO: 拥有 SPI/EXTI endpoint、DMA 缓冲、DR→SPI→UPDATE→NOTIFY
 * 位旗标状态机和 bring-up 状态机。模型只通过 device_imu_io_step() 取得
 * 解码后的未标定样本; 芯片寄存器偏移、帧长只出现在本文件/imu_io.h。
 */

/* BMI088 寄存器读取帧的 SPI 数据偏移 (设备细节) */
#define IMU_IO_BMI088_GYRO_RX_DATA_OFFSET  1u
#define IMU_IO_BMI088_ACCEL_RX_DATA_OFFSET 2u

#define IMU_IO_DR_SHIFT     0u
#define IMU_IO_SPI_SHIFT    1u
#define IMU_IO_UPDATE_SHIFT 2u
#define IMU_IO_NOTIFY_SHIFT 3u

typedef enum {
    IMU_IO_CH_GYRO,
    IMU_IO_CH_ACCEL,
    IMU_IO_CH_ACCEL_TEMP,
    IMU_IO_CH_MAG,
} imu_io_channel_e;

/* One physical IMU is installed on this board. Its endpoints live in device. */
static bsp_spi_endpoint_t gyro_spi;
static bsp_spi_endpoint_t accel_spi;
static bsp_exti_endpoint_t gyro_drdy;
static bsp_exti_endpoint_t accel_drdy;
static bsp_exti_endpoint_t mag_drdy;

static void imu_io_cmd_spi_dma(device_imu_instance_t *dev);

static void imu_io_notify_from_isr(device_imu_instance_t *dev)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    TaskHandle_t task_handle = (TaskHandle_t)dev->notification_target;

    if (task_handle == NULL ||
        xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        return;
    vTaskNotifyGiveFromISR(task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void imu_io_on_data_ready(device_imu_instance_t *dev,
                                 imu_io_channel_e channel)
{
    if (dev == NULL)
        return;

    if (channel == IMU_IO_CH_ACCEL) {
        dev->accel_update_flag |= 1u << IMU_IO_DR_SHIFT;
        dev->accel_temp_update_flag |= 1u << IMU_IO_DR_SHIFT;
        if (dev->start_dma_flag)
            imu_io_cmd_spi_dma(dev);
    } else if (channel == IMU_IO_CH_GYRO) {
        dev->gyro_update_flag |= 1u << IMU_IO_DR_SHIFT;
        if (dev->start_dma_flag)
            imu_io_cmd_spi_dma(dev);
    } else if (channel == IMU_IO_CH_MAG) {
        dev->mag_update_flag |= 1u << IMU_IO_DR_SHIFT;
    }
}

static void imu_io_on_dma_complete(device_imu_instance_t *dev,
                                   imu_io_channel_e channel)
{
    if (dev == NULL)
        return;

    if (channel == IMU_IO_CH_GYRO) {
        if (dev->gyro_update_flag & (1u << IMU_IO_SPI_SHIFT)) {
            dev->gyro_update_flag &= ~(1u << IMU_IO_SPI_SHIFT);
            dev->gyro_update_flag |= 1u << IMU_IO_UPDATE_SHIFT;
        }
    } else if (channel == IMU_IO_CH_ACCEL ||
               channel == IMU_IO_CH_ACCEL_TEMP) {
        if (dev->accel_update_flag & (1u << IMU_IO_SPI_SHIFT)) {
            dev->accel_update_flag &= ~(1u << IMU_IO_SPI_SHIFT);
            dev->accel_update_flag |= 1u << IMU_IO_UPDATE_SHIFT;
        }
        if (dev->accel_temp_update_flag & (1u << IMU_IO_SPI_SHIFT)) {
            dev->accel_temp_update_flag &= ~(1u << IMU_IO_SPI_SHIFT);
            dev->accel_temp_update_flag |= 1u << IMU_IO_UPDATE_SHIFT;
        }
    }

    imu_io_cmd_spi_dma(dev);

    if (dev->gyro_update_flag & (1u << IMU_IO_UPDATE_SHIFT)) {
        dev->gyro_update_flag &= ~(1u << IMU_IO_UPDATE_SHIFT);
        dev->gyro_update_flag |= 1u << IMU_IO_NOTIFY_SHIFT;
        imu_io_notify_from_isr(dev);
    }
}

static void imu_io_cmd_spi_dma(device_imu_instance_t *dev)
{
    if ((dev->gyro_update_flag & (1u << IMU_IO_DR_SHIFT)) &&
        !bsp_spi_dma_is_busy(BSP_SPI1) &&
        !(dev->accel_update_flag & (1u << IMU_IO_SPI_SHIFT)) &&
        !(dev->accel_temp_update_flag & (1u << IMU_IO_SPI_SHIFT))) {
        dev->gyro_update_flag &= ~(1u << IMU_IO_DR_SHIFT);
        dev->gyro_update_flag |= 1u << IMU_IO_SPI_SHIFT;
        if (!bsp_spi_dma_start(&gyro_spi,
                               dev->gyro_dma_tx_buf,
                               dev->gyro_dma_rx_buf,
                               IMU_IO_SPI_DMA_GYRO_LENGTH)) {
            dev->gyro_update_flag &= ~(1u << IMU_IO_SPI_SHIFT);
            dev->gyro_update_flag |= 1u << IMU_IO_DR_SHIFT;
        }
        return;
    }

    if ((dev->accel_update_flag & (1u << IMU_IO_DR_SHIFT)) &&
        !bsp_spi_dma_is_busy(BSP_SPI1) &&
        !(dev->gyro_update_flag & (1u << IMU_IO_SPI_SHIFT)) &&
        !(dev->accel_temp_update_flag & (1u << IMU_IO_SPI_SHIFT))) {
        dev->accel_update_flag &= ~(1u << IMU_IO_DR_SHIFT);
        dev->accel_update_flag |= 1u << IMU_IO_SPI_SHIFT;
        if (!bsp_spi_dma_start(&accel_spi,
                               dev->accel_dma_tx_buf,
                               dev->accel_dma_rx_buf,
                               IMU_IO_SPI_DMA_ACCEL_LENGTH)) {
            dev->accel_update_flag &= ~(1u << IMU_IO_SPI_SHIFT);
            dev->accel_update_flag |= 1u << IMU_IO_DR_SHIFT;
        }
        return;
    }

    if ((dev->accel_temp_update_flag & (1u << IMU_IO_DR_SHIFT)) &&
        !bsp_spi_dma_is_busy(BSP_SPI1) &&
        !(dev->gyro_update_flag & (1u << IMU_IO_SPI_SHIFT)) &&
        !(dev->accel_update_flag & (1u << IMU_IO_SPI_SHIFT))) {
        dev->accel_temp_update_flag &= ~(1u << IMU_IO_DR_SHIFT);
        dev->accel_temp_update_flag |= 1u << IMU_IO_SPI_SHIFT;
        if (!bsp_spi_dma_start(&accel_spi,
                               dev->accel_temp_dma_tx_buf,
                               dev->accel_temp_dma_rx_buf,
                               IMU_IO_SPI_DMA_ACCEL_TEMP_LENGTH)) {
            dev->accel_temp_update_flag &= ~(1u << IMU_IO_SPI_SHIFT);
            dev->accel_temp_update_flag |= 1u << IMU_IO_DR_SHIFT;
        }
    }
}

static void imu_io_exti_callback(void *owner, uint16_t pin)
{
    device_imu_instance_t *dev = (device_imu_instance_t *)owner;

    if (pin == ACC_INT_Pin)
        imu_io_on_data_ready(dev, IMU_IO_CH_ACCEL);
    else if (pin == GYRO_INT_Pin)
        imu_io_on_data_ready(dev, IMU_IO_CH_GYRO);
    else if (pin == DRDY_IST8310_Pin)
        imu_io_on_data_ready(dev, IMU_IO_CH_MAG);
}

static void imu_io_dma_callback(void *owner,
                                const bsp_spi_endpoint_t *endpoint)
{
    device_imu_instance_t *dev = (device_imu_instance_t *)owner;

    if (endpoint == &gyro_spi)
        imu_io_on_dma_complete(dev, IMU_IO_CH_GYRO);
    else if (endpoint == &accel_spi)
        imu_io_on_dma_complete(dev, IMU_IO_CH_ACCEL);
}

uint8_t device_imu_io_configure(device_imu_instance_t *dev,
                                void *notification_target)
{
    static const uint8_t gyro_tx[IMU_IO_SPI_DMA_GYRO_LENGTH] = {
        0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };
    static const uint8_t accel_tx[IMU_IO_SPI_DMA_ACCEL_LENGTH] = {
        0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };
    static const uint8_t accel_temp_tx[IMU_IO_SPI_DMA_ACCEL_TEMP_LENGTH] = {
        0xA2, 0xFF, 0xFF, 0xFF
    };
    bsp_spi_endpoint_config_t spi_config;
    bsp_exti_endpoint_config_t exti_config;

    if (dev == NULL || notification_target == NULL)
        return 0u;

    /* 只初始化设备私有区; algo 由 imu_algorithm_configure 负责。 */
    dev->init_stage = IMU_IO_INIT_GYRO_ACCEL;
    dev->init_retry_count = 0u;
    dev->notification_target = notification_target;
    dev->gyro_update_flag = 0u;
    dev->accel_update_flag = 0u;
    dev->accel_temp_update_flag = 0u;
    dev->mag_update_flag = 0u;
    dev->start_dma_flag = 0u;
    dev->accel_sensor_time = 0.0f;
    memset(&dev->sample, 0, sizeof(dev->sample));
    memcpy(dev->gyro_dma_tx_buf, gyro_tx, sizeof(gyro_tx));
    memcpy(dev->accel_dma_tx_buf, accel_tx, sizeof(accel_tx));
    memcpy(dev->accel_temp_dma_tx_buf, accel_temp_tx, sizeof(accel_temp_tx));

    spi_config.bus = BSP_SPI1;
    spi_config.owner = dev;
    spi_config.complete_callback = imu_io_dma_callback;
    spi_config.cs_port = GYRO_CS_GPIO_Port;
    spi_config.cs_pin = GYRO_CS_Pin;
    if (bsp_spi_endpoint_register(&gyro_spi, &spi_config) != BSP_SPI_ENDPOINT_OK)
        return 0u;

    spi_config.cs_port = ACC_CS_GPIO_Port;
    spi_config.cs_pin = ACC_CS_Pin;
    if (bsp_spi_endpoint_register(&accel_spi, &spi_config) != BSP_SPI_ENDPOINT_OK)
        return 0u;
    BMI088_middleware_bind_spi(&gyro_spi, &accel_spi);

    exti_config.owner = dev;
    exti_config.callback = imu_io_exti_callback;
    exti_config.pin = GYRO_INT_Pin;
    if (bsp_exti_endpoint_register(&gyro_drdy, &exti_config) != BSP_EXTI_ENDPOINT_OK)
        return 0u;
    exti_config.pin = ACC_INT_Pin;
    if (bsp_exti_endpoint_register(&accel_drdy, &exti_config) != BSP_EXTI_ENDPOINT_OK)
        return 0u;
    exti_config.pin = DRDY_IST8310_Pin;
    if (bsp_exti_endpoint_register(&mag_drdy, &exti_config) != BSP_EXTI_ENDPOINT_OK)
        return 0u;
    return 1u;
}

imu_io_init_stage_e device_imu_io_init(device_imu_instance_t *dev)
{
    if (dev == NULL)
        return IMU_IO_INIT_GYRO_ACCEL;

    switch (dev->init_stage) {
    case IMU_IO_INIT_GYRO_ACCEL:
        if (BMI088_init() != 0u) {
            dev->init_retry_count++;
            return dev->init_stage;
        }
        dev->init_stage = IMU_IO_INIT_MAG;
        /* fall through */
    case IMU_IO_INIT_MAG:
        if (ist8310_init() != 0u) {
            dev->init_retry_count++;
            return dev->init_stage;
        }
        dev->init_stage = IMU_IO_INIT_DMA_ON;
        /* fall through */
    case IMU_IO_INIT_DMA_ON:
        /* 首帧阻塞采样: 模型 imu_algorithm_init 的种子数据。 */
        BMI088_read(dev->sample.gyro, dev->sample.accel, &dev->sample.temp);
        if (!bsp_spi_dma_bus_init(BSP_SPI1, BSP_SPI_PRESCALER_8)) {
            dev->init_retry_count++;
            return dev->init_stage;
        }
        dev->start_dma_flag = 1u;
        dev->init_stage = IMU_IO_INIT_DONE;
        break;
    default:
        break;
    }

    return dev->init_stage;
}

void device_imu_io_step(device_imu_instance_t *dev, imu_raw_sample_t *sample)
{
    uint8_t temp_updated = 0u;

    if (dev == NULL || sample == NULL)
        return;

    if (dev->gyro_update_flag & (1u << IMU_IO_NOTIFY_SHIFT)) {
        dev->gyro_update_flag &= ~(1u << IMU_IO_NOTIFY_SHIFT);
        BMI088_gyro_read_over(
            dev->gyro_dma_rx_buf + IMU_IO_BMI088_GYRO_RX_DATA_OFFSET,
            dev->sample.gyro);
    }

    if (dev->accel_update_flag & (1u << IMU_IO_UPDATE_SHIFT)) {
        dev->accel_update_flag &= ~(1u << IMU_IO_UPDATE_SHIFT);
        BMI088_accel_read_over(
            dev->accel_dma_rx_buf + IMU_IO_BMI088_ACCEL_RX_DATA_OFFSET,
            dev->sample.accel,
            &dev->accel_sensor_time);
    }

    if (dev->accel_temp_update_flag & (1u << IMU_IO_UPDATE_SHIFT)) {
        dev->accel_temp_update_flag &= ~(1u << IMU_IO_UPDATE_SHIFT);
        BMI088_temperature_read_over(
            dev->accel_temp_dma_rx_buf + IMU_IO_BMI088_ACCEL_RX_DATA_OFFSET,
            &dev->sample.temp);
        temp_updated = 1u;
    }

    /* 磁力计采集保持关闭, 只维护旗标语义。 */
    if (dev->mag_update_flag & (1u << IMU_IO_DR_SHIFT)) {
        dev->mag_update_flag &= ~(1u << IMU_IO_DR_SHIFT);
        dev->mag_update_flag |= 1u << IMU_IO_SPI_SHIFT;
        /* ist8310_read_over(..., &dev->sample.mag); */
    }

    *sample = dev->sample;
    sample->temp_updated = temp_updated;
}

/* 模型 imu_port.h 的唯一端口: 加热片 PWM。 */
void imu_port_pwm_set(uint16_t pwm)
{
    imu_pwm_set(pwm);
}
