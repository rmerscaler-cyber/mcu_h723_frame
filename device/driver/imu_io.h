#ifndef IMU_IO_H
#define IMU_IO_H

#include "imu.h"

/* IMU SPI DMA 帧长 (与具体传感器寄存器读取帧一致, 设备私有) */
#define IMU_IO_SPI_DMA_GYRO_LENGTH        8u
#define IMU_IO_SPI_DMA_ACCEL_LENGTH       9u
#define IMU_IO_SPI_DMA_ACCEL_TEMP_LENGTH  4u

/* 设备 bring-up 阶段: 通道语义, 不出现芯片名。 */
typedef enum {
    IMU_IO_INIT_GYRO_ACCEL = 0,
    IMU_IO_INIT_MAG,
    IMU_IO_INIT_DMA_ON,
    IMU_IO_INIT_DONE,
} imu_io_init_stage_e;

/*
 * 设备组合实例: imu_algorithm_t 为第一成员, 中间层句柄即 &dev->algo;
 * DMA 缓冲、原始样本、update 旗标和 bring-up 状态都只存在于设备侧,
 * 由 owner 任务 (INS_task) 静态持有。
 */
typedef struct {
    imu_algorithm_t algo;   /* 第一成员: 算法状态与对外快照 */

    /* ---- bring-up 状态与 Watch 探针 ---- */
    imu_io_init_stage_e init_stage;
    uint16_t init_retry_count;
    void *notification_target;   /* IMU owner 任务的通知句柄 */

    /* ---- DMA 收发缓冲 ---- */
    uint8_t gyro_dma_rx_buf[IMU_IO_SPI_DMA_GYRO_LENGTH];
    uint8_t gyro_dma_tx_buf[IMU_IO_SPI_DMA_GYRO_LENGTH];
    uint8_t accel_dma_rx_buf[IMU_IO_SPI_DMA_ACCEL_LENGTH];
    uint8_t accel_dma_tx_buf[IMU_IO_SPI_DMA_ACCEL_LENGTH];
    uint8_t accel_temp_dma_rx_buf[IMU_IO_SPI_DMA_ACCEL_TEMP_LENGTH];
    uint8_t accel_temp_dma_tx_buf[IMU_IO_SPI_DMA_ACCEL_TEMP_LENGTH];

    /* ---- DR/SPI/UPDATE/NOTIFY 位旗标 ---- */
    volatile uint8_t gyro_update_flag;
    volatile uint8_t accel_update_flag;
    volatile uint8_t accel_temp_update_flag;
    volatile uint8_t mag_update_flag;
    volatile uint8_t start_dma_flag;

    /* ---- 解码后的未标定样本 (任务上下文消费) ---- */
    imu_raw_sample_t sample;
    fp32 accel_sensor_time;   /* 加速度计 SENSORTIME, 仅供调试观察 */
} device_imu_instance_t;

/* 注册 SPI/EXTI endpoint 并初始化设备运行态; 与 imu_algorithm_configure 相互独立。 */
uint8_t device_imu_io_configure(device_imu_instance_t *dev,
                                void *notification_target);
/* 非阻塞 bring-up: 返回当前阶段, IMU_IO_INIT_DONE 表示传感器与 DMA 就绪;
 * 失败由调用方延时重试, 重试次数记录在 init_retry_count。 */
imu_io_init_stage_e device_imu_io_init(device_imu_instance_t *dev);
/* 任务上下文: 消费 DMA 完成旗标并解码原始寄存器, 输出当前未标定样本。 */
void device_imu_io_step(device_imu_instance_t *dev, imu_raw_sample_t *sample);

#endif
