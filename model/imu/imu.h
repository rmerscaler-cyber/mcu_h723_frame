#ifndef IMU_H
#define IMU_H

#include "struct_typedef.h"
#include "pid.h"

typedef enum {
    IMU_ANGLE_YAW = 0,
    IMU_ANGLE_PITCH,
    IMU_ANGLE_ROLL,
} imu_angle_index_e;

typedef enum {
    IMU_GYRO_X = 0,
    IMU_GYRO_Y,
    IMU_GYRO_Z,
} imu_gyro_index_e;

typedef struct {
    fp32 quaternion[4];
    fp32 angle[3];       /* yaw, pitch, roll; unit rad */
    fp32 gyro[3];        /* body x, y, z; unit rad/s */
    fp32 accel[3];       /* body x, y, z; unit m/s^2 */
    fp32 mag[3];         /* body x, y, z; unit uT */
    fp32 temperature;
    uint32_t sequence;
    uint8_t ready;
} imu_snapshot_t;

/* 设备层解码后的未标定原始样本；标定、滤波和温控都在算法层完成。 */
typedef struct {
    fp32 gyro[3];
    fp32 accel[3];
    fp32 mag[3];
    fp32 temp;
    uint8_t temp_updated;   /* 本周期温度通道有新帧（温控环只在有帧时推进） */
} imu_raw_sample_t;

typedef struct {
    fp32 gyro_scale_factor[3][3];
    fp32 accel_scale_factor[3][3];
    fp32 mag_scale_factor[3][3];
    pid_config_t temperature_pid;
    fp32 timing_time;
    fp32 control_temperature;
    uint16_t temperature_pwm_max;
} imu_config_t;

/*
 * 算法实例: 一个 IMU owner 任务持有一份。收发缓冲、原始样本、update 旗标和
 * bring-up 状态在设备侧的 device_imu_instance_t；本结构只保存解算状态、标定
 * 零偏和发布快照，不出现任何芯片/总线细节。
 */
typedef struct {
    imu_config_t config;
    imu_snapshot_t snapshot;
    volatile uint32_t snapshot_guard;

    fp32 gyro_offset[3];
    fp32 gyro_cali_offset[3];
    fp32 accel_offset[3];
    fp32 accel_cali_offset[3];
    fp32 mag_offset[3];
    fp32 mag_cali_offset[3];

    fp32 accel_filter_1[3];
    fp32 accel_filter_2[3];
    fp32 accel_filter_3[3];

    pid_type_def temperature_pid;
    uint8_t first_temperature;
    uint8_t temperature_constant_time;

    uint32_t applied_temperature_sequence;
    uint32_t applied_gyro_cali_sequence;
    uint8_t configured;
} imu_algorithm_t;

void imu_algorithm_configure(imu_algorithm_t *imu,
                             const imu_config_t *config);
/* 设备 bring-up 完成后调用一次：用首帧原始样本完成标定解算与 AHRS 种子。 */
void imu_algorithm_init(imu_algorithm_t *imu,
                        const imu_raw_sample_t *sample);
void imu_algorithm_step(imu_algorithm_t *imu,
                        const imu_raw_sample_t *sample);
/* 仅由 IMU owner 任务调用：把 mailbox 中的新输入应用到指定实例。 */
void imu_algorithm_apply_pending_inputs(imu_algorithm_t *imu);

/* Copies one coherent default-instance publication; leaves output unchanged on collision. */
uint8_t imu_read_snapshot(imu_snapshot_t *snapshot);
/* 生产者接口：只提交完整输入，不直接修改 imu_algorithm_t。 */
void imu_submit_control_temperature(fp32 control_temperature);
void imu_submit_gyro_calibration(const fp32 cali_scale[3],
                                 const fp32 cali_offset[3]);
/* 使用最新完整快照按原公式推进一步；快照不可用时返回 0 且不增加计数。 */
uint8_t imu_run_gyro_calibration_step(fp32 cali_scale[3],
                                      fp32 cali_offset[3],
                                      uint16_t *time_count);

#endif
