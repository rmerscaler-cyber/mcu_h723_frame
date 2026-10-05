#include "imu.h"

#include <string.h>

#include "cmsis_gcc.h"
#include "imu_port.h"
#include "ahrs.h"

typedef struct {
    fp32 control_temperature;
    fp32 gyro_cali_scale[3];
    fp32 gyro_cali_offset[3];
    uint32_t temperature_sequence;
    uint32_t gyro_cali_sequence;
} imu_pending_input_t;

static const fp32 accel_filter_num[3] = {
    1.929454039488895f,
    -0.93178349823448126f,
    0.002329458745586203f,
};

/*
 * Default algorithm instance is owned by INS_task. It serves coherent snapshot
 * copies for the current single-IMU application facade.
 */
static imu_algorithm_t *default_imu;
/*
 * 外部任务只能向这份 mailbox 提交输入；只有 INS_task 会调用
 * imu_algorithm_apply_pending_inputs() 把输入写进 imu_algorithm_t。
 */
static volatile imu_pending_input_t pending_input;

static void imu_cali_solve(imu_algorithm_t *imu,
                           const imu_raw_sample_t *sample);
static void imu_temp_control(imu_algorithm_t *imu, fp32 temp);
static void imu_set_control_temperature(imu_algorithm_t *imu,
                                        fp32 control_temperature);
static void imu_apply_gyro_calibration(imu_algorithm_t *imu,
                                       const fp32 cali_scale[3],
                                       const fp32 cali_offset[3]);
static void gyro_offset_calc(fp32 gyro_offset[3],
                             const fp32 gyro[3],
                             uint16_t *offset_time_count);
static uint32_t imu_pending_input_lock(void);
static void imu_pending_input_unlock(uint32_t primask);

void imu_algorithm_configure(imu_algorithm_t *imu,
                             const imu_config_t *config)
{
    if (imu == NULL || config == NULL)
        return;

    memset(imu, 0, sizeof(*imu));
    memcpy(&imu->config, config, sizeof(imu->config));
    imu->configured = 1u;
    default_imu = imu;
}

void imu_algorithm_init(imu_algorithm_t *imu,
                        const imu_raw_sample_t *sample)
{
    if (imu == NULL || sample == NULL || !imu->configured)
        return;

    imu_cali_solve(imu, sample);

    PID_init(&imu->temperature_pid, &imu->config.temperature_pid);
    AHRS_init(imu->snapshot.quaternion,
              imu->snapshot.accel,
              imu->snapshot.mag);

    imu->accel_filter_1[0] = imu->accel_filter_2[0] =
        imu->accel_filter_3[0] = imu->snapshot.accel[0];
    imu->accel_filter_1[1] = imu->accel_filter_2[1] =
        imu->accel_filter_3[1] = imu->snapshot.accel[1];
    imu->accel_filter_1[2] = imu->accel_filter_2[2] =
        imu->accel_filter_3[2] = imu->snapshot.accel[2];

    imu->snapshot.temperature = sample->temp;
    __DMB();
    imu->snapshot.ready = 1u;
}

void imu_algorithm_step(imu_algorithm_t *imu,
                        const imu_raw_sample_t *sample)
{
    uint8_t i;

    if (imu == NULL || sample == NULL || !imu->configured ||
        imu->snapshot.ready == 0u)
        return;

    /* Odd guard means the single IMU writer is updating the publication. */
    imu->snapshot_guard++;
    __DMB();

    imu_cali_solve(imu, sample);

    for (i = 0u; i < 3u; i++) {
        imu->accel_filter_1[i] = imu->accel_filter_2[i];
        imu->accel_filter_2[i] = imu->accel_filter_3[i];
        imu->accel_filter_3[i] =
            imu->accel_filter_2[i] * accel_filter_num[0] +
            imu->accel_filter_1[i] * accel_filter_num[1] +
            imu->snapshot.accel[i] * accel_filter_num[2];
    }

    AHRS_update(imu->snapshot.quaternion,
                imu->config.timing_time,
                imu->snapshot.gyro,
                imu->accel_filter_3,
                imu->snapshot.mag);
    get_angle(imu->snapshot.quaternion,
              imu->snapshot.angle + IMU_ANGLE_YAW,
              imu->snapshot.angle + IMU_ANGLE_PITCH,
              imu->snapshot.angle + IMU_ANGLE_ROLL);

    if (sample->temp_updated)
        imu_temp_control(imu, sample->temp);

    imu->snapshot.temperature = sample->temp;
    imu->snapshot.sequence++;
    __DMB();
    imu->snapshot_guard++;
}

static void imu_set_control_temperature(imu_algorithm_t *imu,
                                        fp32 control_temperature)
{
    if (imu == NULL)
        return;
    imu->config.control_temperature = control_temperature;
}

static void imu_cali_solve(imu_algorithm_t *imu,
                           const imu_raw_sample_t *sample)
{
    uint8_t i;

    for (i = 0u; i < 3u; i++) {
        imu->snapshot.gyro[i] =
            sample->gyro[0] * imu->config.gyro_scale_factor[i][0] +
            sample->gyro[1] * imu->config.gyro_scale_factor[i][1] +
            sample->gyro[2] * imu->config.gyro_scale_factor[i][2] +
            imu->gyro_offset[i];
        imu->snapshot.accel[i] =
            sample->accel[0] * imu->config.accel_scale_factor[i][0] +
            sample->accel[1] * imu->config.accel_scale_factor[i][1] +
            sample->accel[2] * imu->config.accel_scale_factor[i][2] +
            imu->accel_offset[i];
        imu->snapshot.mag[i] =
            sample->mag[0] * imu->config.mag_scale_factor[i][0] +
            sample->mag[1] * imu->config.mag_scale_factor[i][1] +
            sample->mag[2] * imu->config.mag_scale_factor[i][2] +
            imu->mag_offset[i];
    }
}

static void imu_temp_control(imu_algorithm_t *imu, fp32 temp)
{
    uint16_t temp_pwm;

    if (imu->first_temperature) {
        PID_calc(&imu->temperature_pid,
                 temp,
                 imu->config.control_temperature,
                 0);
        if (imu->temperature_pid.out < 0.0f)
            imu->temperature_pid.out = 0.0f;

        temp_pwm = (uint16_t)imu->temperature_pid.out;
        imu_port_pwm_set(temp_pwm);
    } else {
        if (temp > imu->config.control_temperature) {
            imu->temperature_constant_time++;
            if (imu->temperature_constant_time > 200u) {
                imu->first_temperature = 1u;
                imu->temperature_pid.Iout =
                    imu->config.temperature_pwm_max / 2.0f;
            }
        }

        imu_port_pwm_set(imu->config.temperature_pwm_max - 1u);
    }
}

static void gyro_offset_calc(fp32 gyro_offset[3],
                             const fp32 gyro[3],
                             uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
        return;

    gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
    (*offset_time_count)++;
}

static void imu_gyro_calibration_calculate(const fp32 gyro[3],
                                           fp32 cali_scale[3],
                                           fp32 cali_offset[3],
                                           uint16_t *time_count)
{
    if (gyro == NULL || cali_scale == NULL ||
        cali_offset == NULL || time_count == NULL)
        return;

    /* cali_offset 同时是上一步零偏输入和本步输出，计算公式保持原样。 */
    gyro_offset_calc(cali_offset, gyro, time_count);
    cali_scale[0] = 1.0f;
    cali_scale[1] = 1.0f;
    cali_scale[2] = 1.0f;
}

static void imu_apply_gyro_calibration(imu_algorithm_t *imu,
                                       const fp32 cali_scale[3],
                                       const fp32 cali_offset[3])
{
    (void)cali_scale;

    if (imu == NULL)
        return;

    imu->gyro_cali_offset[0] = cali_offset[0];
    imu->gyro_cali_offset[1] = cali_offset[1];
    imu->gyro_cali_offset[2] = cali_offset[2];
    imu->gyro_offset[0] = imu->gyro_cali_offset[0];
    imu->gyro_offset[1] = imu->gyro_cali_offset[1];
    imu->gyro_offset[2] = imu->gyro_cali_offset[2];
}

void imu_submit_control_temperature(fp32 control_temperature)
{
    uint32_t primask = imu_pending_input_lock();

    pending_input.control_temperature = control_temperature;
    pending_input.temperature_sequence++;
    imu_pending_input_unlock(primask);
}

void imu_submit_gyro_calibration(const fp32 cali_scale[3],
                                 const fp32 cali_offset[3])
{
    uint32_t primask;
    uint8_t i;

    if (cali_scale == NULL || cali_offset == NULL)
        return;

    /* sequence 最后更新，消费者只会取得同一次提交的完整三轴参数。 */
    primask = imu_pending_input_lock();
    for (i = 0u; i < 3u; i++) {
        pending_input.gyro_cali_scale[i] = cali_scale[i];
        pending_input.gyro_cali_offset[i] = cali_offset[i];
    }
    pending_input.gyro_cali_sequence++;
    imu_pending_input_unlock(primask);
}

uint8_t imu_run_gyro_calibration_step(fp32 cali_scale[3],
                                      fp32 cali_offset[3],
                                      uint16_t *time_count)
{
    imu_snapshot_t snapshot;

    if (cali_scale == NULL || cali_offset == NULL || time_count == NULL)
        return 0u;
    if (!imu_read_snapshot(&snapshot))
        return 0u;

    /* 使用完整快照保持原零偏公式；结果提交后由 INS_task 下一周期应用。 */
    imu_gyro_calibration_calculate(snapshot.gyro,
                                   cali_scale,
                                   cali_offset,
                                   time_count);
    imu_submit_gyro_calibration(cali_scale, cali_offset);
    return 1u;
}

void imu_algorithm_apply_pending_inputs(imu_algorithm_t *imu)
{
    fp32 control_temperature;
    fp32 cali_scale[3];
    fp32 cali_offset[3];
    uint32_t temperature_sequence;
    uint32_t gyro_cali_sequence;
    uint32_t primask;
    uint8_t i;

    if (imu == NULL || !imu->configured)
        return;

    /* 临界区只复制 mailbox，不在关中断期间运行设备逻辑。 */
    primask = imu_pending_input_lock();
    temperature_sequence = pending_input.temperature_sequence;
    gyro_cali_sequence = pending_input.gyro_cali_sequence;
    control_temperature = pending_input.control_temperature;
    for (i = 0u; i < 3u; i++) {
        cali_scale[i] = pending_input.gyro_cali_scale[i];
        cali_offset[i] = pending_input.gyro_cali_offset[i];
    }
    imu_pending_input_unlock(primask);

    if (temperature_sequence != imu->applied_temperature_sequence) {
        imu_set_control_temperature(imu, control_temperature);
        imu->applied_temperature_sequence = temperature_sequence;
    }

    if (gyro_cali_sequence != imu->applied_gyro_cali_sequence) {
        imu_apply_gyro_calibration(imu, cali_scale, cali_offset);
        imu->applied_gyro_cali_sequence = gyro_cali_sequence;
    }
}

static uint32_t imu_pending_input_lock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    return primask;
}

static void imu_pending_input_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0u)
        __enable_irq();
}

uint8_t imu_read_snapshot(imu_snapshot_t *snapshot)
{
    imu_snapshot_t local_snapshot;
    uint32_t guard_before;
    uint32_t guard_after;

    if (snapshot == NULL || default_imu == NULL)
        return 0u;

    guard_before = default_imu->snapshot_guard;
    if ((guard_before & 1u) != 0u)
        return 0u;

    __DMB();
    local_snapshot = default_imu->snapshot;
    __DMB();

    guard_after = default_imu->snapshot_guard;
    if (guard_before != guard_after || (guard_after & 1u) != 0u ||
        local_snapshot.ready == 0u)
        return 0u;

    *snapshot = local_snapshot;
    return 1u;
}
