#include "INS_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "imu.h"
#include "imu_io.h"

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f},                  \
    {-1.0f, 0.0f, 0.0f},                 \
    {0.0f, 0.0f, 1.0f}

#define IST8310_BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f},                   \
    {0.0f, 1.0f, 0.0f},                   \
    {0.0f, 0.0f, 1.0f}

/* 设备组合实例: algo 为第一成员, 算法句柄即 &ins_imu.algo。 */
static device_imu_instance_t ins_imu;

static const imu_config_t imu_config = {
    .gyro_scale_factor = {BMI088_BOARD_INSTALL_SPIN_MATRIX},
    .accel_scale_factor = {BMI088_BOARD_INSTALL_SPIN_MATRIX},
    .mag_scale_factor = {IST8310_BOARD_INSTALL_SPIN_MATRIX},
    .temperature_pid = {
        .mode = PID_LINEAR,
        .kp = TEMPERATURE_PID_KP,
        .ki = TEMPERATURE_PID_KI,
        .kd = TEMPERATURE_PID_KD,
        .max_out = TEMPERATURE_PID_MAX_OUT,
        .max_iout = TEMPERATURE_PID_MAX_IOUT,
        .d_filter = 0.0f,
        .integral_separation = 0.0f,
    },
    .timing_time = 0.001f,
    .control_temperature = 0.0f,
    .temperature_pwm_max = MPU6500_TEMP_PWM_MAX,
};

void INS_task(void const *pvParameters)
{
    imu_config_t task_config = imu_config;
    imu_raw_sample_t first_sample;
    TaskHandle_t task_handle;

    (void)pvParameters;

    task_handle = xTaskGetHandle(pcTaskGetName(NULL));
    device_imu_io_configure(&ins_imu, task_handle);
    imu_algorithm_configure(&ins_imu.algo, &task_config);

    /*
     * cali_param_init() 早于任务启动；实例配置完成后应用全部暂存输入，并且
     * 必须位于 imu_algorithm_init() 首次姿态解算之前，避免首帧使用默认温度或零偏。
     */
    imu_algorithm_apply_pending_inputs(&ins_imu.algo);

    osDelay(INS_TASK_INIT_TIME);
    while (device_imu_io_init(&ins_imu) != IMU_IO_INIT_DONE)
        osDelay(100);

    device_imu_io_step(&ins_imu, &first_sample);
    imu_algorithm_init(&ins_imu.algo, &first_sample);

    for (;;) {
        imu_raw_sample_t sample;

        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }
        /* 统一在 IMU 任务上下文应用外部输入，之后再推进本周期解算。 */
        imu_algorithm_apply_pending_inputs(&ins_imu.algo);
        device_imu_io_step(&ins_imu, &sample);
        imu_algorithm_step(&ins_imu.algo, &sample);
    }
}
