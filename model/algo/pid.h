#ifndef PID_H
#define PID_H

#include "struct_typedef.h"

#define PID_FEEDFORWARD_MAX_COMP 4u

/** PID 误差的拓扑：普通线性误差，或具有 2π 周期的角度误差。 */
typedef enum
{
    PID_LINEAR = 0,
    PID_ANGLE,
} pid_mode_e;

/** PID 的只读调参项；使用具名成员避免数组下标与单位混淆。 */
typedef struct
{
    pid_mode_e mode;
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 max_out;
    fp32 max_iout;
    fp32 d_filter;             /* D 项一阶低通系数 [0, 1]；0 表示不过滤。 */
    fp32 integral_separation;  /* 积分分离阈值；小于等于 0 表示禁用。 */
} pid_config_t;

typedef enum
{
    PID_ERROR_NONE = 0,
    PID_ERROR_BLOCKED,
} pid_error_e;

typedef struct
{
    fp32 set_gain;
    fp32 comp_gain[PID_FEEDFORWARD_MAX_COMP];
    fp32 comp_value[PID_FEEDFORWARD_MAX_COMP];
    fp32 output;
    uint8_t comp_count;
    uint8_t enabled;
} pid_feedforward_t;

/** 单个 PID 实例的参数快照和跨控制周期状态。 */
typedef struct
{
    pid_mode_e mode;
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
    fp32 max_out;
    fp32 max_iout;
    fp32 d_filter;
    fp32 integral_separation;
    uint8_t integral_separation_enabled;

    fp32 set;
    fp32 fdb;
    fp32 last_fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;       /* 同时保存本周期输出和下周期 D 低通的历史状态。 */
    fp32 error[2];
    fp32 ierror[2];

    pid_feedforward_t feedforward;
    uint32_t block_count;
    pid_error_e error_type;
} pid_type_def;

void PID_init(pid_type_def *pid, const pid_config_t *config);
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set, fp32 direct_delta);
void PID_clear(pid_type_def *pid);
void PID_reset(pid_type_def *pid);

void PID_feedforward_init(pid_type_def *pid,
                          fp32 set_gain,
                          const fp32 *comp_gain,
                          uint8_t comp_count);
void PID_feedforward_set_comp_value(pid_type_def *pid,
                                    uint8_t index,
                                    fp32 value);
void PID_feedforward_clear(pid_type_def *pid);

pid_error_e PID_block_detect(pid_type_def *pid);
void PID_error_reset(pid_type_def *pid);

#endif /* PID_H */
