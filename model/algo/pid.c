#include "pid.h"

#include <math.h>
#include <stddef.h>

#include "user_lib.h"

#define PID_BLOCK_OUTPUT_RATIO  0.01f
#define PID_BLOCK_SET_EPSILON   0.01f
#define PID_BLOCK_ERROR_RATIO   0.90f
#define PID_BLOCK_COUNT_LIMIT   1000u

static fp32 pid_clamp(fp32 value, fp32 limit)
{
    if (limit < 0.0f)
        limit = -limit;
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static fp32 pid_config_filter(fp32 filter)
{
    if (filter < 0.0f)
        return 0.0f;
    if (filter > 1.0f)
        return 1.0f;
    return filter;
}

void PID_init(pid_type_def *pid, const pid_config_t *config)
{
    if (pid == NULL || config == NULL)
        return;

    pid->mode = (config->mode == PID_ANGLE) ? PID_ANGLE : PID_LINEAR;
    pid->Kp = config->kp;
    pid->Ki = config->ki;
    pid->Kd = config->kd;
    pid->max_out = fabsf(config->max_out);
    pid->max_iout = fabsf(config->max_iout);
    pid->d_filter = pid_config_filter(config->d_filter);
    pid->integral_separation = fabsf(config->integral_separation);
    pid->integral_separation_enabled =
        (config->integral_separation > 0.0f) ? 1u : 0u;

    PID_clear(pid);
    PID_feedforward_clear(pid);
}

fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set, fp32 direct_delta)
{
    fp32 raw_dout;
    fp32 feedforward_output = 0.0f;
    uint8_t i;

    if (pid == NULL)
        return 0.0f;

    pid->error[1] = pid->error[0];
    pid->last_fdb = pid->fdb;
    pid->set = set;
    pid->fdb = ref;

    if (pid->mode == PID_ANGLE)
        pid->error[0] = rad_format(set - ref);
    else
        pid->error[0] = set - ref;

    if (pid->integral_separation_enabled &&
        fabsf(pid->error[0]) > pid->integral_separation) {
        pid->ierror[0] = 0.0f;
    } else {
        pid->ierror[0] = pid->ierror[1] + pid->error[0];
    }

    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout = pid_clamp(pid->Ki * pid->ierror[0], pid->max_iout);
    if (pid->Ki != 0.0f)
        pid->ierror[0] = pid->Iout / pid->Ki;
    else
        pid->ierror[0] = 0.0f;
    pid->ierror[1] = pid->ierror[0];

    if (pid->mode == PID_ANGLE)
        raw_dout = -pid->Kd * direct_delta;
    else
        raw_dout = pid->Kd * (pid->last_fdb - pid->fdb);

    pid->Dout = pid->d_filter * pid->Dout
              + (1.0f - pid->d_filter) * raw_dout;

    if (pid->feedforward.enabled) {
        feedforward_output = pid->feedforward.set_gain * set;
        for (i = 0u; i < pid->feedforward.comp_count; i++)
            feedforward_output += pid->feedforward.comp_gain[i]
                                * pid->feedforward.comp_value[i];
        pid->feedforward.output = feedforward_output;
    }

    pid->out = pid_clamp(pid->Pout + pid->Iout + pid->Dout
                       + feedforward_output,
                         pid->max_out);
    return pid->out;
}

void PID_clear(pid_type_def *pid)
{
    uint8_t i;

    if (pid == NULL)
        return;

    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->last_fdb = 0.0f;
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->error[0] = pid->error[1] = 0.0f;
    pid->ierror[0] = pid->ierror[1] = 0.0f;
    pid->feedforward.output = 0.0f;
    for (i = 0u; i < PID_FEEDFORWARD_MAX_COMP; i++)
        pid->feedforward.comp_value[i] = 0.0f;
    pid->block_count = 0u;
    pid->error_type = PID_ERROR_NONE;
}

void PID_reset(pid_type_def *pid)
{
    uint8_t i;

    if (pid == NULL)
        return;

    PID_clear(pid);
    pid->mode = PID_LINEAR;
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->max_out = 0.0f;
    pid->max_iout = 0.0f;
    pid->d_filter = 0.0f;
    pid->integral_separation = 0.0f;
    pid->integral_separation_enabled = 0u;
    pid->feedforward.set_gain = 0.0f;
    pid->feedforward.comp_count = 0u;
    pid->feedforward.enabled = 0u;
    for (i = 0u; i < PID_FEEDFORWARD_MAX_COMP; i++)
        pid->feedforward.comp_gain[i] = 0.0f;
}

void PID_feedforward_init(pid_type_def *pid,
                          fp32 set_gain,
                          const fp32 *comp_gain,
                          uint8_t comp_count)
{
    uint8_t i;

    if (pid == NULL)
        return;

    PID_feedforward_clear(pid);
    if (comp_count > PID_FEEDFORWARD_MAX_COMP)
        comp_count = PID_FEEDFORWARD_MAX_COMP;

    pid->feedforward.set_gain = set_gain;
    pid->feedforward.comp_count = comp_count;
    if (comp_gain != NULL) {
        for (i = 0u; i < comp_count; i++)
            pid->feedforward.comp_gain[i] = comp_gain[i];
    }
    pid->feedforward.enabled = 1u;
}

void PID_feedforward_set_comp_value(pid_type_def *pid,
                                    uint8_t index,
                                    fp32 value)
{
    if (pid == NULL || index >= pid->feedforward.comp_count)
        return;
    pid->feedforward.comp_value[index] = value;
}

void PID_feedforward_clear(pid_type_def *pid)
{
    uint8_t i;

    if (pid == NULL)
        return;

    pid->feedforward.set_gain = 0.0f;
    pid->feedforward.output = 0.0f;
    pid->feedforward.comp_count = 0u;
    pid->feedforward.enabled = 0u;
    for (i = 0u; i < PID_FEEDFORWARD_MAX_COMP; i++) {
        pid->feedforward.comp_gain[i] = 0.0f;
        pid->feedforward.comp_value[i] = 0.0f;
    }
}

pid_error_e PID_block_detect(pid_type_def *pid)
{
    if (pid == NULL)
        return PID_ERROR_NONE;

    /* 堵转检测对正负输出保持对称；持续时间仍按调用周期计数。 */
    if (fabsf(pid->out) < pid->max_out * PID_BLOCK_OUTPUT_RATIO) {
        pid->block_count = 0u;
        return pid->error_type;
    }

    if (fabsf(pid->set) > PID_BLOCK_SET_EPSILON &&
        fabsf(pid->set - pid->fdb) / fabsf(pid->set)
            > PID_BLOCK_ERROR_RATIO) {
        if (pid->block_count < 0xFFFFFFFFu)
            pid->block_count++;
    } else {
        pid->block_count = 0u;
    }

    if (pid->block_count > PID_BLOCK_COUNT_LIMIT)
        pid->error_type = PID_ERROR_BLOCKED;

    return pid->error_type;
}

void PID_error_reset(pid_type_def *pid)
{
    if (pid == NULL)
        return;
    pid->block_count = 0u;
    pid->error_type = PID_ERROR_NONE;
}
