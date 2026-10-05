#include "dji_motor.h"
#include "motor_utils.h"   /* motor_angle_wrap */
#include "motor_port.h"    /* motor_port_send: CAN 发送由设备层适配 */
#include <string.h>       /* memset */
#include <stddef.h>
#include <math.h>          /* fmod: 先按输出轴一圈取余，避免大累计角度丢失单圈精度 */

/*
 * DJI 驱动只解释 DJI 帧格式：把反馈写成统一物理量，并把中间层已经限幅
 * 的 ctrl.output 编成 0x200/0x2FF 电流帧。它不决定 PID，也不读取全局 motor ID。
 */

/* ============ 实现 ============ */
/* 注册条目由中间层通过 dji_motor_driver_ops 逐台填入；驱动不读 motor_reg.h。 */
#define DJI_MOTOR_STATE_MAX  10   /* H7: 8 路底盘电机 + 1 路只接收 yaw 反馈 + 1路拨盘2006*/
static dji_motor_t dji_motors[DJI_MOTOR_STATE_MAX];
static uint8_t dji_motor_cnt = 0;   /* 实际注册数 */

static motor_base_t *dji_motor_register(const motor_reg_t *cfg)
{
    uint8_t i;
    dji_motor_t *m;

    if (cfg == NULL || cfg->reduction <= 0.0f) return NULL;

    /* 幂等: 同一物理 CAN 总线上的同一反馈 ID 才是同一实例。
     * 不同总线允许复用相同 ID（如 CAN1 底盘 0x201 与 CAN3 拨盘 0x201）。
     * 该函数只在冷路径被 motor_register 调用，非控制周期热路径。 */
    for (i = 0; i < dji_motor_cnt; i++) {
        if (dji_motors[i].can_bus == cfg->can_bus &&
            dji_motors[i].can_id == cfg->can_id) {
            return &dji_motors[i].base;
        }
    }
    if (dji_motor_cnt >= DJI_MOTOR_STATE_MAX) return NULL;

    m = &dji_motors[dji_motor_cnt];
    m->base.id    = cfg->id;
    m->base.type  = cfg->type;
    m->can_bus    = cfg->can_bus;
    m->can_id     = cfg->can_id;
    m->cmd_id     = cfg->cmd_id;
    m->cmd_slot   = cfg->cmd_slot;
    m->enable     = 0;   /* 由 DJI ops 的 enable 打开 */
    /* 反馈标定从 registry 拷入 (fb_parse 换算物理量用) */
    m->direction     = cfg->direction;
    m->offset_ecd    = cfg->offset_ecd;
    m->current_scale = cfg->current_scale;
    m->reduction     = cfg->reduction;
    m->wheel_radius  = cfg->wheel_radius;
    dji_motor_cnt++;
    return &m->base;
}

static uint8_t dji_motor_store_protocol_feedback(
    motor_base_t *motor, const uint8_t *data)
{
    dji_motor_t *m;
    int32_t delta_ecd;

    if (motor == NULL || data == NULL) return 0;
    /* motor_base_t 是 dji_motor_t 第一成员，可从已绑定实例还原具体驱动对象。 */
    m = (dji_motor_t *)motor;

    m->raw.ecd = (uint16_t)((data[0] << 8) | data[1]);
    m->raw.speed = (int16_t)((data[2] << 8) | data[3]);
    m->raw.current = (int16_t)((data[4] << 8) | data[5]);
    m->raw.temperature = (int8_t)data[6];

    if (!m->feedback_initialized) {
        /* 首帧只建立相对编码器零点的坐标；上电前转过的圈数不可观测。 */
        m->total_ecd = (int32_t)m->raw.ecd - (int32_t)m->offset_ecd;
        m->feedback_initialized = 1u;
    } else {
        /* 相邻有效帧的转子位移必须小于半圈；正向跨零加，反向跨零减。 */
        delta_ecd = (int32_t)m->raw.ecd - (int32_t)m->last_ecd;
        if (delta_ecd > (int32_t)DJI_ECD_RANGE / 2)
            delta_ecd -= (int32_t)DJI_ECD_RANGE;
        else if (delta_ecd < -(int32_t)DJI_ECD_RANGE / 2)
            delta_ecd += (int32_t)DJI_ECD_RANGE;
        m->total_ecd += delta_ecd;
    }
    m->last_ecd = m->raw.ecd;

    return 1;
}

static uint8_t dji_motor_read_unfiltered_feedback(
    motor_base_t *motor, motor_feedback_t *feedback)
{
    dji_motor_t *m;
    double output_ecd_per_turn;
    double output_rad_per_ecd;

    if (motor == NULL || feedback == NULL) return 0;
    m = (dji_motor_t *)motor;

    /* 累计角度与角速度统一为输出轴坐标，所有实例都除以 reduction。
     * 先用整数累计编码器增量，再换算；不能把包到 [-pi, pi] 的转子角累加。 */
    output_rad_per_ecd = 6.28318530717958647692 / (double)DJI_ECD_RANGE *
                         (double)m->direction / (double)m->reduction;
    output_ecd_per_turn = (double)DJI_ECD_RANGE * (double)m->reduction;
    feedback->total_angle = (fp32)((double)m->total_ecd * output_rad_per_ecd);
    feedback->angle = motor_angle_wrap((fp32)(
        fmod((double)m->total_ecd, output_ecd_per_turn) * output_rad_per_ecd));
    /* DJI 反馈是电机转子转速；统一除以注册表减速比，向上发布输出端角速度。
     * 直驱电机的 reduction 明确配置为 1.0f，因此热路径无需按型号分支。 */
    feedback->speed_rotor =
        (fp32)m->raw.speed * DJI_RPM_TO_RADPS * (fp32)m->direction;
    feedback->speed_angle = feedback->speed_rotor / m->reduction;
    feedback->current =
        (int16_t)((fp32)m->raw.current * m->current_scale);
    feedback->temperature = m->raw.temperature;
    /* 无轮机构的 wheel_radius 配置为 0，乘法结果自然为 0。 */
    feedback->speed_ms = feedback->speed_angle * m->wheel_radius;

    return 1;
}

static void dji_motor_pack_commands(motor_handle_t const *motors, uint8_t count)
{
    uint8_t buf[8];
    uint8_t i, j, first, sent;

    if (motors == NULL) return;

    for (i = 0; i < count; i++) {
        dji_motor_t *group_motor;

        if (motors[i] == NULL || motors[i]->driver != &dji_motor_driver_ops) continue;
        group_motor = (dji_motor_t *)motors[i];

        /* 只对每个 (can_bus, cmd_id) 组的第一台负责组帧, 避免重复发 */
        first = 1;
        for (j = 0; j < i; j++) {
            dji_motor_t *previous_motor;
            if (motors[j] == NULL || motors[j]->driver != &dji_motor_driver_ops) continue;
            previous_motor = (dji_motor_t *)motors[j];
            if (previous_motor->can_bus == group_motor->can_bus &&
                previous_motor->cmd_id  == group_motor->cmd_id) {
                first = 0;
                break;
            }
        }
        if (!first) continue;

        /* 组帧: 只填传入列表内且已使能的电机槽位，有实际填槽才发帧。 */
        memset(buf, 0, sizeof(buf));
        sent = 0;
        for (j = 0; j < count; j++) {
            dji_motor_t *motor;
            if (motors[j] == NULL || motors[j]->driver != &dji_motor_driver_ops) continue;
            motor = (dji_motor_t *)motors[j];
            if (motor->can_bus == group_motor->can_bus &&
                motor->cmd_id  == group_motor->cmd_id) {
                if (motor->enable) {
                    /* 中间层已把 ctrl.output 限幅到本电机电流范围, 驱动只做转换 */
                    motor->current_cmd = (int16_t)motor->base.ctrl.output;
                    buf[motor->cmd_slot * 2]     = (uint8_t)(motor->current_cmd >> 8);
                    buf[motor->cmd_slot * 2 + 1] = (uint8_t)(motor->current_cmd);
                    sent = 1;   /* 只发实例列表中已使能电机的帧，避免全 0 帧覆盖其他任务 */
                }
            }
        }
        if (!sent) continue;

        motor_port_send(group_motor->can_bus, group_motor->cmd_id, buf, 8);
    }
}

static void dji_motor_enable_instance(motor_base_t *motor)
{
    dji_motor_t *m;
    if (motor == NULL) return;
    m = (dji_motor_t *)motor;
    m->enable = 1;
}

static void dji_motor_disable_instance(motor_base_t *motor)
{
    dji_motor_t *m;
    if (motor == NULL) return;
    m = (dji_motor_t *)motor;
    m->enable = 0;
}

const motor_driver_ops_t dji_motor_driver_ops = {
    "DJI",
    dji_motor_register,
    dji_motor_store_protocol_feedback,
    dji_motor_read_unfiltered_feedback,
    dji_motor_pack_commands,
    dji_motor_enable_instance,
    dji_motor_disable_instance,
};

void dji_motor_reset_id(motor_bus_e bus)
{
    uint8_t data[8] = {0};
    (void)motor_port_send(bus, 0x700u, data, 8u);
}
