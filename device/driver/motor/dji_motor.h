#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "motor_driver.h"

/* DJI 测量换算常量 (13bit 编码器 / RPM) */
#define DJI_ECD_TO_RAD    0.0007669903939428206f   /* 2π/8192 */
#define DJI_RPM_TO_RADPS  0.10471975511965977f     /* 2π/60 */
#define DJI_TWO_PI        6.283185307179586f

/* DJI 编码器原始范围 (13bit: 0~8191) */
#define DJI_ECD_RANGE     8192u

/* 编码器值包回 [0, 8192) */
static inline uint16_t dji_ecd_wrap(int32_t ecd)
{
    if (ecd >= (int32_t)DJI_ECD_RANGE) ecd -= (int32_t)DJI_ECD_RANGE;
    else if (ecd < 0)                  ecd += (int32_t)DJI_ECD_RANGE;
    return (uint16_t)ecd;
}

/* DJI 协议原始反馈: CAN 中断写入, 仅本驱动 read_unfiltered_feedback 换算读取。 */
typedef struct {
    uint16_t ecd;          /* 13bit 编码器值 0~8191 */
    int16_t  speed;        /* RPM 协议整数 */
    int16_t  current;      /* 电流协议整数 */
    int8_t   temperature;  /* 温度协议值 */
} dji_motor_raw_t;

/* ============ DJI 电机实例 = 内嵌统一基类 + DJI 私有 ============ */
/* 协议参数(总线/ID/槽位)在注册表 motor_reg.h, 注册时由 DJI ops 从配置填入 */
typedef struct {
    motor_base_t base;        /* 统一: id/type/fb/ctrl/online —— 第一成员, 中间层只认它 */

    /* ---- DJI 私有: 协议参数 (register_instance 从 motor_regs 填入) ---- */
    motor_bus_e  can_bus;      /* 逻辑总线，物理 CAN 映射由设备侧 port 实现 */
    uint16_t     can_id;       /* 反馈帧 ID (3508:0x200+id / 6020:0x204+id) */
    uint16_t     cmd_id;       /* 命令帧 ID (0x200 组 / 0x2FF 单发) */
    uint8_t      cmd_slot;     /* 命令帧内槽位 0-3 (组帧用) */

    /* ---- 反馈标定 (init 从 registry 拷入, fb_parse 换算统一物理量用) ---- */
    int16_t  direction;        /* 正反 +1/-1 */
    uint16_t offset_ecd;       /* 转子编码器零点，单位为原始计数 */
    fp32     current_scale;    /* 电流 LSB→mA */
    fp32     reduction;        /* 转子→输出端减速比；直驱必须为 1.0f */
    fp32     wheel_radius;     /* 轮半径 m (无轮=0) */

    /* ---- DJI 私有原始值与跨帧算法状态 ---- */
    dji_motor_raw_t raw;       /* 协议原始值: CAN 中断写, 驱动内部换算 */
    uint16_t last_ecd;
    int64_t  total_ecd;        /* 相对 offset_ecd 的连续转子计数；读反馈时换算到输出轴 */
    uint8_t  feedback_initialized;

    /* ---- 输出 (闭环在中间层, 驱动只读 ctrl.output) ---- */
    int16_t current_cmd;      /* 电流指令 */
    uint8_t enable;           /* 使能门: 1=发送电流帧 0=不发(置0) */
} dji_motor_t;

/* motor_reg.c 装配到配置行；具体协议函数保持驱动私有。 */
extern const motor_driver_ops_t dji_motor_driver_ops;

/* DJI 广播重设 ID 命令；仅校准任务使用，控制周期仍走统一 handle。 */
void dji_motor_reset_id(motor_bus_e bus);

#endif /* DJI_MOTOR_H */
