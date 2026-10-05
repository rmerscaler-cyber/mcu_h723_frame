/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "math.h"

#include "detect_task.h"
#include "main.h"

// 输入：1=中，2=下，3=上
// 输出：上→1，中→0，下→-1
#define MAP(val)  ((val < 0) ? 1 : (val > 0) ? 2 : 3)

// 摇杆通道值映射宏定义开关
#define MAPPING_ENABLE 1


//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700

//取正函数
static int16_t RC_abs(int16_t value);
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static int16_t map_to_660(const int16_t val);


//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
/* 以下变量去 static 是为了让 Keil watch 窗口能看到（-O0 下 static 文件级变量无法观察） */
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
volatile uint16_t remote_sequence;
volatile uint32_t remote_timestamp_ms;


/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
/* 遥控器帧回调: BSP 在 IDLE 中断里切换完双缓冲后调用, 这里只做协议解析 */
static void rc_sbus_rx_callback(bsp_uart_id_t uartx, uint8_t *data, uint16_t size)
{
    (void)uartx;
    if (size == RC_FRAME_LENGTH)
    {
        sbus_to_rc(data, &rc_ctrl);
        remote_sequence++;
        remote_timestamp_ms = HAL_GetTick();
        detect_hook(DBUS_TOE);
    }
}

void remote_control_init(void)
{
    /* 通过BSP层注册UART5 DMA双缓冲接收, 不再直接触碰 huart5/hdma */
    RC_Init(BSP_UART5, sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    bsp_uart_dma_rx_register_cb(BSP_UART5, rc_sbus_rx_callback);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void remote_control_read_snapshot(remote_input_snapshot_t *snapshot)
{
    uint32_t primask;
    uint8_t i;

    if (snapshot == NULL)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0u; i < 4u; i++) {
        snapshot->ch[i] = rc_ctrl.rc.ch[i];
        snapshot->s[i] = rc_ctrl.rc.s[i];
    }
    snapshot->frame_lost = rc_ctrl.rc.frame_lost;
    snapshot->failsafe = rc_ctrl.rc.failsafe;
    snapshot->sequence = remote_sequence;
    snapshot->timestamp_ms = remote_timestamp_ms;
    snapshot->dbus_lost = toe_is_error(DBUS_TOE) ? 1u : 0u;
    snapshot->online = snapshot->dbus_lost ? 0u : 1u;
    __DMB();
    if (primask == 0u)
        __enable_irq();
}

/* DBUS 失联/坏帧时的安全位: 摇杆回中 + 开关全 UP + 键鼠清零。 */
void rc_safe_reset(void)
{
    for (int i = 0; i < 4; i++)
    {
        rc_ctrl.rc.ch[i] = 0;
        rc_ctrl.rc.s[i]  = RC_SW_UP;
    }
    rc_ctrl.rc.vr[0] = 0;
    rc_ctrl.rc.vr[1] = 0;
    rc_ctrl.rc.failsafe  = 1;
    rc_ctrl.rc.frame_lost = 1;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    /* 坏数据兜底: 统一走安全位 (开关全 UP), 不再沿用 DJI 原版的 DOWN */
    rc_safe_reset();
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // 【保留】帧头帧尾校验（标准 SBUS：0x0F 开头，0x00 结尾）
    if (sbus_buf[0] != 0x0F || sbus_buf[24] != 0x00)
    {
        return;
    }
    // SBUS通道原始解包
    rc_ctrl->rc.ch[0] = (int16_t)(((sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc_ctrl->rc.ch[1] = (int16_t)((((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc_ctrl->rc.ch[2] = (int16_t)((((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc_ctrl->rc.ch[3] = (int16_t)((((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET);

    // 四路拨杆开关解析
    rc_ctrl->rc.s[0] = (int8_t) MAP(((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc_ctrl->rc.s[1] = (int8_t) MAP(((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc_ctrl->rc.s[2] = (int8_t) MAP(((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc_ctrl->rc.s[3] = (int8_t) MAP(((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF) - RC_CH_VALUE_OFFSET);
		//vra，vrb
    rc_ctrl->rc.vr[0] = (int16_t)((sbus_buf[12] | sbus_buf[13] << 8) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.vr[1] = (int16_t)((sbus_buf[13] >> 3 | sbus_buf[14] << 5) & 0x07FF) - RC_CH_VALUE_OFFSET;

#if MAPPING_ENABLE
    // 通道数值映射
    for (int i = 0; i < 4; i++)
    {   
        rc_ctrl->rc.ch[i] = map_to_660(rc_ctrl->rc.ch[i]);
    }
    for (int i = 0; i < 2; i++)
    {
       rc_ctrl->rc.vr[i] = map_to_660(rc_ctrl->rc.vr[i]);
    }

    // 失控、丢帧标志位解析
    const uint8_t flag = sbus_buf[23];
    rc_ctrl->rc.frame_lost = (flag >> 2) & 0x01;
    rc_ctrl->rc.failsafe = (flag >> 3) & 0x01;

#endif
}

/**
 * @brief 为方便适配代码增加该映射-660~660函数,原始数据为-784~783
 * @param val i6x搭配iA6B接收器解包后的初始值
 * @return 映射到-660~660的值
 */
static int16_t map_to_660(const int16_t val)
{
    if (val >= 0)
        return (int16_t)floorf((660.0f / 783.0f) * (float)val + 0.5f);
    else
        return (int16_t)floorf((660.0f / 784.0f) * (float)val + 0.5f);
}

