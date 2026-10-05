/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      ң����������ң������ͨ������SBUS��Э�鴫�䣬����DMA���䷽ʽ��ԼCPU
  *             ��Դ�����ô��ڿ����ж���������������ͬʱ�ṩһЩ��������DMA������
  *             �ķ�ʽ��֤�Ȳ�ε��ȶ��ԡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"
#include "remote_input.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)


/* ----------------------- Internal Data ----------------------------------- */

/* ��ȡң����ҡ��ƫ��ֵ 
   RLR����ҡ�������ƶ�  LUD����ҡ�������ƶ�	*/
#define    RC_CH0_RLR_OFFSET    (get_remote_control_point()->rc.ch[0])
#define    RC_CH1_RUD_OFFSET  	(get_remote_control_point()->rc.ch[1])
#define    RC_CH2_LLR_OFFSET  	(get_remote_control_point()->rc.ch[2])
#define    RC_CH3_LUD_OFFSET  	(get_remote_control_point()->rc.ch[3])
#define	RC_CHX_MAX	660;
#define RC_CH0_OFFSET RC_CH0_RLR_OFFSET
#define RC_CH1_OFFSET RC_CH1_RUD_OFFSET
#define RC_CH2_OFFSET RC_CH2_LLR_OFFSET
#define RC_CH3_OFFSET RC_CH3_LUD_OFFSET
/* ���ң��������״̬ */
#define    IF_RC_SW1_UP      (get_remote_control_point()->rc.s[0] == RC_SW_UP)
#define    IF_RC_SW1_MID     (get_remote_control_point()->rc.s[0] == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (get_remote_control_point()->rc.s[0] == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (get_remote_control_point()->rc.s[1] == RC_SW_UP)
#define    IF_RC_SW2_MID     (get_remote_control_point()->rc.s[1] == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (get_remote_control_point()->rc.s[1] == RC_SW_DOWN)

/* ��ȡ���������ƶ��ٶ� */
#define    MOUSE_X_MOVE_SPEED    (get_remote_control_point()->mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (get_remote_control_point()->mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (get_remote_control_point()->mouse.z)

/* �����갴��״̬ 
   ����Ϊ1��û����Ϊ0*/
#define    IF_MOUSE_PRESSED_LEFT    (get_remote_control_point()->mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (get_remote_control_point()->mouse.press_r == 1)
/* �����̰���״̬ 
   ����Ӧ���������£����߼�����ʽ��ֵΪ1������Ϊ0 */
#define    IF_KEY_PRESSED         (  get_remote_control_point()->key.v  )
#define    IF_KEY_PRESSED_W       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (get_remote_control_point()->key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

/* ----------------------- Data Struct ------------------------------------- */
typedef struct __attribute__((packed))
{
        struct __attribute__((packed))
        {
						int16_t ch[4];
            int8_t s[4];
						int16_t vr[2];
						uint8_t failsafe;  // ʧ�ر�����־λ
						uint8_t frame_lost;
        } rc;
        struct __attribute__((packed))
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        struct __attribute__((packed))
        {
                uint16_t v;
        } key;

} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void rc_safe_reset(void);   /* DBUS lost/bad-frame safe state: sticks centered, all switches UP */
/* 将本地 DBUS 状态复制成与 Upboard 共用的值快照。 */
extern void remote_control_read_snapshot(remote_input_snapshot_t *snapshot);
#endif
