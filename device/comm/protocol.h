/**
  ******************************************************************************
  * @file    protocol.h
  * @author  Karolance Future
  * @version V1.3.0
  * @date    2022/03/21
  * @brief   依据裁判系统 串口协议附录 V1.3
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */

#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "stdint.h"

#define HEADER_SOF                  0xA5

#define SEND_HEADER_SOF 0x50
#define VISION_HEADER_SOF 0x50
#define NAV_HEADER_SOF 0x50
#define VIS_PROTOCOL_SEND_CMDID							0x001		
/***************NAV******************/
#define NAV_PROTOCOL_HEADER_SIZE 						1
#define NAV_PROTOCOL_SEND_DATA_SIZE					16	
#define NAV_PROTOCOL_DATA_SIZE							NAV_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + 6 * sizeof(float)
/***************VISION******************/
#define VIS_PROTOCOL_SEND_DATA_SIZE					16													//发送数据长度												
#define VIS_PROTOCOL_HEADER_SOF_SIZE 				1
#define VIS_PROTOCOL_CMD_SIZE               2
#define VIS_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define VIS_PROTOCOL_LEN_SIZE               1
#define VIS_PROTOCOL_CRC16_SIZE             2
#define VIS_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define VIS_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define VIS_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof((float)))
#define VIS_PROTOCOL_FRAME_MAX_SIZE         128
//实际长度
//#define VIS_PROTOCOL_FRAME_MAX_SIZE         (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t)+VIS_PROTOCOL_SEND_DATA_SIZE)

/***************REFEREE******************/
#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE    sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE     2

#define REF_HEADER_CRC_LEN          (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN    (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN        (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
	GAME_STATE_CMD_ID                 = 0x0001,  //比赛状态数据
	GAME_RESULT_CMD_ID                = 0x0002,  //比赛结果数据
	GAME_ROBOT_HP_CMD_ID              = 0x0003,  //机器人血量数据
//	DART_FLYING_STATE_CMD_ID          = 0x0004,  //飞镖发射后状态
	
	FIELD_EVENTS_CMD_ID               = 0x0101,  //场地事件数据
	SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,  //补给站动作标识
	SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,  //请求补给站补弹数据，由参赛队发送（RM 对抗赛尚未开放）
	REFEREE_WARNING_CMD_ID            = 0x0104,  //裁判警告信息
	DART_REMAINING_TIME_CMD_ID        = 0x0105,  //飞镖发射口倒计时
	
	ROBOT_STATE_CMD_ID                = 0x0201,  //比赛机器人状态
	POWER_HEAT_DATA_CMD_ID            = 0x0202,  //实时功率热量数据
	ROBOT_POS_CMD_ID                  = 0x0203,  //机器人位置
	BUFF_MUSK_CMD_ID                  = 0x0204,  //机器人增益
	AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,  //空中机器人能量状态
	ROBOT_HURT_CMD_ID                 = 0x0206,  //伤害状态
	SHOOT_DATA_CMD_ID                 = 0x0207,  //实时射击信息
	BULLET_REMAINING_CMD_ID           = 0x0208,  //子弹剩余发射数
	ROBOT_RFID_STATE_CMD_ID           = 0x0209,  //机器人RFID状态
	DART_CLIENT_CMD_ID                = 0x020A,  //飞镖机器人客户端指令数据
	SENTRY_INFO_CMD_ID                = 0x020D,   //哨兵机器人战斗状态
	
	STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,  //机器人间通信
  ROBOT_COMMAND_CMD_ID              = 0x0303,  //小地图下发信息标识
  CLIENT_MAP_COMMAND_CMD_ID         = 0x0305,  //小地图接收信息标识
	IDCustomData,
}referee_cmd_id_e;

typedef struct __attribute__((packed))
{
  uint8_t  SOF;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
	STEP_SHORT_DATA_CRC16 = 6,
} unpack_step_e;
typedef struct
{
  frame_header_struct_t *p_header;
  uint8_t  		 SOF;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

typedef struct
{
  frame_header_struct_t p_header;
	uint16_t 			 cmd_id;
  uint8_t        protocol_packet[VIS_PROTOCOL_FRAME_MAX_SIZE];
  uint16_t       index;
} send_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H














//#ifndef ROBOMASTER_PROTOCOL_H
//#define ROBOMASTER_PROTOCOL_H

//#include "struct_typedef.h"

//#define HEADER_SOF 0xA5
//#define VISION_HEADER_SOF 0x50
//#define NAV_HEADER_SOF 0xA4
///***************NAV******************/
//#define NAV_PROTOCOL_HEADER_SIZE 						1
//#define NAV_PROTOCOL_DATA_SIZE							NAV_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + 6 * sizeof(float)
///***************VISION******************/
//#define VIS_PROTOCOL_SEND_DATA_SIZE					16														//发送数据长度
//#define VIS_PROTOCOL_SEND_CMDID							0x001														

//#define VIS_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
//#define VIS_PROTOCOL_CMD_SIZE               2
//#define VIS_PROTOCOL_CRC16_SIZE             2
//#define VIS_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
//#define VIS_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
//#define VIS_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof((float)))

//#define VIS_PROTOCOL_FRAME_MAX_SIZE         128
////实际长度
////#define VIS_PROTOCOL_FRAME_MAX_SIZE         (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t)+VIS_PROTOCOL_SEND_DATA_SIZE)

///***************REFEREE******************/
//#define REF_PROTOCOL_FRAME_MAX_SIZE         128
//#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
//#define REF_PROTOCOL_CMD_SIZE               2
//#define REF_PROTOCOL_CRC16_SIZE             2
//#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
//#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
//#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

//#pragma pack(push, 1)

//typedef enum
//{
//    GAME_STATE_CMD_ID                 = 0x0001,
//    GAME_RESULT_CMD_ID                = 0x0002,
//    GAME_ROBOT_HP_CMD_ID              = 0x0003,
//    FIELD_EVENTS_CMD_ID               = 0x0101,
//    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
//    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
//    REFEREE_WARNING_CMD_ID            = 0x0104,
//    ROBOT_STATE_CMD_ID                = 0x0201,
//    POWER_HEAT_DATA_CMD_ID            = 0x0202,
//    ROBOT_POS_CMD_ID                  = 0x0203,
//    BUFF_MUSK_CMD_ID                  = 0x0204,
//    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
//    ROBOT_HURT_CMD_ID                 = 0x0206,
//    SHOOT_DATA_CMD_ID                 = 0x0207,
//    BULLET_REMAINING_CMD_ID           = 0x0208,
//    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
//    IDCustomData,
//}referee_cmd_id_t;
//typedef  struct
//{
//  uint8_t SOF;
//  uint16_t data_length;
//  uint8_t seq;
//  uint8_t CRC8;
//} frame_header_struct_t;

//typedef enum
//{
//  STEP_HEADER_SOF  = 0,
//  STEP_LENGTH_LOW  = 1,
//  STEP_LENGTH_HIGH = 2,
//  STEP_FRAME_SEQ   = 3,
//  STEP_HEADER_CRC8 = 4,
//  STEP_DATA_CRC16  = 5,
//	STEP_SHORT_DATA_CRC16 = 6,
//} unpack_step_e;


//typedef struct
//{
//  frame_header_struct_t *p_header;
//  uint8_t  		 SOF;
//  uint16_t       data_len;
//  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
//  unpack_step_e  unpack_step;
//  uint16_t       index;
//} unpack_data_t;

//typedef struct
//{
//  frame_header_struct_t p_header;
//	uint16_t 			 cmd_id;
//  uint8_t        protocol_packet[VIS_PROTOCOL_FRAME_MAX_SIZE];
//  uint16_t       index;
//} send_data_t;

//#pragma pack(pop)

//#endif //ROBOMASTER_PROTOCOL_H
