/**
 * @file robot_TEST.c
 * @author Smoaflie
 * @version 0.1
 * @date 2024-01-07
 * @introduce 调试用
 * 因为原封装的变量都使用了static修饰，故采用指针传入的方式进行按需调试
 * 
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 *
 */
#include "main.h"
#include "stdbool.h"
#include "arm_math.h"
#ifndef TEST_H
#define TEST_H


// void TEST_Init();
// void TEST_Task();
//DMA发送配置
#define DMA_Stream_NUC_TX DMA2_Stream7
#define DMA_Stream_NUC_RX DMA2_Stream2


#define NUC_TX_BUFF_SIZE  SEND_DATA_SIZE
#define SEND_DATA_SIZE    sizeof(vision_send_t)//28//24
#define RECEIVE_DATA_SIZE sizeof(vision_receive_t)

#define FRAME_HEADER    0X5A
#define FRAME_END		0X55

// #define NUC_RX_BUFF_SIZE 7+RECEIVE_DATA_SIZE  //NUC通信缓存大小
#define NUC_RX_BUFF_SIZE RECEIVE_DATA_SIZE
#define NUC_TX_BUFF_SIZE SEND_DATA_SIZE

#define SENTRY_TARGET 0
/*
SENTRY_TARGET:
0:全功能模式 导航、自瞄均正常运行，积极占领中央增益点
暂无1:保守模式 导航、自瞄均正常运行，仅导航到增益点外视野开阔处
暂无2:自瞄+磁力计模式 导航故障，自瞄能正常运行，依靠磁力计驶出补给区，不具备返回能力
暂无3:自瞄+小陀螺模式 导航故障，自瞄能正常运行，原地小陀螺，靠其他兵种推动，不具备自主行驶能力
暂无4:自保模式 导航、自瞄均故障，最高转速原地小陀螺
暂无5:调试模式
*/


typedef enum
{
	SENTRY_STOP,
	PUSH_FORWARD,
	PATROL,
	HEALING,
	BACK_FORWARD
}behaviour_state_e;

typedef enum
{
	NO_ROTATE,
	HIGHSPEED_ROTATE,
	LOWSPEED_ROTATE
	
}chassis_state_e;

typedef enum
{
	NO_TARGET,//0
	RECHARGE_AREA,//1
	CENTRAL_RETRY,//2
	CENTRAL_AREA,//3
	RECHARGE_RETRY//4
}navigation_mode_e;

typedef enum
{
	NAVI_PENDING,//0:目标已提交，但尚未被处理(等待中)。
	NAVI_ACTIVE,//1:目标当前正在被处理(导航中)。
	NAVI_PREEMPTED,//2:目标被新的目标抢占(例如，发送了新的目标点)。
	NAVI_SUCCEEDED,//3:目标成功完成(机器人到达目标点)。
	NAVI_ABORTED,//4:目标因某种原因失败(例如，路径规划失败或机器人卡住)。
	NAVI_REJECT,//5:目标被拒绝(例如，目标点不可达)。
	NAVI_PREEMPTING,//6:目标正在被抢占(例如，正在取消当前目标以处理新目标)。
	NAVI_RECALLING,//7:目标正在被召回(例如，用户请求取消目标)。
	NAVI_RECALLED//8:目标已被召回(取消成功)。
}navigation_feed_e;

typedef struct
{
	behaviour_state_e Behaviour_State;
	chassis_state_e Chassis_State;
	navigation_mode_e Navigation_Mode;
	uint32_t Navigation_Wait;
}decision_state_t;

typedef struct 
{
    /* data */
    int vx;
    int vy;
    int wz;
	float pitch;
	float yaw;
	int shot;
	uint8_t detect;
	int16_t delay;
	uint32_t second;
	uint32_t nano_second;
	// uint8_t vision_breath;
	// uint8_t vision_breath_last;
	navigation_feed_e Navigation_Feed;
	uint8_t scanMode;
	uint8_t rotateMode;
	float odomYaw;
}NUC_cmd_t;

extern NUC_cmd_t NUC_cmd;

#pragma pack(1) // 压缩结构体,取消字节对齐
typedef struct
{
	uint8_t head;//0
	uint8_t enemy_color; //0：未开始 1：红色 2：蓝色（敌方颜色）
	uint8_t game_progress;
	uint8_t robot_level;
	uint32_t Rfid_Status;
	uint16_t current_hp;
	uint16_t maximum_hp;
	float pitch;
	float yaw;
	float bullet_speed;
	uint16_t stage_remain_time;
	uint8_t reserve[5];
	uint8_t end;
}vision_send_t;

typedef struct
{
	uint8_t head;
	uint8_t fireadvise;//0：不开火 1：开火
	uint8_t detect;
	uint8_t chassis_status;
	float pitch;
	float yaw;
	uint32_t second;
	uint32_t nano_second;
	float vx;
	float vy;
	uint8_t reserve[3];
	uint8_t end;
}vision_receive_t;
#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

typedef struct
{
	uint8_t head;//0
	uint8_t reserve;//1
	uint8_t spin;
	uint8_t reserve1[13];
	float vx;
	float vy;
	float odomYaw;
	uint8_t reserve2[2];
	uint8_t buffer_type;//0x02
	uint8_t end;
}navigation_receive_t;


void NUC_Send_Data();
// void data_transition(void);
void NUC_init(void);
// void daemon_NUC();
void NUC_offline();
void Decision_Tree();
void USB_Decode(void);
#endif
