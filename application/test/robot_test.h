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
#include "referee_protocol.h"
#ifndef TEST_H
#define TEST_H


// void TEST_Init();
// void TEST_Task();
//DMA发送配置
#define DMA_Stream_NUC_TX DMA2_Stream7
#define DMA_Stream_NUC_RX DMA2_Stream2


#define NUC_TX_BUFF_SIZE  SEND_DATA_SIZE
#define SEND_DATA_SIZE    sizeof(vision_send_t)
#define RECEIVE_DATA_SIZE sizeof(vision_receive_t)
#define NAVIGATION_RECEIVE_DATA_SIZE sizeof(navigation_receive_t)
#define SITUATION_ALPHA_SIZE sizeof(situation_alpha_t)
#define SITUATION_BETA_SIZE sizeof(situation_beta_t)

#define FRAME_HEADER    0X5A
#define FRAME_END		0X55

#define SENTRY_REFEREE_CMD_ID      0x0120
#define SENTRY_REFEREE_RECEIVER_ID 0x8080
#define SENTRY_REFEREE_DATA_LEN    (Interactive_Data_LEN_Head + sizeof(sentry_cmd_t))

// #define NUC_RX_BUFF_SIZE 7+RECEIVE_DATA_SIZE  //NUC通信缓存大小
#define NUC_RX_BUFF_SIZE RECEIVE_DATA_SIZE
#define NUC_TX_BUFF_SIZE SEND_DATA_SIZE

typedef enum
{
	TERRAIN_NORMAL,
	TERRAIN_FORTRESS,	//堡垒
	TERRAIN_BUMP,		//起伏路段
}terrain_state_e;

typedef struct 
{
    /* data */
    float vx;
    float vy;
    float wz;
	float pitch;
	float pitch_vel;
	float pitch_acc;
	float yaw;
	float yaw_vel;
	float yaw_acc;
	int shoot;
	int16_t delay;
	uint64_t time_stamp;
	uint64_t navi_time_stamp;
	uint8_t scanMode;		//0：自瞄装甲板 1：前哨站 2：小能量机关 3：大能量机关
	uint8_t rotateMode;
	terrain_state_e terrain_state;
	float base_yaw;
}NUC_cmd_t;

#pragma pack(1) // 压缩结构体,取消字节对齐

typedef struct
{
	uint32_t sentry_cmd;
}sentry_cmd_t;

typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	ext_student_interactive_header_data_t datahead;
	sentry_cmd_t data;
	uint16_t frametail;
}sentry_referee_send_t;

typedef struct
{
	uint8_t head;
	uint8_t cmdid;
	float DWT_stamp;

	uint8_t enemy_color; //0：未开始 1：红色 2：蓝色（敌方颜色）
	float quat[4];
	float pitch;
	float pitch_gyro;
	float yaw;
	float yaw_gyro;
	float Yaw_diff;
	float bullet_speed;
	uint16_t load_count;

	uint8_t reserve[12];
	uint8_t end;
	uint16_t check_sum;
}vision_send_t;

typedef struct
{
	uint8_t head;
	uint8_t cmd_id;
	uint64_t time_stamp;
	uint8_t fireadvise;//0：未识别到 1：识别到、不开火 2：开火
	float pitch;
	float pitch_vel;
	float pitch_acc;
	float yaw;
	float yaw_vel;
	float yaw_acc;
	uint8_t reserve[26];
	uint16_t check_sum;
	uint8_t end;
}vision_receive_t;

typedef struct
{
	uint8_t head;
	uint8_t cmd_id;
	uint64_t time_stamp;
	uint8_t chassis_status;	//底盘状态：小陀螺转速
	uint8_t sentry_status;	//姿态信息 1：进攻姿态 2：防御姿态 3：移动姿态
	uint8_t scanmode;		//0：自瞄装甲板 1：前哨站 2：小能量机关 3：大能量机关
	float vx;
	float vy;
	float base_yaw;
	uint8_t fortress_mode;	//堡垒
	uint8_t bump_mode;		//起伏路段
	uint8_t reserve[34];
	uint16_t check_sum;
	uint8_t end;
}navigation_receive_t;

typedef struct
{
	uint8_t head;
	uint8_t cmdid;
	float DWT_stamp;

	uint8_t game_progress;
  	uint16_t stage_remain_time;
	uint32_t event_data;
	uint16_t current_hp;
  	uint16_t maximum_hp;
  	uint8_t armor_id;
  	uint8_t hp_deduction_reason;
  	uint8_t disengaged_state;
  	uint8_t current_state;
  	uint8_t ally_power_rune_state; 
	uint32_t rfid_status;

	uint8_t reserve[35];
	uint8_t end;
	uint16_t check_sum;
}situation_alpha_t;

typedef struct
{
	uint8_t head;
	uint8_t cmdid;
	float DWT_stamp;

	float init_sentry_position_x;
  	float init_sentry_position_y;
  	float hero_x;
  	float hero_y;
  	float engineer_x;
  	float engineer_y;
  	float standard_3_x;
  	float standard_3_y;
  	float standard_4_x;
  	float standard_4_y;
	uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
	uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;

	uint8_t reserve[3];
	uint8_t end;
	uint16_t check_sum;
}situation_beta_t;
#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)


void NUC_Send_Data();
// void data_transition(void);
void NUC_init(void);
// void daemon_NUC();
void NUC_offline();
// void Decision_Tree();
void USB_Decode(void);
extern uint8_t Sentry_Energy_Confirm;
#endif
