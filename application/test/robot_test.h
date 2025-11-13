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
#define SEND_DATA_SIZE    sizeof(navigation_send_t)//28//24
#define RECEIVE_DATA_SIZE sizeof(navigation_receive_t)

#define FRAME_HEADER      0X7B //Frame_header 
#define FRAME_TAIL        0X7D //Frame_tail   

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





//存放陀螺仪三轴可直接读取的数据（加速度、角加速度）的结构体//
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2¸ö×Ö½Ú
	float Y_data; //2 bytes //2¸ö×Ö½Ú
	float Z_data; //2 bytes //2¸ö×Ö½Ú
}Mpu6050_Data;

//串口发送数据的总格式（共22位）//
typedef struct _SEND_DATA_
{
		unsigned char Frame_Header; //1¸字节
		short X_speed;	            //2 bytes 
		short Z_speed;              //2 bytes 
		short Y_speed;              //2 bytes 
		short Power_Voltage;        //2 bytes 
		Mpu6050_Data Accelerometer; //6 bytes 
		Mpu6050_Data Gyroscope;     //6 bytes 
		unsigned char Frame_Tail;   //1 bytes 
}SEND_DATA;
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
    float32_t vx;
    float32_t vy;
    float32_t wz;
	float pitch;
	float yaw;
	int shot;
	uint16_t delay;
	uint8_t vision_breath;
	uint8_t vision_breath_last;
	navigation_feed_e Navigation_Feed;
	uint8_t scanMode;
	uint8_t rotateMode;
	float32_t odomYaw;
}NUC_cmd_t;

extern NUC_cmd_t NUC_cmd;

typedef struct 
{
  uint8_t header;
  uint8_t tracking; // 代表当前是否锁定目标
  uint8_t id ;          // 0-outpost 6-guard 7-base
  uint8_t armors_num ;  // 2-balance 3-outpost 4-normal
  uint8_t reserved ;
  float x; // 目标在世界坐标系下的 x 坐标
  float y; // 目标在世界坐标系下的 Y 坐标
  float z; //目标在世界坐标系下的 Z 坐标
  float yaw; // 目标在世界坐标系下的倾斜角度
  float vx; // 目标在世界坐标系下 x 方向的速度
  float vy; // 目标在世界坐标系下 y 方向的速度
  float vz; // 目标在世界坐标系下 z 方向的速度
  float v_yaw; // 目标旋转的角速度
  float r1; // 目标其中一组装甲板相对中心的半径
  float r2; // 目标另一组装甲板相对中心的半径
  float dz; // tracking 中的装甲板的上一块装甲板的 z 轴位置
  uint16_t checksum ;
} __attribute__((packed)) SendPacket_t;

extern SendPacket_t SendPacket;

typedef struct 
{
  uint8_t header ;
  uint8_t detect_color ; // 0: red, 1: blue
  uint8_t reset_tracker;
  uint8_t reserved ;
  float roll; // // 世界坐标系下云台当前的 roll
  float pitch; // 世界坐标系下云台当前的 pitch
  float yaw; // 世界坐标系下云台当前的 yaw
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t reserved_2;
  uint16_t checksum ;
} __attribute__((packed)) ReceivePacket_t;

typedef struct
{
	uint8_t header ;
	uint8_t length;
	uint8_t ID;
	uint8_t crc1;
	uint32_t timestamp;
	float pitch;
	float yaw;
	uint16_t crc2;
}navigation_send_t;

#pragma pack (1)
typedef struct
{
	uint8_t header;
	uint8_t length;
	uint8_t ID;
	uint8_t crc1;
	float32_t vx;
	float32_t vy;
	float64_t odomYaw;
	uint8_t scanMode;
	uint8_t rotateMode;
}navigation_receive_t;
#pragma pack ()	

extern ReceivePacket_t ReceivePacket;

void NUC_Send_Data();
void data_transition(void);
void NUC_init(void);
// void daemon_NUC();
void NUC_offline();
void Decision_Tree();
void USB_Decode(void);
#endif
