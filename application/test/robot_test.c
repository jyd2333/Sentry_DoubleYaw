#include "robot_test.h"
#include "message_center.h"
#include "bsp_usart.h"
#include "ins_task.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "rm_referee.h"
#include "general_def.h"
#include "SolveTrajectory.h"
#include "crc16.h"
NUC_cmd_t NUC_cmd;
extern UART_HandleTypeDef huart1;
extern DJIMotorInstance *yaw_motor;
Chassis_Ctrl_Cmd_s chassis_nuc_send; 
extern referee_info_t referee_info;           // 裁判系统数据
// uint8_t dzx_is_awesome[USART_RXBUFF_LIMIT]; // nuc接收数组
//static 
USARTInstance *NUC_recv;
// static void NUC_Data_Decode(short *buff);

uint8_t NUC_tx_buff[NUC_TX_BUFF_SIZE]={0};
uint8_t NUC_rx_buff[NUC_RX_BUFF_SIZE]={0};
uint8_t *USB_rx_buff;
float current_vx_t,current_vy_t;
int16_t current_wz,current_vx,current_vy;
uint16_t current_pitch,current_yaw;
float beta=0.04,theta=1;
int yaw_temp=0;
// uint8_t NUC_rx_buff[2][NUC_RX_BUFF_SIZE];

uint8_t Flag_Stop=1; //失能标志位

uint16_t Check_Sum(uint8_t* Data ,uint8_t Count);
float GetYawDiff(void);
// static Publisher_t* NUC_pub;
extern INS_Instance *INS;
USART_Init_Config_s NUC_Init_Config;
uint16_t daemon_reload=1000; //允许的串口离线时间
decision_state_t Decision_State={};
vision_send_t Vision_Send;
vision_receive_t Vision_Receive;
navigation_receive_t Navigation_Receive;

float vision_pitch = 0; //输出控制量 pitch绝对角度 弧度
float vision_yaw = 0;   //输出控制量 yaw绝对角度 弧度
USB_Init_Config_s USB_conf = {.rx_cbk = USB_Decode};
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];


void NUC_offline()   //离线处理
{																																																					

            NUC_cmd.vx = 0;
            NUC_cmd.vy = 0;
            NUC_cmd.wz = 0;
            NUC_cmd.pitch = 0;
            NUC_cmd.shot = 0;
            NUC_cmd.yaw = 0;
			// HAL_UART_Init(&huart1);
			__HAL_UART_DISABLE_IT(&huart1,UART_IT_RXNE);

            HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
			
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
// 	if(huart==&huart1)
// 	{
// 		if(NUC_rx_buff[0]==0xA5)
// 		{
// 		memcpy(&Navigation_Receive,NUC_rx_buff,sizeof(Navigation_Receive));
// 		NUC_cmd.vx=Navigation_Receive.vx;
// 		NUC_cmd.vy=-Navigation_Receive.vy;

// 		// NUC_cmd.wz=Navigation_Receive.wz;
// 		HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
// 		}
// 	}
// }

void USB_Decode(void)
{
	// if(NUC_rx_buff[0]==0xFF&&NUC_rx_buff[31]==0x0D)
	// {
	// 	NUC_cmd.delay=1000;
	// }
	memcpy(&Vision_Receive,UserRxBufferFS,sizeof(Vision_Receive));
	if(Vision_Receive.head == FRAME_HEADER && Vision_Receive.end == FRAME_END && Vision_Receive.check_sum == Check_Sum(&Vision_Receive.head,sizeof(vision_receive_t)-3))
	{
		NUC_cmd.vx = Vision_Receive.vx;
		NUC_cmd.vy = -Vision_Receive.vy;
		NUC_cmd.rotateMode = Vision_Receive.chassis_status;
		NUC_cmd.second = Vision_Receive.second;
		NUC_cmd.nano_second = Vision_Receive.nano_second;
		NUC_cmd.detect = Vision_Receive.major_number;
		vision_pitch = Vision_Receive.pitch;
		vision_yaw = Vision_Receive.yaw;
		if(Vision_Receive.major_number)
		{
			NUC_cmd.pitch = vision_pitch;
			NUC_cmd.yaw = vision_yaw;
			NUC_cmd.shot = Vision_Receive.fireadvise;
		}
		else
		{
			NUC_cmd.pitch = 0;
			NUC_cmd.yaw = 0;
			NUC_cmd.shot = 0;
		}
	}
	return;
}

void NUC_init(void)
{
	// HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
	NUC_cmd.delay = 200;
	USB_rx_buff = USBInit(USB_conf);
}

extern DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;

/**
 *@Function:	NUC_Send_Data()
 *@Description:	NUC数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */

void NUC_Send_Data(){
	Vision_Send.head = FRAME_HEADER;
	Vision_Send.enemy_color = (referee_info.referee_id.Robot_ID < 10) ? 2 : 1 ;//Red 1~7 BLUE 101~107本机器人
	Vision_Send.pitch = INS->output.INS_angle[1];
	Vision_Send.yaw = INS->output.INS_angle[2];
	Vision_Send.game_progress = referee_info.GameState.game_progress;
	Vision_Send.robot_level = referee_info.GameRobotStatus.robot_level;
	Vision_Send.Rfid_Status = referee_info.Rfid_Status.rfid_status;
	Vision_Send.current_hp = referee_info.GameRobotStatus.remain_HP;
	Vision_Send.maximum_hp = referee_info.GameRobotStatus.max_HP;
	Vision_Send.bullet_speed = referee_info.ShootData.bullet_speed;
	Vision_Send.stage_remain_time = referee_info.GameState.stage_remain_time;
	Vision_Send.armor_id = referee_info.RobotHurt.armor_id;
	Vision_Send.hurt_type = referee_info.RobotHurt.hurt_type;
	Vision_Send.Yaw_diff = GetYawDiff();
	Vision_Send.check_sum = Check_Sum(&Vision_Send.head,sizeof(vision_send_t)-3);
	Vision_Send.end = FRAME_END;

	memcpy(NUC_tx_buff,&Vision_Send ,sizeof(Vision_Send));
	// HAL_UART_Transmit_IT(&huart1,NUC_tx_buff,sizeof(NUC_tx_buff));
	USBTransmit(NUC_tx_buff, sizeof(NUC_tx_buff));
	// DMA_Cmd(DMA_Stream_NUC_TX, ENABLE);
}

uint16_t Check_Sum(uint8_t* Data ,uint8_t Count)
{
	uint16_t check_sum=0;
	for(uint8_t i = 0; i < Count; i++)
		check_sum += Data[i];
	return check_sum;
}

float GetYawDiff(void)
{
	return (float)(yaw_motor->measure.ecd - YAW_BIG_YAW_ALIGN_ECD) * 2 * PI / 8192;
}
