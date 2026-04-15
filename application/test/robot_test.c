#include "robot_test.h"
#include "message_center.h"
#include "bsp_usart.h"
#include "ins_task.h"
#include "robot_def.h"
#include "shoot.h"
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

uint8_t Check_Sum_8(uint8_t* Data ,uint8_t Count);
uint16_t Check_Sum_16(uint8_t* Data ,uint8_t Count);
void EularAngleToQuaternion(float Y, float P, float R, float *q);
float GetYawDiff(void);
// static Publisher_t* NUC_pub;
extern INS_Instance *INS;
USART_Init_Config_s NUC_Init_Config;
uint16_t daemon_reload=1000; //允许的串口离线时间
vision_send_t Vision_Send = {};
situation_alpha_t Situation_Alpha = {};
situation_beta_t Situation_Beta = {};
vision_receive_t Vision_Receive = {};
navigation_receive_t Navigation_Receive = {};

float vision_pitch = 0; //输出控制量 pitch绝对角度 弧度
float vision_yaw = 0;   //输出控制量 yaw绝对角度 弧度
float quat_tran[4];
extern int32_t load_count;
USB_Init_Config_s USB_conf = {.rx_cbk = USB_Decode};
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];


void NUC_offline()   //离线处理
{																																																					

            NUC_cmd.vx = 0;
            NUC_cmd.vy = 0;
            NUC_cmd.wz = 0;
            NUC_cmd.pitch = 0;
            NUC_cmd.shoot = 0;
            NUC_cmd.yaw = 0;
			// HAL_UART_Init(&huart1);
			// __HAL_UART_DISABLE_IT(&huart1,UART_IT_RXNE);

            // HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
			
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
	switch(UserRxBufferFS[2])
	{
		case 0x01:
			memcpy(&Vision_Receive, UserRxBufferFS, sizeof(Vision_Receive));
			if(Vision_Receive.head == FRAME_HEADER && Vision_Receive.end == FRAME_END && Vision_Receive.check_sum == Check_Sum_16(&Vision_Receive.head,sizeof(vision_receive_t)-3))
			{
				NUC_cmd.time_stamp 	= Vision_Receive.time_stamp;
				vision_pitch 		= -Vision_Receive.pitch;
				vision_yaw 			= -RAD_2_DEGREE * Vision_Receive.yaw;
				NUC_cmd.pitch 		= vision_pitch;
				NUC_cmd.yaw 		= vision_yaw;
				NUC_cmd.shoot 		= Vision_Receive.fireadvise;
				NUC_cmd.pitch_vel 	= -Vision_Receive.pitch_vel;
				NUC_cmd.pitch_acc 	= -Vision_Receive.pitch_acc;
				NUC_cmd.yaw_vel 	= -Vision_Receive.yaw_vel;
				NUC_cmd.yaw_acc 	= -Vision_Receive.yaw_acc;
			}
			break;
		case 0x02:
			memcpy(&Navigation_Receive, UserRxBufferFS, sizeof(Navigation_Receive));
			if(Navigation_Receive.head == FRAME_HEADER && Navigation_Receive.end == FRAME_END && Navigation_Receive.check_sum == Check_Sum_16(&Navigation_Receive.head,sizeof(navigation_receive_t)-3))
			{
				NUC_cmd.time_stamp	= Navigation_Receive.time_stamp;
				NUC_cmd.vx 			= Navigation_Receive.vx;
				NUC_cmd.vy 			= Navigation_Receive.vy;
				NUC_cmd.base_yaw	= Navigation_Receive.base_yaw;
				NUC_cmd.scanMode	= Navigation_Receive.mode;
				NUC_cmd.rotateMode	= Navigation_Receive.chassis_status;
			}
			break;
		default:
			break;
	}
	
	return;
}
uint8_t NUC_send_count = 0;

void NUC_init(void)
{
	// HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
	NUC_cmd.delay = 200;
	USB_rx_buff = USBInit(USB_conf);

	Vision_Send.head 		= FRAME_HEADER;
	Vision_Send.cmdid 		= 0x01;
	Vision_Send.end 		= FRAME_END;

	Situation_Alpha.head 	= FRAME_HEADER;
	Situation_Alpha.cmdid 	= 0x02;
	Situation_Alpha.end 	= FRAME_END;

	Situation_Beta.head 	= FRAME_HEADER;
	Situation_Beta.cmdid 	= 0x03;
	Situation_Beta.end 		= FRAME_END;

}

extern DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;


/**
 *@Function:	NUC_Send_Data()
 *@Description:	NUC数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */

void NUC_Send_Data(){
	Vision_Send.DWT_stamp 		= DWT_GetTimeline_ms();
	Vision_Send.enemy_color 	= (referee_info.referee_id.Robot_ID < 10) ? 2 : 1 ;//Red 1~7 BLUE 101~107本机器人
	EularAngleToQuaternion(-INS->output.INS_angle[1], -INS->output.INS_angle[2], INS->output.INS_angle[0],quat_tran);
	memcpy(Vision_Send.quat,quat_tran,16);
	Vision_Send.pitch 			= INS->output.INS_angle[1];
	Vision_Send.pitch_gyro		= INS->INS_data.INS_gyro[1];
	Vision_Send.yaw 			= INS->output.INS_angle[2];
	Vision_Send.yaw_gyro		= INS->INS_data.INS_gyro[2];
	Vision_Send.Yaw_diff 		= GetYawDiff();
	Vision_Send.bullet_speed 	= referee_info.ShootData.bullet_speed;
	Vision_Send.load_count		= load_count;
	Vision_Send.check_sum 		= Check_Sum_16(&Vision_Send.head,sizeof(vision_send_t)-2);
	
	Situation_Alpha.DWT_stamp 			= DWT_GetTimeline_ms();
	Situation_Alpha.game_progress 		= referee_info.GameState.game_progress;
	Situation_Alpha.stage_remain_time 	= referee_info.GameState.stage_remain_time;
	Situation_Alpha.event_data 			= referee_info.EventData.event_type;
	Situation_Alpha.current_hp 			= referee_info.GameRobotStatus.remain_HP;
	Situation_Alpha.maximum_hp 			= referee_info.GameRobotStatus.max_HP;
	Situation_Alpha.armor_id			= referee_info.RobotHurt.armor_id;
	Situation_Alpha.hp_deduction_reason	= referee_info.RobotHurt.hurt_type;
	Situation_Alpha.disengaged_state	= referee_info.SentryInfo.sentry_info_2 & 0x0001;
	Situation_Alpha.current_state		= (referee_info.SentryInfo.sentry_info_2 >> 12) & 0x0003;
	Situation_Alpha.ally_power_rune_state =	(referee_info.SentryInfo.sentry_info_2 >> 14) & 0x0001;
	Situation_Alpha.rfid_status			= referee_info.Rfid_Status.rfid_status;
	Situation_Alpha.check_sum 			= Check_Sum_16(&Situation_Alpha.head,sizeof(situation_alpha_t)-2);


	Situation_Beta.DWT_stamp 				= DWT_GetTimeline_ms();
	Situation_Beta.init_sentry_position_x 	= referee_info.GameRobotPos.x;
	Situation_Beta.init_sentry_position_y 	= referee_info.GameRobotPos.y;
	Situation_Beta.hero_x					= referee_info.GroundRobotPosition.hero_x;
	Situation_Beta.hero_y					= referee_info.GroundRobotPosition.hero_y;
	Situation_Beta.engineer_x				= referee_info.GroundRobotPosition.engineer_x;
	Situation_Beta.engineer_y				= referee_info.GroundRobotPosition.engineer_y;
	Situation_Beta.standard_3_x				= referee_info.GroundRobotPosition.standard_3_x;
	Situation_Beta.standard_3_y				= referee_info.GroundRobotPosition.standard_3_y;
	Situation_Beta.standard_4_x				= referee_info.GroundRobotPosition.standard_4_x;
	Situation_Beta.standard_4_y				= referee_info.GroundRobotPosition.standard_4_y;
	Situation_Beta.ally_1_robot_HP			= referee_info.GameRobotHP.ally_1_robot_HP;
	Situation_Beta.ally_2_robot_HP			= referee_info.GameRobotHP.ally_2_robot_HP;
	Situation_Beta.ally_3_robot_HP			= referee_info.GameRobotHP.ally_3_robot_HP;
	Situation_Beta.ally_4_robot_HP			= referee_info.GameRobotHP.ally_4_robot_HP;
	Situation_Beta.ally_outpost_HP			= referee_info.GameRobotHP.ally_outpost_HP;
	Situation_Beta.ally_base_HP				= referee_info.GameRobotHP.ally_base_HP;
	Situation_Beta.check_sum 				= Check_Sum_16(&Situation_Beta.head,sizeof(situation_beta_t)-2);

	// memcpy(NUC_tx_buff,&Vision_Send ,sizeof(Vision_Send));
	// HAL_UART_Transmit_IT(&huart1,NUC_tx_buff,sizeof(NUC_tx_buff));
	// DMA_Cmd(DMA_Stream_NUC_TX, ENABLE);
	switch(NUC_send_count)
	{
		case 0:
		case 2:
			memcpy(NUC_tx_buff,&Vision_Send ,sizeof(Vision_Send));
			break;
		case 1:
			memcpy(NUC_tx_buff,&Situation_Alpha ,sizeof(Situation_Alpha));
			break;
		case 3:
			memcpy(NUC_tx_buff,&Situation_Beta ,sizeof(Situation_Beta));
			break;
		default:
			break;
	}
	// memcpy(NUC_tx_buff,&Vision_Send ,sizeof(Vision_Send));
	USBTransmit(NUC_tx_buff, sizeof(NUC_tx_buff));
	NUC_send_count++;
	if(NUC_send_count > 3) NUC_send_count = 0;
}

uint8_t Check_Sum_8(uint8_t* Data ,uint8_t Count)
{
	uint8_t check_sum=0;
	for(uint8_t i = 0; i < Count; i++)
		check_sum += Data[i];
	return check_sum;
}

uint16_t Check_Sum_16(uint8_t* Data ,uint8_t Count)
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

void EularAngleToQuaternion(float Y, float P, float R, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    // Y /= 57.295779513f;
    // P /= 57.295779513f;
    // R /= 57.295779513f;
    cosPitch = arm_cos_f32(P / 2);
    cosYaw   = arm_cos_f32(Y / 2);
    cosRoll  = arm_cos_f32(R / 2);
    sinPitch = arm_sin_f32(P / 2);
    sinYaw   = arm_sin_f32(Y / 2);
    sinRoll  = arm_sin_f32(R / 2);
q[0] = cosYaw * cosPitch * cosRoll + sinYaw * sinPitch * sinRoll;
q[1] = cosYaw * cosPitch * sinRoll - sinYaw * sinPitch * cosRoll;
q[2] = cosYaw * sinPitch * cosRoll + sinYaw * cosPitch * sinRoll;
q[3] = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;
}
