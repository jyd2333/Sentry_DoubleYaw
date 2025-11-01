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
extern DJIMotorInstance *yaw_motor,*pitch_motor;
Chassis_Ctrl_Cmd_s chassis_nuc_send; 
extern referee_info_t referee_info;           // 裁判系统数据
// uint8_t dzx_is_awesome[USART_RXBUFF_LIMIT]; // nuc接收数组
//static 
USARTInstance *NUC_recv;
// static void NUC_Data_Decode(short *buff);

SEND_DATA Send_Data;
uint8_t NUC_tx_buff[NUC_TX_BUFF_SIZE]={0};
uint8_t NUC_rx_buff[NUC_RX_BUFF_SIZE]={0};
uint8_t *USB_rx_buff;
SendPacket_t SendPacket;
ReceivePacket_t ReceivePacket;
float current_vx_t,current_vy_t;
int16_t current_wz,current_vx,current_vy;
uint16_t current_pitch,current_yaw;
float beta=0.04,theta=1;
int yaw_temp=0;
// uint8_t NUC_rx_buff[2][NUC_RX_BUFF_SIZE];

uint8_t Flag_Stop=1; //失能标志位
// static Subscriber_t * IMU_Data_sub;    //接收陀螺仪数据的订阅者实例
//static Gimbal_Upload_Data_s *IMU_Data; // 用于接收陀螺仪数据
static Subscriber_t *chassis_nuc_sub;  // 获取cmd发送给底盘的订阅者实例
static Chassis_Ctrl_Cmd_s *chassis_data_to_nuc; // 用于接收cmd发送给底盘的控制信息 
uint8_t Check_Sum(unsigned char StartNum ,unsigned char  Count_Number,unsigned char Mode);
// static Publisher_t* NUC_pub;
extern INS_Instance *INS;
USART_Init_Config_s NUC_Init_Config;
uint16_t daemon_reload=1000; //允许的串口离线时间
decision_state_t Decision_State={};
navigation_send_t Navigation_Send={};
navigation_receive_t Navigation_Receive={};


extern struct SolveTrajectoryParams st;
// ReceivePacket_t Rec_cmp;
float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
float vision_pitch = 0; //输出控制量 pitch绝对角度 弧度
float vision_yaw = 0;   //输出控制量 yaw绝对角度 弧度
USB_Init_Config_s USB_conf = {.rx_cbk = USB_Decode};
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];


void NUC_offline()   //离线处理
{																																																					

            NUC_cmd.vx=0;
            NUC_cmd.vy=0;
            NUC_cmd.wz=0;
            NUC_cmd.pitch=0;
            NUC_cmd.shot=0;
            NUC_cmd.yaw=0;
			// HAL_UART_Init(&huart1);
			__HAL_UART_DISABLE_IT(&huart1,UART_IT_RXNE);

            HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
			
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1)
	{
		if(NUC_rx_buff[0]==0xA5)
		{
		memcpy(&Navigation_Receive,NUC_rx_buff,sizeof(Navigation_Receive));
		NUC_cmd.vx=Navigation_Receive.vx;
		NUC_cmd.vy=-Navigation_Receive.vy;
		NUC_cmd.wz=Navigation_Receive.wz;
		HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
		}
	}
}

void USB_Decode(void)
{
	memcpy(&Navigation_Receive,UserRxBufferFS ,sizeof(Navigation_Receive));
	if(Navigation_Receive.header == 0x5A)
	{
		NUC_cmd.vx = Navigation_Receive.vx;
		NUC_cmd.vy=-Navigation_Receive.vy;
		NUC_cmd.wz=Navigation_Receive.wz;
	}
}

void NUC_init(void)
{
	
	// HAL_UART_Receive_IT(&huart1,NUC_rx_buff,NUC_RX_BUFF_SIZE);
	NUC_cmd.delay=200;
	USB_rx_buff = USBInit(USB_conf);
    chassis_nuc_sub  = SubRegister("chassis_cmd_2", sizeof(Chassis_Ctrl_Cmd_s));
}
extern DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
/**
 *@Function:	NUC_Send_Data()
 *@Description:	NUC数据发送
 *@Param:       形参
 *@Return:	  	返回值
 */
void data_transition(void)
{

    //SubGetMessage(IMU_Data_sub, &IMU_Data);
    // SubGetMessage(chassis_nuc_sub, &chassis_data_to_nuc);
	chassis_data_to_nuc=&chassis_nuc_send;
    //帧头
	Send_Data.Frame_Header = FRAME_HEADER; 
   //三轴线速度(直接从遥控器获取了，后续改实际反馈值)
	Send_Data.X_speed =  chassis_data_to_nuc->vx ;
	Send_Data.Y_speed =  chassis_data_to_nuc->vy ;
	Send_Data.Z_speed =  chassis_data_to_nuc->wz ;  
    //三轴线加速度
    Send_Data.Accelerometer.X_data =  /*IMU_Data->gimbal_imu_data->*/NUC_SEND_IMU->INS_data.INS_accel[1] ;//加速度x、y轴与ROS轴互换
    Send_Data.Accelerometer.Y_data =  /*IMU_Data->gimbal_imu_data->*/NUC_SEND_IMU->INS_data.INS_accel[0] ;
    Send_Data.Accelerometer.Z_data =  /*IMU_Data->gimbal_imu_data->*/NUC_SEND_IMU->INS_data.INS_accel[2] ;
    //三轴角加速度
    Send_Data.Gyroscope.X_data =  /*IMU_Data->gimbal_imu_data->*/NUC_SEND_IMU->INS_data.INS_gyro[1] ;
    Send_Data.Gyroscope.Y_data =  /*IMU_Data->gimbal_imu_data->*/NUC_SEND_IMU->INS_data.INS_gyro[0] ;
if(Flag_Stop==0) 
    Send_Data.Gyroscope.Z_data =  /*IMU_Data->gimbal_imu_data->*/NUC_SEND_IMU->INS_data.INS_gyro[2] ;
else  
	Send_Data.Gyroscope.Z_data=0;   
    //帧尾
    Send_Data.Frame_Tail = FRAME_TAIL; 
    Send_Data.Power_Voltage = 5200; //放大一千倍，接收后缩小一千倍

	current_vx_t=-0.03702*beta*(motor_lf->measure.speed_rpm-motor_rf->measure.speed_rpm+motor_lb->measure.speed_rpm-motor_rb->measure.speed_rpm);
	if(NUC_cmd.vx!=0)
	{
		if(NUC_cmd.vx>0&&current_vx_t>-1)	current_vx=-1;
		if(NUC_cmd.vx<0&&current_vx_t<1)	current_vx=1;
		if(current_vx_t>=1||current_vx_t<=-1) current_vx=current_vx_t;
	}
	else 
	current_vx=current_vx_t;
	// current_vy*=0.104*RADIUS_WHEEL*0.025;
	//current_vx*=-0.03702*beta;
	current_vy_t=-0.03702*beta*(-motor_lf->measure.speed_rpm-motor_rf->measure.speed_rpm+motor_lb->measure.speed_rpm+motor_rb->measure.speed_rpm);
	if(NUC_cmd.vy!=0)
	{
		if(NUC_cmd.vy>0&&current_vy_t<1)	current_vy=1;
		if(NUC_cmd.vy<0&&current_vy_t>-1)	current_vy=-1;
		if(current_vy_t>=1||current_vy_t<=-1) current_vy=current_vy_t;
	}
	else 
	current_vy=current_vy_t;
	// current_vx*=0.104*RADIUS_WHEEL*0.025;
	// current_vy*=-0.03702*beta;
	current_wz=-0.05234*beta*(-motor_lf->measure.speed_rpm-motor_rf->measure.speed_rpm-motor_lb->measure.speed_rpm-motor_rb->measure.speed_rpm);
	// current_wz*=0.104*RADIUS_WHEEL*0.025/0.15;
	// current_wz*=-0.05234*beta;
	current_yaw=(INS->output.INS_angle[2]+PI)*RAD_2_DEGREE*360/8192;
}


void NUC_Send_Data(){
	
	Navigation_Send.header = 0x5A;
	Navigation_Send.length = 8;
	Navigation_Send.ID = 0x0C;
	Navigation_Send.pitch = INS->output.INS_angle[1];
	Navigation_Send.yaw = INS->output.INS_angle[2];
	// Navigation_Send.test_data=0xAB;
	// Navigation_Send.tail=0x5B;
	memcpy(NUC_tx_buff,&Navigation_Send,sizeof(Navigation_Send));
	USBTransmit(NUC_tx_buff, sizeof(NUC_tx_buff));
	// HAL_UART_Transmit_IT(&huart1,NUC_tx_buff,sizeof(NUC_tx_buff));
	// DMA_Cmd(DMA_Stream_NUC_TX, ENABLE);
}




// float XYZ_Target_Speed_transition(uint8_t High,uint8_t Low) 
// {
//     //高低八位整合
// 	return 
// 		((High<<8)+Low); //mm/s			
// }
// uint8_t HAL_DMA_GetCurrentMemoryTarget(DMA_Stream_TypeDef *DMA_Stream)
// {
//     return (DMA_Stream->CR & DMA_SxCR_CT) ? 1 : 0;
// }


uint8_t Check_Sum(unsigned char StartNum ,unsigned char  Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(Mode==1)
		for(k=StartNum; k < StartNum + Count_Number; k++)
		{
			check_sum=check_sum^NUC_tx_buff[k];
		}
	
	//Verify the data received
	//对要接收的数据进行校验
	// if(Mode==0)
	// 	for(k=StartNum; k < StartNum + Count_Number; k++)
	// 	{
	// 		check_sum=check_sum^NUC_rx_buff[1-HAL_DMA_GetCurrentMemoryTarget(DMA_Stream_NUC_RX)][k];
	// 	}
	return check_sum;
}

// decision_state_e Decision_State=ROBOT_STOP;
// chassis_state_e Chassis_Stage=NO_ROTATE; 

void Decision_Tree()
{
	if(referee_info.GameState.game_progress==4)//比赛开始
	{
		Decision_State.Navigation_Wait++;
		if(referee_info.GameState.stage_remain_time>=290)//前期前压占点
		{
			Decision_State.Behaviour_State = PUSH_FORWARD;
			// Decision_State.Chassis_State=NO_ROTATE;
			if(!(Decision_State.Navigation_Mode==CENTRAL_RETRY||Decision_State.Navigation_Mode==CENTRAL_AREA))
			{
				Decision_State.Navigation_Mode=CENTRAL_AREA;
				Decision_State.Navigation_Wait=0;
			}
		}
		else
		{
			if(referee_info.GameRobotStatus.remain_HP<200)
			{
				Decision_State.Behaviour_State=HEALING;
				// Decision_State.Chassis_State=LOWSPEED_ROTATE;
				if(!(Decision_State.Navigation_Mode==RECHARGE_AREA||Decision_State.Navigation_Mode==RECHARGE_RETRY))
				{
					Decision_State.Navigation_Mode=RECHARGE_AREA;
					Decision_State.Navigation_Wait=0;
				}
			}
			else
			{
				if(referee_info.GameRobotStatus.remain_HP>=200&&Decision_State.Behaviour_State!=HEALING)
				{
					Decision_State.Behaviour_State=PATROL;
					// Decision_State.Chassis_State=LOWSPEED_ROTATE;
					if(!(Decision_State.Navigation_Mode==CENTRAL_AREA||Decision_State.Navigation_Mode==CENTRAL_RETRY))
					{
						Decision_State.Navigation_Mode=CENTRAL_AREA;
						Decision_State.Navigation_Wait=0;
					}
				}
				if(referee_info.GameRobotStatus.remain_HP>=390)
				{
					Decision_State.Behaviour_State=PATROL;
				}
			}
		}
		if(Decision_State.Navigation_Wait>2000&&NUC_cmd.Navigation_Feed==NAVI_ABORTED)//导航失败切换备用方案
		{
			if(Decision_State.Navigation_Mode==CENTRAL_AREA)
				Decision_State.Navigation_Mode=CENTRAL_RETRY;
			else if(Decision_State.Navigation_Mode==CENTRAL_RETRY)
				Decision_State.Navigation_Mode=CENTRAL_AREA;
			if(Decision_State.Navigation_Mode==RECHARGE_AREA)
				Decision_State.Navigation_Mode=RECHARGE_RETRY;
			else if(Decision_State.Navigation_Mode==RECHARGE_RETRY)
				Decision_State.Navigation_Mode=RECHARGE_AREA;
			Decision_State.Navigation_Wait=0;
		}
	}
	else
	{
		Decision_State.Behaviour_State=SENTRY_STOP;
		Decision_State.Chassis_State=NO_ROTATE;
		Decision_State.Navigation_Mode=NO_TARGET;
	}
	return;
}
