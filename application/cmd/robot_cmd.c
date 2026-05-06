// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "omni_UI.h"
#include "robot_test.h"
#include "comm.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "referee_UI.h"
#include "referee_init.h"

#include "tool.h"
#include "super_cap.h"
#include "AHRS_MiddleWare.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#define RC_LOST (rc_data[TEMP].rc.switch_left == 0 && rc_data[TEMP].rc.switch_right == 0)

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#if PITCH_FEED_TYPE                                                  // Pitch电机反馈数据源为陀螺仪
#define PTICH_HORIZON_ANGLE 0                                        // PITCH水平时电机的角度
#if PITCH_ECD_UP_ADD
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#else
#define PITCH_LIMIT_ANGLE_UP   (((PITCH_POS_UP_LIMIT_ECD < PITCH_HORIZON_ECD) ? (PITCH_POS_UP_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_UP_LIMIT_ECD - 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI)       // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (((PITCH_POS_DOWN_LIMIT_ECD > PITCH_HORIZON_ECD) ? (PITCH_POS_DOWN_LIMIT_ECD - PITCH_HORIZON_ECD) : (PITCH_POS_DOWN_LIMIT_ECD + 8192 - PITCH_HORIZON_ECD)) * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif
#else                                                                   // PITCH电机反馈数据源为编码器
#define PTICH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // PITCH水平时电机的角度,0-360
#define PITCH_LIMIT_ANGLE_UP   (PITCH_POS_MAX_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最大角度 0-360
#define PITCH_LIMIT_ANGLE_DOWN (PITCH_POS_MIN_ECD * ECD_ANGLE_COEF_DJI) // 云台竖直方向最小角度 0-360
#endif

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者

extern NUC_cmd_t NUC_cmd;
extern comm_cmd_t comm_cmd_data;
extern comm_upload_t comm_upload_data;
Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

// static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回
static WFLY_ctrl_t *WFLY_data;


HostInstance *host_instance; // 上位机接口

// 这里的四元数以wxyz的顺序
static uint8_t vision_recv_data[9];  // 从视觉上位机接收的数据-绝对角度，第9个字节作为识别到目标的标志位

static Publisher_t *gimbal_cmd_pub  ;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

// static Publisher_t *ui_cmd_pub;        // UI控制消息发布者
// static Subscriber_t *ui_feed_sub;      // UI反馈信息订阅者
// static UI_Cmd_s ui_cmd_send;           // 传递给UI的控制信息
// static UI_Upload_Data_s ui_fetch_data; // 从UI获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static referee_info_t *referee_data; // 用于获取裁判系统的数据

uint8_t UI_SendFlag = 1; // UI发送标志位

uint8_t auto_rune; // 自瞄打符标志位

// float rec_yaw, rec_pitch;
uint8_t i=0;
uint8_t SuperCap_flag_from_user = 0; // 超电标志位
extern DaemonInstance *rc_daemon_instance;

static float yaw_l_limit, yaw_r_limit;

void HOST_RECV_CALLBACK()
{
    memcpy(vision_recv_data, host_instance->comm_instance, host_instance->RECV_SIZE);
    // vision_recv_data[8] = 1;
}
void RobotCMDInit()
{
    WFLY_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#ifdef GIMBAL_BOARD
    HostInstanceConf host_conf = {
        .callback  = HOST_RECV_CALLBACK,
        .comm_mode = HOST_VCP,
        .RECV_SIZE = 8,
    };
    host_instance = HostInit(&host_conf); // 视觉通信串口
    GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#endif
    referee_data = RefereeHardwareInit(&huart6); // 裁判系统初始化,会同时初始化UI
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    i=1;
    
    // ui_cmd_pub  = PubRegister("ui_cmd", sizeof(UI_Cmd_s));
    // ui_feed_sub = SubRegister("ui_feed", sizeof(UI_Upload_Data_s));

// #ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
// #endif // ONE_BOARD

    CommInit();
#if PITCH_FEED_TYPE
    gimbal_cmd_send.pitch = 0;
#else
    gimbal_cmd_send.pitch = PTICH_HORIZON_ANGLE;
#endif

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}
uint16_t real_robot_id = 0;
/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    if(referee_data->GameRobotStatus.robot_id == 7) real_robot_id = 7;//2025裁判系统发送未定义随机ID
    if(referee_data->GameRobotStatus.robot_id == 107) real_robot_id = 107;//哨兵特判，其他兵种请修改
    referee_data->referee_id.Robot_Color       = real_robot_id > 10 ? Robot_Blue : Robot_Red;
    referee_data->referee_id.Cilent_ID         = 0x0100 + real_robot_id; // 计算客户端ID
    referee_data->referee_id.Robot_ID          = real_robot_id;          // 计算机器人ID

    referee_data->referee_id.Receiver_Robot_ID = 0;
}

float yaw_control;   // 遥控器YAW自由度输入值
float pitch_control; // 遥控器PITCH自由度输入值
uint8_t check_count=0;
/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 * @todo  将单圈角度修改为-180~180
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    // static float gimbal_yaw_current_angle;                                                // 云台yaw轴当前角度
    // static float gimbal_yaw_set_angle;                                                    // 云台yaw轴目标角度
    angle                               = gimbal_fetch_data.yaw_motor_single_round_angle - (BIG_YAW_CHASSIS_ALIGN_POS * RAD_2_DEGREE - 90.0f); // 从云台获取的当前yaw电机单圈角度
    // gimbal_yaw_current_angle            = gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET];
    // gimbal_yaw_set_angle                = yaw_control;
    // chassis_cmd_send.gimbal_error_angle = gimbal_yaw_set_angle - gimbal_yaw_current_angle; // 云台误差角

    if(angle < 0) angle += 360;
    chassis_cmd_send.offset_angle = angle;
    // if(gimbal_fetch_data.base_yaw_tilt->alpha > 0.07) angle -= gimbal_fetch_data.base_yaw_tilt->direction;
    for(check_count = 0; check_count < 10; check_count++)//防止阻塞
    {
        if(angle <= 45 && angle >= -45) break;
        if(angle > 45) angle -= 90;
        if(angle < -45) angle += 90;
    }
    chassis_cmd_send.align_angle = angle;
}

/**
 * @brief 对Pitch轴角度变化进行动态限位
 *
 */
static void PitchAngleLimit()
{
    float pitch_angle;
    float pitch_pos;
    float limit_min;
    float limit_max;

    if (gimbal_fetch_data.gimbal_imu_data == NULL) {
        return;
    }

    pitch_angle = gimbal_fetch_data.gimbal_imu_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    pitch_pos   = gimbal_fetch_data.pitch_motor_pos;
    limit_max   = pitch_angle + pitch_pos - PITCH_UP_POS;
    limit_min   = pitch_angle + pitch_pos - PITCH_DOWN_POS;

    if (pitch_control > limit_max)
        pitch_control = limit_max;
    if (pitch_control < limit_min)
        pitch_control = limit_min;
}

/**
 * @brief 云台Yaw轴反馈值改单圈角度后过圈处理
 *
 */
static void YawControlFollowAngle(float yaw_feedback)
{
    yaw_control = yaw_feedback + theta_format(yaw_control - yaw_feedback);
}

static void YawControlProcess()
{
    if (gimbal_fetch_data.gimbal_imu_data == NULL) {
        return;
    }

    YawControlFollowAngle(gimbal_fetch_data.gimbal_imu_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET]);
}

static void HeatControl()
{
    uint16_t heat_limit = referee_data->GameRobotStatus.shooter_barrel_heat_limit;
    float reserve_heat = (float)(SHOOT_ONE_BULLET_HEAT * SHOOT_HEAT_RESERVE_BULLETS);

    if (shoot_cmd_send.friction_mode == FRICTION_OFF) {
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    if (heat_limit == 0) {
        shoot_cmd_send.load_mode = LOAD_STOP;
        return;
    }

    if ((float)heat_limit - shoot_fetch_data.shooter_local_heat <= reserve_heat) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

static void GetSearchRange()
{
    int32_t yaw_ecd_delta;
    float small_yaw_offset;

    if (gimbal_fetch_data.gimbal_imu_data == NULL) {
        return;
    }

    yaw_ecd_delta = (int32_t)gimbal_fetch_data.yaw_ecd - YAW_BIG_YAW_ALIGN_ECD;
    if (yaw_ecd_delta > 4096) {
        yaw_ecd_delta -= 8192;
    } else if (yaw_ecd_delta < -4096) {
        yaw_ecd_delta += 8192;
    }

    small_yaw_offset = (float)yaw_ecd_delta * ECD_ANGLE_COEF_DJI;
    yaw_l_limit = gimbal_fetch_data.gimbal_imu_data->output.Yaw_total_angle_deg
                - small_yaw_offset
                - SEARCH_RANGE / 2.0f;
    yaw_r_limit = yaw_l_limit + SEARCH_RANGE;
}

static void Search()
{
    static int8_t pitch_search_flag = 1;
    static int8_t yaw_search_flag = 1;

    if (gimbal_fetch_data.gimbal_imu_data == NULL) {
        return;
    }

    YawControlFollowAngle(gimbal_fetch_data.gimbal_imu_data->output.Yaw_total_angle_deg);
    yaw_control += -YAW_K * (float)WFLY_data[TEMP].rocker_l_ + (float)yaw_search_flag * SEARCH_YAW_SPEED;
    pitch_control += (float)pitch_search_flag * SEARCH_PITCH_SPEED + PITCH_K * (float)WFLY_data[TEMP].rocker_l1;

    if (yaw_control > yaw_r_limit) {
        yaw_control = yaw_r_limit;
        yaw_search_flag = -1;
    }
    if (yaw_control < yaw_l_limit) {
        yaw_control = yaw_l_limit;
        yaw_search_flag = 1;
    }
    if (pitch_control > 0.15f) {
        pitch_search_flag = -1;
    }
    if (pitch_control < -0.1f) {
        pitch_search_flag = 1;
    }
}

// 底盘模式
static uint8_t rc_mode[5];
#define CHASSIS_FREE     0
#define CHASSIS_ROTATION 1
#define CHASSIS_FOLLOW   2
#define SHOOT_FRICTION   3
#define SHOOT_LOAD       4

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    gimbal_cmd_send.gimbal_mode   = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.friction_mode  = FRICTION_OFF;
    shoot_cmd_send.load_mode      = LOAD_STOP;
    shoot_cmd_send.shoot_mode     = SHOOT_OFF;
  //  SuperCap_flag_from_user       = SUPER_USER_CLOSE;
    memset(rc_mode, 1, sizeof(uint8_t));
    memset(rc_mode + 1, 0, sizeof(uint8_t) * 4);
    LOGERROR("[CMD] emergency stop!");
}
float speed_k=1;
float32_t nuc_yaw=-0.08;//0.003;
float32_t nuc_pitch=0.3;
uint64_t last_time_stamp;
int32_t shoot_wait=0;
// uint16_t shoot_delay=0,fire_flag=0;
uint16_t vision_wait=0;
// int8_t pitch_search_flag=1;//pitch上升下降
// int8_t yaw_search_flag=1;
extern INS_Instance *INS;
int16_t yaw_test_count = 1000, yaw_test_state = 1, yaw_test_range = 10;
int16_t pitch_test_count = 1000,pitch_test_state = 1;
float pitch_test_range = 0.05;
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    shoot_cmd_send.shoot_mode   = SHOOT_ON; // 发射机构常开
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.shoot_rate   = 16;   // 射频默认30Hz
    chassis_cmd_send.control_type = NUC_NORMAL;

    // if (rc_data[TEMP].rc.dial > 400) {
    //     SuperCap_flag_from_user = SUPER_USER_OPEN;
    // } else {
    //     SuperCap_flag_from_user = SUPER_USER_CLOSE; // 默认关闭超电
    // }

    // 使用相对角度控制
    //memcpy(&rec_yaw, vision_recv_data, sizeof(float));
    //memcpy(&rec_pitch, vision_recv_data + 4, sizeof(float));

    

    //修改部分
    // if(rc_data[TEMP].rc.switch_left == RC_SW_DOWN&&rc_data[TEMP].rc.switch_right == RC_SW_MID) chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    if(WFLY_data[TEMP].state_SB == SWITCH_DOWN && WFLY_data[TEMP].state_SC == SWITCH_DOWN && WFLY_data[TEMP].state_SD == SWITCH_UP)
    {    
        // NUC_cmd.delay--;
        // if(NUC_cmd.delay<=0)
        // {
        //     // NUC_offline();
        //     NUC_cmd.delay=1000;
        // }
        // chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        // else
        // {
        chassis_cmd_send.control_type = NUC_CONTROL;
        chassis_cmd_send.vx = 1000.0f * NUC_cmd.vx; // 水平方向
        chassis_cmd_send.vy = 1000.0f * NUC_cmd.vy; // 竖直方向
        // chassis_cmd_send.chassis_mode = NUC_cmd.rotate
        // chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        // chassis_cmd_send.chassis_rotate_speed = 200;
        if(NUC_cmd.rotateMode == 0)
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;//CHASSIS_ROTATE;
            chassis_cmd_send.chassis_rotate_speed = 0;
        }
        else
        {
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            chassis_cmd_send.chassis_rotate_speed = NUC_cmd.rotateMode;
        }
        if(NUC_cmd.time_stamp != last_time_stamp)
        {
            if(NUC_cmd.shoot)
            {
                gimbal_cmd_send.control_type = NUC_CONTROL;
                yaw_control = NUC_cmd.yaw ;
                pitch_control = NUC_cmd.pitch ;
                if(NUC_cmd.shoot==2&&shoot_cmd_send.friction_mode== FRICTION_ON) 
                {
                    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                    shoot_wait = 100;
                }
                else
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                }
            }
            else
                gimbal_cmd_send.control_type = NUC_NORMAL;
        }
        if(shoot_wait>0) shoot_wait--;
        if(shoot_wait<=0) shoot_cmd_send.load_mode = LOAD_STOP;
        last_time_stamp = NUC_cmd.time_stamp;
        yaw_control-= YAW_K * (float)WFLY_data[TEMP].rocker_l_;
        pitch_control+=PITCH_K * (float)WFLY_data[TEMP].rocker_l1;
        if(NUC_cmd.shoot == 0 && gimbal_fetch_data.gimbal_online)
        {
            gimbal_cmd_send.gimbal_mode = GIMBAL_SEARCH_MODE;
            Search();
            // yaw_control += -YAW_K * (float)WFLY_data[TEMP].rocker_l_ + 1  * 0.1;
            // pitch_control += pitch_search_flag * 0.0015f + PITCH_K * (float)WFLY_data[TEMP].rocker_l1;
            // if(pitch_control > -0.15f)
            //     pitch_search_flag=-1;
            // if(pitch_control < -0.4f)
            //     pitch_search_flag=1;
        }
    }


    if(WFLY_data[TEMP].state_SD == SWITCH_DOWN)
    {
        switch(WFLY_data[TEMP].state_SB)
        {
            case SWITCH_DOWN:
                shoot_cmd_send.friction_mode = FRICTION_OFF;
                shoot_cmd_send.load_mode = LOAD_STOP;
                break;
            case SWITCH_MIDDLE:
                shoot_cmd_send.friction_mode = FRICTION_ON;
                shoot_cmd_send.load_mode = LOAD_STOP;
                break;
            case SWITCH_UP:
                shoot_cmd_send.friction_mode = FRICTION_ON;
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                break;
            default:
                break;
        }
        switch(WFLY_data[TEMP].state_SC)
        {
            case SWITCH_DOWN:
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                break;
            case SWITCH_MIDDLE:
                chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;//todo
                break;
            case SWITCH_UP:
                chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
                chassis_cmd_send.chassis_rotate_speed = 255;
                break;
            default:
                break;
        }
        gimbal_cmd_send.control_type = NUC_NORMAL;
        chassis_cmd_send.vx = 5.0f * (float)WFLY_data[TEMP].rocker_r_; // 水平方向
        chassis_cmd_send.vy = -5.0f * (float)WFLY_data[TEMP].rocker_r1; // 竖直方向
        yaw_control-= YAW_K * (float)WFLY_data[TEMP].rocker_l_;
        pitch_control-=PITCH_K * (float)WFLY_data[TEMP].rocker_l1;
    }
    if(WFLY_data[TEMP].state_SD == SWITCH_UP && !(WFLY_data[TEMP].state_SB == SWITCH_DOWN && WFLY_data[TEMP].state_SC == SWITCH_DOWN))
    {
        switch(WFLY_data[TEMP].state_SC)
        {
            case SWITCH_DOWN://未定义：默认状态保护
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                shoot_cmd_send.friction_mode = FRICTION_OFF;
                shoot_cmd_send.load_mode = LOAD_STOP;
                gimbal_cmd_send.control_type = NUC_NORMAL;
                chassis_cmd_send.vx = 5.0f * (float)WFLY_data[TEMP].rocker_r_; // 水平方向
                chassis_cmd_send.vy = -5.0f * (float)WFLY_data[TEMP].rocker_r1; // 竖直方向
                yaw_control-= YAW_K * (float)WFLY_data[TEMP].rocker_l_;
                pitch_control-=PITCH_K * (float)WFLY_data[TEMP].rocker_l1;
                break;
            case SWITCH_MIDDLE://Guide
                shoot_cmd_send.friction_mode = FRICTION_OFF;
                shoot_cmd_send.load_mode = LOAD_STOP;
                gimbal_cmd_send.control_type = NUC_NORMAL;
                chassis_cmd_send.control_type = NUC_CONTROL;
                chassis_cmd_send.vx = 1000.0f * NUC_cmd.vx; // 水平方向
                chassis_cmd_send.vy = 1000.0f * NUC_cmd.vy; // 竖直方向
                if(NUC_cmd.rotateMode == 0)
                {
                    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;//CHASSIS_ROTATE;
                    chassis_cmd_send.chassis_rotate_speed = 0;
                }
                else
                {
                    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
                    chassis_cmd_send.chassis_rotate_speed = NUC_cmd.rotateMode;

                // chassis_cmd_send.chassis_mode = NUC_cmd.rotateMode;
                }
                yaw_control += -YAW_K * (float)WFLY_data[TEMP].rocker_l_;//+(float) NUC_cmd.odomYaw  * 0.001;
                pitch_control-=PITCH_K * (float)WFLY_data[TEMP].rocker_l1;
                break;
            case SWITCH_UP://Vision
                chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
                // chassis_cmd_send.chassis_rotate_speed = 255;
                switch(WFLY_data[TEMP].state_SB)
                {
                    case SWITCH_DOWN:
                        shoot_cmd_send.friction_mode = FRICTION_OFF;
                        shoot_cmd_send.load_mode = LOAD_STOP;
                        break;
                    case SWITCH_MIDDLE:
                        shoot_cmd_send.friction_mode = FRICTION_ON;
                        shoot_cmd_send.load_mode = LOAD_STOP;
                        break;
                    case SWITCH_UP:
                        shoot_cmd_send.friction_mode = FRICTION_ON;
                        if(NUC_cmd.time_stamp != last_time_stamp)
                        {
                            if(NUC_cmd.shoot == 2) 
                            {
                                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                                shoot_wait = 100;
                            }
                            else
                                shoot_cmd_send.load_mode = LOAD_STOP;
                        }
                        break;
                    default:
                        break;
                }
                chassis_cmd_send.vx = 5.0f * (float)WFLY_data[TEMP].rocker_r_; // 水平方向
                chassis_cmd_send.vy = -5.0f * (float)WFLY_data[TEMP].rocker_r1; // 竖直方向
                if(NUC_cmd.time_stamp != last_time_stamp)
                {
                    if(NUC_cmd.shoot)
                    {
                        gimbal_cmd_send.control_type = NUC_CONTROL;
                        yaw_control = NUC_cmd.yaw ;
                        pitch_control = NUC_cmd.pitch ;
                    }
                    else
                        gimbal_cmd_send.control_type = NUC_NORMAL;
                }
                if(shoot_wait>0) shoot_wait--;
                if(shoot_wait<=0)
                {
                    shoot_cmd_send.load_mode = LOAD_STOP;
                    shoot_wait = 0;
                }
                last_time_stamp = NUC_cmd.time_stamp;
                yaw_control-= YAW_K * (float)WFLY_data[TEMP].rocker_l_;
                pitch_control-=PITCH_K * (float)WFLY_data[TEMP].rocker_l1;
                break;
            default:
                break;
        }
    }
    rc_daemon_instance->temp_count--;
    if(rc_daemon_instance->temp_count<=0)
    {
        RCLostCallback(NULL);
    }
    // 云台软件限位
    PitchAngleLimit(); // PITCH限位
    // 云台参数
    YawControlProcess();
    HeatControl();
    // if(yaw_control > 100) yaw_control = 100;
    // if(yaw_control < -100) yaw_control = -100;
    // yaw_test_count--;
    // if(yaw_test_count <= 0)
    // {
    //     yaw_test_count = 1000;
    //     yaw_control += yaw_test_state * yaw_test_range;
    //     yaw_test_state *= -1;
    // }
    // pitch_test_count--;
    // if(pitch_test_count <= 0)
    // {
    //     pitch_test_count = 1000;
    //     pitch_control += pitch_test_state * pitch_test_range;
    //     pitch_test_state *= -1;
    // }
    gimbal_cmd_send.yaw   = yaw_control;
    gimbal_cmd_send.pitch = pitch_control;


}

ramp_t fb_ramp;
ramp_t lr_ramp;
ramp_t slow_ramp;

extern referee_info_t *referee_data_for_ui;

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据

    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    // SubGetMessage(ui_feed_sub, &ui_fetch_data);
    DeterminRobotID();
#ifdef GIMBAL_BOARD
    GetSearchRange();
    // 根据遥控器SA判断是否急停
    if (WFLY_data[TEMP].state_SA == SWITCH_DOWN) {
        EmergencyHandler(); // 调试/疯车时急停
    } else {
        RemoteControlSet();
    }
    NUC_Send_Data();
    CommIsOnline();
#endif
#ifdef CHASSIS_BOARD
// 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    if(CommIsOnline())
    {
        chassis_cmd_send.chassis_mode           = comm_cmd_data.chassis_mode;
        chassis_cmd_send.vx                     = comm_cmd_data.vx;
        chassis_cmd_send.vy                     = comm_cmd_data.vy;
        chassis_cmd_send.chassis_rotate_speed   = comm_cmd_data.chassis_rotate_speed;
        gimbal_cmd_send.gimbal_mode             = comm_cmd_data.gimbal_mode;
    }
    else
    {
        EmergencyHandler(); //通信离线急停
    }
#endif
    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
    // chassis
    // memcpy(&chassis_cmd_send.chassis_power, &referee_data->PowerHeatData.chassis_power, sizeof(float));
    memcpy(&chassis_cmd_send.power_buffer, &referee_data->PowerHeatData.chassis_power_buffer, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.level, &referee_data->GameRobotStatus.robot_level, sizeof(uint8_t));
    memcpy(&chassis_cmd_send.power_limit, &referee_data->GameRobotStatus.chassis_power_limit, sizeof(uint16_t));
    memcpy(&chassis_cmd_send.SuperCap_flag_from_user, &SuperCap_flag_from_user, sizeof(uint8_t));
    // memcpy

    // shoot
    memcpy(&shoot_cmd_send.shooter_heat_cooling_rate, &referee_data->GameRobotStatus.shooter_barrel_cooling_value, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_referee_heat, &referee_data->PowerHeatData.shooter_17mm_barrel_heat, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.shooter_cooling_limit, &referee_data->GameRobotStatus.shooter_barrel_heat_limit, sizeof(uint16_t));
    memcpy(&shoot_cmd_send.bullet_speed, &referee_data->ShootData.bullet_speed, sizeof(float));

    // UI
    // memcpy(referee_data_for_ui, referee_data, sizeof(referee_info_t));
    // referee_data_for_ui = referee_data;
    // memcpy(&ui_cmd_send.ui_send_flag, &UI_SendFlag, sizeof(uint8_t));
    // memcpy(&ui_cmd_send.chassis_mode, &chassis_cmd_send.chassis_mode, sizeof(chassis_mode_e));
    // memcpy(&ui_cmd_send.chassis_attitude_angle, &gimbal_fetch_data.yaw_motor_single_round_angle, sizeof(uint16_t));
    // memcpy(&ui_cmd_send.friction_mode, &shoot_cmd_send.friction_mode, sizeof(friction_mode_e));
    // memcpy(&ui_cmd_send.rune_mode, &auto_rune, sizeof(uint8_t));
    // memcpy(&ui_cmd_send.SuperCap_mode, &chassis_fetch_data.CapFlag_open_from_real, sizeof(uint8_t));
    // memcpy(&ui_cmd_send.SuperCap_voltage, &chassis_fetch_data.cap_voltage, sizeof(float));
    // memcpy(&ui_cmd_send.Chassis_Ctrl_power, &chassis_fetch_data.chassis_power_output, sizeof(float));
    // memcpy(&ui_cmd_send.Cap_absorb_power_limit, &chassis_fetch_data.capget_power_limit, sizeof(uint16_t));
    // memcpy(&ui_cmd_send.Chassis_power_limit, &referee_data->GameRobotState.chassis_power_limit, sizeof(uint16_t));
    // memcpy(&ui_cmd_send.Shooter_heat, &shoot_fetch_data.shooter_local_heat, sizeof(float));
    // memcpy(&ui_cmd_send.Heat_Limit, &referee_data->GameRobotState.shooter_id1_17mm_cooling_limit, sizeof(uint16_t));


    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    // PubPushMessage(ui_cmd_pub, (void *)&ui_cmd_send);
    CommSend();
}
