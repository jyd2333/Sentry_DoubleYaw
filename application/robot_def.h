/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * Copyright (c) 2023 by Alliance-EC, All Rights Reserved.
 *
 */
#pragma once // 可以用 pragma once 代替 #ifndef ROBOT_DEF_H（header guard）
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include <stdint.h>
#include "ins_task.h"
#include "master_process.h"

/* 开发板类型定义，烧录时注意不要弄错对应功能；修改定义后需要重新编译；只能存在一个定义 */
// #define ONE_BOARD // 单板控制整车
#define CHASSIS_BOARD // 底盘板
// #define GIMBAL_BOARD  // 云台板

//#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据


/* 机器人重要参数定义：注意根据不同机器人进行修改；浮点数以 .0 或 f 结尾，无符号数以 u 结尾 */
// 云台参数


#define YAW_CHASSIS_ALIGN_ECD     4302 // 云台和底盘对齐（指向相同方向）时的电机编码器值，机械改动后需重新标定
#define YAW_ECD_GREATER_THAN_4096 1    // ALIGN_ECD 值是否大于 4096：是为 1，否为 0；用于计算云台偏转角度
#define PITCH_HORIZON_ECD         4240 // 云台处于水平位置时的编码器值，机械改动后需重新标定
#define PITCH_POS_UP_LIMIT_ECD    4900 // 云台俯仰上限位的编码器值，机械改动后需重新标定
#define PITCH_POS_DOWN_LIMIT_ECD  3750 // 云台俯仰下限位的编码器值，机械改动后需重新标定

#define PITCH_HORIZON_POS           0.960f  // 云台水平时电机反馈位置
#define PITCH_UP_POS                0.5f  // 云台上限位时电机反馈位置（较机械限位略保守）
#define PITCH_DOWN_POS              1.4f  // 云台下限位时电机反馈位置（较机械限位略保守）
#define YAW_BIG_YAW_ALIGN_ECD       5426    // 大小 Yaw 对齐时小 Yaw 编码器值
#define YAW_LEFT_LIMIT_ECD          7000    //小Yaw左限位时电机反馈位置
#define YAW_RIGHT_LIMIT_ECD         4000    //小Yaw右限位时电机反馈位置
#define BIG_YAW_CHASSIS_ALIGN_POS   1.57983f //大Yaw与底盘对齐时单机反馈位置
#define LOADER_ANGLE_PER_BULLET     3240.0f   // 拨出一发弹丸时拨盘转动角度（36 * 90）
#define SEARCH_RANGE                100.0f  //小yaw搜索范围（注意不要超出机械限位）
#define SEARCH_YAW_SPEED            0.15f
#define SEARCH_PITCH_SPEED          0.0010f


#define STEERING_LF_ECD             4683
#define STEERING_RF_ECD             1325
#define STEERING_RB_ECD             2700
#define STEERING_LB_ECD             8169
#define STEERING_LF_ANGLE           STEERING_LF_ECD * ECD_ANGLE_COEF_DJI
#define STEERING_RF_ANGLE           STEERING_RF_ECD * ECD_ANGLE_COEF_DJI
#define STEERING_RB_ANGLE           STEERING_RB_ECD * ECD_ANGLE_COEF_DJI
#define STEERING_LB_ANGLE           STEERING_LB_ECD * ECD_ANGLE_COEF_DJI

#define PITCH_FEED_TYPE     1// 云台 PITCH 轴反馈值来源：编码器为 0，陀螺仪为 1
#define PITCH_INS_FEED_TYPE 1
 // 云台PITCH轴陀螺仪反馈:角度值为0,弧度制为1
#define PITCH_ECD_UP_ADD    0 // 云台抬升时编码器变化趋势，增为 1，减为 0（陀螺仪变化方向应一致）

// 发射参数
#define ONE_BULLET_DELTA_ANGLE 45    // 发射一发弹丸时拨盘转动距离（由机械设计图纸给出）
#define REDUCTION_RATIO_LOADER 36.0f // 拨盘电机减速比；英雄可按实际电机改为 3508 的 19.0f
#define NUM_PER_CIRCLE         8     // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE             344   // 320.5   // 纵向轴距(前进后退方向)
#define TRACK_WIDTH            344   // 320.5   // 横向轮距(左右平移方向)
#define CHASSIS_R              243.245f   //轮子到车体中心的距离
#define CENTER_GIMBAL_OFFSET_X 0     // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0     // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL           55.0f   // 轮子半径
#define REDUCTION_RATIO_WHEEL  15.76f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define SPEED_TO_DJI_MOTOR_APS (REDUCTION_RATIO_WHEEL * 360.0f / PERIMETER_WHEEL)

#define CHASSIS_SPEED          40000 // 键盘控制不限功率时底盘最大移动速度
#define CHASSIS_SPEED_MEASURE_WINDOW_SIZE 20U
#define CHASSIS_HIGH_SPEED                  3000.0f     //mm/s
#define CHASSIS_ACCELERATION_HIGH_LIMIT      50000.0f   // mm/(s*s)
#define CHASSIS_ACCELERATION_LOW_LIMIT      300000.0f   // mm/(s*s)
#define YAW_K                  0.0004f
#define PITCH_K                0.000004f

// 模拟小电脑负载 652.2
// 其他参数（尽量把所有参数集中到此文件）
#define BUZZER_SILENCE 0 // 蜂鸣器静音：1 为静音，0 为正常

#define IMU_DEF_PARAM_WARNING
// 编译warning,提醒开发者修改传感器参数
#ifndef IMU_DEF_PARAM_WARNING
#define IMU_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !IMU_DEF_PARAM_WARNING

// 陀螺仪校准数据，开启陀螺仪校准后可从 INS 中获取
#ifdef GIMBAL_BOARD
#define BMI088_PRE_CALI_GYRO_X_OFFSET 0.000715131406f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET 0.000697459152f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET -0.002f
#endif

#ifdef CHASSIS_BOARD
#define BMI088_PRE_CALI_GYRO_X_OFFSET -0.00242434512f
#define BMI088_PRE_CALI_GYRO_Y_OFFSET -0.000412355439f
#define BMI088_PRE_CALI_GYRO_Z_OFFSET -0.00428985665f
#endif
// 陀螺仪默认环境温度
#define BMI088_AMBIENT_TEMPERATURE 25.0f
// 设置陀螺仪数据相较于云台 yaw/pitch/roll 的方向
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, -1.0f, 0.0f},                 \
    {1.0f, 0.0f, 0.0f},                  \
    {0.0f, 0.0f, 1.0f }                  \

    

#define INS_YAW_ADDRESS_OFFSET   2 // 陀螺仪数据相较于云台 yaw 的方向
#define INS_PITCH_ADDRESS_OFFSET 1 // 陀螺仪数据相较于云台 pitch 的方向
#define INS_ROLL_ADDRESS_OFFSET  0 // 陀螺仪数据相较于云台 roll 的方向

// 检查是否出现主控板定义冲突，只允许一个开发板定义存在，否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体并取消字节对齐，下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续可考虑改为云台跟随底盘，而不是底盘追云台（通常云台惯量更小）
 *
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0,    // 电流零输出
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
    CHASSIS_REVERSE_ROTATE,    // 反方向小陀螺
} chassis_mode_e;

// 云台模式设置
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输出
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_SEARCH_MODE,
} gimbal_mode_e;
typedef enum {
    NUC_NORMAL = 0,
    NUC_CONTROL,
} NUC_mode_e;

// 发射模式设置
#define SHOOT_ONE_BULLET_HEAT      10
#define SHOOT_HEAT_RESERVE_BULLETS 3

typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum {
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef enum {
    LOAD_UNINIT = 0,     // 未初始化
    LOAD_REINIT,         // 预初始化
    LOAD_INFRARED_INIT,  // 对射式红外传感器初始化
} loader_state_e;

typedef struct {
    float vx;
    float vy;
    float vt;
    float angle_measure;
    float angle_speed;
    uint8_t reverse_flag;
    float rotate_range;
    float angle_target;
    float angle_ref;
    float angle_diff;
} steering_wheelset_t;

typedef struct {
    float real_vx;
    float real_vy;
    float real_wz;
    float vx_window[CHASSIS_SPEED_MEASURE_WINDOW_SIZE];
    float vy_window[CHASSIS_SPEED_MEASURE_WINDOW_SIZE];
    float wz_window[CHASSIS_SPEED_MEASURE_WINDOW_SIZE];
    float vx_sum;
    float vy_sum;
    float wz_sum;
    uint8_t window_index;
    uint8_t window_count;
} chassis_speed_measure_t;

/* ----------------CMD 应用发布的控制数据，应由 gimbal/chassis/shoot/UI 订阅---------------- */
/**
 * @brief 双板情况下：遥控器和 PC 在云台板，裁判系统在底盘板
 *
 */
// CMD 发布的底盘控制数据（由 chassis 订阅）
typedef struct
{
    // 控制部分
    uint16_t power_buffer;           // 60 焦耳缓冲能量
    float chassis_power;             // 底盘瞬时功率
    uint8_t level;                   // 机器人等级
    uint16_t power_limit;            // 底盘功率限制
    uint8_t SuperCap_flag_from_user; // 超电的标志位
    float vx;                        // 前进方向速度
    float vy;                        // 横移方向速度
    float wz;                        // 旋转速度
    float offset_angle;              // 底盘和归中位置的夹角
    float align_angle;
    float gimbal_error_angle;        // 云台当前位置与目标（归中）位置的夹角
    chassis_mode_e chassis_mode;
    NUC_mode_e control_type;
    uint8_t chassis_rotate_speed;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// CMD 发布的云台控制数据（由 gimbal 订阅）
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    gimbal_mode_e gimbal_mode;
    NUC_mode_e control_type;

} Gimbal_Ctrl_Cmd_s;

// CMD 发布的发射控制数据（由 shoot 订阅）
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    friction_mode_e friction_mode;
    uint8_t rest_heat;
    uint16_t shooter_heat_cooling_rate; // 枪口热量冷却
    uint16_t shooter_referee_heat;      // 17mm枪口热量
    uint16_t shooter_cooling_limit;     // 枪口热量上限
    float shoot_rate;                   // 连续发射射速（unit per s，发/秒）
    float bullet_speed;                 // 子弹速度
} Shoot_Ctrl_Cmd_s;

// cmd发布的UI数据,由UI订阅
typedef struct
{
    uint8_t ui_send_flag; // UI发送标志位
    chassis_mode_e chassis_mode;
    uint16_t chassis_attitude_angle; // 底盘姿态角
    friction_mode_e friction_mode;
    uint8_t rune_mode;
    uint8_t SuperCap_mode;           // 开关指示（未开启为 1）
    float SuperCap_voltage;          // 超电电压
    float Chassis_Ctrl_power;        // 底盘控制功率
    uint16_t Cap_absorb_power_limit; // 超电吸收功率
    float Chassis_voltage;           // 底盘电压
    uint16_t Chassis_power_limit;    // 底盘功率
    float Shooter_heat;              // 枪口热量
    uint16_t Heat_Limit;             // 热量上限
} UI_Cmd_s;

/* ----------------gimbal/shoot/chassis/UI 发布的反馈数据---------------- */
/**
 * @brief 由 cmd 订阅，其他应用也可按需获取
 *
 */

typedef struct
{
// #if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
//     // attitude_t chassis_imu_data;
// #endif
    // 后续增加底盘的真实速度
    float real_vx;
    float real_vy;
    float real_wz;
    uint8_t CapFlag_open_from_real;
    float cap_voltage;
    uint16_t capget_power_limit;
    float chassis_power_output;
    float chassis_voltage;
    float tilt_direction;
    float tilt_angle;
} Chassis_Upload_Data_s;

typedef struct
{
    INS_Instance *gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    uint16_t yaw_ecd;
    uint8_t gimbal_online;
    float pitch_motor_pos;
} Gimbal_Upload_Data_s;

typedef struct
{
    int shooter_heat_control; // 热量控制
    float shooter_local_heat; // 本地热量
} Shoot_Upload_Data_s;

typedef struct
{
    // code to go here
} UI_Upload_Data_s;

#pragma pack() // 开启字节对齐；结束前面的 #pragma pack(1)

#endif // !ROBOT_DEF_H

