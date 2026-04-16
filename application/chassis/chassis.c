/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用，负责接收 robot_cmd 的控制命令并根据命令进行运动学解算，得到输出
 *        注意底盘采用右手坐标系：平面视图下，底盘前方为 x 正方向，右侧为 y 正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_init.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "power_calc.h"
#include "tool.h"

/* 根据 robot_def.h 中的宏自动计算参数 */
#define HALF_WHEEL_BASE  (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)    // 半轮距
#define PERIMETER_WHEEL  (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define LF               0
#define RF               1
#define RB               2
#define LB               3

/* 底盘应用包含的模块和信息存储，底盘为单例模式，因此不需要单独结构体 */
static INS_Instance *chassis_IMU_data; // 底盘IMU数据
static Publisher_t *chassis_pub;                    // 用于发布底盘数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘控制命令
// #endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

SuperCapInstance *cap;                                              // 超级电容
// 驱动与转向电机实例
DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
DJIMotorInstance *steering_lf, *steering_rf, *steering_rb, *steering_lb;
steering_wheelset_t wheelset_lf, wheelset_rf, wheelset_rb, wheelset_lb;
// 为了方便调试加入的量
static uint8_t center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X; // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
static uint8_t center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y; // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0

// 跟随模式底盘的pid
// 目前未严格约定单位，后续如有需要再统一规范
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 50,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0.3, // 0.0,  // 0.07,  // 0
    .DeadBand      =  0.75,  //跟随模式设置了死区，防止抖动
    .IntegralLimit = 3000,
    .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .MaxOut        = 10000,
    

};

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中间变量，设为静态以避免参数传递开销 */
static float chassis_vx, chassis_vy, chassis_vw; // 将云台坐标系速度投影到底盘坐标系
static float vt_lf, vt_rf, vt_lb, vt_rb;         // 底盘速度解算后的临时输出，待进行限幅
static ramp_t rotate_ramp;
void ChassisInit()
{
#ifdef CHASSIS_BOARD
    BMI088_Init_Config_s config = {
        .acc_int_config  = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_4},
        .gyro_int_config = {.GPIOx = GPIOC, .GPIO_Pin = GPIO_PIN_5},
        .heat_pid_config = {
            .Kp            = 0.32f,
            .Ki            = 0.0004f,
            .Kd            = 0,
            .Improve       = PID_IMPROVE_NONE,
            .IntegralLimit = 0.90f,
            .MaxOut        = 0.95f,
        },
        .heat_pwm_config = {
            .htim      = &htim10,
            .channel   = TIM_CHANNEL_1,
            .dutyratio = 0,
            .period    = 5000 - 1,
        },
        .spi_acc_config = {
            .GPIOx      = GPIOA,
            .cs_pin     = GPIO_PIN_4,
            .spi_handle = &hspi1,
        },
        .spi_gyro_config = {
            .GPIOx      = GPIOB,
            .cs_pin     = GPIO_PIN_0,
            .spi_handle = &hspi1,
        },
        .cali_mode = BMI088_LOAD_PRE_CALI_MODE,
        .work_mode = BMI088_BLOCK_PERIODIC_MODE,

    };
    chassis_IMU_data = INS_Init(BMI088Register(&config)); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // 四个轮子的参数一致，只需修改 tx_id 和反转标志位
    Motor_Init_Config_s chassis_motor_config = {
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 1.0, // 4.5
                .Ki            = 0,   // 0
                .Kd            = 0,   // 0
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
            }
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s steering_config = {
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 150,//12, // 0.24, // 0.31, // 0.45
                .Ki            = 0,
                .Kd            = 0,//0.02,//0.01,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement,
                .IntegralLimit = 20, 
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp            = 1,//6000,//10000, //11000,
                .Ki            = 0,    // 0
                .Kd            = 0,//5, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 3000,
                .MaxOut        = 3000 // 20000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020
    };
    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 1;
    steering_lf                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 2;
    steering_rf                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 3;
    steering_rb                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 4;
    steering_lb                                                             = DJIMotorInit(&steering_config);

    //  @todo: 当前未统一电机正反方向，仍需手动处理 reference 正负号，待电机模块支持后修复
    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    // motor_lf                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    // motor_rf                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    // motor_rb                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    // motor_lb                                                                = DJIMotorInit(&chassis_motor_config);

    // SuperCap_Init_Config_s cap_conf = {
    //     .can_config = {
    //         .can_handle = &hcan1,
    //         .tx_id      = 0X427, // 超级电容默认接收id
    //         .rx_id      = 0x300, // 超级电容默认发送 ID，注意 tx/rx 对设备视角是相反的
    //     }};
    // cap = SuperCapInit(&cap_conf); // 超级电容初始化
    ramp_init(&rotate_ramp, 1000);
#endif
    // 发布订阅初始化；如果为双板则需要 CAN Comm 传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

#define LF_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE - center_gimbal_offset_y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - center_gimbal_offset_x + HALF_WHEEL_BASE + center_gimbal_offset_y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机输出（正运动学解算）
 *        通过宏预替换减小开销，具体推导可参考机械/运动学文档
 */
static void MecanumCalculate()
{
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}

static void SteeringCalculate(void)
{
    wheelset_lf.vx = chassis_vx + chassis_cmd_recv.wz * 0.707f;
    wheelset_lf.vy = chassis_vy - chassis_cmd_recv.wz * 0.707f;
    wheelset_lf.angle_measure   = ((float)steering_lf->measure.ecd - STEERING_LF_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_lf.angle_speed     = atan2f(wheelset_lf.vy, wheelset_lf.vx);
    wheelset_lf.rotate_range    = wheelset_lf.angle_speed * RAD_2_DEGREE - wheelset_lf.angle_measure;
    if (wheelset_lf.rotate_range > 180.0f)
        wheelset_lf.rotate_range -= 360.0f;
    if (wheelset_lf.rotate_range < -180.0f)
        wheelset_lf.rotate_range += 360.0f;
    wheelset_lf.reverse_flag = 0;
    if (fabsf(wheelset_lf.rotate_range) > 90.0f) {
        wheelset_lf.reverse_flag = 1;
        if (wheelset_lf.rotate_range > 0.0f) {
            wheelset_lf.angle_target = wheelset_lf.angle_measure + wheelset_lf.rotate_range - 180.0f;
            wheelset_lf.angle_ref = steering_lf->measure.total_angle + wheelset_lf.rotate_range - 180.0f;
        } else {
            wheelset_lf.angle_target = wheelset_lf.angle_measure + wheelset_lf.rotate_range + 180.0f;
            wheelset_lf.angle_ref = steering_lf->measure.total_angle + wheelset_lf.rotate_range + 180.0f;
        }
    } else {
        wheelset_lf.angle_target = wheelset_lf.angle_measure + wheelset_lf.rotate_range;
        wheelset_lf.angle_ref = steering_lf->measure.total_angle + wheelset_lf.rotate_range;
    }
    wheelset_lf.angle_diff      = RAD_2_DEGREE * wheelset_lf.angle_speed - wheelset_lf.angle_measure;
    wheelset_lf.vt              = arm_cos_f32(DEGREE_2_RAD * wheelset_lf.angle_diff) * sqrtf(wheelset_lf.vx * wheelset_lf.vx + wheelset_lf.vy * wheelset_lf.vy);
    
    wheelset_rf.vx = chassis_vx + chassis_cmd_recv.wz * 0.707f;
    wheelset_rf.vy = chassis_vy + chassis_cmd_recv.wz * 0.707f;
    wheelset_rf.angle_measure   = ((float)steering_rf->measure.ecd - STEERING_RF_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_rf.angle_speed     = atan2f(wheelset_rf.vy, wheelset_rf.vx);
    wheelset_rf.rotate_range    = wheelset_rf.angle_speed * RAD_2_DEGREE - wheelset_rf.angle_measure;
    if (wheelset_rf.rotate_range > 180.0f)
        wheelset_rf.rotate_range -= 360.0f;
    if (wheelset_rf.rotate_range < -180.0f)
        wheelset_rf.rotate_range += 360.0f;
    wheelset_rf.reverse_flag = 0;
    if (fabsf(wheelset_rf.rotate_range) > 90.0f) {
        wheelset_rf.reverse_flag = 1;
        if (wheelset_rf.rotate_range > 0.0f) {
            wheelset_rf.angle_target = wheelset_rf.angle_measure + wheelset_rf.rotate_range - 180.0f;
            wheelset_rf.angle_ref = steering_rf->measure.total_angle + wheelset_rf.rotate_range - 180.0f;
        } else {
            wheelset_rf.angle_target = wheelset_rf.angle_measure + wheelset_rf.rotate_range + 180.0f;
            wheelset_rf.angle_ref = steering_rf->measure.total_angle + wheelset_rf.rotate_range + 180.0f;
        }
    } else {
        wheelset_rf.angle_target = wheelset_rf.angle_measure + wheelset_rf.rotate_range;
        wheelset_rf.angle_ref = steering_rf->measure.total_angle + wheelset_rf.rotate_range;
    }
    wheelset_rf.angle_diff      = RAD_2_DEGREE * wheelset_rf.angle_speed - wheelset_rf.angle_measure;
    wheelset_rf.vt              = arm_cos_f32(DEGREE_2_RAD * wheelset_rf.angle_diff) * sqrtf(wheelset_rf.vx * wheelset_rf.vx + wheelset_rf.vy * wheelset_rf.vy);

    wheelset_rb.vx = chassis_vx - chassis_cmd_recv.wz * 0.707f;
    wheelset_rb.vy = chassis_vy + chassis_cmd_recv.wz * 0.707f;
    wheelset_rb.angle_measure   = ((float)steering_rb->measure.ecd - STEERING_RB_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_rb.angle_speed     = atan2f(wheelset_rb.vy, wheelset_rb.vx);
    wheelset_rb.rotate_range    = wheelset_rb.angle_speed * RAD_2_DEGREE - wheelset_rb.angle_measure;
    if (wheelset_rb.rotate_range > 180.0f)
        wheelset_rb.rotate_range -= 360.0f;
    if (wheelset_rb.rotate_range < -180.0f)
        wheelset_rb.rotate_range += 360.0f;
    wheelset_rb.reverse_flag = 0;
    if (fabsf(wheelset_rb.rotate_range) > 90.0f) {
        wheelset_rb.reverse_flag = 1;
        if (wheelset_rb.rotate_range > 0.0f) {
            wheelset_rb.angle_target = wheelset_rb.angle_measure + wheelset_rb.rotate_range - 180.0f;
            wheelset_rb.angle_ref = steering_rb->measure.total_angle + wheelset_rb.rotate_range - 180.0f;
        } else {
            wheelset_rb.angle_target = wheelset_rb.angle_measure + wheelset_rb.rotate_range + 180.0f;
            wheelset_rb.angle_ref = steering_rb->measure.total_angle + wheelset_rb.rotate_range + 180.0f;
        }
    } else {
        wheelset_rb.angle_target = wheelset_rb.angle_measure + wheelset_rb.rotate_range;
        wheelset_rb.angle_ref = steering_rb->measure.total_angle + wheelset_rb.rotate_range;
    }
    wheelset_rb.angle_diff      = RAD_2_DEGREE * wheelset_rb.angle_speed - wheelset_rb.angle_measure;
    wheelset_rb.vt              = arm_cos_f32(DEGREE_2_RAD * wheelset_rb.angle_diff) * sqrtf(wheelset_rb.vx * wheelset_rb.vx + wheelset_rb.vy * wheelset_rb.vy);

    wheelset_lb.vx = chassis_vx - chassis_cmd_recv.wz * 0.707f;
    wheelset_lb.vy = chassis_vy - chassis_cmd_recv.wz * 0.707f;
    wheelset_lb.angle_measure   = ((float)steering_lb->measure.ecd - STEERING_LB_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_lb.angle_speed     = atan2f(wheelset_lb.vy, wheelset_lb.vx);
    wheelset_lb.rotate_range    = wheelset_lb.angle_speed * RAD_2_DEGREE - wheelset_lb.angle_measure;
    if (wheelset_lb.rotate_range > 180.0f)
        wheelset_lb.rotate_range -= 360.0f;
    if (wheelset_lb.rotate_range < -180.0f)
        wheelset_lb.rotate_range += 360.0f;
    wheelset_lb.reverse_flag = 0;
    if (fabsf(wheelset_lb.rotate_range) > 90.0f) {
        wheelset_lb.reverse_flag = 1;
        if (wheelset_lb.rotate_range > 0.0f) {
            wheelset_lb.angle_target = wheelset_lb.angle_measure + wheelset_lb.rotate_range - 180.0f;
            wheelset_lb.angle_ref = steering_lb->measure.total_angle + wheelset_lb.rotate_range - 180.0f;
        } else {
            wheelset_lb.angle_target = wheelset_lb.angle_measure + wheelset_lb.rotate_range + 180.0f;
            wheelset_lb.angle_ref = steering_lb->measure.total_angle + wheelset_lb.rotate_range + 180.0f;
        }
    } else {
        wheelset_lb.angle_target = RAD_2_DEGREE * wheelset_lb.angle_speed + wheelset_lb.rotate_range;
        wheelset_lb.angle_ref = steering_lb->measure.total_angle + wheelset_lb.rotate_range;
    }
    wheelset_lb.angle_diff      = wheelset_lb.angle_target - wheelset_lb.angle_measure;
    wheelset_lb.vt              = arm_cos_f32(DEGREE_2_RAD * wheelset_lb.angle_diff) * sqrtf(wheelset_lb.vx * wheelset_lb.vx + wheelset_lb.vy * wheelset_lb.vy);
}
static ramp_t super_ramp;
static float Power_Output;
/**
 * @brief 根据裁判系统和电容剩余容量限制输出，并设置电机参考值
 * @param
 * @param
 *
 */
 static void LimitChassisOutput()
 {
     static float Plimit;

//     // 缓冲能量闭环
//     // if (chassis_cmd_recv.power_buffer < 50 && chassis_cmd_recv.power_buffer >= 40)
//     //     Plimit = 0.9 + (chassis_cmd_recv.power_buffer - 40) * 0.01;
//     // else if (chassis_cmd_recv.power_buffer < 40 && chassis_cmd_recv.power_buffer >= 35)
//     //     Plimit = 0.75 + (chassis_cmd_recv.power_buffer - 35) * (0.15f / 5);
//     // else if (chassis_cmd_recv.power_buffer < 35 && chassis_cmd_recv.power_buffer >= 30)
//     //     Plimit = 0.6 + (chassis_cmd_recv.power_buffer - 30) * (0.15 / 5);
//     // else if (chassis_cmd_recv.power_buffer < 30 && chassis_cmd_recv.power_buffer >= 20)
//     //     Plimit = 0.35 + (chassis_cmd_recv.power_buffer - 20) * (0.25f / 10);
//     // else if (chassis_cmd_recv.power_buffer < 20 && chassis_cmd_recv.power_buffer >= 10)
//     //     Plimit = 0.15 + (chassis_cmd_recv.power_buffer - 10) * 0.01;
//     // else if (chassis_cmd_recv.power_buffer < 10 && chassis_cmd_recv.power_buffer > 0)
//     //     Plimit = 0.05 + chassis_cmd_recv.power_buffer * 0.01;
//     // else if (chassis_cmd_recv.power_buffer == 60)
//     //     Plimit = 1;
    // chassis_cmd_recv.power_buffer = 60;
    // 固定功率限制策略（当前调试配置）
    Plimit = 0;
    chassis_cmd_recv.power_limit = 60;
     Power_Output = chassis_cmd_recv.power_limit - 10 + 20 * Plimit;
     PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL);

     ramp_init(&super_ramp, 300);
 }

// 提高功率上限，飞坡或跑路
// static void SuperLimitOutput()
// {
//     static float power_output;
//     Power_Output = (power_output + (250 - power_output) * ramp_calc(&super_ramp));
//     // Power_Output = (power_output + (250 - 20 + 40 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f - power_output) * ramp_calc(&super_ramp));
//     PowerControlupdate(Power_Output, 1.0f / REDUCTION_RATIO_WHEEL);

//     power_output = Power_Output;
// }

/**
 * @brief 超电控制算法
 *
 *
 */
// uint8_t Super_Voltage_Allow_Flag;
// static SuperCap_State_e SuperCap_state = SUPER_STATE_LOW;
 void Super_Cap_control()
 {
//     // 状态机逻辑,滞回
//     switch (SuperCap_state) {
//         case SUPER_STATE_LOW:
//             if (cap->cap_msg_s.CapVot > SUPER_VOLTAGE_THRESHOLD_HIGH) {
//                 SuperCap_state = SUPER_STATE_HIGH;
//             }
//             break;
//         case SUPER_STATE_HIGH:
//             if (cap->cap_msg_s.CapVot < SUPER_VOLTAGE_THRESHOLD_LOW) {
//                 SuperCap_state = SUPER_STATE_LOW;
//             }
//             break;
//         default:
//             SuperCap_state = SUPER_STATE_LOW;
//             break;
//     }

//     // 小于12V关闭
//     if (SuperCap_state == SUPER_STATE_LOW) {
//         Super_Voltage_Allow_Flag = SUPER_VOLTAGE_CLOSE;
//     } else if (SuperCap_state == SUPER_STATE_HIGH) {
//         Super_Voltage_Allow_Flag = SUPER_VOLTAGE_OPEN;
//     } else {
//         // none
//     }

//     // 用户允许开启超级电容，且电压充足
//     if (chassis_cmd_recv.SuperCap_flag_from_user == SUPER_USER_OPEN) {
//         cap->cap_msg_g.enabled = SUPER_CMD_OPEN;
//         SuperLimitOutput();
//     } else {
//         cap->cap_msg_g.enabled = SUPER_CMD_CLOSE;
         LimitChassisOutput();
//     }

     // 设定速度参考值
    //  DJIMotorSetRef(motor_lf, vt_lf);
    //  DJIMotorSetRef(motor_rf, vt_rf);
    //  DJIMotorSetRef(motor_lb, vt_lb);
    //  DJIMotorSetRef(motor_rb, vt_rb);
 }

// 获取功率裆位
 static void Power_get()
 {
     cap->cap_msg_g.power_limit = chassis_cmd_recv.power_limit - 30 + 30 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f;
 }

// float offset_angle_watch;
uint8_t chassis_rate=100;
int8_t chassis_flag=1;
/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续可增加“未收到消息”时的处理（双板场景）
    // 获取新的控制信息

    SubGetMessage(chassis_sub, &chassis_cmd_recv);

#ifdef CHASSIS_BOARD

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现关键模块离线或遥控器急停，则关闭电机输出
        // DJIMotorStop(motor_lf);
        // DJIMotorStop(motor_rf);
        // DJIMotorStop(motor_lb);
        // DJIMotorStop(motor_rb);
        DJIMotorStop(steering_lf);
        DJIMotorStop(steering_rf);
        DJIMotorStop(steering_rb);
        DJIMotorStop(steering_lb);
    } else { // 正常工作
        // DJIMotorEnable(motor_lf);
        // DJIMotorEnable(motor_rf);
        // DJIMotorEnable(motor_lb);
        // DJIMotorEnable(motor_rb);
        DJIMotorEnable(steering_lf);
        DJIMotorEnable(steering_rf);
        DJIMotorEnable(steering_rb);
        DJIMotorEnable(steering_lb);
    }
    static float offset_angle;
    static float sin_theta, cos_theta;
    //
    static float current_speed_vw, vw_set;
    // static ramp_t rotate_ramp;

    offset_angle       = chassis_cmd_recv.offset_angle;// + chassis_cmd_recv.gimbal_error_angle;
    // offset_angle_watch = offset_angle;
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW:
            // 底盘不自旋，但保持全向机动，一般用于调整云台姿态
            // 当前模式下保留外部 wz 指令（不强制置零）
            //chassis_cmd_recv.wz = 0;

            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台

            //  if (chassis_cmd_recv.offset_angle <= 90 && chassis_cmd_recv.offset_angle >= -90) // 0附近
            //     offset_angle = -chassis_cmd_recv.offset_angle;
            //  else {
            //      offset_angle = -(chassis_cmd_recv.offset_angle >= 0 ? chassis_cmd_recv.offset_angle - 180 : chassis_cmd_recv.offset_angle + 180);
            //  }

            chassis_cmd_recv.wz = PIDCalculate(&Chassis_Follow_PID, -chassis_cmd_recv.align_angle, 0);

            cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
            sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

            ramp_init(&rotate_ramp, 250);
            break;
            // 旋转模式调参说明
        case CHASSIS_ROTATE: // 自旋，同时保持全向机动；当前 wz 保持定速，后续可增加不规则变速策略
           //  if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_PMOS_OPEN) {
           //      vw_set = 7000;
           //  } else {
                vw_set = 3800 * ((float)chassis_cmd_recv.chassis_rotate_speed / 255.0f);
            // }
            // if(vw_set<=3000) chassis_flag=1;
            // if(vw_set>=6500) chassis_flag=-1;
            // vw_set+=2*chassis_flag;

            chassis_vw       = (current_speed_vw + (vw_set - current_speed_vw) * ramp_calc(&rotate_ramp));
            current_speed_vw = chassis_vw;
            

            chassis_cmd_recv.wz = chassis_vw;
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle + 0) * DEGREE_2_RAD); // 矫正小陀螺偏航
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle + 0) * DEGREE_2_RAD);
            chassis_cmd_recv.vx *= 0.6;
            chassis_cmd_recv.vy *= 0.6;
            break;
            
        case CHASSIS_REVERSE_ROTATE:
            chassis_cmd_recv.wz = -2500;
            cos_theta           = arm_cos_f32((chassis_cmd_recv.offset_angle + 22) * DEGREE_2_RAD); // 矫正小陀螺偏航
            sin_theta           = arm_sin_f32((chassis_cmd_recv.offset_angle + 22) * DEGREE_2_RAD);
        default:
            break;
    }

    // 根据云台与底盘的角度 offset 将控制量映射到底盘坐标系
    // 底盘逆时针为角度正方向；云台指向作为 x 轴，按右手系确定 y 轴方向
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 根据控制模式进行正运动学解算,计算底盘输出
    SteeringCalculate();
    DJIMotorSetRef(steering_lf, wheelset_lf.angle_ref);
    DJIMotorSetRef(steering_rf, wheelset_rf.angle_ref);
    DJIMotorSetRef(steering_rb, wheelset_rb.angle_ref);
    DJIMotorSetRef(steering_lb, wheelset_lb.angle_ref);
    // DJIMotorSetRef(steering_lf, 0);
    // 根据裁判系统与电容反馈对输出限幅，并设定闭环参考
    Super_Cap_control();

    // 获得给电容传输的电容吸取功率等级
    Power_get();

    // 给电容发送数据
    //SuperCapSend(cap, (uint8_t *)&cap->cap_msg_g);

    // 推送反馈消息
    //memcpy(&chassis_feedback_data.CapFlag_open_from_real, &cap->cap_msg_s.SuperCap_open_flag_from_real, sizeof(uint8_t));
    //memcpy(&chassis_feedback_data.cap_voltage, &cap->cap_msg_s.CapVot, sizeof(float));
   // memcpy(&chassis_feedback_data.chassis_power_output, &Power_Output, sizeof(float));
   // memcpy(&chassis_feedback_data.chassis_voltage, &cap->cap_msg_s.chassis_voltage_from_cap, sizeof(float));
#endif
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}

