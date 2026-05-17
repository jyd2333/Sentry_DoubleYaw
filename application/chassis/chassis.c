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
#include "bsp_dwt.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "power_calc.h"
#include "tool.h"
#include "comm.h"
#include "robot_test.h"

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
extern comm_cmd_t comm_cmd_data;
extern comm_upload_t comm_upload_data;
extern referee_info_t referee_info;           // 裁判系统数据
SuperCapInstance *cap;                                              // 超级电容
// 驱动与转向电机实例
DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
DJIMotorInstance *steering_lf, *steering_rf, *steering_rb, *steering_lb;
steering_wheelset_t wheelset_lf, wheelset_rf, wheelset_rb, wheelset_lb;
chassis_speed_measure_t speed_measure;
static float chassis_tilt_deadband_deg = 4.0f;
static uint8_t last_navi_stamp;
static uint32_t last_navi_DWT_stamp;
static uint8_t navi_speed_valid;
static float navi_raw_vx, navi_raw_vy, navi_raw_yaw;
// 为了方便调试加入的量
static uint8_t center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X; // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
static uint8_t center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y; // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0

// 跟随模式底盘的pid
// 目前未严格约定单位，后续如有需要再统一规范
static PIDInstance Chassis_Follow_PID = {
    .Kp            = 0.1f,   // 25,//25, // 50,//70, // 4.5
    .Ki            = 0,    // 0
    .Kd            = 0, // 0.0,  // 0.07,  // 0
    .DeadBand      = 1.5f,  //跟随模式设置了死区，防止抖动
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
static float offset_angle;
static float sin_theta, cos_theta;
static float chassis_speed_direction;
static volatile float chassis_offset_delay_s        = 0.4f;
static volatile float chassis_offset_ff_limit_deg   = 90.0f;
static volatile float chassis_offset_ff_min_speed   = 100.0f;
static volatile float chassis_offset_ff_full_speed  = CHASSIS_HIGH_SPEED * 2.0f;
static volatile float steering_ff_delay_s           = 0.0f;
static volatile float steering_ff_limit_deg         = 15.0f;
static volatile float steering_ff_min_speed         = 100.0f;
static float steering_lf_angle_ff, steering_rf_angle_ff, steering_rb_angle_ff, steering_lb_angle_ff;
static float steering_lf_last_angle, steering_rf_last_angle, steering_rb_last_angle, steering_lb_last_angle;
static uint8_t steering_ff_valid;
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
                .Kp            = 60,//12, // 0.24, // 0.31, // 0.45
                .Ki            = 7,
                .Kd            = 0,//0.02,//0.01,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement,
                .IntegralLimit = 20, 
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp            = 18,//6000,//10000, //11000,
                .Ki            = 8,    // 0
                .Kd            = 0.01,//5, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 3000,
                .MaxOut        = 20000 // 20000
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
    steering_config.can_init_config.can_handle                              = &hcan2;
    steering_config.can_init_config.tx_id                                   = 1;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = LF;
    steering_lf                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan2;
    steering_config.can_init_config.tx_id                                   = 4;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = RF;
    steering_rf                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 3;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = RB;
    steering_rb                                                             = DJIMotorInit(&steering_config);

    steering_config.can_init_config.can_handle                              = &hcan1;
    steering_config.can_init_config.tx_id                                   = 2;
    steering_config.power_limit_group                                       = DJI_POWER_LIMIT_STEER;
    steering_config.power_limit_index                                       = LB;
    steering_lb                                                             = DJIMotorInit(&steering_config);

    //  @todo: 当前未统一电机正反方向，仍需手动处理 reference 正负号，待电机模块支持后修复
    chassis_motor_config.can_init_config.can_handle                         = &hcan2;
    chassis_motor_config.can_init_config.tx_id                              = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = LF;
    motor_lf                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan2;
    chassis_motor_config.can_init_config.tx_id                              = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = RF;
    motor_rf                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = RB;
    motor_rb                                                                = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.can_handle                         = &hcan1;
    chassis_motor_config.can_init_config.tx_id                              = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag  = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.power_limit_group                                  = DJI_POWER_LIMIT_WHEEL;
    chassis_motor_config.power_limit_index                                  = LB;
    motor_lb                                                                = DJIMotorInit(&chassis_motor_config);

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0X180, // 超级电容默认接收id
            .rx_id      = 0x185, // 超级电容默认发送 ID，注意 tx/rx 对设备视角是相反的
        }};
    cap = SuperCapRegister(&cap_conf); // 超级电容初始化
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
    vt_lf = chassis_vx + chassis_vy + chassis_vw * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_vw * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy + chassis_vw * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_vw * RB_CENTER;
}

static float ChassisLimitFloat(float value, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }

    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float ChassisAngleDiffRad(float target, float measure)
{
    float diff = target - measure;

    while (diff > PI) {
        diff -= 2.0f * PI;
    }
    while (diff < -PI) {
        diff += 2.0f * PI;
    }
    return diff;
}

static uint8_t ChassisIsRotateMode(void)
{
    return chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE ||
           chassis_cmd_recv.chassis_mode == CHASSIS_REVERSE_ROTATE;
}

static void ChassisOffsetAngleFeedforwardCalc(void)
{
    float offset_ff_deg = 0.0f;
    float move_speed;
    float speed_factor;

    if (ChassisIsRotateMode()) {
        move_speed = sqrtf(chassis_cmd_recv.vx * chassis_cmd_recv.vx + chassis_cmd_recv.vy * chassis_cmd_recv.vy);
        if (move_speed > chassis_offset_ff_min_speed) {
            if (chassis_offset_ff_full_speed > chassis_offset_ff_min_speed) {
                speed_factor = (move_speed - chassis_offset_ff_min_speed) /
                               (chassis_offset_ff_full_speed - chassis_offset_ff_min_speed);
                speed_factor = ChassisLimitFloat(speed_factor, 1.0f);
            } else {
                speed_factor = 1.0f;
            }
        } else {
            speed_factor = 0.0f;
        }

        offset_ff_deg = chassis_vw * chassis_offset_delay_s * RAD_2_DEGREE * speed_factor;
        offset_ff_deg = ChassisLimitFloat(offset_ff_deg, chassis_offset_ff_limit_deg);
    }
    comm_upload_data.debug_1 = offset_ff_deg;
    offset_angle += offset_ff_deg;
    cos_theta = arm_cos_f32(offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(offset_angle * DEGREE_2_RAD);
}

static void ChassisSteeringFeedforwardReset(void)
{
    steering_lf_angle_ff = 0.0f;
    steering_rf_angle_ff = 0.0f;
    steering_rb_angle_ff = 0.0f;
    steering_lb_angle_ff = 0.0f;
    steering_ff_valid    = 0;
}

static float ChassisSteeringAngleFeedforward(float target_angle, float last_angle, float target_speed)
{
    float target_angle_rate;
    float angle_ff_limit_rad = fabsf(steering_ff_limit_deg) * DEGREE_2_RAD;
    float min_speed          = fabsf(steering_ff_min_speed);

    if (target_speed < min_speed || angle_ff_limit_rad <= 0.0f) {
        return 0.0f;
    }

    target_angle_rate = ChassisAngleDiffRad(target_angle, last_angle) * 1000.0f;
    return ChassisLimitFloat(target_angle_rate * steering_ff_delay_s, angle_ff_limit_rad);
}

static void ChassisSteeringFeedforwardCalc(void)
{
    const float half_side = CHASSIS_R * 0.707f;
    float lf_vx, rf_vx, rb_vx, lb_vx;
    float lf_vy, rf_vy, rb_vy, lb_vy;
    float lf_angle, rf_angle, rb_angle, lb_angle;
    float lf_speed, rf_speed, rb_speed, lb_speed;

    lf_vx = chassis_vx + chassis_vw * half_side;
    lf_vy = chassis_vy - chassis_vw * half_side;
    rf_vx = chassis_vx - chassis_vw * half_side;
    rf_vy = chassis_vy - chassis_vw * half_side;
    rb_vx = chassis_vx - chassis_vw * half_side;
    rb_vy = chassis_vy + chassis_vw * half_side;
    lb_vx = chassis_vx + chassis_vw * half_side;
    lb_vy = chassis_vy + chassis_vw * half_side;

    lf_angle = -atan2f(lf_vy, lf_vx);
    rf_angle = -atan2f(rf_vy, rf_vx);
    rb_angle = -atan2f(rb_vy, rb_vx);
    lb_angle = -atan2f(lb_vy, lb_vx);

    if (!ChassisIsRotateMode() || fabsf(steering_ff_delay_s) < 0.000001f) {
        steering_lf_last_angle = lf_angle;
        steering_rf_last_angle = rf_angle;
        steering_rb_last_angle = rb_angle;
        steering_lb_last_angle = lb_angle;
        ChassisSteeringFeedforwardReset();
        return;
    }

    if (!steering_ff_valid) {
        steering_lf_last_angle = lf_angle;
        steering_rf_last_angle = rf_angle;
        steering_rb_last_angle = rb_angle;
        steering_lb_last_angle = lb_angle;
        steering_lf_angle_ff   = 0.0f;
        steering_rf_angle_ff   = 0.0f;
        steering_rb_angle_ff   = 0.0f;
        steering_lb_angle_ff   = 0.0f;
        steering_ff_valid      = 1;
        return;
    }

    lf_speed = sqrtf(lf_vx * lf_vx + lf_vy * lf_vy);
    rf_speed = sqrtf(rf_vx * rf_vx + rf_vy * rf_vy);
    rb_speed = sqrtf(rb_vx * rb_vx + rb_vy * rb_vy);
    lb_speed = sqrtf(lb_vx * lb_vx + lb_vy * lb_vy);

    steering_lf_angle_ff = ChassisSteeringAngleFeedforward(lf_angle, steering_lf_last_angle, lf_speed);
    steering_rf_angle_ff = ChassisSteeringAngleFeedforward(rf_angle, steering_rf_last_angle, rf_speed);
    steering_rb_angle_ff = ChassisSteeringAngleFeedforward(rb_angle, steering_rb_last_angle, rb_speed);
    steering_lb_angle_ff = ChassisSteeringAngleFeedforward(lb_angle, steering_lb_last_angle, lb_speed);

    steering_lf_last_angle = lf_angle;
    steering_rf_last_angle = rf_angle;
    steering_rb_last_angle = rb_angle;
    steering_lb_last_angle = lb_angle;
}

static void SteeringCalculate(void)
{
    wheelset_lf.vx = chassis_vx + chassis_vw * 0.707f;
    wheelset_lf.vy = chassis_vy - chassis_vw * 0.707f;
    wheelset_lf.angle_measure   = ((float)steering_lf->measure.ecd - STEERING_LF_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_lf.angle_speed     = -atan2f(wheelset_lf.vy, wheelset_lf.vx) + steering_lf_angle_ff;
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
    
    wheelset_rf.vx = chassis_vx - chassis_vw * 0.707f;
    wheelset_rf.vy = chassis_vy - chassis_vw * 0.707f;
    wheelset_rf.angle_measure   = ((float)steering_rf->measure.ecd - STEERING_RF_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_rf.angle_speed     = -atan2f(wheelset_rf.vy, wheelset_rf.vx) + steering_rf_angle_ff;
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

    wheelset_rb.vx = chassis_vx - chassis_vw * 0.707f;
    wheelset_rb.vy = chassis_vy + chassis_vw * 0.707f;
    wheelset_rb.angle_measure   = ((float)steering_rb->measure.ecd - STEERING_RB_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_rb.angle_speed     = -atan2f(wheelset_rb.vy, wheelset_rb.vx) + steering_rb_angle_ff;
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

    wheelset_lb.vx = chassis_vx + chassis_vw * 0.707f;
    wheelset_lb.vy = chassis_vy + chassis_vw * 0.707f;
    wheelset_lb.angle_measure   = ((float)steering_lb->measure.ecd - STEERING_LB_ECD) * ECD_ANGLE_COEF_DJI;
    wheelset_lb.angle_speed     = -atan2f(wheelset_lb.vy, wheelset_lb.vx) + steering_lb_angle_ff;
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
    wheelset_lb.angle_diff      = RAD_2_DEGREE * wheelset_lb.angle_speed - wheelset_lb.angle_measure;
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
    DJIMotorPowerControlEnable(1);
    DJIMotorPowerControlSetMaxPower(Power_Output);
    ramp_init(&super_ramp, 300);
 }

// 提高功率上限，飞坡或跑路
// static void SuperLimitOutput()
// {
//     static float power_output;
//     Power_Output = (power_output + (250 - power_output) * ramp_calc(&super_ramp));
//     // Power_Output = (power_output + (250 - 20 + 40 * (cap->cap_msg_s.CapVot - 17.0f) / 6.0f - power_output) * ramp_calc(&super_ramp));

//     power_output = Power_Output;
// }
// uint8_t Super_Voltage_Allow_Flag;
// static SuperCap_State_e SuperCap_state = SUPER_STATE_LOW;
static uint8_t supercap_task_div_cnt = 0;
/**
 * @brief 超电控制算法
 *
 *
 */
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
    cap->tx_data.enable_flag = SUPERCAP_ENABLE;
    cap->tx_data.charge_flag = 1;
    cap->tx_data.power_limit = referee_info.GameRobotStatus.chassis_power_limit;
    if (++supercap_task_div_cnt >= 5) {
        supercap_task_div_cnt = 0;
        SuperCapTask();
    }
     // 设定速度参考值
     DJIMotorSetRef(motor_lf, wheelset_lf.vt);
     DJIMotorSetRef(motor_rf, wheelset_rf.vt);
     DJIMotorSetRef(motor_lb, wheelset_lb.vt);
     DJIMotorSetRef(motor_rb, wheelset_rb.vt);
 }


/**
 * @brief 底盘速度方向映射与死区处理
 *
 */
static void ChassisSpeedMap()
{
    const float speed_deadband  = 10.0f / SPEED_TO_DJI_MOTOR_APS;
    const float rotate_deadband = 10.0f / (CHASSIS_R * SPEED_TO_DJI_MOTOR_APS);

    // 根据云台与底盘的角度 offset 将控制量映射到底盘坐标系
    // 底盘逆时针为角度正方向；云台指向作为 y 轴
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    if (fabsf(chassis_vx) < speed_deadband && fabsf(chassis_vy) < speed_deadband && fabsf(chassis_vw) < rotate_deadband)
    {
        chassis_vx = 0;
        chassis_vy = 0;
        chassis_vw = 1.0f / (CHASSIS_R * SPEED_TO_DJI_MOTOR_APS);
    }
}

/**
 * @brief 底盘速度转换
 * 将vx、vy、vw转换为电机转子角速度（单位：aps）
 * (原vx、vy单位：mm/s，vw单位：rad/s)
 */
static void SpeedUnitsConvert()
{
    chassis_vx = chassis_vx * SPEED_TO_DJI_MOTOR_APS;
    chassis_vy = chassis_vy * SPEED_TO_DJI_MOTOR_APS;
    chassis_vw = chassis_vw * CHASSIS_R * SPEED_TO_DJI_MOTOR_APS;
}

static void ChassisAccelerationPlan()
{
    float current_speed     = sqrtf(speed_measure.real_vx * speed_measure.real_vx + speed_measure.real_vy * speed_measure.real_vy);
    float acceleration_limit = CHASSIS_ACCELERATION_LOW_LIMIT;
    float max_delta_speed;
    float target_vx         = chassis_vx;
    float target_vy         = chassis_vy;
    float delta_vx          = target_vx - speed_measure.real_vx;
    float delta_vy          = target_vy - speed_measure.real_vy;
    float delta_speed       = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);

    if (current_speed >= CHASSIS_HIGH_SPEED) {
        acceleration_limit = CHASSIS_ACCELERATION_HIGH_LIMIT;
    } else if (current_speed > 0.000001f && CHASSIS_HIGH_SPEED > 0.000001f) {
        acceleration_limit = CHASSIS_ACCELERATION_LOW_LIMIT +
                             (CHASSIS_ACCELERATION_HIGH_LIMIT - CHASSIS_ACCELERATION_LOW_LIMIT) *
                             current_speed / CHASSIS_HIGH_SPEED;
    }

    max_delta_speed = acceleration_limit * 0.001f;

    if (delta_speed > max_delta_speed && delta_speed > 0.000001f) {
        float scale = max_delta_speed / delta_speed;
        target_vx   = speed_measure.real_vx + delta_vx * scale;
        target_vy   = speed_measure.real_vy + delta_vy * scale;
    }

    chassis_vx = target_vx;
    chassis_vy = target_vy;
}

static void ChassisSpeedMeasure()
{
    const float half_side = CHASSIS_R * 0.70710678f;
    float lf_speed, rf_speed, rb_speed, lb_speed;
    float lf_angle, rf_angle, rb_angle, lb_angle;
    float lf_vx, rf_vx, rb_vx, lb_vx;
    float lf_vy, rf_vy, rb_vy, lb_vy;
    float raw_vx, raw_vy, raw_wz;
    float sample_count;

    if (motor_lf == NULL || motor_rf == NULL || motor_rb == NULL || motor_lb == NULL ||
        steering_lf == NULL || steering_rf == NULL || steering_rb == NULL || steering_lb == NULL) {
        speed_measure.real_vx = 0.0f;
        speed_measure.real_vy = 0.0f;
        speed_measure.real_wz = 0.0f;
        chassis_feedback_data.real_vx = 0.0f;
        chassis_feedback_data.real_vy = 0.0f;
        chassis_feedback_data.real_wz = 0.0f;
        return;
    }

    lf_speed = motor_lf->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;
    rf_speed = motor_rf->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;
    rb_speed = motor_rb->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;
    lb_speed = motor_lb->measure.speed_rpm / REDUCTION_RATIO_WHEEL / 60.0f * PERIMETER_WHEEL;

    lf_angle = -((float)steering_lf->measure.ecd - STEERING_LF_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;
    rf_angle = -((float)steering_rf->measure.ecd - STEERING_RF_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;
    rb_angle = -((float)steering_rb->measure.ecd - STEERING_RB_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;
    lb_angle = -((float)steering_lb->measure.ecd - STEERING_LB_ECD) * ECD_ANGLE_COEF_DJI * DEGREE_2_RAD;

    lf_vx = lf_speed * arm_cos_f32(lf_angle);
    rf_vx = rf_speed * arm_cos_f32(rf_angle);
    rb_vx = rb_speed * arm_cos_f32(rb_angle);
    lb_vx = lb_speed * arm_cos_f32(lb_angle);
    lf_vy = lf_speed * arm_sin_f32(lf_angle);
    rf_vy = rf_speed * arm_sin_f32(rf_angle);
    rb_vy = rb_speed * arm_sin_f32(rb_angle);
    lb_vy = lb_speed * arm_sin_f32(lb_angle);

    raw_vx = (lf_vx + rf_vx + rb_vx + lb_vx) * 0.25f;
    raw_vy = (lf_vy + rf_vy + rb_vy + lb_vy) * 0.25f;
    raw_wz = (lf_vx + lb_vx - rf_vx - rb_vx + rb_vy + lb_vy - lf_vy - rf_vy) / (8.0f * half_side);

    speed_measure.vx_sum -= speed_measure.vx_window[speed_measure.window_index];
    speed_measure.vy_sum -= speed_measure.vy_window[speed_measure.window_index];
    speed_measure.wz_sum -= speed_measure.wz_window[speed_measure.window_index];
    speed_measure.vx_window[speed_measure.window_index] = raw_vx;
    speed_measure.vy_window[speed_measure.window_index] = raw_vy;
    speed_measure.wz_window[speed_measure.window_index] = raw_wz;
    speed_measure.vx_sum += raw_vx;
    speed_measure.vy_sum += raw_vy;
    speed_measure.wz_sum += raw_wz;

    if (speed_measure.window_count < CHASSIS_SPEED_MEASURE_WINDOW_SIZE) {
        speed_measure.window_count++;
    }

    speed_measure.window_index++;
    if (speed_measure.window_index >= CHASSIS_SPEED_MEASURE_WINDOW_SIZE) {
        speed_measure.window_index = 0;
    }

    sample_count = (float)speed_measure.window_count;
    speed_measure.real_vx = speed_measure.vx_sum / sample_count;
    speed_measure.real_vy = speed_measure.vy_sum / sample_count;
    speed_measure.real_wz = speed_measure.wz_sum / sample_count;

    chassis_feedback_data.real_vx = speed_measure.real_vx;
    chassis_feedback_data.real_vy = speed_measure.real_vy;
    chassis_feedback_data.real_wz = speed_measure.real_wz;

}

/**
 * @brief 底盘速度插值
 *
 */
static void ChassisSpeedInterpolate()
{
    uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
    float delta_yaw;
    float sin_delta_yaw, cos_delta_yaw;

    if (comm_cmd_data.navi_stamp == 0 || chassis_IMU_data == NULL) {
        last_navi_stamp = comm_cmd_data.navi_stamp;
        last_navi_DWT_stamp = now_ms;
        navi_speed_valid = 0;
        return;
    }

    if (!navi_speed_valid || last_navi_stamp != comm_cmd_data.navi_stamp)
    {
        navi_raw_vx = chassis_vx;
        navi_raw_vy = chassis_vy;
        navi_raw_yaw = chassis_IMU_data->output.Yaw_total_angle;
        last_navi_DWT_stamp = now_ms;
        last_navi_stamp = comm_cmd_data.navi_stamp;
        navi_speed_valid = 1;
        return;
    }

    if (now_ms - last_navi_DWT_stamp > 1000U) {
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        return;
    }

    delta_yaw = navi_raw_yaw - chassis_IMU_data->output.Yaw_total_angle;
    sin_delta_yaw = arm_sin_f32(delta_yaw);
    cos_delta_yaw = arm_cos_f32(delta_yaw);
    chassis_vx = navi_raw_vx * cos_delta_yaw - navi_raw_vy * sin_delta_yaw;
    chassis_vy = navi_raw_vx * sin_delta_yaw + navi_raw_vy * cos_delta_yaw;
}

/**
 * @brief 底盘梯度方向
 *
 */
static void ChassisTiltCalc()
{
    float pitch, roll;
    float sin_pitch, sin_roll, cos_pitch, cos_roll;
    float tilt_x, tilt_y;
    float tilt_alpha;
    float tilt_direction;

    if (chassis_IMU_data == NULL) {
        chassis_feedback_data.tilt_direction = 0.0f;
        chassis_feedback_data.tilt_angle = 0.0f;
        return;
    }

    pitch     = chassis_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET];
    roll      = chassis_IMU_data->output.INS_angle[INS_ROLL_ADDRESS_OFFSET];
    sin_pitch = arm_sin_f32(pitch);
    sin_roll  = arm_sin_f32(roll);
    cos_pitch = arm_cos_f32(pitch);
    cos_roll  = arm_cos_f32(roll);
    tilt_x    = -sin_pitch;
    tilt_y    = sin_roll * cos_pitch;

    tilt_alpha = atan2f(sqrtf(tilt_x * tilt_x + tilt_y * tilt_y), cos_roll * cos_pitch) * RAD_2_DEGREE;
    if (tilt_alpha < chassis_tilt_deadband_deg) {
        chassis_feedback_data.tilt_direction = 0.0f;
        chassis_feedback_data.tilt_angle = 0.0f;
        return;
    }

    tilt_direction = atan2f(tilt_y, tilt_x) * RAD_2_DEGREE;
    if (tilt_direction < 0.0f) {
        tilt_direction += 360.0f;
    }

    chassis_feedback_data.tilt_direction    = tilt_direction;
    chassis_feedback_data.tilt_angle        = tilt_alpha;
}

/**
 * @brief 底盘速度方向
 *
 */
static void GetChassisSpeedDirection()
{
    const float speed_deadband = 10.0f / SPEED_TO_DJI_MOTOR_APS;
    float chassis_speed;
    float speed_direction;

    chassis_speed = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
    if (chassis_speed < speed_deadband) {
        return;
    }

    speed_direction = atan2f(chassis_vy, chassis_vx) * RAD_2_DEGREE;
    speed_direction = theta_format(speed_direction);

    while (speed_direction > 90.0f) {
        speed_direction -= 180.0f;
    }
    while (speed_direction < -90.0f) {
        speed_direction += 180.0f;
    }

    chassis_speed_direction = speed_direction;
}

/**
 * @brief 跨越地形时速度控制
 *
 */
static void TerrainSpeedControl()
{
    float chassis_speed;
    float scale;

    if(!comm_cmd_data.terrain_state)
        return;
    
    chassis_speed = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);

    if(comm_cmd_data.terrain_state == TERRAIN_BUMP)
    {
        if (chassis_speed <= 0.0f) {
            return;
        }

        scale = 2500.0f / chassis_speed;
        chassis_vx *= scale;
        chassis_vy *= scale;
    }
    if(comm_cmd_data.terrain_state == TERRAIN_FORTRESS)
    {
        if (chassis_speed <= 500.0f) {
            return;
        }

        scale = 500.0f / chassis_speed;
        chassis_vx *= scale;
        chassis_vy *= scale;
    }
}

/**
 * @brief 堡垒上坡助力
 *
 */
static void FortressAssist()
{
    if(comm_cmd_data.terrain_state != TERRAIN_FORTRESS)
        return;

    const float speed_deadband = 10.0f / SPEED_TO_DJI_MOTOR_APS;
    const float tilt_angle_cap = 25.0f;
    float chassis_speed;
    float speed_direction;
    float delta_direction;
    float dir_factor;
    float tilt_factor;
    float assist_scale;

    chassis_speed = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);
    if (chassis_speed < speed_deadband) {
        return;
    }

    if (chassis_feedback_data.tilt_angle <= 0.0f) {
        return;
    }

    speed_direction = theta_format(-atan2f(chassis_vy, chassis_vx) * RAD_2_DEGREE);
    delta_direction = speed_direction - chassis_feedback_data.tilt_direction;

    while (delta_direction > 180.0f) {
        delta_direction -= 360.0f;
    }
    while (delta_direction < -180.0f) {
        delta_direction += 360.0f;
    }

    dir_factor = arm_cos_f32(delta_direction * DEGREE_2_RAD);
    if (dir_factor <= 0.0f) {
        return;
    }

    tilt_factor = chassis_feedback_data.tilt_angle / tilt_angle_cap;
    tilt_factor = ChassisLimitFloat(tilt_factor, 1.0f);
    if (tilt_factor <= 0.0f) {
        return;
    }

    assist_scale = 1.0f + 0.5f * dir_factor * tilt_factor;
    chassis_vx *= assist_scale;
    chassis_vy *= assist_scale;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 获取新的控制信息

    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    
#ifdef CHASSIS_BOARD
    ChassisSpeedMeasure();
    ChassisTiltCalc();
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现关键模块离线或遥控器急停，则关闭电机输出
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        DJIMotorStop(steering_lf);
        DJIMotorStop(steering_rf);
        DJIMotorStop(steering_rb);
        DJIMotorStop(steering_lb);
        chassis_vw = 0;
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        DJIMotorEnable(steering_lf);
        DJIMotorEnable(steering_rf);
        DJIMotorEnable(steering_rb);
        DJIMotorEnable(steering_lb);
    }

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
            chassis_vw = 0;
            ramp_init(&rotate_ramp, 250);
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台

            //  if (chassis_cmd_recv.offset_angle <= 90 && chassis_cmd_recv.offset_angle >= -90) // 0附近
            //     offset_angle = -chassis_cmd_recv.offset_angle;
            //  else {
            //      offset_angle = -(chassis_cmd_recv.offset_angle >= 0 ? chassis_cmd_recv.offset_angle - 180 : chassis_cmd_recv.offset_angle + 180);
            //  }
            if(comm_cmd_data.terrain_state == TERRAIN_BUMP)
            {
                const float speed_deadband = 10.0f / SPEED_TO_DJI_MOTOR_APS;
                float chassis_speed = sqrtf(chassis_vx * chassis_vx + chassis_vy * chassis_vy);

                if (chassis_speed >= speed_deadband) {
                    chassis_vw = PIDCalculate(&Chassis_Follow_PID, chassis_speed_direction, 0);
                } else {
                    chassis_vw = 1.0f / (CHASSIS_R * SPEED_TO_DJI_MOTOR_APS);
                }
            }
            else
                chassis_vw = PIDCalculate(&Chassis_Follow_PID, chassis_cmd_recv.align_angle, 0);


            ramp_init(&rotate_ramp, 250);
            break;
            // 旋转模式调参说明
        case CHASSIS_ROTATE: // 自旋，同时保持全向机动；当前 wz 保持定速，后续可增加不规则变速策略
           //  if (cap->cap_msg_s.SuperCap_open_flag_from_real == SUPERCAP_PMOS_OPEN) {
           //      vw_set = 7000;
           //  } else {
                vw_set = 10 * ((float)chassis_cmd_recv.chassis_rotate_speed / 255.0f);
            // }


            chassis_vw       = (current_speed_vw + (vw_set - current_speed_vw) * ramp_calc(&rotate_ramp));
            current_speed_vw = chassis_vw;
            

            // chassis_cmd_recv.wz = chassis_vw;
            chassis_cmd_recv.vx *= 0.6;
            chassis_cmd_recv.vy *= 0.6;
            break;
            
        case CHASSIS_REVERSE_ROTATE:
            // chassis_cmd_recv.wz = -2500;
            chassis_vw          = -2;
            offset_angle       += 22.0f;
            break;
        default:
            break;
    }
    ChassisOffsetAngleFeedforwardCalc();
    ChassisSpeedMap();
    GetChassisSpeedDirection();
    ChassisSpeedInterpolate();
    TerrainSpeedControl();
    FortressAssist();
    ChassisAccelerationPlan();
    ChassisSteeringFeedforwardCalc();
    SpeedUnitsConvert();
    // 根据控制模式进行正运动学解算,计算底盘输出
    SteeringCalculate();
    DJIMotorSetRef(steering_lf, wheelset_lf.angle_ref);
    DJIMotorSetRef(steering_rf, wheelset_rf.angle_ref);
    DJIMotorSetRef(steering_rb, wheelset_rb.angle_ref);
    DJIMotorSetRef(steering_lb, wheelset_lb.angle_ref);
    // DJIMotorSetRef(steering_lf, 0);
    // 根据裁判系统与电容反馈对输出限幅，并设定闭环参考
    Super_Cap_control();



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

