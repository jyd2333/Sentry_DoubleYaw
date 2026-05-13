#include "stdio.h"

#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "robot_test.h"
#include "comm.h"
#include "bmi088.h"
#include "referee_UI.h"

#include "DMmotor.h"
#include "math.h"
#include "arm_math.h"
#include "bsp_dwt.h"

#define SEARCH_MOTOR_OFFLINE_TIMEOUT_MS 10.0f

static INS_Instance *gimbal_IMU_data; // 云台IMU数据
DJIMotorInstance *yaw_motor;
DMMotorInstance *pitch_motor, *big_yaw_motor;
extern DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;
extern comm_cmd_t comm_cmd_data;
extern comm_upload_t comm_upload_data;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
float base_yaw_vel_feedforward = 0;
float pitch_tor_feedforward = 0;
float pitch_tor_feedforward_ori = 0;
float pitch_vel_feedforward = 0;
float yaw_vel_feedforward = 0;
float yaw_tor_feedforward = 0;
volatile static float yaw_tor_k = 60.0f;
extern  NUC_cmd_t NUC_cmd;
extern chassis_speed_measure_t speed_measure;

#ifdef CHASSIS_BOARD
static uint8_t SearchMotorIsOnline(const DMMotorInstance *motor)
{
    float now_ms;

    if (motor == NULL || motor->last_feedback_ms <= 0.0f) {
        return 0;
    }

    now_ms = DWT_GetTimeline_ms();
    return (now_ms - motor->last_feedback_ms) <= SEARCH_MOTOR_OFFLINE_TIMEOUT_MS;
}
#endif

void GimbalInit()
{
#ifdef GIMBAL_BOARD
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
    gimbal_IMU_data = INS_Init(BMI088Register(&config)); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    //YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id      = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp            = 2,//12, // 0.24, // 0.31, // 0.45
                .Ki            = 0.0f,
                .Kd            = 0,//0.02,//0.01,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement,
                .IntegralLimit = 20, 
                .MaxOut = 1000,
            },
            .speed_PID = {
                .Kp            = 2000,//6000,//10000, //11000,
                .Ki            = 0,    // 0
                .Kd            = 0,//5, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 3000,
                .MaxOut        = 10000, // 20000
            },
            .mpc_speed_PID = {
                .Kp            = 5000,
                .Ki            = 0,
                .Kd            = 0,
                .MaxOut        = 10000,
            },
            .other_angle_feedback_ptr   = &gimbal_IMU_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET], // yaw反馈角度值
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr   = &gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],
            .mpc_speed_ref_ptr          = &yaw_vel_feedforward,
            .current_feedforward_ptr    = &yaw_tor_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
            .mpc_type              = MPC_SPEED_PARALLEL,
        },
        .motor_type = GM6020};
    yaw_motor   = DJIMotorInit(&yaw_config);

    Motor_Init_Config_s pitch_motor_config = {//DM4310
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x02,
            .rx_id = 0x12,
        },
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = 50,
                .Ki = 100,
                .Kd = 0.0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 3,
                .MaxOut = 30,
            },
            .speed_PID = {
                .Kp = 1,
                .Ki = 0.8,
                .Kd = 0.001,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 1,
                .MaxOut = 4,
            },
            .mpc_speed_PID = {
                .Kp = 0.0f,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 0,
            },
             .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch反馈弧度制
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET],
            // .speed_feedforward_ptr = &pitch_vel_feedforward,
            .current_feedforward_ptr = &pitch_tor_feedforward,
            .mpc_speed_ref_ptr = &pitch_vel_feedforward,

        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            .feedforward_flag      = CURRENT_FEEDFORWARD,
            .mpc_type              = MPC_SPEED_PARALLEL,
            .control_range = {
                .P_max = 12.5,
                .V_max = 30,
                .T_max = 10,
            },
        },
    };
    pitch_motor = DMMotorInit(&pitch_motor_config);
#endif
#ifdef CHASSIS_BOARD
    Motor_Init_Config_s big_yaw_motor_config = {//DM6006
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x01,
            .rx_id = 0x11,
        },
        .motor_type = DM_Motor,
        .controller_param_init_config ={
            .angle_PID = {
                .Kp = 5,
                .Ki = 0.1,
                .Kd = 0.5,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 3,
                .MaxOut = 30,
            },
            .speed_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit ,
                .IntegralLimit = 0,
                .MaxOut = 10,
            },
            .speed_feedforward_ptr = &base_yaw_vel_feedforward,
        },
        .controller_setting_init_config ={
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD,
            .control_range = {
                .P_max = 12.5663704,
                .V_max = 45,
                .T_max = 12,
            },
            
        },
    };
    big_yaw_motor = DMMotorInit(&big_yaw_motor_config);
#endif
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
float pitch_target,big_yaw_target;
float big_yaw_kp = 5;
float big_yaw_fetch_angle;
int32_t big_yaw_fetch_angle_single;
// base_yaw_tilt_s base_yaw_tilt;
uint8_t last_NUC_detect = 0;
gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE;
// base_yaw_tilt_s* GetBaseYawTilt(void)
// {
//     static float gimbal_yaw_pitch,gimbal_yaw_roll;
//     static float gimbal_yaw_tilt_direction, gimbal_yaw_tilt_alpha;
//     gimbal_yaw_pitch            =  - gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET] - (pitch_motor->measure.pos - PITCH_HORIZON_POS);
//     gimbal_yaw_roll             = gimbal_IMU_data->output.INS_angle[INS_ROLL_ADDRESS_OFFSET];
//     gimbal_yaw_tilt_direction   = atan2f(arm_sin_f32(gimbal_yaw_roll)*arm_cos_f32(gimbal_yaw_pitch), -arm_sin_f32(gimbal_yaw_pitch));
//     gimbal_yaw_tilt_alpha       = atan2f(sqrtf(arm_sin_f32(gimbal_yaw_pitch)*arm_sin_f32(gimbal_yaw_pitch) + arm_sin_f32(gimbal_yaw_roll)*arm_sin_f32(gimbal_yaw_roll)*arm_cos_f32(gimbal_yaw_pitch)*arm_cos_f32(gimbal_yaw_pitch)), arm_cos_f32(gimbal_yaw_roll)*arm_cos_f32(gimbal_yaw_pitch));
//     base_yaw_tilt.direction     = RAD_2_DEGREE * (gimbal_yaw_tilt_direction - (float)(yaw_motor->measure.ecd - YAW_BIG_YAW_ALIGN_ECD) * 2 * PI / 8192);
//     base_yaw_tilt.alpha         = gimbal_yaw_tilt_alpha;
//     return &base_yaw_tilt;
// }

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // if (yaw_motor == NULL || pitch_motor == NULL || big_yaw_motor == NULL) {
    //     return;
    // }

    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

#ifdef GIMBAL_BOARD
    pitch_tor_feedforward_ori = 0.8 * tan(0.82 - abs(gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET]));
    if(gimbal_cmd_recv.control_type == NUC_CONTROL)
    {
        pitch_vel_feedforward   = NUC_cmd.pitch_vel;
        pitch_tor_feedforward   = pitch_tor_feedforward_ori;
        pitch_motor->motor_controller.mpc_speed_PID.MaxOut = 0;
        yaw_vel_feedforward     = -NUC_cmd.yaw_vel;
        yaw_tor_feedforward     = yaw_tor_k * NUC_cmd.yaw_acc;
        yaw_motor->motor_controller.mpc_speed_PID.MaxOut = 5000;
    }
    else
    {
        pitch_vel_feedforward   = 0;
        pitch_tor_feedforward   = pitch_tor_feedforward_ori;
        pitch_motor->motor_controller.mpc_speed_PID.MaxOut = 0;
        yaw_vel_feedforward     = 0;
        yaw_tor_feedforward     = 0;
        yaw_motor->motor_controller.mpc_speed_PID.MaxOut = 0;
    }
    if(NUC_cmd.shoot != 0 && last_NUC_detect == 0)
    {
        yaw_motor->motor_controller.angle_PID.Iout = 0;
        yaw_motor->motor_controller.speed_PID.Iout = 0;
    }
    switch (gimbal_cmd_recv.gimbal_mode) {
        // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            DMMotorStop(pitch_motor);
            // DMMotorStop(big_yaw_motor);
            // big_yaw_motor->motor_controller.angle_PID.Iout  = 0;
            yaw_motor->motor_controller.angle_PID.Iout      = 0;
            pitch_motor->motor_controller.angle_PID.Iout    = 0;
            pitch_motor->motor_controller.speed_PID.Iout    = 0;
            break;
        //使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE:
        case GIMBAL_SEARCH_MODE:
            DJIMotorEnable(yaw_motor);
            DMMotorEnable1(pitch_motor);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            pitch_motor->motor_controller.pid_ref = gimbal_cmd_recv.pitch;
            break;
        default:
            break;
    }
    last_NUC_detect = NUC_cmd.shoot;
    if(yaw_motor->dt < 0.1) gimbal_feedback_data.gimbal_online = 1;
    else gimbal_feedback_data.gimbal_online = 0;
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;
    gimbal_feedback_data.pitch_motor_pos              = pitch_motor->measure.pos;
    gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;
#endif

#ifdef CHASSIS_BOARD
    if(last_gimbal_mode != GIMBAL_SEARCH_MODE && gimbal_cmd_recv.gimbal_mode == GIMBAL_SEARCH_MODE)
        big_yaw_target = big_yaw_motor->measure.total_pos;
    if(gimbal_cmd_recv.gimbal_mode == GIMBAL_SEARCH_MODE)
    {
        if (SearchMotorIsOnline(big_yaw_motor)) {
            big_yaw_target += 1.0f * 0.001f;
            big_yaw_target += speed_measure.real_wz * 0.001f;
        }
        big_yaw_kp = 10.0f;
    }
    else
    {
        big_yaw_target = big_yaw_motor->measure.total_pos + 1 * comm_cmd_data.yaw_diff;
        big_yaw_kp = 3.0f + (fabsf(comm_cmd_data.yaw_diff) > PI / 3.0f ? PI / 3.0f : fabsf(comm_cmd_data.yaw_diff)) / (PI / 3.0f) * 12.0f;
    }

    base_yaw_vel_feedforward = speed_measure.real_wz;
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
        // 停止
        case GIMBAL_ZERO_FORCE:
            DMMotorStop(big_yaw_motor);
            big_yaw_motor->motor_controller.angle_PID.Iout  = 0;
            break;
        //使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE:
        case GIMBAL_SEARCH_MODE:
            DMMotorEnable1(big_yaw_motor);
            big_yaw_motor->motor_controller.angle_PID.Kp = big_yaw_kp;
            big_yaw_motor->motor_controller.pid_ref = big_yaw_target;
            break;
        default:
            break;
    }
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...
    
    // 设置反馈数据,主要是imu和yaw的ecd
    
    big_yaw_fetch_angle = big_yaw_motor->measure.pos * RAD_2_DEGREE;
    big_yaw_fetch_angle_single = ((int32_t)big_yaw_fetch_angle + 180) % 360;
    if(big_yaw_fetch_angle_single < 0) big_yaw_fetch_angle_single += 360;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)big_yaw_fetch_angle_single; // 推送消息
    
    // gimbal_feedback_data.base_yaw_tilt                = GetBaseYawTilt();
#endif
    last_gimbal_mode = gimbal_cmd_recv.gimbal_mode;
    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
