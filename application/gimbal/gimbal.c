#include "stdio.h"

#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "bmi088.h"
#include "referee_UI.h"

#include "DMmotor.h"

static INS_Instance *gimbal_IMU_data; // 云台IMU数据
DJIMotorInstance *yaw_motor;
DMMotorInstance *pitch_motor, *big_yaw_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

void GimbalInit()
{
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
                .Kp            = 12, // 0.24, // 0.31, // 0.45
                .Ki            = 0.2,
                .Kd            = 0.02,//0.01,
                .DeadBand      = 0.0f,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement,
                .IntegralLimit = 20, 

                .MaxOut = 1000,
            },
            .speed_PID = {
                .Kp            = 250,//6000,//10000, //11000,
                .Ki            = 0,    // 0
                .Kd            = 8,//5, // 30
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit ,//| PID_Derivative_On_Measurement | PID_OutputFilter,
                .IntegralLimit = 5000,
                .MaxOut        = 20000, // 20000
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle_deg[INS_YAW_ADDRESS_OFFSET], // yaw反馈角度值
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_YAW_ADDRESS_OFFSET],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = GM6020};
    yaw_motor   = DJIMotorInit(&yaw_config);
    // PITCH
    // Motor_Init_Config_s pitch_config = {
    //     .can_init_config = {    
    //         .can_handle = &hcan2,
    //         .tx_id      = 4,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp            = 15, // 35, // 40, // 10
    //             .Ki            = 0,
    //             .Kd            = 0,
    //             .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 8,
    //             .MaxOut        = 20
    //         },
    //         .speed_PID = {
    //             .Kp            = 4000,//5000, // 10500, // 13000,//10500,  // 10500
    //             .Ki            = 0,    // 12000, // 10000, // 10000
    //             .Kd            = 0.5,    // 0
    //             .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
    //             .IntegralLimit = 5000,
    //             .MaxOut        = 16000,
    //         },
    //         .other_angle_feedback_ptr = &gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET], // pitch反馈弧度制
    //         // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
    //         .other_speed_feedback_ptr = &gimbal_IMU_data->INS_data.INS_gyro[INS_PITCH_ADDRESS_OFFSET],
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = OTHER_FEED,
    //         .speed_feedback_source = OTHER_FEED,
    //         .outer_loop_type       = ANGLE_LOOP,
    //         .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
    //         .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
    //     },
    //     .motor_type = GM6020
    // };
    
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    // pitch_motor = DJIMotorInit(&pitch_config);

    Motor_Init_Config_s pitch_motor_config = {//DM4310
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x02,
            .rx_id = 0x12,
        },
        .motor_type = DM_Motor,
        .controller_setting_init_config = {
            .control_range = {
                .P_max = 12.5,
                .V_max = 30,
                .T_max = 10,
            },
        },
    };
    pitch_motor = DMMotorInit(&pitch_motor_config);

    Motor_Init_Config_s big_yaw_motor_config = {//DM6006
        .can_init_config = {
            .can_handle = &hcan1,
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
                .IntegralLimit = 5,
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
        },
        .controller_setting_init_config ={
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
            .control_range = {
                .P_max = 12.5,
                .V_max = 45,
                .T_max = 12,
            },
        },
    };
    big_yaw_motor = DMMotorInit(&big_yaw_motor_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}
float pitch_target,big_yaw_target;
float big_yaw_kp = 5;
float big_yaw_fetch_angle;
int32_t big_yaw_fetch_angle_single;
/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    big_yaw_target = big_yaw_motor->measure.pos + 1 * (float)(yaw_motor->measure.ecd - YAW_BIG_YAW_ALIGN_ECD) * 2 * PI / 8192;
    big_yaw_kp = 1 + (float)abs(yaw_motor->measure.ecd - YAW_BIG_YAW_ALIGN_ECD) / abs(YAW_LEFT_LIMIT_ECD - YAW_RIGHT_LIMIT_ECD) * 2 * 15;
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode) {
        // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            // DJIMotorStop(pitch_motor);
            DMMotorStop(pitch_motor);
            DMMotorStop(big_yaw_motor);
            break;
        //使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE: // 后续只保留此模式
            DJIMotorEnable(yaw_motor);
           //DJIMotorStop(yaw_motor);
            DMMotorEnable1(pitch_motor);
            DMMotorEnable1(big_yaw_motor);
           // DJIMotorStop(pitch_motor);
            //DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            //DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
            //DJIMotorOuterLoop(yaw_motor, SPEED_LOOP);
            //DJIMotorEnable(yaw_motor);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
            pitch_target = pitch_motor->measure.pos - (gimbal_cmd_recv.pitch - gimbal_IMU_data->output.INS_angle[INS_PITCH_ADDRESS_OFFSET]);

            pitch_motor->ctrl.kp_set = 30;
            pitch_motor->ctrl.kd_set = 2;
            if(pitch_target < PITCH_UP_POS) pitch_target = PITCH_UP_POS;        //todo:待修改为单独函数并判断电机转向（或许无意义）
            if(pitch_target >PITCH_DOWN_POS) pitch_target = PITCH_DOWN_POS;
            pitch_motor->ctrl.pos_set = pitch_target;

            // big_yaw_motor->ctrl.kp_set = 5;
            // big_yaw_motor->ctrl.kd_set = 1;
            // big_yaw_motor->ctrl.pos_set = big_yaw_target;
            // big_yaw_motor->motor_controller.angle_PID.Kp = big_yaw_kp;
            big_yaw_motor->motor_controller.pid_ref = big_yaw_target;
            break;
        default:
            break;
    }
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data              = gimbal_IMU_data;

    big_yaw_fetch_angle = big_yaw_motor->measure.pos * RAD_2_DEGREE;
    big_yaw_fetch_angle_single = ((int32_t)big_yaw_fetch_angle + 180) % 360;
    if(big_yaw_fetch_angle_single < 0) big_yaw_fetch_angle_single += 360;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)big_yaw_fetch_angle_single; // 推送消息
    gimbal_feedback_data.yaw_ecd                      = yaw_motor->measure.ecd;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}