#include "comm.h"

#include <string.h>
#include "message_center.h"
#include "rm_referee.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "dji_motor.h"
#include "crc_ref.h"

static Subscriber_t *chassis_monitor_sub;                   // 用于订阅底盘控制命令
static Subscriber_t *gimbal_monitor_sub;                  // cmd控制消息订阅者
static Subscriber_t *shoot_monitor_sub;

static Chassis_Ctrl_Cmd_s chassis_cmd_monitor;         // 底盘接收到的控制命令
static Gimbal_Ctrl_Cmd_s gimbal_cmd_monitor;         // 来自cmd的控制信息
static Shoot_Ctrl_Cmd_s shoot_cmd_monitor;         // 来自cmd的发射控制信息

comm_cmd_t comm_cmd_data = {};
comm_upload_t comm_upload_data = {};
static referee_upload_A_t referee_upload_A = {};
static referee_upload_B_t referee_upload_B = {};
static referee_upload_C_t referee_upload_C = {};
static referee_cmd_t referee_cmd = {};

extern referee_info_t referee_info;           // 裁判系统数据
extern int32_t load_count;
extern DJIMotorInstance *yaw_motor;

static CommInstance_t comm_instance;

static void CommRestartRxInternal(void);
static void CommHandleOffline(CommInstance_t *comm);
static void CommFeedWatchdogInternal(void);
#if defined(GIMBAL_BOARD)
static void CommRecieve(const uint8_t *buf)
{
    if (buf[0] == COMM_HEADER && Verify_CRC8_Check_Sum((uint8_t *)buf, COMM_UPDATA_SIZE))
    {
        memcpy(&comm_upload_data, buf, COMM_UPDATA_SIZE);
        referee_info.ShootData.bullet_speed                         = comm_upload_data.bullet_speed;
        referee_info.GameRobotStatus.shooter_barrel_cooling_value   = comm_upload_data.shooter_heat_cooling_rate;
        referee_info.PowerHeatData.shooter_17mm_barrel_heat         = comm_upload_data.shooter_referee_heat;
        referee_info.GameRobotStatus.shooter_barrel_heat_limit      = comm_upload_data.shooter_cooling_limit;
        load_count = comm_upload_data.load_count;
        switch (comm_upload_data.referee_upload[0])
        {
            case 0x01:
                memcpy(&referee_upload_A, comm_upload_data.referee_upload, REFEREE_UPLOAD_A);
                referee_info.referee_id.Robot_ID            = referee_upload_A.Robot_ID;
                referee_info.GameRobotStatus.robot_id       = referee_upload_A.Robot_ID;
                referee_info.GameState.game_progress        = referee_upload_A.game_progress;
                referee_info.GameState.stage_remain_time    = referee_upload_A.stage_remain_time;
                referee_info.EventData.event_type           = referee_upload_A.event_data;
                referee_info.GameRobotStatus.remain_HP      = referee_upload_A.current_hp;
                referee_info.GameRobotStatus.max_HP         = referee_upload_A.maximum_hp;
                referee_info.RobotHurt.armor_id             = referee_upload_A.armor_id;
                referee_info.RobotHurt.hurt_type            = referee_upload_A.hp_deduction_reason;
                referee_info.SentryInfo.sentry_info_2       &= (uint16_t)~0x7001u;
                referee_info.SentryInfo.sentry_info_2       |= (uint16_t)(referee_upload_A.disengaged_state & 0x01u);
                referee_info.SentryInfo.sentry_info_2       |= (uint16_t)((referee_upload_A.current_state & 0x03u) << 12);
                referee_info.SentryInfo.sentry_info_2       |= (uint16_t)((referee_upload_A.ally_power_rune_state & 0x01u) << 14);
                referee_info.Rfid_Status.rfid_status        = referee_upload_A.rfid_status;
                referee_info.GameRobotPos.x                 = referee_upload_A.init_sentry_position_x;
                referee_info.GameRobotPos.y                 = referee_upload_A.init_sentry_position_y;
                break;
            case 0x02:
                memcpy(&referee_upload_B, comm_upload_data.referee_upload, REFEREE_UPLOAD_B);
                referee_info.GroundRobotPosition.hero_x         = referee_upload_B.hero_x;
                referee_info.GroundRobotPosition.hero_y         = referee_upload_B.hero_y;
                referee_info.GroundRobotPosition.engineer_x     = referee_upload_B.engineer_x;
                referee_info.GroundRobotPosition.engineer_y     = referee_upload_B.engineer_y;
                referee_info.GroundRobotPosition.standard_3_x   = referee_upload_B.standard_3_x;
                referee_info.GroundRobotPosition.standard_3_y   = referee_upload_B.standard_3_y;
                break;
            case 0x03:
                memcpy(&referee_upload_C, comm_upload_data.referee_upload, REFEREE_UPLOAD_C);
                referee_info.GroundRobotPosition.standard_4_x   = referee_upload_C.standard_4_x;
                referee_info.GroundRobotPosition.standard_4_y   = referee_upload_C.standard_4_y;
                referee_info.GameRobotHP.ally_1_robot_HP        = referee_upload_C.ally_1_robot_HP;
                referee_info.GameRobotHP.ally_2_robot_HP        = referee_upload_C.ally_2_robot_HP;
                referee_info.GameRobotHP.ally_3_robot_HP        = referee_upload_C.ally_3_robot_HP;
                referee_info.GameRobotHP.ally_4_robot_HP        = referee_upload_C.ally_4_robot_HP;
                referee_info.GameRobotHP.ally_outpost_HP        = referee_upload_C.ally_outpost_HP;
                referee_info.GameRobotHP.ally_base_HP           = referee_upload_C.ally_base_HP;
                break;
            default:
                break;
        }
    }
}

static void CommSend(void)
{
    SubGetMessage(chassis_monitor_sub, &chassis_cmd_monitor);
    SubGetMessage(gimbal_monitor_sub, &gimbal_cmd_monitor);
    SubGetMessage(shoot_monitor_sub, &shoot_cmd_monitor);
    
    comm_cmd_data.head = COMM_HEADER;
    comm_cmd_data.vx = chassis_cmd_monitor.vx;
    comm_cmd_data.vy = chassis_cmd_monitor.vy;
    comm_cmd_data.chassis_mode = chassis_cmd_monitor.chassis_mode;
    comm_cmd_data.chassis_rotate_speed = chassis_cmd_monitor.chassis_rotate_speed;
    comm_cmd_data.yaw_diff = (float)(yaw_motor->measure.ecd - YAW_BIG_YAW_ALIGN_ECD) * 2 * PI / 8192;
    referee_cmd.cmdid = 0x01;
    referee_cmd.ally_power_rune_active = 0;
    memcpy(comm_cmd_data.referee_cmd, &referee_cmd, REFEREE_CMD_SIZE);
    Append_CRC8_Check_Sum((uint8_t *)&comm_cmd_data, COMM_CMD_SIZE);
    CommSendDMA(&comm_upload_data, COMM_UPDATA_SIZE);
}
#endif

#if defined(CHASSIS_BOARD)
static void CommRecieve(const uint8_t *buf)
{
    if(buf[0] == COMM_HEADER && Verify_CRC8_Check_Sum((uint8_t *)buf,COMM_CMD_SIZE))
    {
        memcpy(&comm_cmd_data, buf, COMM_CMD_SIZE);
        switch(comm_cmd_data.referee_cmd[0])
        {
            case 0x01:
                memcpy(&referee_cmd, comm_cmd_data.referee_cmd, REFEREE_CMD_SIZE);
                break;
            default:
                break;
        }
    }
}
uint8_t commSendCount = 0;
static void CommSend(void)
{
    SubGetMessage(chassis_monitor_sub, &chassis_cmd_monitor);
    SubGetMessage(gimbal_monitor_sub, &gimbal_cmd_monitor);
    SubGetMessage(shoot_monitor_sub, &shoot_cmd_monitor);

    comm_upload_data.head = COMM_HEADER;
    comm_upload_data.real_vx = 0.0f;
    comm_upload_data.real_vy = 0.0f;
    comm_upload_data.bullet_speed = shoot_cmd_monitor.bullet_speed;
    comm_upload_data.load_count = (uint16_t)load_count;
    comm_upload_data.shooter_heat_cooling_rate = shoot_cmd_monitor.shooter_heat_cooling_rate;
    comm_upload_data.shooter_referee_heat = shoot_cmd_monitor.shooter_referee_heat;
    comm_upload_data.shooter_cooling_limit = shoot_cmd_monitor.shooter_cooling_limit;
    comm_upload_data.cap_voltage = 0.0f;
    memset(comm_upload_data.reserve, 0, sizeof(comm_upload_data.reserve));

    if (commSendCount >= 3)
        commSendCount = 0;

    switch(commSendCount)
    {
        case 0:
            memset(&referee_upload_A, 0, sizeof(referee_upload_A));
            referee_upload_A.cmdid = 0x01;
            referee_upload_A.Robot_ID = referee_info.referee_id.Robot_ID;
            referee_upload_A.game_progress = referee_info.GameState.game_progress;
            referee_upload_A.stage_remain_time = referee_info.GameState.stage_remain_time;
            referee_upload_A.event_data = referee_info.EventData.event_type;
            referee_upload_A.current_hp = referee_info.GameRobotStatus.remain_HP;
            referee_upload_A.maximum_hp = referee_info.GameRobotStatus.max_HP;
            referee_upload_A.armor_id = referee_info.RobotHurt.armor_id;
            referee_upload_A.hp_deduction_reason = referee_info.RobotHurt.hurt_type;
            referee_upload_A.disengaged_state = (uint8_t)(referee_info.SentryInfo.sentry_info_2 & 0x01u);
            referee_upload_A.current_state = (uint8_t)((referee_info.SentryInfo.sentry_info_2 >> 12) & 0x03u);
            referee_upload_A.ally_power_rune_state = (uint8_t)((referee_info.SentryInfo.sentry_info_2 >> 14) & 0x01u);
            referee_upload_A.rfid_status = referee_info.Rfid_Status.rfid_status;
            referee_upload_A.init_sentry_position_x = referee_info.GameRobotPos.x;
            referee_upload_A.init_sentry_position_y = referee_info.GameRobotPos.y;
            memcpy(comm_upload_data.referee_upload, &referee_upload_A, REFEREE_UPLOAD_A);
            break;
        case 1:
            memset(&referee_upload_B, 0, sizeof(referee_upload_B));
            referee_upload_B.cmdid = 0x02;
            referee_upload_B.hero_x = referee_info.GroundRobotPosition.hero_x;
            referee_upload_B.hero_y = referee_info.GroundRobotPosition.hero_y;
            referee_upload_B.engineer_x = referee_info.GroundRobotPosition.engineer_x;
            referee_upload_B.engineer_y = referee_info.GroundRobotPosition.engineer_y;
            referee_upload_B.standard_3_x = referee_info.GroundRobotPosition.standard_3_x;
            referee_upload_B.standard_3_y = referee_info.GroundRobotPosition.standard_3_y;
            memcpy(comm_upload_data.referee_upload, &referee_upload_B, REFEREE_UPLOAD_B);
            break;
        case 2:
        default:
            memset(&referee_upload_C, 0, sizeof(referee_upload_C));
            referee_upload_C.cmdid = 0x03;
            referee_upload_C.standard_4_x = referee_info.GroundRobotPosition.standard_4_x;
            referee_upload_C.standard_4_y = referee_info.GroundRobotPosition.standard_4_y;
            referee_upload_C.ally_1_robot_HP = referee_info.GameRobotHP.ally_1_robot_HP;
            referee_upload_C.ally_2_robot_HP = referee_info.GameRobotHP.ally_2_robot_HP;
            referee_upload_C.ally_3_robot_HP = referee_info.GameRobotHP.ally_3_robot_HP;
            referee_upload_C.ally_4_robot_HP = referee_info.GameRobotHP.ally_4_robot_HP;
            referee_upload_C.ally_outpost_HP = referee_info.GameRobotHP.ally_outpost_HP;
            referee_upload_C.ally_base_HP = referee_info.GameRobotHP.ally_base_HP;
            memcpy(comm_upload_data.referee_upload, &referee_upload_C, REFEREE_UPLOAD_C);
            break;
    }
    Append_CRC8_Check_Sum((uint8_t *)&comm_upload_data, COMM_UPDATA_SIZE);
    commSendCount++;
    if (commSendCount >= 3)
        commSendCount = 0;
    CommSendDMA(&comm_upload_data, COMM_UPDATA_SIZE);
}
#endif

static uint32_t CommGetTimelineMs(void)
{
    return (uint32_t)DWT_GetTimeline_ms();
}

static void CommUartRxCallback(void)
{

    if (comm_instance.uart == NULL) {
        return;
    }

    memcpy(comm_instance.rx_buf, comm_instance.uart->recv_buff, sizeof(comm_instance.rx_buf));
    comm_instance.rx_updated = 1;
    CommFeedWatchdogInternal();

    if (comm_instance.rx_cb != NULL) {
        comm_instance.rx_cb(comm_instance.rx_buf);
    }
}

static void CommRestartRxInternal(void)
{
    if (comm_instance.uart != NULL) {
        USARTServiceInit(comm_instance.uart);
    }
}

static void CommHandleOffline(CommInstance_t *comm)
{
    memset(comm->rx_buf, 0, sizeof(comm->rx_buf));
    comm->rx_updated = 0;
    comm->online = 0;

    if (comm->uart != NULL) {
        USARTServiceInit(comm->uart);
    }
}

void CommInit(void)
{
    USART_Init_Config_s uart_config;

    if (comm_instance.uart != NULL) {
        return;
    }

    memset(&comm_instance, 0, sizeof(comm_instance));
    memset(&uart_config, 0, sizeof(uart_config));

    uart_config.recv_buff_size = COMM_RX_BUF_SIZE;
    uart_config.usart_handle = COMM_UART_HANDLE;
    uart_config.module_callback = CommUartRxCallback;
    comm_instance.uart = USARTRegister(&uart_config);
    comm_instance.rx_cb = CommRecieve;

    chassis_monitor_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    gimbal_monitor_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    shoot_monitor_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    CommFeedWatchdogInternal();
}

uint8_t CommSendDMA(const uint8_t *data, uint16_t len)
{
    if (comm_instance.uart == NULL || data == NULL) {
        return 0;
    }

    if (len == 0 || len > COMM_TX_BUF_SIZE) {
        return 0;
    }

    if (COMM_UART_HANDLE->gState != HAL_UART_STATE_READY) {
        return 0;
    }

    memcpy(comm_instance.tx_buf, data, len);
    USARTSend(comm_instance.uart, comm_instance.tx_buf, len, USART_TRANSFER_DMA);
    return 1;
}

const uint8_t *CommGetRxBuffer(void)
{
    return comm_instance.rx_buf;
}

uint8_t CommHasNewData(void)
{
    return comm_instance.rx_updated;
}

void CommClearRxFlag(void)
{
    comm_instance.rx_updated = 0;
}

void CommSetRxCallback(CommRxCallback_t cb)
{
    comm_instance.rx_cb = cb;
}

static void CommFeedWatchdogInternal(void)
{
    comm_instance.last_feed_ms = CommGetTimelineMs();
    comm_instance.online = 1;
}

uint8_t CommIsOnline(void)
{
    uint32_t now_ms;

    if (comm_instance.uart == NULL) {
        return 0;
    }

    if (!comm_instance.online) {
        return 0;
    }

    now_ms = CommGetTimelineMs();
    if ((now_ms - comm_instance.last_feed_ms) > COMM_WATCHDOG_TIMEOUT_MS) {
        CommHandleOffline(&comm_instance);
        return 0;
    }

    return 1;
}

void CommRestartRx(void)
{
    CommRestartRxInternal();
}
