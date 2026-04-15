#ifndef COMM_H
#define COMM_H

#include <stdint.h>

#include "usart.h"
#include "robot_def.h"

#define COMM_UART_HANDLE    (&huart1)
#define COMM_RX_BUF_SIZE    64
#define COMM_TX_BUF_SIZE    64
#define COMM_WATCHDOG_TIMEOUT_MS 100U

#define COMM_CMD_SIZE       sizeof(comm_cmd_t)
#define COMM_UPDATA_SIZE    sizeof(comm_upload_t)
#define REFEREE_UPLOAD_A    sizeof(referee_upload_A_t)
#define REFEREE_UPLOAD_B    sizeof(referee_upload_B_t)
#define REFEREE_UPLOAD_C    sizeof(referee_upload_C_t)
#define REFEREE_CMD_SIZE    sizeof(referee_cmd_t)

typedef void (*CommRxCallback_t)(const uint8_t *buf);

#pragma pack(1) // 压缩结构体,取消字节对齐
typedef struct
{
    uint8_t head;
    float vx;                        // 前进方向速度
    float vy;                        // 横移方向速度
    chassis_mode_e chassis_mode;
    uint8_t chassis_rotate_speed;
    uint16_t yaw_diff_ECD;
    uint8_t referee_cmd[32];
    uint8_t reserve[1];
    uint8_t crc8;
} comm_cmd_t;

typedef struct
{
    uint8_t head;
    float real_vx;
    float real_vy;
    float real_wz;
    float bullet_speed;
	uint16_t load_count;
    float cap_voltage;
    uint8_t referee_upload[32];
    // uint8_t reserve[1];
    uint8_t crc8;
} comm_upload_t;

typedef struct
{
    uint8_t cmdid;
    uint16_t Robot_ID;
    uint8_t game_progress;
  	uint16_t stage_remain_time;
	uint32_t event_data;
	uint16_t current_hp;
  	uint16_t maximum_hp;
  	uint8_t armor_id;
  	uint8_t hp_deduction_reason;
  	uint8_t disengaged_state;
  	uint8_t current_state;
  	uint8_t ally_power_rune_state; 
	uint32_t rfid_status;
    float init_sentry_position_x;
  	float init_sentry_position_y;
    uint8_t reserve[1];
} referee_upload_A_t;

typedef struct
{
    uint8_t cmdid;
    float hero_x;
  	float hero_y;
  	float engineer_x;
  	float engineer_y;
  	float standard_3_x;
  	float standard_3_y;
  	uint8_t reserve[7];
} referee_upload_B_t;

typedef struct
{
    uint8_t cmdid;
    float standard_4_x;
  	float standard_4_y;
	uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
	uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
    uint8_t reserve[11];
} referee_upload_C_t;

typedef struct
{
    uint8_t cmdid;
    uint8_t ally_power_rune_active;
    uint8_t reserve[30];
}referee_cmd_t;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

void CommInit(void);
uint8_t CommSendDMA(const uint8_t *data, uint16_t len);
const uint8_t *CommGetRxBuffer(void);
uint8_t CommHasNewData(void);
void CommClearRxFlag(void);
void CommSetRxCallback(CommRxCallback_t cb);
uint8_t CommIsOnline(void);
void CommRestartRx(void);

#endif
