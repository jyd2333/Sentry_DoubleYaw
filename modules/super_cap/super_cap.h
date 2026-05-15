#ifndef SUPER_CUP_H
#define SUPER_CUP_H

#include "bsp_can.h"
#include "daemon.h"

#define SUPERCAP_MAX_VOLTAGE 24.0f
#define SUPERCAP_MIN_VOLTAGE 16.0f
#define SUPERCAP_HIGHER_THRESHOLD_ENERGY 90.0f
#define SUPERCAP_LOWER_THRESHOLD_ENERGY 10.0f

#define SOFTWARE_UVP_VCAP 6.0f  //最小电容组放电截至电压，这里设定为6V。
#define SOFTWARE_OVP_VCAP 20.0f //最大电容组充电电压，电容组过压保护
#define SUPERCAP_AVAILABLE_VOLTAGE  (SOFTWARE_OVP_VCAP - SOFTWARE_UVP_VCAP) //电容组可用电压范围，用于粗略计算电容组的能量百分比

typedef enum
{
    SUPERCAP_DISABLE = 0,
    SUPERCAP_ENABLE = 1,
}SuperCap_Enable_Flag_e;

#pragma pack(1)
typedef struct 
{
    SuperCap_Enable_Flag_e enable_flag;
    uint8_t charge_flag;
    uint8_t power_limit;
    uint8_t reserved[5];
}SuperCap_Tx_Data_s;

typedef enum
{
  SUPERCAP_STATE_DISCHARGE = 0,
  SUPERCAP_STATE_CHARGE = 1,
  SUPERCAP_STATE_WAIT = 2,
  SUPERCAP_STATE_SOFRSTART_PROTECTION = 3,
  SUPERCAP_STATE_OVER_LOAD_PROTECTTION = 4,
  SUPERCAP_STATE_OVP_BAT_PROTECTION = 5,
  SUPERCAP_STATE_UVP_BAT_PROTECTION = 6,
  SUPERCAP_STATE_UVP_CAP_PROTECTION = 7,
  SUPERCAP_STATE_OTP_PROTECTION = 8,
  SUPERCAP_STATE_BOOM_PROTECTION = 9,
  SUPERCAP_STATE_CAN_OFFLINE = 10,
} SuperCap_State_e;

typedef struct 
{
    uint8_t ready_flag;//超级电容【可用标志】：1为可用，0为不可用
    SuperCap_State_e SuperCapState;//超级电容【状态标志】：各个状态对应的状态码查看E_SuperCapState枚举。
    uint8_t energy;//超级电容可用能量：0-100%(仅电压计算)
    uint16_t chassis_real_power; //底盘功率，将0~512W映射到uint16_t范围
    uint8_t bat_voltage; //通过超级电容监控电池电压*10，
    uint8_t bat_power;
    uint8_t reserved;
}SuperCap_Rx_Data_s;

#pragma pack()

typedef struct 
{
    CAN_Init_Config_s can_config;
    Daemon_Init_Config_s daemon_config;
}SuperCap_Init_Config_s;

typedef struct
{
    CANInstance *can_instance;
    DaemonInstance *daemon_instance;
    SuperCap_Tx_Data_s tx_data;
    SuperCap_Rx_Data_s rx_data;
    float chassis_real_power;       //映射回浮点型
    float real_energy;              //换算至真实能量的百分比
}SuperCapInstance;

SuperCapInstance *SuperCapRegister(SuperCap_Init_Config_s *config);
void SuperCapTask(void);
void SuperCapEnable(SuperCapInstance *instance);
void SuperCapDisable(SuperCapInstance *instance);
void SuperCapSetPowerLimit(SuperCapInstance *instance, uint8_t power_limit);
uint8_t SuperCapIsOnline(SuperCapInstance *instance);
float SuperCapGetChassisRealPower(SuperCapInstance *instance);
uint8_t SuperCapGetCapEnergy(SuperCapInstance *instance);
uint8_t SuperCapGetReadyFlag(SuperCapInstance *instance);

#endif