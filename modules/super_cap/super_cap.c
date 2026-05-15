#include <memory.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "super_cap.h"

static SuperCapInstance *supercap = NULL;

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 电容能量换算
 *
 *
 */
static float CapEnergyTransform(uint8_t raw_energy)
{
    float raw_energy_f = (float)raw_energy;
    float vcap;
    float uvp_square = SOFTWARE_UVP_VCAP * SOFTWARE_UVP_VCAP;
    float ovp_square = SOFTWARE_OVP_VCAP * SOFTWARE_OVP_VCAP;
    float energy;

    if (raw_energy_f < 0.0f) {
        raw_energy_f = 0.0f;
    } else if (raw_energy_f > 100.0f) {
        raw_energy_f = 100.0f;
    }

    vcap = SOFTWARE_UVP_VCAP + raw_energy_f * SUPERCAP_AVAILABLE_VOLTAGE / 100.0f;
    energy = (vcap * vcap - uvp_square) * 100.0f / (ovp_square - uvp_square);

    if (energy < 0.0f) {
        energy = 0.0f;
    } else if (energy > 100.0f) {
        energy = 100.0f;
    }

    return energy;
}

void SuperCapEnable(SuperCapInstance *instance)
{
    instance->tx_data.enable_flag = SUPERCAP_ENABLE;
}

void SuperCapDisable(SuperCapInstance *instance)
{
    instance->tx_data.enable_flag = SUPERCAP_DISABLE;
}

void SuperCapSetPowerLimit(SuperCapInstance *instance, uint8_t power_limit)
{
    instance->tx_data.power_limit = power_limit;
}

static void SuperCapRxCallback(CANInstance *instance)
{
    DaemonReload(supercap->daemon_instance);
    memcpy(&supercap->rx_data, instance->rx_buff, sizeof(SuperCap_Rx_Data_s));
    supercap->chassis_real_power = uint_to_float(supercap->rx_data.chassis_real_power, 0, 512, 16);
    supercap->real_energy = CapEnergyTransform(supercap->rx_data.energy);
    return;
}

static void SuperCapLostCallback(void *instance)
{
    memset(&supercap->rx_data, 0, sizeof(SuperCap_Rx_Data_s));
    supercap->chassis_real_power = 0;
    supercap->real_energy = 0;
    return;
}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    return DaemonIsOnline(instance->daemon_instance);
}

SuperCapInstance *SuperCapRegister(SuperCap_Init_Config_s *config)
{
    if (!supercap)
    {
        supercap = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
        memset(supercap, 0, sizeof(SuperCapInstance));

        config->can_config.id = supercap;
        config->can_config.can_module_callback = SuperCapRxCallback;
        supercap->can_instance = CANRegister(&config->can_config);

        config->daemon_config.callback = SuperCapLostCallback;
        config->daemon_config.init_count = 200;
        config->daemon_config.owner_id = (void *)supercap;
        config->daemon_config.reload_count = 100;
        supercap->daemon_instance = DaemonRegister(&config->daemon_config);
    }
    return supercap;
}

void SuperCapTask(void)
{
    static uint8_t counter = 0;
    if (counter % 8 == 0) // 200ms周期发送一次控制命令,上位机控制频率较低
    {
         memcpy(supercap->can_instance->tx_buff, &supercap->tx_data, sizeof(SuperCap_Tx_Data_s));
         CANTransmit(supercap->can_instance, 1);
    }
    counter++;
}

float SuperCapGetChassisRealPower(SuperCapInstance *instance)
{
    return (float)(instance->rx_data.chassis_real_power << 1); // 接收的时候右移了一位,所以要左移还原
}

uint8_t SuperCapGetCapEnergy(SuperCapInstance *instance)
{
    return instance->rx_data.energy;
}

uint8_t SuperCapGetReadyFlag(SuperCapInstance *instance)
{
    return instance->rx_data.ready_flag;
}
