#pragma once

#include <stdint.h>

#define POWER_LIMITATOR_MOTOR_COUNT 4u
#define POWER_LIMITATOR_DEFAULT_CURRENT_LIMIT 16384.0f

typedef enum
{
    POWER_LIMITATOR_NOT_RESTRICTED = 0,
    POWER_LIMITATOR_RESTRICTED = 1,
} PowerLimitator_Status_e;

typedef struct
{
    float k0; /* command current to torque coefficient */
    float k1; /* velocity loss coefficient */
    float k2; /* torque squared loss coefficient */
    float k3; /* constant loss of the motor group */
    float current_limit;
} PowerLimitator_Params_s;

typedef struct
{
    PowerLimitator_Params_s params;
    float max_power;
    float cmd_power[POWER_LIMITATOR_MOTOR_COUNT];
    float current_power[POWER_LIMITATOR_MOTOR_COUNT];
    float estimated_power;
    float command_power;
    PowerLimitator_Status_e status;
    uint8_t active[POWER_LIMITATOR_MOTOR_COUNT];
} PowerLimitator_s;

extern const PowerLimitator_Params_s POWER_LIMITATOR_STEER_6020_DEFAULT;
extern const PowerLimitator_Params_s POWER_LIMITATOR_WHEEL_3508_DEFAULT;

void PowerLimitatorInit(PowerLimitator_s *limitator, const PowerLimitator_Params_s *params);

void PowerLimitatorUpdateStatus(PowerLimitator_s *limitator,
                                float updated_max_power,
                                const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                const float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT]);

void PowerLimitatorUpdateStatusMasked(PowerLimitator_s *limitator,
                                      float updated_max_power,
                                      const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                      const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                      const float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT],
                                      const uint8_t active[POWER_LIMITATOR_MOTOR_COUNT]);

void PowerLimitatorGetDecayCurrent(PowerLimitator_s *limitator,
                                   const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                   float decay_current[POWER_LIMITATOR_MOTOR_COUNT]);

void PowerLimitatorLimitCurrents(PowerLimitator_s *limitator,
                                 float updated_max_power,
                                 const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                 const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                 float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT]);

void PowerLimitatorLimitCurrentsMasked(PowerLimitator_s *limitator,
                                       float updated_max_power,
                                       const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                       const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                       float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT],
                                       const uint8_t active[POWER_LIMITATOR_MOTOR_COUNT]);

float PowerLimitatorCalcPower(const PowerLimitator_Params_s *params,
                              float motor_speed_rpm,
                              float torque_current);

float PowerLimitatorClampCurrent(const PowerLimitator_Params_s *params, float current);
