#include "power_calc.h"
#include "robot_def.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define POWER_RPM_TO_RAD_PER_SEC 0.10471975511965977f
#define POWER_FLOAT_EPSILON 1.0e-6f
#define DJI_GM6020_TORQUE_CONSTANT_NM_PER_A 0.741f
#define DJI_GM6020_CURRENT_A_PER_RAW 0.0001f
#define DJI_M3508_TORQUE_CONSTANT_NM_PER_A_AT_19_RATIO 0.3f
#define DJI_C620_CURRENT_A_PER_RAW (20.0f / 16384.0f)

const PowerLimitator_Params_s POWER_LIMITATOR_STEER_6020_DEFAULT = {
    .k0 = DJI_GM6020_TORQUE_CONSTANT_NM_PER_A * DJI_GM6020_CURRENT_A_PER_RAW,
    .k1 = 0.659f,
    .k2 = 31.5f,
    .k3 = 6.4f / 2.0f,
    .current_limit = POWER_LIMITATOR_DEFAULT_CURRENT_LIMIT,
};

const PowerLimitator_Params_s POWER_LIMITATOR_WHEEL_3508_DEFAULT = {
    .k0 = DJI_M3508_TORQUE_CONSTANT_NM_PER_A_AT_19_RATIO * (REDUCTION_RATIO_WHEEL / 19.0f) * DJI_C620_CURRENT_A_PER_RAW,
    .k1 = 0.22f,
    .k2 = 2.135f,
    .k3 = 6.4f / 2.0f,
    .current_limit = POWER_LIMITATOR_DEFAULT_CURRENT_LIMIT,
};

static float PowerLimitatorAbs(float value)
{
    return value >= 0.0f ? value : -value;
}

static uint8_t PowerLimitatorFloatEqual(float value, float target)
{
    return PowerLimitatorAbs(value - target) < POWER_FLOAT_EPSILON;
}

static float PowerLimitatorRpmToRad(float rpm)
{
    return rpm * POWER_RPM_TO_RAD_PER_SEC;
}

static float PowerLimitatorCurrentToTorque(const PowerLimitator_Params_s *params, float current)
{
    return current * params->k0;
}

float PowerLimitatorClampCurrent(const PowerLimitator_Params_s *params, float current)
{
    float limit = POWER_LIMITATOR_DEFAULT_CURRENT_LIMIT;

    if (params != NULL && params->current_limit > 0.0f) {
        limit = params->current_limit;
    }

    if (current > limit) {
        return limit;
    }
    if (current < -limit) {
        return -limit;
    }
    return current;
}

float PowerLimitatorCalcPower(const PowerLimitator_Params_s *params,
                              float motor_speed_rpm,
                              float torque_current)
{
    float w;
    float torque;

    if (params == NULL) {
        return 0.0f;
    }

    w = PowerLimitatorRpmToRad(motor_speed_rpm);
    torque = PowerLimitatorCurrentToTorque(params, torque_current);

    return torque * w +
           params->k1 * PowerLimitatorAbs(w) +
           params->k2 * torque * torque +
           params->k3 / (float)POWER_LIMITATOR_MOTOR_COUNT;
}

void PowerLimitatorInit(PowerLimitator_s *limitator, const PowerLimitator_Params_s *params)
{
    if (limitator == NULL) {
        return;
    }

    memset(limitator, 0, sizeof(*limitator));
    if (params != NULL) {
        limitator->params = *params;
    } else {
        limitator->params = POWER_LIMITATOR_WHEEL_3508_DEFAULT;
    }
    if (limitator->params.current_limit <= 0.0f) {
        limitator->params.current_limit = POWER_LIMITATOR_DEFAULT_CURRENT_LIMIT;
    }
    limitator->status = POWER_LIMITATOR_NOT_RESTRICTED;
}

void PowerLimitatorUpdateStatus(PowerLimitator_s *limitator,
                                float updated_max_power,
                                const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                const float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT])
{
    uint8_t active[POWER_LIMITATOR_MOTOR_COUNT] = {1u, 1u, 1u, 1u};

    PowerLimitatorUpdateStatusMasked(limitator,
                                     updated_max_power,
                                     motor_speed_rpm,
                                     torque_current_feedback,
                                     torque_current_command,
                                     active);
}

void PowerLimitatorUpdateStatusMasked(PowerLimitator_s *limitator,
                                      float updated_max_power,
                                      const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                      const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                      const float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT],
                                      const uint8_t active[POWER_LIMITATOR_MOTOR_COUNT])
{
    float estimated_power = 0.0f;
    float command_power = 0.0f;

    if (limitator == NULL || motor_speed_rpm == NULL ||
        torque_current_feedback == NULL || torque_current_command == NULL) {
        return;
    }

    limitator->max_power = updated_max_power > 0.0f ? updated_max_power : 0.0f;

    for (uint8_t i = 0; i < POWER_LIMITATOR_MOTOR_COUNT; ++i) {
        limitator->active[i] = (active == NULL || active[i]) ? 1u : 0u;
        if (!limitator->active[i]) {
            limitator->current_power[i] = 0.0f;
            limitator->cmd_power[i] = 0.0f;
            continue;
        }
        limitator->current_power[i] = PowerLimitatorCalcPower(&limitator->params,
                                                              motor_speed_rpm[i],
                                                              torque_current_feedback[i]);
        limitator->cmd_power[i] = PowerLimitatorCalcPower(&limitator->params,
                                                          motor_speed_rpm[i],
                                                          torque_current_command[i]);
        estimated_power += limitator->current_power[i];
        command_power += limitator->cmd_power[i];
    }

    limitator->estimated_power = estimated_power;
    limitator->command_power = command_power;
}

void PowerLimitatorGetDecayCurrent(PowerLimitator_s *limitator,
                                   const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                   float decay_current[POWER_LIMITATOR_MOTOR_COUNT])
{
    float allocatable_power;
    float positive_power_required = 0.0f;

    if (limitator == NULL || motor_speed_rpm == NULL || decay_current == NULL) {
        return;
    }

    allocatable_power = limitator->max_power;
    for (uint8_t i = 0; i < POWER_LIMITATOR_MOTOR_COUNT; ++i) {
        if (!limitator->active[i]) {
            continue;
        }
        if (limitator->cmd_power[i] > 0.0f) {
            positive_power_required += limitator->cmd_power[i];
        } else {
            allocatable_power -= limitator->cmd_power[i];
        }
    }

    if (limitator->command_power <= limitator->max_power ||
        positive_power_required <= POWER_FLOAT_EPSILON) {
        limitator->status = POWER_LIMITATOR_NOT_RESTRICTED;
        for (uint8_t i = 0; i < POWER_LIMITATOR_MOTOR_COUNT; ++i) {
            if (!limitator->active[i]) {
                continue;
            }
            decay_current[i] = PowerLimitatorClampCurrent(&limitator->params, decay_current[i]);
        }
        return;
    }

    limitator->status = POWER_LIMITATOR_RESTRICTED;

    for (uint8_t i = 0; i < POWER_LIMITATOR_MOTOR_COUNT; ++i) {
        float cur_av;
        float power_weight;
        float delta;
        float root;

        if (!limitator->active[i]) {
            continue;
        }

        if (PowerLimitatorFloatEqual(limitator->cmd_power[i], 0.0f) ||
            limitator->cmd_power[i] < 0.0f) {
            decay_current[i] = PowerLimitatorClampCurrent(&limitator->params, decay_current[i]);
            continue;
        }

        cur_av = PowerLimitatorRpmToRad(motor_speed_rpm[i]);
        power_weight = limitator->cmd_power[i] / positive_power_required;
        delta = cur_av * cur_av -
                4.0f * limitator->params.k2 *
                    (limitator->params.k1 * PowerLimitatorAbs(cur_av) +
                     limitator->params.k3 / (float)POWER_LIMITATOR_MOTOR_COUNT -
                     power_weight * allocatable_power);

        if (PowerLimitatorFloatEqual(delta, 0.0f) || delta < 0.0f) {
            root = 0.0f;
        } else if (decay_current[i] > 0.0f) {
            root = (-cur_av + sqrtf(delta)) / (2.0f * limitator->params.k2) / limitator->params.k0;
        } else {
            root = (-cur_av - sqrtf(delta)) / (2.0f * limitator->params.k2) / limitator->params.k0;
        }

        decay_current[i] = PowerLimitatorClampCurrent(&limitator->params, root);
    }
}

void PowerLimitatorLimitCurrents(PowerLimitator_s *limitator,
                                 float updated_max_power,
                                 const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                 const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                 float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT])
{
    PowerLimitatorUpdateStatus(limitator,
                               updated_max_power,
                               motor_speed_rpm,
                               torque_current_feedback,
                               torque_current_command);
    PowerLimitatorGetDecayCurrent(limitator, motor_speed_rpm, torque_current_command);
}

void PowerLimitatorLimitCurrentsMasked(PowerLimitator_s *limitator,
                                       float updated_max_power,
                                       const float motor_speed_rpm[POWER_LIMITATOR_MOTOR_COUNT],
                                       const float torque_current_feedback[POWER_LIMITATOR_MOTOR_COUNT],
                                       float torque_current_command[POWER_LIMITATOR_MOTOR_COUNT],
                                       const uint8_t active[POWER_LIMITATOR_MOTOR_COUNT])
{
    PowerLimitatorUpdateStatusMasked(limitator,
                                     updated_max_power,
                                     motor_speed_rpm,
                                     torque_current_feedback,
                                     torque_current_command,
                                     active);
    PowerLimitatorGetDecayCurrent(limitator, motor_speed_rpm, torque_current_command);
}
